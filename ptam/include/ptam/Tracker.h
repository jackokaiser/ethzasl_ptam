//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
//
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to
// do simple patch tracking across a stereo pair. This is handled
// by the TrackForInitialMap() method and associated sub-methods.
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef __TRACKER_H
#define __TRACKER_H

#include "MapMaker.h"
#include "ATANCamera.h"
#include "MiniPatch.h"
#include "Relocaliser.h"
#include "ptam/Params.h"

#include <sstream>
#include <vector>
#include <list>

struct TrackerData;
struct Trail    // This struct is used for initial correspondences of the first stereo pair.
{
  MiniPatch mPatch;
  CVD::ImageRef irCurrentPos;
  CVD::ImageRef irInitialPos;
};

typedef std::vector<CVD::ImageRef> Bearing;    // Position of one corner over multiple observations, used by Closed Form

class Tracker
{
public:
  Tracker(CVD::ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm);

  // TrackFrame is the main working part of the tracker: call this every frame.
  void TrackFrame(CVD::Image<CVD::byte> &imFrame, bool bDraw, const ros::Time &timestamp);
  void TrackFrame(CVD::Image<CVD::byte> &imFrame, bool bDraw, const TooN::SO3<double> & imuOrientation, const ros::Time &timestamp);

  inline SE3<> GetCurrentPose() { return mse3CamFromWorld;}
  //Weiss{
  inline Matrix<6> GetCurrentCov() { return mmCovariances;}
  inline KeyFrame::Ptr GetCurrentKF() { return mCurrentKF;}
  Vector<3> CalcSBIRotation(SmallBlurryImage *SBI1, SmallBlurryImage *SBI2);
  //}
  // Gets messages to be printed on-screen for the user.
  std::string GetMessageForUser();
  int getTrackingQuality(){return mTrackingQuality;}

  void command(const std::string & params);
  std::list<Trail> & getTrails(){return  mlTrails;};
  std::list<Bearing> & getBearings(){return  mlBearings;};

  bool getTrailTrackingStarted(){return mnInitialStage == TRAIL_TRACKING_STARTED;};
  bool getTrailTrackingComplete(){return mnInitialStage == TRAIL_TRACKING_COMPLETE;};
  CVD::Image<TooN::Vector<2> > & ComputeGrid();             // Computes the reference grid;

protected:
  CVD::Image<TooN::Vector<2> > mProjVertices;
  KeyFrame::Ptr mCurrentKF;            // The current working frame as a keyframe struct

  // The major components to which the tracker needs access:
  Map &mMap;                      // The map, consisting of points and keyframes
  MapMaker &mMapMaker;            // The class which maintains the map
  ATANCamera mCamera;             // Projection model
  Relocaliser mRelocaliser;       // Relocalisation module

  CVD::ImageRef mirSize;          // Image size of whole image

  void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.
  void RenderGrid();              // Draws the reference grid

  // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
  void TrackForInitialMap(const ros::Time& timestamp);      // This is called by TrackFrame if there is not a map yet.
  enum {TRAIL_TRACKING_NOT_STARTED,
    TRAIL_TRACKING_STARTED,
    TRAIL_TRACKING_COMPLETE} mnInitialStage;  // How far are we towards making the initial map?
    void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
    int  TrailTracking_Advance();   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
    std::list<Trail> mlTrails;      // Used by trail tracking
    std::list<Bearing> mlBearings;      // Used by closed form for map initialization
    std::vector<ros::Time> bearingTimestamps;
    ros::Time initialTimestamp;

    KeyFrame::Ptr mFirstKF;              // First of the stereo pair
    KeyFrame::Ptr mPreviousFrameKF;      // Used by trail tracking to check married matches

    // Methods for tracking the map once it has been made:
    void TrackMap();                // Called by TrackFrame if there is a map.
    void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
    void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
    void UpdateMotionModel();       // Motion model is updated after TrackMap
    int SearchForPoints(std::vector<TrackerData*> &vTD,
                        int nRange,
                        int nFineIts);  // Finds points in the image
    Vector<6> CalcPoseUpdate(std::vector<TrackerData*> vTD,
                             double dOverrideSigma = 0.0,
                             bool bMarkOutliers = false); // Updates pose from found points.
    SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.
    SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
    Vector<6> mv6CameraVelocity;    // Motion model
    double mdVelocityMagnitude;     // Used to decide on coarse tracking
    double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
    bool mbDidCoarse;               // Did tracking use the coarse tracking stage?

    bool mbDraw;                    // Should the tracker draw anything to OpenGL?

    // achtelik{
    SO3<> mso3CurrentImu;
    SO3<> mso3LastImu;

    //}


    // Weiss{
    Matrix<6>	mmCovariances;		// covariance of current converged pose estimate
    KeyFrame::Ptr mOldKF;
    bool mAutoreset;				// indicates if user pressed reset or not
    //}

    // Interface with map maker:
    int mnFrame;                    // Frames processed since last reset
    int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.
    void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe

    // Tracking quality control:
    int manMeasAttempted[LEVELS];
    int manMeasFound[LEVELS];
    enum {BAD, DODGY, GOOD} mTrackingQuality;
    int mnLostFrames;

    // Relocalisation functions:
    bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.
    bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

    // Frame-to-frame motion init:
    SmallBlurryImage *mpSBILastFrame;
    SmallBlurryImage *mpSBIThisFrame;
    void CalcSBIRotation();
    Vector<6> mv6SBIRot;
    bool mbUseSBIInit;

    // User interaction for initial tracking:
    bool mbUserPressedSpacebar;
    std::ostringstream mMessageForUser;

    // GUI interface:
    void GUICommandHandler(std::string sCommand, std::string sParams);
    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
    struct Command {std::string sCommand; std::string sParams; };
    std::vector<Command> mvQueuedCommands;


};

#endif
