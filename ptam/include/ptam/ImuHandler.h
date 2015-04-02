// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// ImuHandler.h
//
// Defines the ImuHandler class, singleton pattern
//
// This handles everything related to the IMU in PTAM
// for motion prediction and closed form.
//
#ifndef __IMU_HANDLER_H
#define __IMU_HANDLER_H

#include <sensor_msgs/Imu.h>
#include <queue>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/so3.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class ImuHandler
{
  typedef std::queue<sensor_msgs::Imu> ImuQueue;

public:
  static ImuHandler& getInstance()
  {
    static ImuHandler instance; // Guaranteed to be destroyed.
    // Instantiated on first use.
    return instance;
  }

  void setLimitImuQueue (unsigned int limit);
  void unsetLimitImuQueue ();
  void setCollectImuMsg(bool b);
  void flushMsgs ();
  void imuCallback(const sensor_msgs::ImuConstPtr & msg);

  void getImuTransform (TooN::SO3<double>& lastOrientation, TooN::SO3<double>& newOrientation);
  void computeImuTransform(const tf::TransformListener& tf_sub, const ros::Time& timestamp, const std::string& frame_id);
  bool transformQuaternion(const tf::TransformListener& tf_sub, const std::string & target_frame, const std_msgs::Header & header,
                           const geometry_msgs::Quaternion & _q_in, TooN::SO3<double> & r_out);
  /// finds object in queue with timestamp closest to timestamp. Requires that T has a std_msgs::header field named "header"
  template<class T> bool findClosest(const ros::Time & timestamp, std::queue<T> & queue, T * obj, const double & max_delay = 0.01);

  void quaternionToRotationMatrix(const geometry_msgs::Quaternion & q, TooN::SO3<double> & R);

private:
  ImuHandler() {};                   // Constructor? (the {} brackets) are needed here.

  /* prevent copy */
  ImuHandler(ImuHandler const&);              // Don't Implement
  void operator=(ImuHandler const&); // Don't implement

  TooN::SO3<double> currentImuTransform;
  TooN::SO3<double> lastImuTransform;
  ImuQueue imuMsgs;
  unsigned int limitImuQueue;
  bool isQueueLimited;
  bool isCollectingImuMsg;
};

#endif
