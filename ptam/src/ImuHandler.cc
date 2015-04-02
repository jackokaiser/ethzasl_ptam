#include "ptam/ImuHandler.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

using namespace std;

void ImuHandler::imuCallback(const sensor_msgs::ImuConstPtr & msg)
{
  if (isCollectingImuMsg)
  {
    imuMsgs.push(*msg);
    if (isQueueLimited && imuMsgs.size() > limitImuQueue)
    {
      imuMsgs.pop();
    }
  }
}

ImuHandler::ImuQueue ImuHandler::getMsgs()
{
  return imuMsgs;
}

void ImuHandler::flushMsgs()
{
  ImuQueue empty;
  std::swap( imuMsgs, empty );
}

void ImuHandler::setLimitImuQueue(unsigned int limit)
{
  limitImuQueue = limit;
  isQueueLimited = true;
}

void ImuHandler::unsetLimitImuQueue()
{
  isQueueLimited = false;
}
void ImuHandler::setCollectImuMsg(bool b)
{
  isCollectingImuMsg = b;
}

void ImuHandler::getImuTransform(TooN::SO3<double>& mso3LastImu, TooN::SO3<double>& mso3CurrentImu)
{
  mso3LastImu = lastImuTransform;
  mso3CurrentImu = currentImuTransform;
}

void ImuHandler::computeImuTransform(const tf::TransformListener& tf_sub, const ros::Time& timestamp, const string& frame_id)
{
  lastImuTransform = currentImuTransform;
  sensor_msgs::Imu imu;
  if (!findClosest(timestamp, imuMsgs, &imu, 0.01))
  {
    ROS_WARN("no imu match, skipping frame");
    return;
  }
  if (!transformQuaternion(tf_sub, frame_id, imu.header, imu.orientation, currentImuTransform))
  {
    return;
  }
}

bool ImuHandler::transformQuaternion(const tf::TransformListener& tf_sub, const std::string & target_frame, const std_msgs::Header & header,
                                     const geometry_msgs::Quaternion & _q_in, TooN::SO3<double> & r_out)
{
  geometry_msgs::QuaternionStamped q_in, q_out;
  q_in.header = header;
  q_in.quaternion = _q_in;
  try
  {
    tf_sub.transformQuaternion(target_frame, q_in, q_out);
    quaternionToRotationMatrix(q_out.quaternion, r_out);
    return true;
  }
  catch (tf::TransformException & e)
  {
    ROS_WARN_STREAM("could not transform quaternion: "<<e.what());
    return false;
  }
  return true;
}

void ImuHandler::quaternionToRotationMatrix(const geometry_msgs::Quaternion & q, TooN::SO3<double> & R)
{
  // stolen from Eigen3 and adapted to TooN

  TooN::Matrix<3, 3, double> res;

  const double tx = 2 * q.x;
  const double ty = 2 * q.y;
  const double tz = 2 * q.z;
  const double twx = tx * q.w;
  const double twy = ty * q.w;
  const double twz = tz * q.w;
  const double txx = tx * q.x;
  const double txy = ty * q.x;
  const double txz = tz * q.x;
  const double tyy = ty * q.y;
  const double tyz = tz * q.y;
  const double tzz = tz * q.z;

  res(0, 0) = 1 - (tyy + tzz);
  res(0, 1) = txy - twz;
  res(0, 2) = txz + twy;
  res(1, 0) = txy + twz;
  res(1, 1) = 1 - (txx + tzz);
  res(1, 2) = tyz - twx;
  res(2, 0) = txz - twy;
  res(2, 1) = tyz + twx;
  res(2, 2) = 1 - (txx + tyy);

  R = res;

  //  R = TooN::SO3<double>::exp(TooN::makeVector<double>(q.x, q.y, q.z) * acos(q.w) * 2.0 / sqrt(q.x * q.x + q.y * q.y + q.z * q.z));
}


template<class T>
bool ImuHandler::findClosest(const ros::Time & timestamp, std::queue<T> & queue, T * obj, const double & max_delay)
{
  double best_dt(1e9);
  double tmp_dt;
  //  size_t qs_before = queue.size();
  //  int i = 0;
  while (!queue.empty())
  {
    const T & curr_obj = queue.front();
    tmp_dt = (timestamp - curr_obj.header.stamp).toSec();

    if (tmp_dt < -max_delay)
      break;
    if (std::abs(tmp_dt) < best_dt)
    {
      best_dt = std::abs(tmp_dt);
      *obj = curr_obj;
      //      i++;
    }
    queue.pop();
  }
  if (best_dt > max_delay)
  {
    //    ROS_WARN("dt(%f) > 0.01 qs:%d, %d/%d", best_dt, queue.size(), qs_before, i);
    return false;
  }
  else
  {
    return true;
  };
}
