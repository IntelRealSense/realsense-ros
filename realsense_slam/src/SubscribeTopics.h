#pragma once
#include <ros/ros.h>
#include <librealsense/slam/slam.h>
#include <rs_core.h>
#include <rs_utils.h>
#include <realsense_camera/StreamInfo.h>
#include <realsense_camera/MotionInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class SubscribeTopics
{
public:
  SubscribeTopics();
  ~SubscribeTopics();
  ros::NodeHandle l_nh;
  rs::slam::slam* l_slam;
  std::mutex mut_depth, mut_fisheye, mut_gyro_imu, mut_accel_imu;
  std::vector<void *> data_list;
  ros::Subscriber l_motion_gyro_sub, l_motion_accel_sub, l_fisheye_sub, l_depth_sub;
  void subscribeStreamMessages();
  void subscribeMotion();
  void GetStreamSample(const realsense_camera::StreamInfoConstPtr & imageMsg, rs::core::stream_type stream);
  void depthMessageCallback(const realsense_camera::StreamInfoConstPtr & depthImageMsg);
  void fisheyeMessageCallback(const realsense_camera::StreamInfoConstPtr & fisheyeImageMsg);
  void motion_gyroCallback(const realsense_camera::MotionInfoConstPtr & motionInfoMsg);
  void motion_accelCallback(const realsense_camera::MotionInfoConstPtr & motionInfoMsg);
  void GetMotionSample(const realsense_camera::MotionInfoConstPtr & motionInfoMsg, rs::core::motion_type motionType);
  void onInit(ros::NodeHandle & nh, rs::slam::slam * slam);
};
