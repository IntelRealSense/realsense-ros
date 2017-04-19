// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once
#include <iostream>
#include <functional>
#include <iomanip>
#include <map>
#include <atomic>
#include <thread>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <rs_sdk.h>
#include <librealsense/slam/slam.h>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Eigen>
#include "ParamTypes.h"
#include "2DMapCommon.h"
#include <ros/ros.h>
#include <librealsense/slam/slam.h>
#include <rs_core.h>
#include <rs_utils.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <realsense_ros_camera/Extrinsics.h>
#include <realsense_ros_camera/IMUInfo.h>
#include <realsense_ros_slam/Reset.h>
#include <realsense_ros_slam/SaveOutput.h>

namespace realsense_ros_slam
{
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
  void getStreamSample(const sensor_msgs::ImageConstPtr &imageMsg, rs::core::stream_type stream);
  void depthMessageCallback(const sensor_msgs::ImageConstPtr &depthImageMsg);
  void fisheyeMessageCallback(const sensor_msgs::ImageConstPtr &fisheyeImageMsg);
  void motionGyroCallback(const sensor_msgs::ImuConstPtr &imuMsg);
  void motionAccelCallback(const sensor_msgs::ImuConstPtr &imuMsg);
  void getMotionSample(const sensor_msgs::ImuConstPtr &imuMsg, rs::core::motion_type motionType);
  void onInit(ros::NodeHandle & nh, rs::slam::slam * slam);
};

class SNodeletSlam: public nodelet::Nodelet
{
public:
  SNodeletSlam();
  ~SNodeletSlam();
  ros::NodeHandle nh;
  ros::Subscriber sub_depthInfo, sub_fisheyeInfo, sub_accelInfo, sub_gyroInfo, sub_fe2imu, sub_fe2depth;
  rs::core::video_module_interface::supported_module_config supported_config;
  rs::core::video_module_interface::actual_module_config actual_config;
  SubscribeTopics sub;

  void onInit()override;
private:
  sensor_msgs::CameraInfoConstPtr depthCameraInfo_, fisheyeCameraInfo_;
  realsense_ros_camera::IMUInfoConstPtr accelInfo_, gyroInfo_;
  realsense_ros_camera::ExtrinsicsConstPtr fe2imu_, fe2depth_;
  std::mutex mut_init_;
  std::unique_ptr<rs::slam::slam> slam_;
  void depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& depthCameraInfo);
  void fisheyeInfoCallback(const sensor_msgs::CameraInfoConstPtr& fisheyeCameraInfo);
  void accelInfoCallback(const realsense_ros_camera::IMUInfoConstPtr& accelInfo);
  void gyroInfoCallback(const realsense_ros_camera::IMUInfoConstPtr& gyroInfo);
  void fe2ImuCallback(const realsense_ros_camera::ExtrinsicsConstPtr& fe2imu);
  void fe2depthCallback(const realsense_ros_camera::ExtrinsicsConstPtr& fe2depth);
  void startIfReady();  
  void startSlam();
  void setCalibrationData(const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg, rs::core::intrinsics & cameraInfo);
  void setStreamConfigIntrin(rs::core::stream_type stream, std::map< rs::core::stream_type, rs::core::intrinsics > intrinsics);
  void setMotionData(realsense_ros_camera::IMUInfo& imu_res, rs::core::motion_device_intrinsics& motion_intrin);
  void setMotionConfigIntrin(rs::core::motion_type motion, std::map< rs::core::motion_type, rs::core::motion_device_intrinsics > motion_intrinsics);
  void setExtrinData(realsense_ros_camera::Extrinsics& fe_res, rs::core::extrinsics& extrinsics);  
  bool reset(realsense_ros_slam::Reset::Request &req, realsense_ros_slam::Reset::Response &resp); 
  bool saveOutput(realsense_ros_slam::SaveOutput::Request &req, realsense_ros_slam::SaveOutput::Response &resp);
};//end class
}
