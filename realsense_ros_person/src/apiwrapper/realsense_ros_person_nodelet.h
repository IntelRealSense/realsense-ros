// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <vector>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "ros/ros.h"
#include "ros/package.h"
#include "ros/time.h"
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "opencv2/imgproc/types_c.h"
#include "opencv2/opencv.hpp"
#include <librealsense/rs.hpp>
#include "rs_sdk.h"
#include "person_tracking_video_module_factory.h"
#include "PersonTrackingHelper.h"

#include "PersonTrackingServer.h"
#include "PersonTrackingDefaultPublisher.h"
#include "Ros2RealsenseSdkConverter.h"

namespace realsense_ros_person
{
class TNodeletPt: public nodelet::Nodelet
{
public:
  TNodeletPt();
  ros::NodeHandle nh;
  void onInit()override;
private:
  std::unique_ptr<rs::person_tracking::person_tracking_video_module_interface> ptModule;


  std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> mColorCameraInfoSubscriber;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> mDepthCameraInfoSubscriber;
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>> mTimeSynchronizer_info;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> mDepthSubscriber;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> mColorSubscriber;
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>> mTimeSynchronizer;

  std::mutex mProcessingMutex;
  std::unique_ptr<AbsPersonTrackingPublisher> mPublisher;
  PersonTrackingServer mServer;
  uint64_t seq;

  //set ros params
  void setParams(PersonTrackingConfig& config);

  void personTrackingCallback(const sensor_msgs::ImageConstPtr& depthImageMsg, const sensor_msgs::ImageConstPtr& colorImageMsg);
  void subscribeFrameMessages();
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& colorCameraInfo, const sensor_msgs::CameraInfoConstPtr& depthCameraInfo);
  PersonTrackingConfig config;

  static const std::string DEFAULT_PUBLISHER;
  std::unique_ptr<realsense_ros_person::Ros2RealSenseSdkConverter> m_ros2realsense;
  bool m_isTestMode;
};
}

