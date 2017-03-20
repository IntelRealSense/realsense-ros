// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once
#include "ros/ros.h"
#include "person_tracking_video_module_factory.h"
#include <sensor_msgs/Image.h>
#include "realsense_ros_person/User.h"
#include "realsense_ros_person/Frame.h"

namespace realsense_ros_person
{
class AbsPersonTrackingPublisher
{
public:
  AbsPersonTrackingPublisher() = default;

  virtual void publishOutput(Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
                             Intel::RealSense::PersonTracking::PersonTrackingData &trackingData) = 0;

  virtual void publishTestOutput(Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
                                 Intel::RealSense::PersonTracking::PersonTrackingData &trackingData,
                                 const sensor_msgs::ImageConstPtr &colorImage) = 0;

  virtual void onInit(ros::NodeHandle &nodeHandle) = 0;

  virtual ~AbsPersonTrackingPublisher() = default;

protected:
  ros::Publisher mPublisher;
  ros::Publisher mTestPublisher;
};
}