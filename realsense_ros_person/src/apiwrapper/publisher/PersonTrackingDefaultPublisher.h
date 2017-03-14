// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once

#include <memory>

#include "AbsPersonTrackingPublisher.h"
#include "PersonTrackingPublisherHelper.h"

namespace realsense_ros_person
{
class PersonTrackingDefaultPublisher : public AbsPersonTrackingPublisher
{
public:
  PersonTrackingDefaultPublisher() = default;

  virtual void publishOutput(Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
                             Intel::RealSense::PersonTracking::PersonTrackingData &trackingData) override;

  virtual void publishTestOutput(Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
                                 Intel::RealSense::PersonTracking::PersonTrackingData &trackingData,
                                 const sensor_msgs::ImageConstPtr &colorImage) override;

  virtual void onInit(ros::NodeHandle &nodeHandle) override;

  virtual  ~PersonTrackingDefaultPublisher() = default;

protected:
  PersonTrackingPublisherHelper mPtPublisherHelper;
  ros::Publisher mPersonModuleStatePublisher;
  void FillFrameData(realsense_ros_person::Frame &frame,
                     Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
                     Intel::RealSense::PersonTracking::PersonTrackingData &trackingData);
};
}