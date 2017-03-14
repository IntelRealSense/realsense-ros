// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once

#include <PersonTracking2RosHelper.h>
#include "RealSense/PersonTracking/PersonTrackingConfiguration.h"
#include "RealSense/PersonTracking/PersonTrackingData.h"

#include "realsense_ros_person/User.h"
#include "realsense_ros_person/PersonModuleState.h"

namespace realsense_ros_person
{
class PersonTrackingPublisherHelper
{
public:
  PersonTrackingPublisherHelper() = default;

  ~PersonTrackingPublisherHelper() = default;

  virtual std::vector<realsense_ros_person::User>
  BuildUsersVector(Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
                   Intel::RealSense::PersonTracking::PersonTrackingData &trackingData);

  virtual void addSkeletonToOutput(Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
                                   Intel::RealSense::PersonTracking::PersonTrackingData::Person *const personData,
                                   realsense_ros_person::User &user);

  virtual void addGesturesToOutput(Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
                                   Intel::RealSense::PersonTracking::PersonTrackingData::Person *const personData,
                                   realsense_ros_person::User &user);

  virtual void
  addLandmarksToOutput(Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
                       Intel::RealSense::PersonTracking::PersonTrackingData::Person *const personData,
                       realsense_ros_person::User &user);

  virtual void
  addHeadPoseToOutput(Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
                      Intel::RealSense::PersonTracking::PersonTrackingData::Person *const personData,
                      realsense_ros_person::User &user);

  virtual
  realsense_ros_person::PersonModuleState BuildPersonModuleState(
    Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
    Intel::RealSense::PersonTracking::PersonTrackingData &trackingData);

  PersonTracking2RosHelper mPt2rosHelper;
};
}