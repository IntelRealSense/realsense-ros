// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once

#include <string>
#include <ros/ros.h>
#include "RealSense/PersonTracking/PersonTrackingConfiguration.h"

namespace realsense_ros_person
{
struct PersonTrackingConfig
{
  bool pointingGestureEnabled;
  bool waveGestureEnabled;
  bool skeletonEnabled;
  bool recognitionEnabled;
  bool trackingEnabled;
  bool headPoseEnabled;
  bool headBoundingBoxEnabled;
  bool landmarksEnabled;

public:
  PersonTrackingConfig() :
    pointingGestureEnabled(false),
    waveGestureEnabled(false),
    skeletonEnabled(false),
    recognitionEnabled(false),
    trackingEnabled(false),
    headPoseEnabled(false),
    headBoundingBoxEnabled(false),
    landmarksEnabled(false) {}
};

//parse command line arguments and configure person tracking
void ConfigurePersonTracking(PersonTrackingConfig config,
                             Intel::RealSense::PersonTracking::PersonTrackingConfiguration *ptConfiguration);

//load person tracking recognition database from file
bool
LoadDatabase(std::string dbPath, Intel::RealSense::PersonTracking::PersonTrackingConfiguration *ptConfiguration);

//save person tracking recognition database from file
bool
SaveDatabase(std::string dbPath, Intel::RealSense::PersonTracking::PersonTrackingConfiguration *ptConfiguration);
}