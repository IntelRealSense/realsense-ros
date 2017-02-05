#pragma once

#include <string>
#include <ros/ros.h>
#include "RealSense/PersonTracking/PersonTrackingConfiguration.h"

namespace realsense_ros_person {
    struct PersonTrackingConfig {
        bool gesturesEnabled;
        bool skeletonEnabled;
        bool recognitionEnabled;
        bool trackingEnabled;
        bool headPositionEnabled;
        bool headBoundingBoxEnabled;
        bool landmarksEnabled;

    public:
        PersonTrackingConfig() :
                gesturesEnabled(false),
                skeletonEnabled(false),
                recognitionEnabled(false),
                trackingEnabled(false),
                headPositionEnabled(false),
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