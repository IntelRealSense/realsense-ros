#pragma once

#include <string>
#include "PersonTrackingConfig.h"
#include <ros/ros.h>
#include "RealSense/PersonTracking/PersonTrackingConfiguration.h"

//parse command line arguments and configure person tracking
void ConfigurePersonTracking(PersonTrackingConfig config, Intel::RealSense::PersonTracking::PersonTrackingConfiguration* ptConfiguration);

//load person tracking recognition database from file
void LoadDatabase(std::string dbPath, Intel::RealSense::PersonTracking::PersonTrackingConfiguration* ptConfiguration);

//save person tracking recognition database from file
void SaveDatabase(std::string dbPath, Intel::RealSense::PersonTracking::PersonTrackingConfiguration* ptConfiguration);

void SetTracking(const PersonTrackingConfig& config, Intel::RealSense::PersonTracking::PersonTrackingData* trackingData);

