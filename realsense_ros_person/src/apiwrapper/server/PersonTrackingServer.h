// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once
#include <ros/ros.h>
#include <realsense_ros_person/StartTracking.h>
#include <realsense_ros_person/StopTracking.h>
#include <realsense_ros_person/Recognition.h>
#include <realsense_ros_person/RecognitionRegister.h>
#include <realsense_ros_person/TrackingConfig.h>
#include <realsense_ros_person/LoadRecognitionDB.h>
#include <realsense_ros_person/SaveRecognitionDB.h>
#include <PersonTracking2RosHelper.h>
#include "opencv2/opencv.hpp"

#include "person_tracking_video_module_factory.h"
#include "PersonTrackingHelper.h"

namespace realsense_ros_person
{
class PersonTrackingServer
{
public:
  typedef Intel::RealSense::PersonTracking::PersonTrackingData PXCPersonTrackingData;
  typedef Intel::RealSense::PersonTracking::PersonTrackingConfiguration PXCPersonTrackingConfiguration;

  PersonTrackingServer();

  void
  onInit(ros::NodeHandle &nodeHandle, rs::person_tracking::person_tracking_video_module_interface *personTracking,
         PersonTrackingConfig &config);

private:
  bool trackingConfigRequestCallback(realsense_ros_person::TrackingConfig::Request &request,
                                     realsense_ros_person::TrackingConfig::Response &response);

  bool recognitionRequestCallback(realsense_ros_person::RecognitionRequest &request,
                                  realsense_ros_person::Recognition::Response &response);

  bool recognitionRegisterRequestCallback(realsense_ros_person::RecognitionRegister::Request &request,
                                          realsense_ros_person::RecognitionRegister::Response &response);

  bool startTrackingRequestCallback(realsense_ros_person::StartTracking::Request &request,
                                    realsense_ros_person::StartTracking::Response &response);

  bool stopTrackingRequestCallback(realsense_ros_person::StopTracking::Request &request,
                                   realsense_ros_person::StopTracking::Response &response);

  bool startStopTracking(bool isStart, int personId);

  bool saveRecognitionDbCallback(realsense_ros_person::SaveRecognitionDB::Request &request,
                                 realsense_ros_person::SaveRecognitionDB::Response &response);

  bool loadRecognitionDbCallback(realsense_ros_person::LoadRecognitionDB::Request &request,
                                 realsense_ros_person::LoadRecognitionDB::Response &response);


  ros::ServiceServer mTrackingConfigService;
  ros::ServiceServer mRecognitionRequestService;
  ros::ServiceServer mRecognitionRegisterRequestService;
  ros::ServiceServer mStartTrackingService;
  ros::ServiceServer mStopTrackingService;
  ros::ServiceServer mSaveRecognitionDbService;
  ros::ServiceServer mLoadRecognitionDbService;
  ros::ServiceServer mRecognitionImgRequestService;

  realsense_ros_person::PersonTracking2RosHelper m_pt2rosHelper;
  rs::person_tracking::person_tracking_video_module_interface *mPersonTracking;
  PersonTrackingConfig mConfig;
};
}