#pragma once
#include <ros/ros.h>
#include <realsense_srvs/TrackingRequest.h>
#include <realsense_srvs/RecognitionRequest.h>
#include <realsense_srvs/TrackingConfig.h>
#include <realsense_srvs/LoadRecognitionDB.h>
#include <realsense_srvs/SaveRecognitionDB.h>
#include "opencv2/opencv.hpp"

#include <realsense_srvs/RecognitionImgRequest.h>
#include "person_tracking_video_module_factory.h"
#include "PersonTrackingHelper.h"

class PersonTrackingServer
{
public:
  typedef Intel::RealSense::PersonTracking::PersonTrackingData PXCPersonTrackingData;
  typedef Intel::RealSense::PersonTracking::PersonTrackingConfiguration PXCPersonTrackingConfiguration;
  PersonTrackingServer();
  void onInit(ros::NodeHandle & nodeHandle, rs::person_tracking::person_tracking_video_module_interface * personTracking,  PersonTrackingConfig & config);
  bool autoImgRecog;
private:
  void CreateDepthImage(rs::core::correlated_sample_set & sampleSet, const void * depthData, int width, int height);
  void CreateColorImage(rs::core::correlated_sample_set & sampleSet, const void * colorData, int width, int height);
  bool trackingConfigRequestCallback(realsense_srvs::TrackingConfig::Request & request, realsense_srvs::TrackingConfig::Response & response);
  bool recognitionRequestCallback(realsense_srvs::RecognitionRequest::Request & request, realsense_srvs::RecognitionRequest::Response & response);
  bool trackingRequestCallback(realsense_srvs::TrackingRequest::Request & request, realsense_srvs::TrackingRequest::Response & response);
  bool saveRecognitionDbCallback(realsense_srvs::SaveRecognitionDB::Request & request, realsense_srvs::SaveRecognitionDB::Response & response);
  bool loadRecognitionDbCallback(realsense_srvs::LoadRecognitionDB::Request & request, realsense_srvs::LoadRecognitionDB::Response & response);
  bool recognitionImgRequestCallback(realsense_srvs::RecognitionImgRequest::Request & request, realsense_srvs::RecognitionImgRequest::Response & response);
  ros::ServiceServer mTrackingConfigService;
  ros::ServiceServer mRecognitionRequestService;
  ros::ServiceServer mTrackingRequestService;
  ros::ServiceServer mSaveRecognitionDbService;
  ros::ServiceServer mLoadRecognitionDbService;
  ros::ServiceServer mRecognitionImgRequestService;
  rs::person_tracking::person_tracking_video_module_interface* mPersonTracking;
  PersonTrackingConfig mConfig;
};
