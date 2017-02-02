#pragma once
#include <ros/ros.h>
#include <realsense_ros_person/TrackingRequest.h>
#include <realsense_ros_person/RecognitionRequest.h>
#include <realsense_ros_person/TrackingConfig.h>
#include <realsense_ros_person/LoadRecognitionDB.h>
#include <realsense_ros_person/SaveRecognitionDB.h>
#include "opencv2/opencv.hpp"

#include <realsense_ros_person/RecognitionImgRequest.h>
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
  bool trackingConfigRequestCallback(realsense_ros_person::TrackingConfig::Request & request, realsense_ros_person::TrackingConfig::Response & response);
  bool recognitionRequestCallback(realsense_ros_person::RecognitionRequest::Request & request, realsense_ros_person::RecognitionRequest::Response & response);
  bool trackingRequestCallback(realsense_ros_person::TrackingRequest::Request & request, realsense_ros_person::TrackingRequest::Response & response);
  bool saveRecognitionDbCallback(realsense_ros_person::SaveRecognitionDB::Request & request, realsense_ros_person::SaveRecognitionDB::Response & response);
  bool loadRecognitionDbCallback(realsense_ros_person::LoadRecognitionDB::Request & request, realsense_ros_person::LoadRecognitionDB::Response & response);
  bool recognitionImgRequestCallback(realsense_ros_person::RecognitionImgRequest::Request & request, realsense_ros_person::RecognitionImgRequest::Response & response);
  ros::ServiceServer mTrackingConfigService;
  ros::ServiceServer mRecognitionRequestService;
  ros::ServiceServer mTrackingRequestService;
  ros::ServiceServer mSaveRecognitionDbService;
  ros::ServiceServer mLoadRecognitionDbService;
  ros::ServiceServer mRecognitionImgRequestService;
  rs::person_tracking::person_tracking_video_module_interface* mPersonTracking;
  PersonTrackingConfig mConfig;
};
