// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once
#include "ros/ros.h"
#include "realsense_ros_person/FrameTest.h"
#include "TrackingRenderer/TrackingRenderer.h"
#include <mutex>

const char WINDOW_NAME[] = "PersonTracking";

class PersonTrackingSample
{
public:
  PersonTrackingSample();

  void ProcessCommandLineArgs();

  void InitMessaging(ros::NodeHandle& nodeHandle);

private:
  void EnableTrackingFeatures(ros::NodeHandle& nodeHandle);

  void PersonTrackingCallback(const realsense_ros_person::FrameTest& msg);
  void DrawDetectionResults(cv::Mat& colorImage, const realsense_ros_person::FrameTest& msg);
  void DrawPersonResults(cv::Mat& colorImage, realsense_ros_person::User& user);



  void DrawPersonSkeleton(cv::Mat& colorImage, realsense_ros_person::User& user);
  void DrawPersonGestures(cv::Mat& colorImage, realsense_ros_person::User& user);
  void DrawPersonLandmarks(cv::Mat& colorImage, realsense_ros_person::User& user);
  void DrawFace(cv::Mat& colorImage, realsense_ros_person::User& user);
  void DrawPersonSummaryReport(cv::Mat image, realsense_ros_person::User &user);

  void PersonSelectedHandler(PersonData& data, TrackingRenderer::SelectType type);
  void GlobalHandler(TrackingRenderer::SelectType type);


  bool mEnableSkeleton;
  bool mEnableRecognition;
  bool mEnableLandmarks;
  bool mEnableHeadBoundingBox;
  bool mEnableHeadPose;
  bool mEnablePointingGesture;
  bool mEnableWaveGesture;

  const int JOINT_CONFIDENCE_THR = 90;
  const int LANDMARKS_CONFIDENCE_THR = 90;
  const int HEAD_BOUNDING_BOX_THR = 90;

  ros::Subscriber mTrackingOutputSubscriber;
  ros::ServiceClient mRecognitionRequestClient;
  ros::ServiceClient mStartTrackingRequestClient;
  ros::ServiceClient mStopTrackingRequestClient;
  ros::ServiceClient mRegisterRequestClient;
  ros::ServiceClient mConfigClient;

  std::mutex mMutex;

  Viewer m_viewer;
  TrackingRenderer m_trackingRenderer;

  static std::string PERSON_MODULE_STATE_TOPIC;
};
