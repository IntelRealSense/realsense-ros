// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "PersonTrackingHelper.h"

#include <fstream>

namespace realsense_ros_person
{
void ConfigurePersonTracking(PersonTrackingConfig config,
                             Intel::RealSense::PersonTracking::PersonTrackingConfiguration *ptConfiguration)
{

  if (config.recognitionEnabled)
  {
    ROS_INFO("recognition enabled");
    ptConfiguration->QueryRecognition()->Enable();
  }
  else
  {
    ROS_INFO("recognition disabled");
    ptConfiguration->QueryRecognition()->Disable();
  }

  if (config.pointingGestureEnabled)
  {
    ROS_INFO("pointing gesture enabled");
    ptConfiguration->QueryGestures()->Enable();
    ptConfiguration->QueryGestures()->EnableGesture(Intel::RealSense::PersonTracking::GestureType::Pointing);
  }
  else
  {
    ROS_INFO("pointing gesture disabled");
    ptConfiguration->QueryGestures()->DisableGesture(Intel::RealSense::PersonTracking::GestureType::Pointing);
  }

  if (config.waveGestureEnabled)
  {
    ROS_INFO("wave gesture enabled");
    ptConfiguration->QueryGestures()->Enable();
    ptConfiguration->QueryGestures()->EnableGesture(Intel::RealSense::PersonTracking::GestureType::Wave);
  }
  else
  {
    ROS_INFO("wave gesture disabled");
    ptConfiguration->QueryGestures()->DisableGesture(Intel::RealSense::PersonTracking::GestureType::Wave);
  }

  if (config.skeletonEnabled)
  {
    ROS_INFO("skeleton enabled");
    ptConfiguration->QuerySkeletonJoints()->Enable();
  }
  else
  {
    ROS_INFO("skeleton disabled");
    ptConfiguration->QuerySkeletonJoints()->Disable();
  }

  if (config.trackingEnabled)
  {
    ROS_INFO("tracking enabled");
    ptConfiguration->QueryTracking()->Enable();
  }
  else
  {
    ROS_INFO("tracking disabled");
    ptConfiguration->QueryTracking()->Disable();
  }

  if (config.headPoseEnabled)
  {
    ROS_INFO("head pose enabled");
    ptConfiguration->QueryTracking()->Enable();
    ptConfiguration->QueryFace()->EnableHeadPose();
  }
  else
  {
    ROS_INFO("head pose disabled");
    ptConfiguration->QueryFace()->DisableHeadPose();
  }


  if (config.headBoundingBoxEnabled)
  {
    ROS_INFO("head bounding box enabled");
    ptConfiguration->QueryTracking()->Enable();
    ptConfiguration->QueryTracking()->EnableHeadBoundingBox();
  }
  else
  {
    ROS_INFO("head bounding box disabled");
    ptConfiguration->QueryTracking()->DisableHeadBoundingBox();
  }


  if (config.landmarksEnabled)
  {
    ROS_INFO("landmarks enabled");
    ptConfiguration->QueryTracking()->Enable();
    ptConfiguration->QueryFace()->EnableFaceLandmarks();
  }
  else
  {
    ROS_INFO("landmarks disabled");
    ptConfiguration->QueryFace()->DisableFaceLandmarks();
  }
}

bool
LoadDatabase(std::string dbPath, Intel::RealSense::PersonTracking::PersonTrackingConfiguration *ptConfiguration)
{
  ROS_INFO_STREAM("loading database from: " << dbPath);
  auto database = ptConfiguration->QueryRecognition()->QueryDatabase();
  auto dbUtils = ptConfiguration->QueryRecognition()->QueryDatabaseUtilities();

  std::filebuf fb;
  if (!fb.open(dbPath, std::ios::in | std::ios::binary))
  {
    throw std::runtime_error("failed to open database file");
  }
  std::istream is(&fb);
  is.seekg(0, std::ios::end);
  int size = is.tellg();
  is.seekg(0, std::ios::beg);
  std::vector<char> buffer(size);
  is.read(buffer.data(), buffer.size());
  fb.close();
  if (!dbUtils->DeserializeDatabase(database, (unsigned char *) buffer.data(), (int) buffer.size()))
  {
    ROS_ERROR("failed to load database");
    return false;
  }
  return true;
}

bool
SaveDatabase(std::string dbPath, Intel::RealSense::PersonTracking::PersonTrackingConfiguration *ptConfiguration)
{
  ROS_INFO_STREAM("save database to: " << dbPath);
  auto db = ptConfiguration->QueryRecognition()->QueryDatabase();
  auto dbUtils = ptConfiguration->QueryRecognition()->QueryDatabaseUtilities();

  const size_t dbSize = dbUtils->GetDatabaseMemorySize(db);

  std::vector<uint8_t> buffer(dbSize, 0);
  int32_t writtenSize;
  dbUtils->SerializeDatabase(db, buffer.data(), (int32_t) buffer.size(), &writtenSize);

  std::ofstream out(dbPath, std::ios::binary);
  if (out)
  {
    out.write((char *) buffer.data(), buffer.size());
  }
  else
  {
    ROS_ERROR("fail to save database");
    return false;
  }
  return true;
}

std::string RecognitionStatusToString(
  Intel::RealSense::PersonTracking::PersonTrackingData::PersonRecognition::RecognitionStatus status)
{
  using Intel::RealSense::PersonTracking::PersonTrackingData;
  switch (status)
  {
  case PersonTrackingData::PersonRecognition::RecognitionPassedPersonRecognized:
    return "RecognitionPassedPersonRecognized";
  case PersonTrackingData::PersonRecognition::RecognitionPassedPersonNotRecognized:
    return "RecognitionPassedPersonNotRecognized";
  case PersonTrackingData::PersonRecognition::RecognitionFailed:
    return "RecognitionFailed";
  case PersonTrackingData::PersonRecognition::RecognitionFailedFaceNotDetected:
    return "RecognitionFailedFaceNotDetected";
  case PersonTrackingData::PersonRecognition::RecognitionFailedFaceNotClear:
    return "RecognitionFailedFaceNotClear";
  case PersonTrackingData::PersonRecognition::RecognitionFailedPersonTooFar:
    return "RecognitionFailedPersonTooFar";
  case PersonTrackingData::PersonRecognition::RecognitionFailedPersonTooClose:
    return "RecognitionFailedPersonTooClose";
  default:
    return "unknown recognition status please check the code";
  }
}
}