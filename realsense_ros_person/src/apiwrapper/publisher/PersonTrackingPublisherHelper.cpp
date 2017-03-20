// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <ros/console.h>

#include "PersonTrackingPublisherHelper.h"
#include "realsense_ros_person/SkeletonJoint.h"
#include "PersonTrackingHelper.h"

namespace realsense_ros_person
{
void PersonTrackingPublisherHelper::addSkeletonToOutput(
  Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
  Intel::RealSense::PersonTracking::PersonTrackingData::Person *personData, realsense_ros_person::User &user)
{
  using Intel::RealSense::PersonTracking::PersonTrackingData;
  if (!ptConfiguration.QuerySkeletonJoints()->IsEnabled()) return;

  PersonTrackingData::PersonJoints *personJoints = personData->QuerySkeletonJoints();
  int numberOfJoints = personJoints->QueryNumJoints();

  if (numberOfJoints == 0) return;

  std::vector<PersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(numberOfJoints);
  if (!personJoints->QueryJoints(skeletonPoints.data()))
  {
    return;
  }


  for (PersonTrackingData::PersonJoints::SkeletonPoint &skeletonPoint : skeletonPoints)
  {
    realsense_ros_person::SkeletonJoint joint;
    joint.type = mPt2rosHelper.SkeletonJointType2RosSkeletonJointType(skeletonPoint.jointType);
    joint.confidence = skeletonPoint.confidenceImage;
    joint.location.x = skeletonPoint.image.x;
    joint.location.y = skeletonPoint.image.y;
    joint.realWorldCoordinates.x = skeletonPoint.world.x;
    joint.realWorldCoordinates.y = skeletonPoint.world.y;
    joint.realWorldCoordinates.z = skeletonPoint.world.z;
    user.skeletonJoints.push_back(joint);
  }
}

void PersonTrackingPublisherHelper::addGesturesToOutput(
  Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
  Intel::RealSense::PersonTracking::PersonTrackingData::Person *const personData,
  realsense_ros_person::User &user)
{
  using Intel::RealSense::PersonTracking::PersonTrackingData;

  user.gestures.pointing.confidence = 0;
  user.gestures.wave.type = mPt2rosHelper.WaveGesture2RosWaveGesture(PersonTrackingData::PersonGestures::WaveType::WAVE_NOT_DETECTED);

  if (!ptConfiguration.QueryGestures()->IsEnabled()) return;

  PersonTrackingData::PersonGestures *personGestures = personData->QueryGestures();

  if (personGestures == nullptr)
  {
    ROS_WARN("personGestures == nullptr");
    return;
  }

  if (personGestures->IsPointing())
  {
    PersonTrackingData::PersonGestures::PointingInfo poitingInfo = personGestures->QueryPointingInfo();
    user.gestures.pointing.originColor.x = poitingInfo.colorPointingData.origin.x;
    user.gestures.pointing.originColor.y = poitingInfo.colorPointingData.origin.y;
    user.gestures.pointing.originWorld.x = poitingInfo.worldPointingData.origin.x;
    user.gestures.pointing.originWorld.y = poitingInfo.worldPointingData.origin.y;
    user.gestures.pointing.originWorld.z = poitingInfo.worldPointingData.origin.z;
    user.gestures.pointing.orientationColor.x = poitingInfo.colorPointingData.direction.x;
    user.gestures.pointing.orientationColor.y = poitingInfo.colorPointingData.direction.y;
    user.gestures.pointing.orientationWorld.x = poitingInfo.worldPointingData.direction.x;
    user.gestures.pointing.orientationWorld.y = poitingInfo.worldPointingData.direction.y;
    user.gestures.pointing.orientationWorld.z = poitingInfo.worldPointingData.direction.z;
    user.gestures.pointing.confidence = poitingInfo.confidence;
  }

  user.gestures.wave.type = mPt2rosHelper.WaveGesture2RosWaveGesture(personGestures->QueryWaveGesture());
}

std::vector<realsense_ros_person::User> PersonTrackingPublisherHelper::BuildUsersVector(
  Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
  Intel::RealSense::PersonTracking::PersonTrackingData &trackingData)
{
  using Intel::RealSense::PersonTracking::PersonTrackingData;
  std::vector<realsense_ros_person::User> usersVector;
  int numberOfUsers = trackingData.QueryNumberOfPeople();

  for (int index = 0; index < numberOfUsers; ++index)
  {
    PersonTrackingData::Person *personData = trackingData.QueryPersonData(
          PersonTrackingData::AccessOrderType::ACCESS_ORDER_BY_INDEX, index);

    if (personData)
    {
      realsense_ros_person::User user;

      PersonTrackingData::PersonTracking *personTrackingData = personData->QueryTracking();
      PersonTrackingData::BoundingBox2D box = personTrackingData->Query2DBoundingBox();
      PersonTrackingData::BoundingBox2D headBoundingBox = personTrackingData->QueryHeadBoundingBox();

      PersonTrackingData::PointCombined centerMass = personTrackingData->QueryCenterMass();

      user.userInfo.Id = personTrackingData->QueryId();

      user.userRect = mPt2rosHelper.BoundingBox2D2RosRectWithConfidence(box);
      user.headBoundingBox = mPt2rosHelper.BoundingBox2D2RosRectWithConfidence(headBoundingBox);

      user.centerOfMassImage.x = centerMass.image.point.x;
      user.centerOfMassImage.y = centerMass.image.point.y;
      user.centerOfMassImage.z = centerMass.image.point.z;

      user.centerOfMassWorld.x = centerMass.world.point.x;
      user.centerOfMassWorld.y = centerMass.world.point.y;
      user.centerOfMassWorld.z = centerMass.world.point.z;

      addSkeletonToOutput(ptConfiguration, personData, user);
      addGesturesToOutput(ptConfiguration, personData, user);
      addLandmarksToOutput(ptConfiguration, personData, user);
      addHeadPoseToOutput(ptConfiguration, personData, user);
      usersVector.push_back(user);
    }
  }
  return usersVector;
}

void PersonTrackingPublisherHelper::addLandmarksToOutput(
  Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
  Intel::RealSense::PersonTracking::PersonTrackingData::Person *const personData,
  realsense_ros_person::User &user)
{
  if (!ptConfiguration.QueryFace()->IsFaceLandmarksEnabled())
  {
    return;
  }

  Intel::RealSense::PersonTracking::PersonTrackingData::PersonFace *faceData = personData->QueryFace();
  if (faceData == nullptr)
  {
    return;
  }

  Intel::RealSense::PersonTracking::PersonTrackingData::PersonFace::LandmarksInfo* landmarksInfo = faceData->QueryLandmarks();
  if (landmarksInfo == nullptr)
  {
    return;
  }
  if (landmarksInfo->landmarks == nullptr)
  {
    return;
  }

  int32_t numLandmarks = landmarksInfo->numLandmarks;
  user.landmarksInfo.confidence = landmarksInfo->confidence;
  if (landmarksInfo->confidence > 0)
  {
    for (int landmarkNb = 0; landmarkNb < numLandmarks; ++landmarkNb)
    {
      realsense_ros_person::Landmark landmark;
      landmark.location.x = landmarksInfo->landmarks[landmarkNb].image.x;
      landmark.location.y = landmarksInfo->landmarks[landmarkNb].image.y;
      landmark.realWorldCoordinates.x = landmarksInfo->landmarks[landmarkNb].world.x;
      landmark.realWorldCoordinates.y = landmarksInfo->landmarks[landmarkNb].world.y;
      landmark.realWorldCoordinates.z = landmarksInfo->landmarks[landmarkNb].world.z;
      user.landmarksInfo.landmarks.push_back(landmark);
    }
  }
}

void PersonTrackingPublisherHelper::addHeadPoseToOutput(
  Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
  Intel::RealSense::PersonTracking::PersonTrackingData::Person *const personData,
  realsense_ros_person::User &user)
{

  Intel::RealSense::PersonTracking::PersonTrackingData::PersonFace *faceData = personData->QueryFace();
  if (faceData == nullptr)
  {
    user.headPose.confidence = 0;
    return;
  }
  Intel::RealSense::PersonTracking::PersonTrackingData::PoseEulerAngles angles;
  if (faceData->QueryHeadPose(angles))
  {
    user.headPose.angles.yaw = angles.yaw;
    user.headPose.angles.pitch = angles.pitch;
    user.headPose.angles.roll = angles.roll;
    user.headPose.confidence = 100;
  }
  else
  {
    user.headPose.confidence = 0;
  }
  return;
}

realsense_ros_person::PersonModuleState PersonTrackingPublisherHelper::BuildPersonModuleState(
  Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
  Intel::RealSense::PersonTracking::PersonTrackingData &trackingData)
{
  realsense_ros_person::PersonModuleState state;
  state.isRecognitionEnabled = ptConfiguration.QueryRecognition()->IsEnabled();
  state.isSkeletonEnabled = ptConfiguration.QuerySkeletonJoints()->IsEnabled();
  state.isGesturesEnabled = ptConfiguration.QueryGestures()->IsEnabled();
  state.isLandmarksEnabled = ptConfiguration.QueryFace()->IsFaceLandmarksEnabled();
  state.isHeadBoundingBoxEnabled =  ptConfiguration.QueryTracking()->IsHeadBoundingBoxEnabled();
  state.isTrackingEnabled = ptConfiguration.QueryTracking()->IsEnabled();
  state.trackingState = mPt2rosHelper.TrackingState2RosTrackingState(trackingData.GetTrackingState());
  return  state;
}
}