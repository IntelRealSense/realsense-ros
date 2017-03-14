// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once

#include <RealSense/PersonTracking/PersonTrackingConfiguration.h>

#include "realsense_ros_person/RecognitionResponse.h"
#include "realsense_ros_person/RecognitionRegisterResponse.h"
#include "realsense_ros_person/Wave.h"
#include "realsense_ros_person/RectWithConfidence.h"
#include "realsense_ros_person/SkeletonJoint.h"
#include "realsense_ros_person/PersonModuleState.h"

namespace realsense_ros_person
{
class PersonTracking2RosHelper
{
public:
  int RecognitionStatus2RosRecognitionStatus(
    Intel::RealSense::PersonTracking::PersonTrackingData::PersonRecognition::RecognitionStatus ptRegistrationStatus)
  {
    using namespace Intel::RealSense::PersonTracking;
    switch (ptRegistrationStatus)
    {
    case PersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionPassedPersonRecognized:
      return realsense_ros_person::RecognitionResponse::RECOGNITION_PASSED_PERSON_RECOGNIZED;
    case PersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionPassedPersonNotRecognized:
      return realsense_ros_person::RecognitionResponse::RECOGNITION_PASSED_PERSON_NOT_RECOGNIZED;
    case PersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionFailed:
      return realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED;
    case PersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionFailedFaceNotDetected:
      return realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_FACE_NOT_DETECTED;
    case PersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionFailedFaceNotClear:
      return realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_FACE_NOT_CLEAR;
    case PersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionFailedPersonTooFar:
      return realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_PERSON_TOO_FAR;
    case PersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionFailedPersonTooClose:
      return realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_PERSON_TOO_CLOSE;
    case PersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionFailedFaceAmbiguity:
      return realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_FACE_AMBIGUITY;
    default:
      throw std::runtime_error("unsupported value");
    }
  }

  int RegistrationStatus2RosRecognitionStatus(
    Intel::RealSense::PersonTracking::PersonTrackingData::PersonRecognition::RegistrationStatus ptRegistrationStatus)
  {
    using namespace Intel::RealSense::PersonTracking;
    switch (ptRegistrationStatus)
    {
    case PersonRecognition::RegistrationSuccessful:
      return  realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_SUCCESSFULL;
    case PersonRecognition::RegistrationFailed:
      return  realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED;
    case PersonRecognition::RegistrationFailedAlreadyRegistered:
      return  realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_ALREADY_REGISTERED;
    case PersonRecognition::RegistrationFailedFaceNotDetected:
      return  realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_FACE_NOT_DETECTED;
    case PersonRecognition::RegistrationFailedFaceNotClear:
      return  realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_FACE_NOT_CLEAR;
    case PersonRecognition::RegistrationFailedPersonTooFar:
      return  realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_PERSON_TO_FAR;
    case PersonRecognition::RegistrationFailedPersonTooClose:
      return  realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_PERSON_TO_CLOSE;
    default:
      throw std::runtime_error("unsupported value");
    }
  }

  int WaveGesture2RosWaveGesture(Intel::RealSense::PersonTracking::PersonTrackingData::PersonGestures::WaveType waveType)
  {
    using namespace Intel::RealSense::PersonTracking;
    switch (waveType)
    {
    case PersonTrackingData::PersonGestures::WaveType::WAVE_LEFT_LA:
      return realsense_ros_person::Wave::WAVE_LEFT_LA;
    case PersonTrackingData::PersonGestures::WaveType ::WAVE_RIGHT_LA:
      return  realsense_ros_person::Wave::WAVE_RIGHT_LA;
    case PersonTrackingData::PersonGestures::WaveType::WAVE_LEFT_RA:
      return  realsense_ros_person::Wave::WAVE_LEFT_RA;
    case PersonTrackingData::PersonGestures::WaveType::WAVE_RIGHT_RA:
      return  realsense_ros_person::Wave::WAVE_RIGHT_RA;
    case PersonTrackingData::PersonGestures::WaveType::WAVE_NOT_DETECTED:
      return  realsense_ros_person::Wave::WAVE_NOT_DETECTED;
    default:
      throw std::runtime_error("unsupported value");
    }
  }

  realsense_ros_person::RectWithConfidence BoundingBox2D2RosRectWithConfidence(
    Intel::RealSense::PersonTracking::PersonTrackingData::BoundingBox2D& boundingBox2D)
  {
    realsense_ros_person::RectWithConfidence rectWithConfidence;
    rectWithConfidence.rectCorners[0].x =  boundingBox2D.rect.x;
    rectWithConfidence.rectCorners[0].y =  boundingBox2D.rect.y;
    rectWithConfidence.rectCorners[1].x =  boundingBox2D.rect.x + boundingBox2D.rect.w;
    rectWithConfidence.rectCorners[1].y =  boundingBox2D.rect.y + boundingBox2D.rect.h;
    rectWithConfidence.confidence = boundingBox2D.confidence;
    return rectWithConfidence;
  }

  int SkeletonJointType2RosSkeletonJointType(Intel::RealSense::PersonTracking::PersonJoints::JointType jointType)
  {
    using namespace Intel::RealSense::PersonTracking;
    switch (jointType)
    {
    case PersonJoints::JointType::JOINT_ANKLE_LEFT:
      return realsense_ros_person::SkeletonJoint::JOINT_ANKLE_LEFT;
    case PersonJoints::JointType::JOINT_ANKLE_RIGHT:
      return realsense_ros_person::SkeletonJoint::JOINT_ANKLE_RIGHT;
    case PersonJoints::JointType::JOINT_ELBOW_LEFT:
      return realsense_ros_person::SkeletonJoint::JOINT_ELBOW_LEFT;
    case PersonJoints::JointType::JOINT_ELBOW_RIGHT:
      return realsense_ros_person::SkeletonJoint::JOINT_ELBOW_RIGHT;
    case PersonJoints::JointType::JOINT_FOOT_LEFT:
      return realsense_ros_person::SkeletonJoint::JOINT_FOOT_LEFT;
    case PersonJoints::JointType::JOINT_FOOT_RIGHT:
      return realsense_ros_person::SkeletonJoint::JOINT_FOOT_RIGHT;
    case PersonJoints::JointType::JOINT_HAND_LEFT:
      return realsense_ros_person::SkeletonJoint::JOINT_HAND_LEFT;
    case PersonJoints::JointType::JOINT_HAND_RIGHT:
      return realsense_ros_person::SkeletonJoint::JOINT_HAND_RIGHT;
    case PersonJoints::JointType::JOINT_HAND_TIP_LEFT:
      return realsense_ros_person::SkeletonJoint::JOINT_HAND_TIP_LEFT;
    case PersonJoints::JointType::JOINT_HAND_TIP_RIGHT:
      return realsense_ros_person::SkeletonJoint::JOINT_HAND_TIP_RIGHT;
    case PersonJoints::JointType::JOINT_HEAD:
      return realsense_ros_person::SkeletonJoint::JOINT_HEAD;
    case PersonJoints::JointType::JOINT_HIP_LEFT:
      return realsense_ros_person::SkeletonJoint::JOINT_HIP_LEFT;
    case PersonJoints::JointType::JOINT_HIP_RIGHT:
      return realsense_ros_person::SkeletonJoint::JOINT_HIP_RIGHT;
    case PersonJoints::JointType::JOINT_KNEE_LEFT:
      return realsense_ros_person::SkeletonJoint::JOINT_KNEE_LEFT;
    case PersonJoints::JointType::JOINT_KNEE_RIGHT:
      return realsense_ros_person::SkeletonJoint::JOINT_KNEE_RIGHT;
    case PersonJoints::JointType::JOINT_NECK:
      return realsense_ros_person::SkeletonJoint::JOINT_NECK;
    case PersonJoints::JointType::JOINT_SHOULDER_LEFT:
      return realsense_ros_person::SkeletonJoint::JOINT_SHOULDER_LEFT;
    case PersonJoints::JointType::JOINT_SHOULDER_RIGHT:
      return realsense_ros_person::SkeletonJoint::JOINT_SHOULDER_RIGHT;
    case PersonJoints::JointType::JOINT_SPINE_BASE:
      return realsense_ros_person::SkeletonJoint::JOINT_SPINE_BASE;
    case PersonJoints::JointType::JOINT_SPINE_MID:
      return realsense_ros_person::SkeletonJoint::JOINT_SPINE_MID;
    case PersonJoints::JointType::JOINT_SPINE_SHOULDER:
      return realsense_ros_person::SkeletonJoint::JOINT_SPINE_SHOULDER;
    case PersonJoints::JointType::JOINT_THUMB_LEFT:
      return realsense_ros_person::SkeletonJoint::JOINT_THUMB_LEFT;
    case PersonJoints::JointType::JOINT_THUMB_RIGHT:
      return realsense_ros_person::SkeletonJoint::JOINT_THUMB_RIGHT;
    case PersonJoints::JointType::JOINT_WRIST_LEFT:
      return realsense_ros_person::SkeletonJoint::JOINT_WRIST_LEFT;
    case PersonJoints::JointType::JOINT_WRIST_RIGHT:
      return realsense_ros_person::SkeletonJoint::JOINT_WRIST_RIGHT;
    default:
      return realsense_ros_person::SkeletonJoint::JOINT_UNKNOWN;
    }
  }

  int TrackingState2RosTrackingState(Intel::RealSense::PersonTracking::TrackingState trackingState)
  {
    using namespace Intel::RealSense::PersonTracking;
    switch (trackingState)
    {
    case TrackingState::TRACKING_STATE_TRACKING:
      return realsense_ros_person::PersonModuleState::TRACKING_STATE_TRACKING;
    case TrackingState::TRACKING_STATE_DETECTING:
      return realsense_ros_person::PersonModuleState::TRACKING_STATE_DETECTING;
    default:
      throw std::runtime_error("unsupported value");
    }
  }

};
}