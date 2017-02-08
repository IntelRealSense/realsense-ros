#pragma once

#include <RealSense/PersonTracking/PersonTrackingConfiguration.h>

#include "realsense_ros_person/RecognitionResponse.h"
#include "realsense_ros_person/RecognitionRegisterResponse.h"
#include "realsense_ros_person/Wave.h"
#include "realsense_ros_person/RectWithConfidence.h"

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
    };
}