// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved

#include "PersonTrackingServer.h"

#include "PersonTracking2RosHelper.h"

namespace realsense_ros_person
{
    PersonTrackingServer::PersonTrackingServer()
    {}

    void PersonTrackingServer::onInit(ros::NodeHandle &nodeHandle,
                                      rs::person_tracking::person_tracking_video_module_interface *personTracking,
                                      PersonTrackingConfig &config)
    {
        mPersonTracking = personTracking;
        mConfig = config;
        mTrackingConfigService = nodeHandle.advertiseService("person_tracking/tracking_config",
                                                             &PersonTrackingServer::trackingConfigRequestCallback,
                                                             this);
        mRecognitionRequestService = nodeHandle.advertiseService("person_tracking/recognition_request",
                                                                 &PersonTrackingServer::recognitionRequestCallback,
                                                                 this);
        mRecognitionRegisterRequestService = nodeHandle.advertiseService("person_tracking/register_request",
                                                                         &PersonTrackingServer::recognitionRegisterRequestCallback,
                                                                         this);
        mTrackingRequestService = nodeHandle.advertiseService("person_tracking/tracking_request",
                                                              &PersonTrackingServer::trackingRequestCallback, this);
        mSaveRecognitionDbService = nodeHandle.advertiseService("person_tracking/save_recognition",
                                                                &PersonTrackingServer::saveRecognitionDbCallback, this);
        mLoadRecognitionDbService = nodeHandle.advertiseService("person_tracking/load_recognition",
                                                                &PersonTrackingServer::loadRecognitionDbCallback, this);
    }

    bool PersonTrackingServer::trackingConfigRequestCallback(realsense_ros_person::TrackingConfig::Request &request,
                                                             realsense_ros_person::TrackingConfig::Response &response)
    {
        mConfig.recognitionEnabled = request.enableRecognition;
        mConfig.pointingGestureEnabled = request.enablePointingGesture;
        mConfig.waveGestureEnabled = request.enableWaveGesture;
        mConfig.skeletonEnabled = request.enableSkeleton;
        mConfig.headBoundingBoxEnabled = request.enableHeadBoundingBox;
        mConfig.landmarksEnabled = request.enableLandmarks;
        mConfig.headPoseEnabled = request.enableHeadPose;
        mConfig.trackingEnabled = true;
        ConfigurePersonTracking(mConfig, mPersonTracking->QueryConfiguration());
        response.status = true;
        return true;
    }

    bool PersonTrackingServer::recognitionRequestCallback(realsense_ros_person::Recognition::Request &request,
                                                          realsense_ros_person::Recognition::Response &response)
    {
        if (!mPersonTracking->QueryConfiguration()->QueryRecognition()->IsEnabled())
        {
            ROS_ERROR("Recognition is not enabled");
            response.status = -1;
            return true;
        }

        ROS_INFO_STREAM("Received recognition request for person: " << request.personId);
        PXCPersonTrackingData *trackingData = mPersonTracking->QueryOutput();
        PXCPersonTrackingData::Person *personData = trackingData->QueryPersonDataById(request.personId);
        if (!personData)
        {
            ROS_ERROR_STREAM("Couldn't find recognition request target");
            response.status = -1;
            return true;
        }
        ROS_INFO_STREAM("Found recognition request target");
        PXCPersonTrackingData::PersonRecognition *recognition = personData->QueryRecognition();
        ROS_INFO("User recognition");
        PXCPersonTrackingData::PersonRecognition::RecognizerData result;
        auto status = recognition->RecognizeUser(&result);
        response.status = m_pt2rosHelper.RecognitionStatus2RosRecognitionStatus(status);
        response.recognitionId = -1;
        response.similarityScore = 0;
        switch (status)
        {
            case PXCPersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionPassedPersonRecognized:
                ROS_INFO_STREAM("Recognized person: " << result.recognitionId << " TrackingId: " << result.trackingId);
                response.recognitionId = result.recognitionId;
                response.similarityScore = result.similarityScore;

                break;
            case PXCPersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionPassedPersonNotRecognized:
                ROS_INFO_STREAM("Recognition passed. person not recognized");
                break;
            case PXCPersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionFailed:
                ROS_INFO_STREAM("Recognition failed.");
                break;
            case PXCPersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionFailedFaceNotDetected:
                ROS_INFO_STREAM("Recognition failed. Face not detected");
                break;
            case PXCPersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionFailedFaceNotClear:
                ROS_INFO_STREAM("Recognition failed. Face not clear");
                break;
            case PXCPersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionFailedPersonTooFar:
                ROS_INFO_STREAM("Recognition failed. Person too far");
                break;
            case PXCPersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionFailedPersonTooClose:
                ROS_INFO_STREAM("Recognition failed. Person too close");
                break;
            case PXCPersonTrackingData::PersonRecognition::RecognitionStatus::RecognitionFailedFaceAmbiguity:
                ROS_INFO_STREAM("Recognition failed. Face ambiguity");
                break;
        }

        return true;
    }

    bool
    PersonTrackingServer::recognitionRegisterRequestCallback(realsense_ros_person::RecognitionRegister::Request &request,
                                                             realsense_ros_person::RecognitionRegister::Response &response)
    {
        if (!mPersonTracking->QueryConfiguration()->QueryRecognition()->IsEnabled())
        {
            ROS_INFO("Recognition is not enabled");
            response.status = false;
            return true;
        }

        ROS_INFO_STREAM("Received register request for person: " << request.personId);
        PXCPersonTrackingData *trackingData = mPersonTracking->QueryOutput();
        PXCPersonTrackingData::Person *personData = trackingData->QueryPersonDataById(request.personId);
        if (!personData)
        {
            ROS_ERROR_STREAM("Couldn't find recognition request target");
            response.status = false;
            return true;
        }
        ROS_INFO_STREAM("Found register request target");
        PXCPersonTrackingData::PersonRecognition *recognition = personData->QueryRecognition();
        int32_t outputRecognitionId;
        int32_t outputTrackingId;
        int32_t outDescriptorId;
        ROS_INFO("User registration");
        auto status = recognition->RegisterUser(&outputRecognitionId, &outputTrackingId, &outDescriptorId);
        response.status = m_pt2rosHelper.RegistrationStatus2RosRecognitionStatus(status);
        switch (status)
        {
            case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationSuccessful:
                ROS_INFO_STREAM("Registered person: " << outputRecognitionId << " TrackingId: " << outputTrackingId);
                response.recognitionId = outputRecognitionId;
                break;
            case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedAlreadyRegistered:
                ROS_INFO_STREAM(
                        "Person already registered: " << outputRecognitionId << " TrackingId: " << outputTrackingId);
                break;
            case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailed:
                ROS_INFO_STREAM(
                        "Registration failed: person: " << outputRecognitionId << " TrackingId: " << outputTrackingId);
                break;
            case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedFaceNotDetected:
                ROS_INFO_STREAM(
                        "Registration failed: face not detected, person: " << outputRecognitionId << " TrackingId: "
                                                                           << outputTrackingId);
                break;
            case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedFaceNotClear:
                ROS_INFO_STREAM(
                        "Registration failed: face not clear, person: " << outputRecognitionId << " TrackingId: "
                                                                        << outputTrackingId);
                break;
            case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedPersonTooFar:
                ROS_INFO_STREAM(
                        "Registration failed: person too far, person: " << outputRecognitionId << " TrackingId: "
                                                                        << outputTrackingId);
                break;
            case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedPersonTooClose:
                ROS_INFO_STREAM(
                        "Registration failed: person to close, person: " << outputRecognitionId << " TrackingId: "
                                                                         << outputTrackingId);
                break;
        }

        return true;
    }

    bool PersonTrackingServer::trackingRequestCallback(realsense_ros_person::TrackingRequest::Request &request,
                                                       realsense_ros_person::TrackingRequest::Response &response)
    {
        ROS_INFO_STREAM("Received tracking request for person: " << request.personId);
        PXCPersonTrackingData *trackingData = mPersonTracking->QueryOutput();
        PXCPersonTrackingData::Person *personData = trackingData->QueryPersonDataById(request.personId);
        if (!personData)
        {
            ROS_ERROR_STREAM("Couldn't find tracking request target");
            response.status = false;
            return true;
        }
        ROS_INFO_STREAM("Found tracking request target");
        PXCPersonTrackingData::TrackingState trackingState = trackingData->GetTrackingState();
        if (trackingState == PXCPersonTrackingData::TRACKING_STATE_DETECTING)
        {
            ROS_INFO_STREAM("start tracking");
            trackingData->StartTracking(request.personId);
        }
        else
        {
            ROS_INFO_STREAM("stop tracking");
            trackingData->StopTracking(request.personId);
        }

        response.status = true;

        return true;
    }

    bool PersonTrackingServer::saveRecognitionDbCallback(realsense_ros_person::SaveRecognitionDB::Request &request,
                                                         realsense_ros_person::SaveRecognitionDB::Response &response)
    {
        std::string dbPath = request.saveToPath;

        ROS_INFO_STREAM("Save database to: " << dbPath);
        try
        {
            response.status = SaveDatabase(dbPath, mPersonTracking->QueryConfiguration());
        }
        catch (const std::runtime_error &error)
        {
            ROS_ERROR_STREAM("Failed to save database to: " << dbPath);
            response.status = false;
        }
        return true;
    }

    bool PersonTrackingServer::loadRecognitionDbCallback(realsense_ros_person::LoadRecognitionDB::Request &request,
                                                         realsense_ros_person::LoadRecognitionDB::Response &response)
    {
        std::string dbPath = request.loadFromPath;
        ROS_INFO_STREAM("Loading database from: " << dbPath);
        try
        {
            response.status = LoadDatabase(dbPath, mPersonTracking->QueryConfiguration());;
        }
        catch (const std::runtime_error &error)
        {
            ROS_ERROR_STREAM("Failed to load database from: " << dbPath);
            response.status = false;
        }
        return true;
    }
}