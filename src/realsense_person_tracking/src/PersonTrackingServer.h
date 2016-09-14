#pragma once

#include "PersonTrackingConfigurator.h"
#include "ros/ros.h"
#include <realsense_srvs/RecognitionRequest.h>
#include <realsense_srvs/TrackingRequest.h>
#include <realsense_srvs/LoadRecognitionDB.h>
#include <realsense_srvs/SaveRecognitionDB.h>
#include <realsense_srvs/ResetTrackingRequest.h>
#include <rs/core/video_module_interface.h>
#include <person_tracking_video_module_interface.h>

namespace PT = Intel::RealSense::PersonTracking;

class PersonTrackingServer
{
public:

    PersonTrackingServer();

    void onInit(ros::NodeHandle& nodeHandle, rs::person_tracking::person_tracking_video_module_interface* personTracking, PersonTrackingConfigurator* configurator);

private:

    bool trackingConfigRequestCallback(realsense_srvs::TrackingConfig::Request& request,
                                       realsense_srvs::TrackingConfig::Response& response);

    bool recognitionRequestCallback(realsense_srvs::RecognitionRequest::Request& request,
                                    realsense_srvs::RecognitionRequest::Response& response);

    bool trackingRequestCallback(realsense_srvs::TrackingRequest::Request& request,
                                realsense_srvs::TrackingRequest::Response& response);

    bool saveRecognitionDbCallback(realsense_srvs::SaveRecognitionDB::Request& request,
                                   realsense_srvs::SaveRecognitionDB::Response& response);

    bool loadRecognitionDbCallback(realsense_srvs::LoadRecognitionDB::Request& request,
                                   realsense_srvs::LoadRecognitionDB::Response& response);

    bool resetTrackingCallback(realsense_srvs::ResetTrackingRequest::Request& request,
                               realsense_srvs::ResetTrackingRequest::Response& response);

    void registerPerson(PT::PersonTrackingData::PersonRecognition* recognition, realsense_srvs::RecognitionRequest::Response& response);
    void reinforceRecognition(PT::PersonTrackingData::PersonRecognition* recognition, int rid);

    ros::ServiceServer mTrackingConfigService;
    ros::ServiceServer mRecognitionRequestService;
    ros::ServiceServer mTrackingRequestService;
    ros::ServiceServer mSaveRecognitionDbService;
    ros::ServiceServer mLoadRecognitionDbService;
    ros::ServiceServer mResetTrackingRequestService;

    rs::person_tracking::person_tracking_video_module_interface* mPersonTrackingVideoModule;
    PersonTrackingConfigurator* mConfigurator;
};
