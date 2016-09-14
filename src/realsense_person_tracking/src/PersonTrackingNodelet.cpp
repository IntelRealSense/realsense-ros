/******************************************************************************
 INTEL CORPORATION PROPRIETARY INFORMATION
 This software is supplied under the terms of a license agreement or nondisclosure
 agreement with Intel Corporation and may not be copied or disclosed except in
 accordance with the terms of that agreement
 Copyright(c) 2011-2015 Intel Corporation. All Rights Reserved.
 *******************************************************************************/

#include "ros/ros.h"

#include "PersonTrackingNodelet.h"
#include "person_tracking_video_module_factory.h"
#include "rs/core/projection_interface.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <std_msgs/String.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <vector>
#include <cstring>

using namespace PT;
using namespace Intel::RealSense;

namespace person_tracking
{
    PersonTrackingNodelet::PersonTrackingNodelet()
        : mNodeHandle(nullptr),
          mProjection(nullptr),
          mColorInfoReceived(false), mDepthInfoReceived(false), mIrStreamsEnabled(false), mIsTrackingModeInteractive(true)
{
    mPersonTrackingVideoModule.reset(rs::person_tracking::person_tracking_video_module_factory::create_person_tracking_video_module(L"/usr/share/librealsense/pt/data/"));
}

    PersonTrackingNodelet::~PersonTrackingNodelet()
    {
        if (mProjection != nullptr)
        {
            mProjection->release();
        }
    }

    void PersonTrackingNodelet::onInit(ros::NodeHandle& nodeHandle)
    {
        processCommandLineArguments();
        initPublishSubscribe(nodeHandle);
    }

    void PersonTrackingNodelet::processCommandLineArguments()
    {
        ros::NodeHandle nodeHandle("~");

        realsense_srvs::TrackingConfig::Request request;
        bool recognition = false;
        nodeHandle.getParam("recognition", recognition);
        request.enableRecognition = recognition;
        bool segmentation = false;
        nodeHandle.getParam("segmentation", segmentation);
        request.enableSegmentation = segmentation;
        bool blob = false;
        nodeHandle.getParam("blob", blob);
        request.enableBlob = blob;
        bool skeleton = false;
        nodeHandle.getParam("skeleton", skeleton);
        request.enableSkeleton = skeleton;
        bool gestures = false;
        nodeHandle.getParam("gestures", gestures);
        request.enableGestures = gestures;
        int trackingMode = 0;
        nodeHandle.getParam("trackingMode", trackingMode);
        request.trackingMode = trackingMode;
        configureTracking(request);

        bool isDebug = false;
        nodeHandle.getParam("debug", isDebug);
        mPublisher.setDebug(isDebug);

        nodeHandle.getParam("ir", mIrStreamsEnabled);

        ROS_INFO_STREAM("Configuration -- recognition  = " << recognition);
        ROS_INFO_STREAM("Configuration -- segmentation = " << segmentation);
        ROS_INFO_STREAM("Configuration -- skeleton     = " << skeleton);
        ROS_INFO_STREAM("Configuration -- gestures     = " << gestures);
        ROS_INFO_STREAM("Configuration -- trackingMode = " << trackingMode);
        ROS_INFO_STREAM("Configuration -- debug        = " << isDebug);
        ROS_INFO_STREAM("Configuration -- ir streams   = " << mIrStreamsEnabled);
    }

    void PersonTrackingNodelet::configureTracking(realsense_srvs::TrackingConfig::Request& request)
    {
        if (request.enableRecognition)
        {
            ROS_INFO("Enable recognition");
            mPersonTrackingVideoModule->QueryConfiguration()->QueryRecognition()->Enable();
        }
        if (request.enableSegmentation || request.enableBlob)
        {
            ROS_INFO("Enable segmentation");
            mPersonTrackingVideoModule->QueryConfiguration()->QueryTracking()->EnableSegmentation();
        }
        if (request.enableSkeleton)
        {
            ROS_INFO("Enable skeleton");
            mPersonTrackingVideoModule->QueryConfiguration()->QuerySkeletonJoints()->Enable();
        }
        if (request.enableGestures)
        {
            ROS_INFO("Enable gestures");
            mPersonTrackingVideoModule->QueryConfiguration()->QueryGestures()->Enable();
            mPersonTrackingVideoModule->QueryConfiguration()->QueryGestures()->EnableGesture(PersonTrackingData::PersonGestures::Pointing);
        }
        mPublisher.enableBlob(request.enableBlob, request.enableSegmentation);
        configureTrackingMode(request.trackingMode);
    }

    void PersonTrackingNodelet::configureTrackingMode(int trackingMode)
    {
        mIsTrackingModeInteractive = (trackingMode == 1);
    }

    void PersonTrackingNodelet::initPublishSubscribe(ros::NodeHandle& nodeHandle)
    {
        mNodeHandle = &nodeHandle;

        std::string colorCameraInfoStream = "camera/color/camera_info";
        std::string depthCameraInfoStream = "camera/depth/camera_info";

        ROS_INFO("Initializing User Tracking Nodelet...");
        ROS_INFO_STREAM("Listening on " << colorCameraInfoStream);
        ROS_INFO_STREAM("Listening on " << depthCameraInfoStream);

        // Subcribe to camera info
        mColorCameraInfoSubscriber = nodeHandle.subscribe(colorCameraInfoStream, 1, &PersonTrackingNodelet::colorCameraInfoCallback, this);
        mDepthCameraInfoSubscriber = nodeHandle.subscribe(depthCameraInfoStream, 1, &PersonTrackingNodelet::depthCameraInfoCallback, this);

        // Init server
        mServer.onInit(nodeHandle, mPersonTrackingVideoModule.get(), this);

        // Init publisher
        mPublisher.onInit(nodeHandle, mPersonTrackingVideoModule.get());
    }

    void PersonTrackingNodelet::colorCameraInfoCallback(const sensor_msgs::CameraInfo colorCameraInfo)
    {
        if (mColorInfoReceived) return;
        mColorInfoReceived = true;
        ROS_INFO("Received color camera info. %dx%d", colorCameraInfo.width, colorCameraInfo.height);
        mColorCameraInfo = colorCameraInfo;
        ConfigurePersonTrackingVideoModule();
    }

    void PersonTrackingNodelet::depthCameraInfoCallback(const sensor_msgs::CameraInfo depthCameraInfo)
    {
        if (mDepthInfoReceived) return;
        mDepthInfoReceived = true;
        ROS_INFO("Received depth camera info. %dx%d", depthCameraInfo.width, depthCameraInfo.height);
        mDepthCameraInfo = depthCameraInfo;
        ConfigurePersonTrackingVideoModule();
    }

    void PersonTrackingNodelet::ConfigurePersonTrackingVideoModule()
    {
        if (!mColorInfoReceived || !mDepthInfoReceived)
        {
            return;
        }

        rs::core::intrinsics colorIntrinsics = {};
        rs::core::intrinsics depthIntrinsics = {};
        rs::core::extrinsics extrisincs = {};
        memset(&colorIntrinsics, 0, sizeof(colorIntrinsics));
        memset(&depthIntrinsics, 0, sizeof(depthIntrinsics));
        memset(&extrisincs, 0, sizeof(extrisincs));

        CameraInfo2Intrinsics(mColorCameraInfo, colorIntrinsics);
        CameraInfo2Intrinsics(mDepthCameraInfo, depthIntrinsics);

        extrisincs.translation[0] = mDepthCameraInfo.P.at(3);
        extrisincs.translation[1] = mDepthCameraInfo.P.at(7);
        extrisincs.translation[2] = mDepthCameraInfo.P.at(11);
        for(int idx = 0; idx < 9; ++idx)
        {
            extrisincs.rotation[idx] = mDepthCameraInfo.R.at(idx);
        }

        //configure person tracking video module
        rs::core::video_module_interface::actual_module_config actualModuleConfig = {};
        actualModuleConfig.image_streams_configs[static_cast<uint32_t>(rs::core::stream_type::color)].size.height = colorIntrinsics.height;
        actualModuleConfig.image_streams_configs[static_cast<uint32_t>(rs::core::stream_type::color)].size.width = colorIntrinsics.width;
        actualModuleConfig.image_streams_configs[static_cast<uint32_t>(rs::core::stream_type::color)].frame_rate = 30;
        actualModuleConfig.image_streams_configs[static_cast<uint32_t>(rs::core::stream_type::color)].is_enabled = true;
        //not extrinsics
        //no intrinsics
        //no flags

        actualModuleConfig.image_streams_configs[static_cast<uint32_t>(rs::core::stream_type::depth)].size.height = depthIntrinsics.height;
        actualModuleConfig.image_streams_configs[static_cast<uint32_t>(rs::core::stream_type::depth)].size.width = depthIntrinsics.width;
        actualModuleConfig.image_streams_configs[static_cast<uint32_t>(rs::core::stream_type::depth)].frame_rate = 30;
        actualModuleConfig.image_streams_configs[static_cast<uint32_t>(rs::core::stream_type::depth)].is_enabled = true;
        //not extrinsics
        //no intrinsics
        //no flags

        actualModuleConfig.projection = rs::core::projection_interface::create_instance(&colorIntrinsics, &depthIntrinsics, &extrisincs);
        mProjection = actualModuleConfig.projection;
        mPersonTrackingVideoModule->set_module_config(actualModuleConfig);

        ROS_INFO("Done set capture profile - subscribing to frame messages");
        subscribeFrameMessages();
        mColorCameraInfoSubscriber.shutdown();
        mDepthCameraInfoSubscriber.shutdown();
    }

    void PersonTrackingNodelet::subscribeFrameMessages()
    {
        std::string depthImageStream =  "camera/depth/image_raw";
        std::string colorImageStream =  "camera/color/image_raw";
        std::string leftImageStream =   "/camera/infrared1/image_raw";
        std::string rightImageStream =  "/camera/infrared2/image_raw";

        ROS_INFO_STREAM("Listening on " << depthImageStream);
        ROS_INFO_STREAM("Listening on " << colorImageStream);
        ROS_INFO_STREAM("Listening on " << leftImageStream);
        ROS_INFO_STREAM("Listening on " << rightImageStream);

        // Subscribe to color, point cloud and depth using synchronization filter
        mDepthSubscriber = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
            (new message_filters::Subscriber<sensor_msgs::Image>(*mNodeHandle, depthImageStream, 1));
        mColorSubscriber = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
            (new message_filters::Subscriber<sensor_msgs::Image>(*mNodeHandle, colorImageStream, 1));
        m_LeftSubscriber = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
            (new message_filters::Subscriber<sensor_msgs::Image>(*mNodeHandle, leftImageStream, 1));
        m_RightSubscriber = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
            (new message_filters::Subscriber<sensor_msgs::Image>(*mNodeHandle, rightImageStream, 1));

        if (mIrStreamsEnabled)
        {
            mTimeSynchronizerIr = std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>>
                (new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>(*mDepthSubscriber, *mColorSubscriber, *m_LeftSubscriber, *m_RightSubscriber, 40));

            mTimeSynchronizerIr->registerCallback(boost::bind(&PersonTrackingNodelet::personTrackingCallback, this, _1, _2, _3, _4));
        }
        else
        {
            mTimeSynchronizer = std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>>
                (new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>(*mDepthSubscriber, *mColorSubscriber, 40));

            mTimeSynchronizer->registerCallback(boost::bind(&PersonTrackingNodelet::personTrackingCallback, this, _1, _2));
        }
    }

    void PersonTrackingNodelet::personTrackingCallback(const sensor_msgs::ImageConstPtr& depthImageMsg, const sensor_msgs::ImageConstPtr& colorImageMsg)
    {
        sensor_msgs::ImageConstPtr leftImageMsg;
        sensor_msgs::ImageConstPtr rightImageMsg;
        personTrackingCallback(depthImageMsg, colorImageMsg, leftImageMsg, rightImageMsg);
    }

    void PersonTrackingNodelet::personTrackingCallback(const sensor_msgs::ImageConstPtr& depthImageMsg, const sensor_msgs::ImageConstPtr& colorImageMsg,
                                                       const sensor_msgs::ImageConstPtr& leftImageMsg, const sensor_msgs::ImageConstPtr& rightImageMsg)
    {
        if (!mProcessingMutex.try_lock()) return;

        if (!handleCallbackParameters(depthImageMsg, colorImageMsg, leftImageMsg, rightImageMsg))
        {
            ROS_ERROR("Failed to handle callback parameters");
        }
        else
        {
            PersonTrackingData* trackingData = processFrame();

            if (trackingData)
            {
                publishOutput(colorImageMsg, *trackingData);
            }
        }

        mProcessingMutex.unlock();
    }

    bool PersonTrackingNodelet::handleCallbackParameters(const sensor_msgs::ImageConstPtr& depthImageMsg, const sensor_msgs::ImageConstPtr& colorImageMsg,
                                                         const sensor_msgs::ImageConstPtr& leftImageMsg, const sensor_msgs::ImageConstPtr& rightImageMsg)
    {
        static bool firstTime = true;
        if (firstTime)
        {
            firstTime = false;
            ROS_INFO("Handling first images callback...");
            ROS_INFO_STREAM("Color image: size " << colorImageMsg->width << "x" << colorImageMsg->height << ". encoding: " << colorImageMsg->encoding);
            ROS_INFO_STREAM("Depth image: size " << depthImageMsg->width << "x" << depthImageMsg->height << ". encoding: " << depthImageMsg->encoding);
        }

        //fill color image at mSampleSet - all images from mSampleSet will be released while callling to process_sample_set_sync
        rs::core::image_info colorInfo = {
               static_cast<int32_t>(colorImageMsg->width),
               static_cast<int32_t>(colorImageMsg->height),
               rs::core::pixel_format::rgb8, //TODO assumes that color format is RGB
               colorImageMsg->step
        };
        mSampleSet[rs::core::stream_type::color] = rs::core::image_interface::create_instance_from_raw_data(
                &colorInfo,
                rs::core::image_interface::image_data_with_data_releaser(colorImageMsg->data.data(), nullptr),
                rs::core::stream_type::color,
                rs::core::image_interface::flag::any,
                0,
                0,
                nullptr);

        //fill depth image at mSampleSet - all images from mSampleSet will be released while callling to process_sample_set_sync
        rs::core::image_info depthInfo = {
               static_cast<int32_t>(depthImageMsg->width),
               static_cast<int32_t>(depthImageMsg->height),
               rs::core::pixel_format::z16,
               depthImageMsg->step
        };
        mSampleSet[rs::core::stream_type::depth] = rs::core::image_interface::create_instance_from_raw_data(
                &depthInfo,
                rs::core::image_interface::image_data_with_data_releaser(depthImageMsg->data.data(), nullptr),
                rs::core::stream_type::depth,
                rs::core::image_interface::flag::any,
                0,
                0,
                nullptr);
        return true;
    }

    PersonTrackingData* PersonTrackingNodelet::processFrame()
    {
        auto status = mPersonTrackingVideoModule->process_sample_set_sync(&mSampleSet);
        if (status != rs::core::status::status_no_error)
        {
            ROS_ERROR_STREAM("Error in process image: " << status);
            return nullptr;
        }

        PersonTrackingData* trackingData = mPersonTrackingVideoModule->QueryOutput();
        if (!trackingData)
        {
            ROS_ERROR("trackingData == null");
            return nullptr;
        }

        SimulateInteractiveTrackingMode(trackingData);
        int numDetectedPeople = trackingData->QueryNumberOfPeople();
        ROS_DEBUG_STREAM("Detected " << numDetectedPeople << " people");

        handleTrackFirst(trackingData);

        return trackingData;
    }

    void PersonTrackingNodelet::handleTrackFirst(PersonTrackingData* trackingData)
    {
        if (!mTrackFirstMode || trackingData->GetTrackingState() == PersonTrackingData::TrackingState::TRACKING_STATE_TRACKING) return;

        for (int index = 0; index < trackingData->QueryNumberOfPeople(); ++index)
        {
            PersonTrackingData::Person* personData = trackingData->QueryPersonData(PersonTrackingData::ACCESS_ORDER_NEAR_TO_FAR, index);
            if (personData)
            {
                trackingData->StartTracking(personData->QueryTracking()->QueryId());
                break;
            }
        }
    }

    void PersonTrackingNodelet::publishOutput(const sensor_msgs::ImageConstPtr& colorImageMsg, PersonTrackingData& trackingData)
    {
        mPublisher.publishOutput(colorImageMsg, trackingData);
    }

    void PersonTrackingNodelet::CameraInfo2Intrinsics(const sensor_msgs::CameraInfo& colorCameraInfo, rs::core::intrinsics& intrinsics)
    {
        memset(&intrinsics, sizeof(intrinsics), 0);
        intrinsics.width = colorCameraInfo.width;
        intrinsics.height = colorCameraInfo.height;
        intrinsics.fx = colorCameraInfo.K[0];
        intrinsics.fy = colorCameraInfo.K[1*3 + 1];;
        intrinsics.ppx = colorCameraInfo.K[2];
        intrinsics.ppy = colorCameraInfo.K[1*3 + 2];
    }


    void PersonTrackingNodelet::SimulateInteractiveTrackingMode(PT::PersonTrackingData* trackingData)
    {
        if (mIsTrackingModeInteractive &&
            trackingData->GetTrackingState() == PT::PersonTrackingData::TrackingState::TRACKING_STATE_DETECTING &&
            trackingData->QueryNumberOfPeople() > 0)
        {
            auto personData = trackingData->QueryPersonData(PT::PersonTrackingData::ACCESS_ORDER_BY_ID, 0);
            if (personData)
            {
                trackingData->StartTracking(personData->QueryTracking()->QueryId());
            }
        }
    }
}

