/******************************************************************************
 INTEL CORPORATION PROPRIETARY INFORMATION
 This software is supplied under the terms of a license agreement or nondisclosure
 agreement with Intel Corporation and may not be copied or disclosed except in
 accordance with the terms of that agreement
 Copyright(c) 2011-2015 Intel Corporation. All Rights Reserved.
 *******************************************************************************/

#include "ros/ros.h"

#include "PersonTrackingNodelet.h"
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
        : mNodeHandle(nullptr), mPersonTracking(PtOpencvAdapter::CreateInstance(L"/usr/local")),
          mColorInfoReceived(false), mDepthInfoReceived(false), mIrStreamsEnabled(false) {}

    PersonTrackingNodelet::~PersonTrackingNodelet(){}

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
            mPersonTracking->QueryConfiguration()->QueryRecognition()->Enable();
        }
        if (request.enableSegmentation || request.enableBlob)
        {
            ROS_INFO("Enable segmentation");
            mPersonTracking->QueryConfiguration()->QueryTracking()->EnableSegmentation();
        }
        if (request.enableSkeleton)
        {
            ROS_INFO("Enable skeleton");
            mPersonTracking->QueryConfiguration()->QuerySkeletonJoints()->Enable();
        }
        if (request.enableGestures)
        {
            ROS_INFO("Enable gestures");
            mPersonTracking->QueryConfiguration()->QueryGestures()->Enable();
            mPersonTracking->QueryConfiguration()->QueryGestures()->EnableGesture(PersonTrackingData::PersonGestures::Pointing);
        }
        mPublisher.enableBlob(request.enableBlob, request.enableSegmentation);
        configureTrackingMode(request.trackingMode);
    }

    void PersonTrackingNodelet::configureTrackingMode(int trackingMode)
    {
        mTrackFirstMode = trackingMode == 2;
        trackingMode = mTrackFirstMode ? 0 : trackingMode;
        mPersonTracking->QueryConfiguration()->QueryTracking()->SetTrackingMode(PersonTrackingConfiguration::TrackingConfiguration::TrackingMode(trackingMode));
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
        mServer.onInit(nodeHandle, mPersonTracking.get(), this);

        // Init publisher
        mPublisher.onInit(nodeHandle, mPersonTracking.get());
    }

    void PersonTrackingNodelet::colorCameraInfoCallback(const sensor_msgs::CameraInfo colorCameraInfo)
    {
        if (mColorInfoReceived) return;
        mColorInfoReceived = true;
        ROS_INFO("Received color camera info. %dx%d", colorCameraInfo.width, colorCameraInfo.height);
        setCalibrationData(colorCameraInfo, mCalibrationData.colorInfo);
        setCaptureProfile();
    }

    void PersonTrackingNodelet::depthCameraInfoCallback(const sensor_msgs::CameraInfo depthCameraInfo)
    {
        if (mDepthInfoReceived) return;
        mDepthInfoReceived = true;
        ROS_INFO("Received depth camera info. %dx%d", depthCameraInfo.width, depthCameraInfo.height);
        setCalibrationData(depthCameraInfo, mCalibrationData.depthInfo);

        mCalibrationData.irInfo.width = (mIrStreamsEnabled) ? mCalibrationData.depthInfo.width : 0;
        mCalibrationData.irInfo.height = (mIrStreamsEnabled) ? mCalibrationData.depthInfo.height : 0;

        setCaptureProfile();
    }

    void PersonTrackingNodelet::setCalibrationData(const sensor_msgs::CameraInfo& cameraInfoMsg, PtProjection::CameraInfo& cameraInfo)
    {
        cameraInfo.width = cameraInfoMsg.width;
        cameraInfo.height = cameraInfoMsg.height;

        cameraInfo.cameraMatrix.focalLengthX = (float)cameraInfoMsg.K[0];
        cameraInfo.cameraMatrix.focalLengthY = (float)cameraInfoMsg.K[4];
        cameraInfo.cameraMatrix.opticalCenterX = (float)cameraInfoMsg.K[2];
        cameraInfo.cameraMatrix.opticalCenterY = (float)cameraInfoMsg.K[5];

        memcpy(cameraInfo.distortionMatrix, cameraInfoMsg.D.data(), sizeof(double) * 5);
        memcpy(cameraInfo.rotationMatrix, cameraInfoMsg.R.data(), sizeof(double) * 9);

        cameraInfo.translationMatrix[0] = (float)cameraInfoMsg.P[3] * 1000;
        cameraInfo.translationMatrix[1] = (float)cameraInfoMsg.P[7] * 1000;
        cameraInfo.translationMatrix[2] = (float)cameraInfoMsg.P[11] * 1000;
    }

    void PersonTrackingNodelet::setCaptureProfile()
    {
        if (!mColorInfoReceived || !mDepthInfoReceived) return;

        Status status = mPersonTracking->SetCalibrationData(mCalibrationData);
        if (status != PXC_STATUS_NO_ERROR)
        {
            ROS_ERROR_STREAM("Failed to set capture profile with error: " << status);
            mColorInfoReceived = false;
            mDepthInfoReceived = false;
        }
        else
        {
            ROS_INFO("Done set capture profile - subscribing to frame messages");
            subscribeFrameMessages();
            mColorCameraInfoSubscriber.shutdown();
            mDepthCameraInfoSubscriber.shutdown();
        }
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

        mColorImage = cv::Mat(colorImageMsg->height, colorImageMsg->width, CV_8UC3, (void*)colorImageMsg->data.data());
        mDepthImage = cv::Mat(depthImageMsg->height, depthImageMsg->width, CV_16UC1, (void*)depthImageMsg->data.data());
        if (mIrStreamsEnabled)
        {
            mLeftImage  = cv::Mat(leftImageMsg->height, leftImageMsg->width, CV_8UC1, (void*)leftImageMsg->data.data());
            mRightImage  = cv::Mat(rightImageMsg->height, rightImageMsg->width, CV_8UC1, (void*)rightImageMsg->data.data());
        }

        return true;
    }

    PersonTrackingData* PersonTrackingNodelet::processFrame()
    {
        cv::Mat bgrImage;
        cv::cvtColor(mColorImage, bgrImage, CV_RGB2BGR);

        PtOpencvAdapter::Images images = { bgrImage, mDepthImage, mLeftImage, mRightImage };

        Status status = mPersonTracking->ProcessImage(images);
        if (status != PXC_STATUS_NO_ERROR)
        {
            ROS_ERROR_STREAM("Error in process image: " << status);
            return nullptr;
        }

        PersonTrackingData* trackingData = mPersonTracking->QueryOutput();
        if (!trackingData)
        {
            ROS_ERROR("trackingData == null");
            return nullptr;
        }

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
}

