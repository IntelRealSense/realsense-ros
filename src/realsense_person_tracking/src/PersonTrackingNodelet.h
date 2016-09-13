/******************************************************************************
 INTEL CORPORATION PROPRIETARY INFORMATION
 This software is supplied under the terms of a license agreement or nondisclosure
 agreement with Intel Corporation and may not be copied or disclosed except in
 accordance with the terms of that agreement
 Copyright(c) 2011-2015 Intel Corporation. All Rights Reserved.
 *******************************************************************************/

#pragma once

#include "PtOpencvAdapter.h"
#include "PersonTrackingServer.h"
#include "PersonTrackingPublisher.h"
#include "PersonTrackingConfigurator.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <realsense_msgs/User.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "opencv2/core/core.hpp"

#include <vector>
#include <string>
#include <memory>
#include <mutex>

namespace person_tracking
{
    class PersonTrackingNodelet : public PersonTrackingConfigurator
	{
        public:

			PersonTrackingNodelet();

			~PersonTrackingNodelet();			

            void onInit(ros::NodeHandle& nodeHandle);

            void configureTracking(realsense_srvs::TrackingConfig::Request& request) override;

		private:

            /***********************
             *  Configuration
             * *********************/

            void processCommandLineArguments();
            void configureTrackingMode(int trackingMode);

            /***********************
             *  Subscribe
             * *********************/

            void initPublishSubscribe(ros::NodeHandle& nodeHandle);
			
			void subscribeFrameMessages();

            /***********************
             *  Info callbacks
             * *********************/

            void colorCameraInfoCallback(const sensor_msgs::CameraInfo colorCameraInfo);
            void depthCameraInfoCallback(const sensor_msgs::CameraInfo depthCameraInfo);

            void setCalibrationData(const sensor_msgs::CameraInfo& cameraInfoMsg, PtProjection::CameraInfo& cameraInfo);

            void setCaptureProfile();

            /***********************
             *  Image callbacks
             * *********************/

            void personTrackingCallback(const sensor_msgs::ImageConstPtr& depthImageMsg, const sensor_msgs::ImageConstPtr& colorImageMsg);

            void personTrackingCallback(const sensor_msgs::ImageConstPtr& depthImageMsg, const sensor_msgs::ImageConstPtr& colorImageMsg,
                                        const sensor_msgs::ImageConstPtr& leftImageMsg, const sensor_msgs::ImageConstPtr& rightImageMsg);

            bool handleCallbackParameters(const sensor_msgs::ImageConstPtr& depthImageMsg, const sensor_msgs::ImageConstPtr& colorImageMsg,
                                          const sensor_msgs::ImageConstPtr& leftImageMsg, const sensor_msgs::ImageConstPtr& rightImageMsg);

            /***********************
             *  Process frame
             * *********************/

            PT::PersonTrackingData* processFrame();
            void handleTrackFirst(PT::PersonTrackingData* trackingData);

            /***********************
             *  Output
             * *********************/

            void publishOutput(const sensor_msgs::ImageConstPtr& colorImageMsg, PT::PersonTrackingData& trackingData);

            /***********************
             *  Members
             * *********************/

            PersonTrackingServer mServer;
            PersonTrackingPublisher mPublisher;

			ros::NodeHandle* mNodeHandle;

            cv::Mat mDepthImage;
            cv::Mat mColorImage;
            cv::Mat mLeftImage;
            cv::Mat mRightImage;

            ros::Subscriber mColorCameraInfoSubscriber;
            ros::Subscriber mDepthCameraInfoSubscriber;

            std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> 			mDepthSubscriber;
			std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>			mColorSubscriber;
            std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>            m_LeftSubscriber;
            std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>            m_RightSubscriber;
            std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>> mTimeSynchronizer;
            std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>> mTimeSynchronizerIr;
			
            PtProjection::CalibrationData mCalibrationData;

            std::mutex mProcessingMutex;
            std::shared_ptr<PtOpencvAdapter> mPersonTracking;

            bool mColorInfoReceived;
            bool mDepthInfoReceived;

            bool mTrackFirstMode;
            bool mIrStreamsEnabled;
	};
}


