// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "PersonTrackingDefaultPublisher.h"

#include "realsense_ros_person/FrameTest.h"
#include "realsense_ros_person/PersonModuleState.h"

namespace realsense_ros_person
{
void PersonTrackingDefaultPublisher::onInit(ros::NodeHandle &nodeHandle)
{
  std::string personTrackingTopic = "person_tracking_output";
  std::string personTrackingTestTopic = "person_tracking_output_test";
  std::string personTrackingModuleStateTopic = "/person_tracking/module_state";
  ROS_INFO_STREAM("Publishing to " << personTrackingTopic);
  mPublisher = nodeHandle.advertise<realsense_ros_person::Frame>(personTrackingTopic, 1);
  mTestPublisher = nodeHandle.advertise<realsense_ros_person::FrameTest>(personTrackingTestTopic,
                   1);
  mPersonModuleStatePublisher = nodeHandle.advertise<realsense_ros_person::PersonModuleState>(personTrackingModuleStateTopic, 1);
}

void PersonTrackingDefaultPublisher::publishOutput(
  Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
  Intel::RealSense::PersonTracking::PersonTrackingData &trackingData)
{

  realsense_ros_person::Frame frame;
  FillFrameData(frame, ptConfiguration, trackingData);
  mPublisher.publish(frame);
  mPersonModuleStatePublisher.publish(mPtPublisherHelper.BuildPersonModuleState(ptConfiguration, trackingData));
}

void PersonTrackingDefaultPublisher::publishTestOutput(
  Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
  Intel::RealSense::PersonTracking::PersonTrackingData &trackingData,
  const sensor_msgs::ImageConstPtr &colorImage)
{
  realsense_ros_person::FrameTest frameTest;
  //fill regular person tracking data
  FillFrameData(frameTest.frameData, ptConfiguration, trackingData);
  //fill color image
  frameTest.colorImage.height = colorImage->height;
  frameTest.colorImage.width = colorImage->width;
  frameTest.colorImage.encoding = colorImage->encoding;
  frameTest.colorImage.step = colorImage->step;
  frameTest.colorImage.data = colorImage->data;

  mTestPublisher.publish(frameTest);
}

void PersonTrackingDefaultPublisher::FillFrameData(realsense_ros_person::Frame &frame,
    Intel::RealSense::PersonTracking::PersonTrackingConfiguration &ptConfiguration,
    Intel::RealSense::PersonTracking::PersonTrackingData &trackingData)
{
  using Intel::RealSense::PersonTracking::PersonTrackingData;
  frame.numberOfUsers = trackingData.QueryNumberOfPeople();
  std::vector<realsense_ros_person::User> usersVector = mPtPublisherHelper.BuildUsersVector(ptConfiguration,
      trackingData);
  std::copy(usersVector.begin(), usersVector.end(), std::back_inserter(frame.usersData));
}
}
