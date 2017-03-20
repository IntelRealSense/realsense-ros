// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <sensor_msgs/Image.h>
#include <mutex>

#include "realsense_ros_person/Frame.h"
#include "realsense_ros_person/Recognition.h"
#include "realsense_ros_person/RecognitionRegister.h"
#include "realsense_ros_person/PersonModuleState.h"
#include "realsense_ros_person/StartTracking.h"


#include <fstream> //TODO remove this include

std::string DETECTION_TOPIC = "/person_tracking_output";
std::string REGISTER_SERVICE = "/person_tracking/register_request";
std::string RECOGNIZE_SERVICE = "/person_tracking/recognition_request";
std::string START_TRACKING_SERVICE = "/person_tracking/start_tracking_request";

bool gDetectionRecv = false;
bool gDetectionBBoxRecv = false;
bool gDetectionComRecv = false;
bool gWaveRecv = false;

realsense_ros_person::User gUsers;
ros::ServiceClient gRegisterClient;
ros::ServiceClient gRecognizeClient;
ros::ServiceClient gStartTrackingClient;

realsense_ros_person::Frame g_latestFrameData;
std::mutex g_latestFrameDataMutex;

void detectionCallback(const realsense_ros_person::Frame& frame)
{
  gDetectionRecv = true;
  {
    std::lock_guard<std::mutex> lockGuard(g_latestFrameDataMutex);
    g_latestFrameData = frame;
  }
  
  for (realsense_ros_person::User person: frame.usersData)
  {
    if ((person.userRect.rectCorners[0].x != 0) || (person.userRect.rectCorners[0].y != 0) ||
        (person.userRect.rectCorners[1].x != 0) || (person.userRect.rectCorners[1].y != 0))
    {
      gDetectionBBoxRecv = true;
    }
    else
    {
      gDetectionBBoxRecv = false;
    }

    if ((person.centerOfMassWorld.x != 0) || (person.centerOfMassWorld.y != 0) || (person.centerOfMassWorld.z != 0) ||
        (person.centerOfMassImage.x != 0) || (person.centerOfMassImage.y != 0))
    {
      gDetectionComRecv = true;
    }
    else
    {
      gDetectionComRecv = false;
    }

    //if received minimum once while test - test passed
    if (!gWaveRecv && person.gestures.wave.type != realsense_ros_person::Wave::WAVE_NOT_DETECTED)
    {
      gWaveRecv = true;
    }

    if ((gDetectionBBoxRecv == false) || (gDetectionComRecv == false))
    {
      break;
    }
  }
}

TEST(PersonTests, PersonDetection)
{
  EXPECT_TRUE(gDetectionRecv);
  EXPECT_TRUE(gDetectionBBoxRecv);
  EXPECT_TRUE(gDetectionComRecv);
}


TEST(PersonTests, WaveDetection)
{
  EXPECT_TRUE(gDetectionRecv);
  EXPECT_TRUE(gDetectionBBoxRecv);
  EXPECT_TRUE(gDetectionComRecv);
  EXPECT_TRUE(gWaveRecv);
}

TEST(PersonTests, Recognition)
{
  //register person
  realsense_ros_person::RecognitionRegister registerRequest;
  {
    std::lock_guard <std::mutex> lockGuard(g_latestFrameDataMutex);
    EXPECT_TRUE(g_latestFrameData.numberOfUsers > 0);
    registerRequest.request.personId = g_latestFrameData.usersData[0].userInfo.Id;
  }
  gRegisterClient.call(registerRequest);


  //recognize person
  realsense_ros_person::Recognition recognitionRequest;
  {
    std::lock_guard<std::mutex> lockGuard(g_latestFrameDataMutex);
    EXPECT_TRUE(g_latestFrameData.numberOfUsers > 0);
    recognitionRequest.request.personId = g_latestFrameData.usersData[0].userInfo.Id;
  }
  gRecognizeClient.call(recognitionRequest);

  ASSERT_EQ(recognitionRequest.response.recognitionId, registerRequest.response.recognitionId);
}

TEST(PersonTests, Tracking)
{
  int personTrackingId = -1;
  {
    std::lock_guard <std::mutex> lockGuard(g_latestFrameDataMutex);
    EXPECT_TRUE(g_latestFrameData.numberOfUsers > 0);
    personTrackingId = g_latestFrameData.usersData[0].userInfo.Id;
  }

  realsense_ros_person::StartTracking startTrackingRequest;
  startTrackingRequest.request.personId = personTrackingId;
  gStartTrackingClient.call(startTrackingRequest);

  EXPECT_TRUE(startTrackingRequest.response.status);
}

int main(int argc, char **argv) try
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_topics");

  ros::NodeHandle nh;
  ROS_INFO_STREAM("RealSense person test - Initializing Tests...");

  ros::Subscriber detection_sub = nh.subscribe(DETECTION_TOPIC, 1, detectionCallback);
  gRegisterClient = nh.serviceClient<realsense_ros_person::RecognitionRegister>(REGISTER_SERVICE);
  gRecognizeClient = nh.serviceClient<realsense_ros_person::Recognition>(RECOGNIZE_SERVICE);
  gStartTrackingClient = nh.serviceClient<realsense_ros_person::StartTracking>(START_TRACKING_SERVICE);

  ros::Rate r(10);
  for (int i = 0; i< 100; ++i)
  {
    ros::spinOnce();
    r.sleep();
  }

  return RUN_ALL_TESTS();
}
catch(...) {} // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
