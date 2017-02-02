/******************************************************************************
 Copyright (c) 2017, Intel Corporation
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/
#include "person_detection.h"

using namespace std;
using namespace realsense_person;

void detectionCallback(const realsense_person::PersonDetection &msg)
{
  g_detection_recv = true;
  g_detection_bbox_recv = true;
  g_detection_com_recv = true;

  for (realsense_person::Person person: msg.persons)
  {
    if ((person.bounding_box.top_left.x != 0) || (person.bounding_box.top_left.y != 0) ||
        (person.bounding_box.bottom_right.x != 0) || (person.bounding_box.bottom_right.y != 0))
    {
      g_detection_bbox_recv = true;
    }
    else
    {
      g_detection_bbox_recv = false;
    }

    if ((person.center_of_mass.world.x != 0) || (person.center_of_mass.world.y != 0) ||
        (person.center_of_mass.world.z != 0) || (person.center_of_mass.image.x != 0) ||
        (person.center_of_mass.image.y != 0))
    {
      g_detection_com_recv = true;
    }
    else
    {
      g_detection_com_recv = false;
    }

    if ((g_detection_bbox_recv == false) || (g_detection_com_recv == false))
    {
      break;
    }
  }
}

void detectionImageCallback(const sensor_msgs::Image &msg)
{
  g_detection_image_recv = true;
}

TEST(RealsensePerson, DetectionTopic)
{
  EXPECT_TRUE(g_detection_recv);
  EXPECT_TRUE(g_detection_bbox_recv);
  EXPECT_TRUE(g_detection_com_recv);
}

TEST(RealsensePerson, DetectionImageTopic)
{
  EXPECT_TRUE(g_detection_image_recv);
}

TEST(RealsensePerson, RegisterService)
{
  realsense_person::GetTrackingState get_tracking_state_service;
  g_get_tracking_state_client.call(get_tracking_state_service);
  g_tracking_ids = get_tracking_state_service.response.tracking_ids;

  bool registered_all = true;

  for (auto tid: g_tracking_ids)
  {
    bool registered_one = false;
    realsense_person::Register service;
    service.request.tracking_id = tid;
    for (int i = 0; i < g_service_call_count; ++i)
    {
      g_register_client.call(service);
      if (service.response.status == 0)
      {
        registered_one = true;
        realsense_person::PersonId person_id;
        person_id.tracking_id = tid;
        person_id.recognition_id = service.response.recognition_id;
        g_person_ids.push_back(person_id);
        break;
      }
    }

    if (registered_one)
    {
      registered_all = true;
    }
    else
    {
      registered_all = false;
    }
  }

  EXPECT_TRUE(registered_all);
}

TEST(RealsensePerson, RecognizeService)
{
  bool recognized_all = true;

  for (realsense_person::PersonId pid: g_person_ids)
  {
    bool recognized_one = false;
    realsense_person::Recognize service;
    service.request.tracking_id = pid.tracking_id;
    for (int i = 0; i < g_service_call_count; ++i)
    {
      g_recognize_client.call(service);
      if (service.response.status == 0)
      {
        if (service.response.recognition_id == pid.recognition_id)
        {
          recognized_one = true;
        }
        else
        {
          recognized_one = false;
        }
        break;
      }
    }

    if (recognized_one)
    {
      recognized_all = true;
    }
    else
    {
      recognized_all = false;
    }
  }

  EXPECT_TRUE(recognized_all);
}

TEST(RealsensePerson, ReinforceService)
{
  bool reinforced_all = true;

  for (realsense_person::PersonId pid: g_person_ids)
  {
    bool reinforced_one = false;
    realsense_person::Reinforce service;
    service.request.tracking_id = pid.tracking_id;
    service.request.recognition_id = pid.recognition_id;
    for (int i = 0; i < g_service_call_count; ++i)
    {
      g_reinforce_client.call(service);
      if (service.response.status == 0)
      {
        reinforced_one = true;
        break;
      }
    }

    if (reinforced_one)
    {
      reinforced_all = true;
    }
    else
    {
      reinforced_all = false;
    }
  }

  EXPECT_TRUE(reinforced_all);
}

void getStaticParameters()
{
  ros::NodeHandle pnh("~");
  pnh.param("camera", g_camera_ns, CAMERA_NAMESPACE);
  pnh.param("person", g_person_ns, PERSON_NAMESPACE);
  pnh.param("ros_sleep_time", g_ros_sleep_time, ROS_SLEEP_TIME);
  pnh.param("service_call_count", g_service_call_count, SERVICE_CALL_COUNT);
}

int main(int argc, char **argv) try
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_topics");

  getStaticParameters();
  std::string ns_prefix = "/" + g_camera_ns + "/" + g_person_ns + "/";

  ros::NodeHandle nh;
  ros::Subscriber detection_sub = nh.subscribe(ns_prefix + DETECTION_TOPIC, 1, detectionCallback);
  ros::Subscriber detection_image_sub = nh.subscribe(ns_prefix + DETECTION_IMAGE_TOPIC, 1, detectionImageCallback);
  g_get_tracking_state_client = nh.serviceClient<realsense_person::GetTrackingState>(ns_prefix + GET_TRACKING_STATE_SERVICE);
  g_register_client = nh.serviceClient<realsense_person::Register>(ns_prefix + REGISTER_SERVICE);
  g_recognize_client = nh.serviceClient<realsense_person::Recognize>(ns_prefix + RECOGNIZE_SERVICE);
  g_reinforce_client = nh.serviceClient<realsense_person::Reinforce>(ns_prefix + REINFORCE_SERVICE);

  ros::Duration duration;
  duration.sec = g_ros_sleep_time;
  duration.sleep();
  ros::spinOnce();

  return RUN_ALL_TESTS();
}
catch(...) {} // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest

