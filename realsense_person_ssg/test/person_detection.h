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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <sensor_msgs/Image.h>
#include <realsense_person/constants.h>
#include <realsense_person/person_paramsConfig.h>
#include <realsense_person/PersonDetection.h>
#include <realsense_person/PersonId.h>
#include <realsense_person/Person.h>
#include <realsense_person/RegisteredPoint.h>
#include <realsense_person/BoundingBox.h>
#include <realsense_person/Pixel.h>
#include <realsense_person/GetTrackingState.h>
#include <realsense_person/Recognize.h>
#include <realsense_person/Register.h>
#include <realsense_person/Reinforce.h>

using namespace realsense_person;

const std::string CAMERA_NAMESPACE = "camera";
const int ROS_SLEEP_TIME = 10;
const int SERVICE_CALL_COUNT = 50;

std::string g_node_name;
std::string g_camera_ns;
std::string g_person_ns;
int g_ros_sleep_time;
int g_service_call_count;

bool g_detection_recv = false;
bool g_detection_image_recv = false;

bool g_detection_bbox_recv = false;
bool g_detection_com_recv = false;

std::vector<int32_t> g_tracking_ids;
std::vector<realsense_person::PersonId> g_person_ids;

ros::ServiceClient g_get_tracking_state_client;
ros::ServiceClient g_register_client;
ros::ServiceClient g_recognize_client;
ros::ServiceClient g_reinforce_client;
