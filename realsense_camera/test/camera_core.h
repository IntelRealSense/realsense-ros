/******************************************************************************
 Copyright (c) 2016, Intel Corporation
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

#include <cstdlib>
#include <cctype>
#include <sstream>
#include <iostream>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <realsense_camera/CameraConfiguration.h>
#include <realsense_camera/IsPowered.h>
#include <realsense_camera/SetPower.h>
#include <realsense_camera/ForcePower.h>
#include <tf/transform_listener.h>
#include <librealsense/rs.h>
#include <realsense_camera/constants.h>

using namespace realsense_camera;

//utest commandline args
int g_color_height_exp = 0;
int g_color_width_exp = 0;
int g_depth_height_exp = 0;
int g_depth_width_exp = 0;
uint32_t g_depth_step_exp; // Expected depth step.
uint32_t g_color_step_exp; // Expected color step.
uint32_t g_infrared1_step_exp; // Expected infrared1 step.

bool g_enable_color = true;
bool g_enable_depth = true;
bool g_enable_fisheye = false;
bool g_enable_imu = false;
bool g_enable_pointcloud = false;

std::string g_depth_encoding_exp; // Expected depth encoding.
std::string g_color_encoding_exp; // Expected color encoding.
std::string g_infrared1_encoding_exp; // Expected infrared1 encoding.

image_transport::CameraSubscriber g_camera_subscriber[STREAM_COUNT];
ros::Subscriber g_sub_imu;
ros::Subscriber g_sub_pc;

ros::ServiceClient g_settings_srv_client;
ros::ServiceClient g_ispowered_srv_client;
ros::ServiceClient g_setpower_srv_client;
ros::ServiceClient g_forcepower_srv_client;

std::map<std::string, std::string> g_config_args;
double g_max_z = R200_MAX_Z * 1000.0f; // Converting meter to mm.

bool g_depth_recv = false;
bool g_color_recv = false;
bool g_infrared1_recv = false;
bool g_infrared2_recv = false;
bool g_fisheye_recv = false;
bool g_imu_recv = false;
bool g_pc_recv = false;

float g_depth_avg = 0.0f;
float g_color_avg = 0.0f;
float g_infrared1_avg = 0.0f;
float g_infrared2_avg = 0.0f;
float g_fisheye_avg = 0.0f;
float g_pc_depth_avg = 0.0f;

int g_height_recv[STREAM_COUNT] = {0};
int g_width_recv[STREAM_COUNT] = {0};
uint32_t g_step_recv[STREAM_COUNT] = {0}; // Received stream step.

std::string g_encoding_recv[STREAM_COUNT]; // Expected stream encoding.

int g_caminfo_height_recv[STREAM_COUNT] = {0};
int g_caminfo_width_recv[STREAM_COUNT] = {0};
double g_color_caminfo_D_recv[5] = {0.0};
double g_depth_caminfo_D_recv[5] = {0.0};
double g_infrared1_caminfo_D_recv[5] = {0.0};
double g_infrared2_caminfo_D_recv[5] = {0.0};
double g_fisheye_caminfo_D_recv[5] = {0.0};

double g_caminfo_rotation_recv[STREAM_COUNT][9] = {{0.0}};
double g_caminfo_projection_recv[STREAM_COUNT][12] = {{0.0}};

std::string g_dmodel_recv[STREAM_COUNT];
std::string g_camera_type;

realsense_camera::CameraConfiguration g_setting_srv;
realsense_camera::IsPowered g_ispowered_srv;
realsense_camera::SetPower g_setpower_srv;
realsense_camera::ForcePower g_forcepower_srv;
