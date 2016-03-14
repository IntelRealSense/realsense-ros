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
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <camera_info_manager/camera_info_manager.h>
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <pcl-1.7/pcl/conversions.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "std_msgs/Float32MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <realsense_camera/cameraConfiguration.h>
#include <tf/transform_listener.h>

const char *DEPTH_TOPIC = "camera/depth/image_raw";
const char *COLOR_TOPIC = "camera/color/image_raw";
const char *IR1_TOPIC = "camera/infrared1/image_raw";
const char *IR2_TOPIC = "camera/infrared2/image_raw";
const char *PC_TOPIC = "camera/depth/points";
const char *SETTINGS_SERVICE = "camera/get_settings";
const char *R200 = "R200";
const int R200_DEPTH_MAX = 10000;

const char *BASE_DEF_FRAME = "camera_link";
const char *DEPTH_DEF_FRAME = "camera_depth_frame";
const char *COLOR_DEF_FRAME = "camera_rgb_frame";
const char *DEPTH_OPTICAL_DEF_FRAME = "camera_depth_optical_frame";
const char *COLOR_OPTICAL_DEF_FRAME = "camera_rgb_optical_frame";
const double ROTATION_IDENTITY[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

//utest commandline args
int color_height_exp = 0;
int color_width_exp = 0;
int depth_height_exp = 0;
int depth_width_exp = 0;
uint32_t depth_step_exp; 	// Expected depth step.
uint32_t color_step_exp; 	// Expected color step.
uint32_t infrared1_step_exp; 	// Expected infrared1 step.
uint32_t infrared2_step_exp; 	// Expected infrared2 step.

bool enable_color = true;
bool enable_depth = true;
bool enable_pointcloud = true;

std::string depth_encoding_exp; // Expected depth encoding.
std::string color_encoding_exp; // Expected color encoding.
std::string infrared1_encoding_exp; // Expected infrared1 encoding.
std::string infrared2_encoding_exp; // Expected infrared2 encoding.

image_transport::CameraSubscriber camera_subscriber[4];
ros::Subscriber m_sub_pc;

std::map<std::string, std::string> config_args;

bool depth_recv = false;
bool color_recv = false;
bool infrared1_recv = false;
bool infrared2_recv = false;
bool pc_recv = false;

float depth_avg = 0;
float color_avg = 0;
float infrared1_avg = 0;
float infrared2_avg = 0;
float pc_depth_avg = 0;

int depth_height_recv = 0;
int depth_width_recv = 0;
int color_height_recv = 0;
int color_width_recv = 0;
int infrared1_width_recv = 0;
int infrared1_height_recv = 0;
int infrared2_width_recv = 0;
int infrared2_height_recv = 0;

uint32_t depth_step_recv = 0;	// Received depth step.
uint32_t color_step_recv = 0;	// Received depth step.
uint32_t infrared1_step_recv = 0;	// Received depth step.
uint32_t infrared2_step_recv = 0;	// Received depth step.

std::string depth_encoding_recv; // Expected depth encoding.
std::string infrared1_encoding_recv; // Expected depth encoding.
std::string infrared2_encoding_recv; // Expected depth encoding.
std::string color_encoding_recv; // Expected color encoding.


int depth_caminfo_height_recv = 0;
int depth_caminfo_width_recv = 0;
double depth_caminfo_rotation_recv[9] = {0};
double depth_caminfo_projection_recv[12] = {0};
int color_caminfo_height_recv = 0;
int color_caminfo_width_recv = 0;
double color_caminfo_rotation_recv[9] = {0};
double color_caminfo_projection_recv[12] = {0};
int inf1_caminfo_height_recv = 0;
int inf1_caminfo_width_recv = 0;
double inf1_caminfo_rotation_recv[9] = {0};
double inf1_caminfo_projection_recv[12] = {0};
int inf2_caminfo_height_recv = 0;
int inf2_caminfo_width_recv = 0;
double inf2_caminfo_rotation_recv[9] = {0};
double inf2_caminfo_projection_recv[12] = {0};

std::string inf1_dmodel_recv;
std::string inf2_dmodel_recv;
std::string depth_dmodel_recv;
std::string color_dmodel_recv;
std::string camera = "R200";

realsense_camera::cameraConfiguration srv;

