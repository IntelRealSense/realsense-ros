/******************************************************************************
 INTEL CORPORATION PROPRIETARY INFORMATION
 This software is supplied under the terms of a license agreement or nondisclosure
 agreement with Intel Corporation and may not be copied or disclosed except in
 accordance with the terms of that agreement
 Copyright(c) 2011-2016 Intel Corporation. All Rights Reserved.
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

const char *DEPTH_TOPIC = "camera/depth/image_raw";
const char *COLOR_TOPIC = "camera/color/image_raw";
const char *IR1_TOPIC = "camera/infrared1/image_raw";
const char *IR2_TOPIC = "camera/infrared2/image_raw";
const char *PC_TOPIC = "camera/depth/points";
const char *SETTINGS_SERVICE = "camera/get_settings";
const char *R200 = "R200";
const int R200_DEPTH_MAX = 10000;

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
int color_caminfo_height_recv = 0;
int color_caminfo_width_recv = 0;
int inf1_caminfo_height_recv = 0;
int inf1_caminfo_width_recv = 0;
int inf2_caminfo_height_recv = 0;
int inf2_caminfo_width_recv = 0;

std::string inf1_dmodel_recv;
std::string inf2_dmodel_recv;
std::string depth_dmodel_recv;
std::string color_dmodel_recv;
std::string camera = "R200";

realsense_camera::cameraConfiguration srv;

