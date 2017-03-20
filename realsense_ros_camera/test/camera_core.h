// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once
#ifndef CAMERA_CORE_H  // NOLINT(build/header_guard)
#define CAMERA_CORE_H

#include <cstdlib>
#include <cctype>
#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <librealsense/rs.h>
#include <realsense_ros_camera/constants.h>

const int STREAM_COUNT = 5;

// utest commandline args
int g_color_height_exp = 0;
int g_color_width_exp = 0;
int g_depth_height_exp = 0;
int g_depth_width_exp = 0;
int g_fisheye_height_exp = 0;
int g_fisheye_width_exp = 0;
uint32_t g_depth_step_exp;  // Expected depth step.
uint32_t g_color_step_exp;  // Expected color step.

bool g_enable_color = true;
bool g_enable_depth = true;
bool g_enable_fisheye = false;
bool g_enable_imu = false;

std::string g_depth_encoding_exp;  // Expected depth encoding.
std::string g_color_encoding_exp;  // Expected color encoding.

std::map<std::string, std::string> g_config_args;
const float R200_MAX_Z = 10.0f;
double g_max_z = R200_MAX_Z * 1000.0f;  // Converting meter to mm.

bool g_depth_recv = false;
bool g_color_recv = false;
bool g_fisheye_recv = false;
bool g_accel_recv = false;
bool g_gyro_recv = false;

float g_depth_avg = 0.0f;
float g_color_avg = 0.0f;
float g_fisheye_avg = 0.0f;
float g_pc_depth_avg = 0.0f;

int g_height_recv[STREAM_COUNT] = {0};
int g_width_recv[STREAM_COUNT] = {0};
uint32_t g_step_recv[STREAM_COUNT] = {0};  // Received stream step.

std::string g_encoding_recv[STREAM_COUNT];  // Expected stream encoding.

int g_caminfo_height_recv[STREAM_COUNT] = {0};
int g_caminfo_width_recv[STREAM_COUNT] = {0};
double g_color_caminfo_D_recv[5] = {0.0};
double g_depth_caminfo_D_recv[5] = {0.0};
double g_fisheye_caminfo_D_recv[5] = {0.0};

double g_caminfo_rotation_recv[STREAM_COUNT][9] = {{0.0}};
double g_caminfo_projection_recv[STREAM_COUNT][12] = {{0.0}};

std::string g_dmodel_recv[STREAM_COUNT];
std::string g_camera_type;


#endif  // CAMERA_CORE_H  // NOLINT(build/header_guard)
