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

#pragma once
#ifndef RGBD_TOPICS_H  // NOLINT(build/header_guard)
#define RGBD_TOPICS_H

#include <string>

#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <gtest/gtest.h>

static const int TOPIC_COUNT = 13;

std::string RGB_IMAGE_MONO = "image_mono";
std::string RGB_IMAGE_COLOR = "image_color";
std::string RGB_IMAGE_RECT_MONO = "image_rect_mono";
std::string RGB_IMAGE_RECT_COLOR = "image_rect_color";
std::string DEPTH_IMAGE_RECT_RAW = "image_rect_raw";
std::string DEPTH_IMAGE_RECT = "image_rect";
std::string DEPTH_IMAGE = "image";
std::string DEPTH_POINTS = "points";
std::string IR_IMAGE_RECT_IR = "image_rect_ir";
std::string DEPTH_REG_SW_REG_IMAGE_RECT_RAW = "sw_registered/image_rect_raw";
std::string DEPTH_REG_SW_REG_CAMERA_INFO = "sw_registered/camera_info";
std::string DEPTH_REG_POINTS = "points";
std::string DEPTH_REG_SW_REG_IMAGE_RECT = "sw_registered/image_rect";

std::string CAMERA = "camera";
std::string RGB = "rgb";
std::string DEPTH = "depth";
std::string IR = "ir";
std::string DEPTH_REGISTERED = "depth_registered";

std::string camera;
std::string rgb;
std::string depth;
std::string ir;
std::string depth_registered;

std::string rgb_image_mono;
std::string rgb_image_color;
std::string rgb_image_rect_mono;
std::string rgb_image_rect_color;
std::string depth_image_rect_raw;
std::string depth_image_rect;
std::string depth_image;
std::string depth_points;
std::string ir_image_rect_ir;
std::string depth_reg_sw_reg_image_rect_raw;
std::string depth_reg_sw_reg_camera_info;
std::string depth_reg_points;
std::string depth_reg_sw_reg_image_rect;

bool topic_0_recv = false;
bool topic_1_recv = false;
bool topic_2_recv = false;
bool topic_3_recv = false;
bool topic_4_recv = false;
bool topic_5_recv = false;
bool topic_6_recv = false;
bool topic_7_recv = false;
bool topic_8_recv = false;
bool topic_9_recv = false;
bool topic_10_recv = false;
bool topic_11_recv = false;
bool topic_12_recv = false;

ros::Subscriber subscriber[TOPIC_COUNT];
#endif  // RGBD_TOPICS_H  // NOLINT(build/header_guard)
