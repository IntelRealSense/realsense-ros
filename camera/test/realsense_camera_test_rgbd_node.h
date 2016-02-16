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

#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <gtest/gtest.h>

const static int TOPIC_COUNT = 13;

const char *RGB_IMAGE_MONO = "/camera/rgb/image_mono";
const char *RGB_IMAGE_COLOR = "/camera/rgb/image_color";
const char *RGB_IMAGE_RECT_MONO = "/camera/rgb/image_rect_mono";
const char *RGB_IMAGE_RECT_COLOR = "/camera/rgb/image_rect_color";
const char *DEPTH_IMAGE_RECT_RAW = "/camera/depth/image_rect_raw";
const char *DEPTH_IMAGE_RECT = "/camera/depth/image_rect";
const char *DEPTH_IMAGE = "/camera/depth/image";
const char *DEPTH_POINTS = "/camera/depth/points";
const char *IR_IMAGE_RECT_IR = "/camera/ir/image_rect_ir";
const char *DEPTH_REG_SW_REG_IMAGE_RECT_RAW = "/camera/depth_registered/sw_registered/image_rect_raw";
const char *DEPTH_REG_SW_REG_CAMERA_INFO = "/camera/depth_registered/sw_registered/camera_info";
const char *DEPTH_REG_POINTS = "/camera/depth_registered/points";
const char *DEPTH_REG_SW_REG_IMAGE_RECT = "/camera/depth_registered/sw_registered/image_rect";

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
