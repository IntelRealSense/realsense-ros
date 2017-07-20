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

#include <iostream>
#include <string>
#include <utility>
#include <map>

#pragma once
#ifndef REALSENSE_CAMERA_CONSTANTS_H
#define REALSENSE_CAMERA_CONSTANTS_H

namespace realsense_camera
{
    // Default Constants.
    const int STREAM_COUNT = 5;
    const int DEPTH_WIDTH = 480;
    const int DEPTH_HEIGHT = 360;
    const int COLOR_WIDTH = 640;
    const int COLOR_HEIGHT = 480;
    const int FISHEYE_WIDTH = 640;
    const int FISHEYE_HEIGHT = 480;
    const int DEPTH_FPS = 60;
    const int COLOR_FPS = 60;
    const int FISHEYE_FPS = 60;
    const bool ENABLE_DEPTH = true;
    const bool ENABLE_COLOR = true;
    const bool ENABLE_IR = false;
    const bool ENABLE_IR2 = false;
    const bool ENABLE_FISHEYE = true;
    const bool ENABLE_IMU = true;
    const bool ENABLE_PC = false;
    const bool ENABLE_TF = true;
    const bool ENABLE_TF_DYNAMIC = false;
    const double TF_PUBLICATION_RATE = 1.0;
    const std::string DEFAULT_MODE = "preset";
    const std::string DEFAULT_BASE_FRAME_ID = "camera_link";
    const std::string DEFAULT_DEPTH_FRAME_ID = "camera_depth_frame";
    const std::string DEFAULT_COLOR_FRAME_ID = "camera_rgb_frame";
    const std::string DEFAULT_IR_FRAME_ID = "camera_ir_frame";
    const std::string DEFAULT_DEPTH_OPTICAL_FRAME_ID = "camera_depth_optical_frame";
    const std::string DEFAULT_COLOR_OPTICAL_FRAME_ID = "camera_rgb_optical_frame";
    const std::string DEFAULT_IR_OPTICAL_FRAME_ID = "camera_ir_optical_frame";
    const std::string DEPTH_NAMESPACE = "depth";
    const std::string DEPTH_TOPIC = "image_raw";
    const std::string PC_TOPIC = "points";
    const std::string COLOR_NAMESPACE = "color";
    const std::string COLOR_TOPIC = "image_raw";
    const std::string IR_NAMESPACE = "ir";
    const std::string IR_TOPIC = "image_raw";
    const std::string SETTINGS_SERVICE = "get_settings";
    const std::string CAMERA_IS_POWERED_SERVICE = "is_powered";
    const std::string CAMERA_SET_POWER_SERVICE = "set_power";
    const std::string CAMERA_FORCE_POWER_SERVICE = "force_power";
    const std::string STREAM_DESC[STREAM_COUNT] = {"Depth", "Color", "IR", "IR2", "Fisheye"};
    const int EVENT_COUNT = 2;
    const double ROTATION_IDENTITY[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const float MILLIMETER_METERS  = 0.001;

    // R200 and ZR300 Constants.
    const std::string IR2_NAMESPACE = "ir2";
    const std::string IR2_TOPIC = "image_raw";
    const std::string DEFAULT_IR2_FRAME_ID = "camera_ir2_frame";
    const std::string DEFAULT_IR2_OPTICAL_FRAME_ID = "camera_ir2_optical_frame";



    // R200 Constants.
    // Indoor Range: 0.7m - 3.5m, Outdoor Range: 10m
    const float R200_MAX_Z = 10.0f;   // in meters
    const std::string R200_CAMERA_FW_VERSION = "1.0.72.06";
    // LR200 Constants.
    const std::string LR200_CAMERA_FW_VERSION = "2.0.71.18";
    // F200 Constants.
    // Indoor Range: 0.2m – 1.0m, Outdoor Range: n/a
    const float F200_MAX_Z = 1.0f;    // in meters
    const std::string F200_CAMERA_FW_VERSION = "2.60.0.0";

    // SR300 Constants.
    // Indoor Range: 0.2m – 1.5m, Outdoor Range: n/a
    const float SR300_MAX_Z = 1.5f;   // in meters
    const std::string SR300_CAMERA_FW_VERSION = "3.10.10.0";

    // ZR300 Constants.
    // Indoor Range: 0.7m - 3.5m, Outdoor Range: 10m
    const float ZR300_MAX_Z = 10.0f;  // in meters
    const std::string FISHEYE_NAMESPACE = "fisheye";
    const std::string FISHEYE_TOPIC = "image_raw";
    const std::string IMU_NAMESPACE = "imu";
    const std::string IMU_TOPIC = "data_raw";
    const std::string IMU_INFO_SERVICE = "get_imu_info";
    const std::string DEFAULT_FISHEYE_FRAME_ID = "camera_fisheye_frame";
    const std::string DEFAULT_IMU_FRAME_ID = "camera_imu_frame";
    const std::string DEFAULT_FISHEYE_OPTICAL_FRAME_ID = "camera_fisheye_optical_frame";
    const std::string DEFAULT_IMU_OPTICAL_FRAME_ID = "camera_imu_optical_frame";
    const std::string IMU_ACCEL = "IMU_ACCEL";
    const std::string IMU_GYRO = "IMU_GYRO";
    const double IMU_UNITS_TO_MSEC = 0.00003125;
    const std::string ZR300_CAMERA_FW_VERSION = "2.0.71.28";
    const std::string ZR300_ADAPTER_FW_VERSION = "1.29.0.0";
    const std::string ZR300_MOTION_MODULE_FW_VERSION = "1.25.0.0";

    // map the camera name to its validated firmware
    typedef std::pair<std::string, std::string> stringpair;
    const stringpair MAP_START_VALUES[] =
    {
      stringpair("Intel RealSense R200_camera", R200_CAMERA_FW_VERSION),
      stringpair("Intel RealSense F200_camera", F200_CAMERA_FW_VERSION),
      stringpair("Intel RealSense SR300_camera", SR300_CAMERA_FW_VERSION),
      stringpair("Intel RealSense ZR300_camera", ZR300_CAMERA_FW_VERSION),
      stringpair("Intel RealSense ZR300_adapter", ZR300_ADAPTER_FW_VERSION),
      stringpair("Intel RealSense ZR300_motion_module", ZR300_MOTION_MODULE_FW_VERSION),
      stringpair("Intel RealSense LR200_camera", LR200_CAMERA_FW_VERSION)
    };

    const int MAP_START_VALUES_SIZE = sizeof(MAP_START_VALUES) /
          sizeof(MAP_START_VALUES[0]);

    extern const std::map<std::string, std::string> CAMERA_NAME_TO_VALIDATED_FIRMWARE;

}  // namespace realsense_camera
#endif  // REALSENSE_CAMERA_CONSTANTS_H
