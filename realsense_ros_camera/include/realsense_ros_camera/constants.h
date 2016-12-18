// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved

#include <iostream>
#include <string>
#include <utility>
#include <map>

#pragma once
#ifndef REALSENSE_CAMERA_CONSTANTS_H
#define REALSENSE_CAMERA_CONSTANTS_H

namespace realsense_ros_camera
{
    // Default Constants.
<<<<<<< HEAD
    const int STREAM_COUNT = 5;
=======
    const std::string DEFAULT_SERIAL_NO = "";

>>>>>>> master
    const int DEPTH_WIDTH = 480;
    const int DEPTH_HEIGHT = 360;
    const int COLOR_WIDTH = 640;
    const int COLOR_HEIGHT = 480;
    const int FISHEYE_WIDTH = 640;
    const int FISHEYE_HEIGHT = 480;
    const int DEPTH_FPS = 30;
    const int COLOR_FPS = 30;
    const int FISHEYE_FPS = 30;
    const bool ENABLE_DEPTH = true;
    const bool ENABLE_COLOR = true;
<<<<<<< HEAD
    const bool ENABLE_IR = false;
    const bool ENABLE_IR2 = false;
    const bool ENABLE_FISHEYE = true;
    const bool ENABLE_IMU = true;
    const bool ENABLE_PC = false;
    const bool ENABLE_TF = true;
    const bool ENABLE_TF_DYNAMIC = false;
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

=======
    const bool ENABLE_FISHEYE = true;

    const double ROTATION_IDENTITY[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const float MILLIMETER_METERS  = 0.001;
    
    const std::string DEFAULT_BASE_FRAME_ID = "camera_link";
    const std::string DEFAULT_DEPTH_FRAME_ID = "camera_depth_frame";
    const std::string DEFAULT_COLOR_FRAME_ID = "camera_rgb_frame";
    const std::string DEFAULT_FISHEYE_FRAME_ID = "camera_fisheye_frame";
    const std::string DEFAULT_IMU_FRAME_ID = "camera_imu_frame";
    const std::string DEFAULT_DEPTH_OPTICAL_FRAME_ID = "camera_depth_optical_frame";
    const std::string DEFAULT_COLOR_OPTICAL_FRAME_ID = "camera_rgb_optical_frame";
    const std::string DEFAULT_FISHEYE_OPTICAL_FRAME_ID = "camera_fisheye_optical_frame";
    const std::string DEFAULT_IMU_OPTICAL_FRAME_ID = "camera_imu_optical_frame";
>>>>>>> master
}  // namespace realsense_camera
#endif  // REALSENSE_CAMERA_CONSTANTS_H
