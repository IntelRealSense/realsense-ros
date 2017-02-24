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
    const std::string DEFAULT_SERIAL_NO = "";

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
    const bool ENABLE_FISHEYE = true;

    const double ROTATION_IDENTITY[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const float MILLIMETER_METERS  = 0.001;
    
    const std::string DEFAULT_DEPTH_OPTICAL_FRAME_ID = "camera_depth_optical_frame";
    const std::string DEFAULT_COLOR_OPTICAL_FRAME_ID = "camera_rgb_optical_frame";
    const std::string DEFAULT_FISHEYE_OPTICAL_FRAME_ID = "camera_fisheye_optical_frame";
    const std::string DEFAULT_ACCEL_OPTICAL_FRAME_ID = "camera_accel_optical_frame";
    const std::string DEFAULT_GYRO_OPTICAL_FRAME_ID = "camera_gyro_optical_frame";
}  // namespace realsense_camera
#endif  // REALSENSE_CAMERA_CONSTANTS_H
