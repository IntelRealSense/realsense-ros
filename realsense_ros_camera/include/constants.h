// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <string>

#pragma once
#ifndef REALSENSE_CAMERA_CONSTANTS_H
#define REALSENSE_CAMERA_CONSTANTS_H

#define REALSENSE_ROS_MAJOR_VERSION    2
#define REALSENSE_ROS_MINOR_VERSION    0
#define REALSENSE_ROS_PATCH_VERSION    1

#define STRINGIFY(arg) #arg
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
/* Return version in "X.Y.Z" format */
#define REALSENSE_ROS_VERSION_STR (VAR_ARG_STRING(REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))

namespace realsense_ros_camera
{
    const bool POINTCLOUD     = false;
    const bool SYNC_FRAMES    = false;

    const int DEPTH_WIDTH     = 640;
    const int DEPTH_HEIGHT    = 480;

    const int INFRA1_WIDTH    = 640;
    const int INFRA1_HEIGHT   = 480;

    const int INFRA2_WIDTH    = 640;
    const int INFRA2_HEIGHT   = 480;

    const int COLOR_WIDTH     = 640;
    const int COLOR_HEIGHT    = 480;

    const int FISHEYE_WIDTH   = 640;
    const int FISHEYE_HEIGHT  = 480;


    const int DEPTH_FPS       = 30;
    const int INFRA1_FPS      = 30;
    const int INFRA2_FPS      = 30;
    const int COLOR_FPS       = 30;
    const int FISHEYE_FPS     = 30;
    const int GYRO_FPS        = 1000;
    const int ACCEL_FPS       = 1000;


    const bool ENABLE_DEPTH   = true;
    const bool ENABLE_INFRA1  = true;
    const bool ENABLE_INFRA2  = true;
    const bool ENABLE_COLOR   = true;
    const bool ENABLE_FISHEYE = true;
    const bool ENABLE_IMU     = true;


    const std::string DEFAULT_BASE_FRAME_ID            = "camera_link";
    const std::string DEFAULT_DEPTH_FRAME_ID           = "camera_depth_frame";
    const std::string DEFAULT_INFRA1_FRAME_ID          = "camera_infra1_frame";
    const std::string DEFAULT_INFRA2_FRAME_ID          = "camera_infra2_frame";
    const std::string DEFAULT_COLOR_FRAME_ID           = "camera_color_frame";
    const std::string DEFAULT_FISHEYE_FRAME_ID         = "camera_fisheye_frame";
    const std::string DEFAULT_IMU_FRAME_ID             = "camera_imu_frame";
    const std::string DEFAULT_DEPTH_OPTICAL_FRAME_ID   = "camera_depth_optical_frame";
    const std::string DEFAULT_INFRA1_OPTICAL_FRAME_ID  = "camera_infra1_optical_frame";
    const std::string DEFAULT_INFRA2_OPTICAL_FRAME_ID  = "camera_infra2_optical_frame";
    const std::string DEFAULT_COLOR_OPTICAL_FRAME_ID   = "camera_color_optical_frame";
    const std::string DEFAULT_FISHEYE_OPTICAL_FRAME_ID = "camera_fisheye_optical_frame";
    const std::string DEFAULT_ACCEL_OPTICAL_FRAME_ID   = "camera_accel_optical_frame";
    const std::string DEFAULT_GYRO_OPTICAL_FRAME_ID    = "camera_gyro_optical_frame";
    const std::string DEFAULT_IMU_OPTICAL_FRAME_ID     = "camera_imu_optical_frame";
}  // namespace realsense_ros_camera
#endif  // REALSENSE_CAMERA_CONSTANTS_H
