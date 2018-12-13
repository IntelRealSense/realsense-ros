// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include <string>

#define REALSENSE_ROS_MAJOR_VERSION    2
#define REALSENSE_ROS_MINOR_VERSION    1
#define REALSENSE_ROS_PATCH_VERSION    2

#define STRINGIFY(arg) #arg
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
/* Return version in "X.Y.Z" format */
#define REALSENSE_ROS_VERSION_STR (VAR_ARG_STRING(REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))

namespace realsense2_camera
{
    const uint16_t SR300_PID        = 0x0aa5; // SR300
    const uint16_t RS400_PID        = 0x0ad1; // PSR
    const uint16_t RS410_PID        = 0x0ad2; // ASR
    const uint16_t RS415_PID        = 0x0ad3; // ASRC
    const uint16_t RS430_PID        = 0x0ad4; // AWG
    const uint16_t RS430_MM_PID     = 0x0ad5; // AWGT
    const uint16_t RS_USB2_PID      = 0x0ad6; // USB2
    const uint16_t RS420_PID        = 0x0af6; // PWG
    const uint16_t RS420_MM_PID     = 0x0afe; // PWGT
    const uint16_t RS410_MM_PID     = 0x0aff; // ASR
    const uint16_t RS400_MM_PID     = 0x0b00; // PSR
    const uint16_t RS430_MM_RGB_PID = 0x0b01; // AWGCT
    const uint16_t RS460_PID        = 0x0b03; // DS5U
    const uint16_t RS435_RGB_PID    = 0x0b07; // AWGC
    const uint16_t RS405_PID        = 0x0b0c; // DS5U

    const bool ALIGN_DEPTH    = false;
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

    const std::string DEFAULT_ALIGNED_DEPTH_TO_COLOR_FRAME_ID = "camera_aligned_depth_to_color_frame";
    const std::string DEFAULT_ALIGNED_DEPTH_TO_INFRA1_FRAME_ID = "camera_aligned_depth_to_infra1_frame";
    const std::string DEFAULT_ALIGNED_DEPTH_TO_INFRA2_FRAME_ID = "camera_aligned_depth_to_infra2_frame";
    const std::string DEFAULT_ALIGNED_DEPTH_TO_FISHEYE_FRAME_ID = "camera_aligned_depth_to_fisheye_frame";

    const std::string DEFAULT_FILTERS                  = "";

    using stream_index_pair = std::pair<rs2_stream, int>;
}  // namespace realsense2_camera
