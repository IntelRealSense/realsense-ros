// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include <string>

#define REALSENSE_ROS_MAJOR_VERSION    3
#define REALSENSE_ROS_MINOR_VERSION    2
#define REALSENSE_ROS_PATCH_VERSION    3

#define STRINGIFY(arg) #arg
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
/* Return version in "X.Y.Z" format */
#define REALSENSE_ROS_VERSION_STR (VAR_ARG_STRING(REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))

#define ROS_DEBUG(...) RCLCPP_DEBUG(_logger, __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(_logger, __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(_logger, __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(_logger, __VA_ARGS__)

#ifdef DASHING
// Based on: https://docs.ros2.org/dashing/api/rclcpp/logging_8hpp.html
#define MSG(msg) (static_cast<std::ostringstream&&>(std::ostringstream() << msg)).str()
#define ROS_DEBUG_STREAM(msg) RCLCPP_DEBUG(_logger, MSG(msg))
#define ROS_INFO_STREAM(msg) RCLCPP_INFO(_logger, MSG(msg))
#define ROS_WARN_STREAM(msg) RCLCPP_WARN(_logger, MSG(msg))
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR(_logger, MSG(msg))
#define ROS_FATAL_STREAM(msg) RCLCPP_FATAL(_logger, MSG(msg))
#define ROS_DEBUG_STREAM_ONCE(msg) RCLCPP_DEBUG_ONCE(_logger, MSG(msg))
#define ROS_INFO_STREAM_ONCE(msg) RCLCPP_INFO_ONCE(_logger, MSG(msg))
#define ROS_WARN_STREAM_COND(cond, msg) RCLCPP_WARN_EXPRESSION(_logger, cond, MSG(msg))
#else
// Based on: https://docs.ros2.org/foxy/api/rclcpp/logging_8hpp.html
#define ROS_DEBUG_STREAM(msg) RCLCPP_DEBUG_STREAM(_logger, msg)
#define ROS_INFO_STREAM(msg) RCLCPP_INFO_STREAM(_logger, msg)
#define ROS_WARN_STREAM(msg) RCLCPP_WARN_STREAM(_logger, msg)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(_logger, msg)
#define ROS_FATAL_STREAM(msg) RCLCPP_FATAL_STREAM(_logger, msg)
#define ROS_DEBUG_STREAM_ONCE(msg) RCLCPP_DEBUG_STREAM_ONCE(_logger, msg)
#define ROS_INFO_STREAM_ONCE(msg) RCLCPP_INFO_STREAM_ONCE(_logger, msg)
#define ROS_WARN_STREAM_COND(cond, msg) RCLCPP_WARN_STREAM_EXPRESSION(_logger, cond, msg)
#endif

#define ROS_WARN_ONCE(msg) RCLCPP_WARN_ONCE(_logger, msg)

namespace realsense2_camera
{
    const uint16_t SR300_PID        = 0x0aa5; // SR300
    const uint16_t SR300v2_PID      = 0x0B48; // SR305
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
    const uint16_t RS435i_RGB_PID   = 0x0B3A; // AWGC_MM
    const uint16_t RS465_PID        = 0x0b4d; // D465
    const uint16_t RS416_RGB_PID    = 0x0B52; // F416 RGB
    const uint16_t RS405_PID        = 0x0B5B; // DS5U
    const uint16_t RS455_PID        = 0x0B5C; // D455
    const uint16_t RS_T265_PID      = 0x0b37; // 
    const uint16_t RS_L515_PID_PRE_PRQ = 0x0B3D; // 
    const uint16_t RS_L515_PID      = 0x0B64; // 
    const uint16_t RS_L535_PID      = 0x0b68;
    

    const bool ALIGN_DEPTH             = false;
    const bool POINTCLOUD              = false;
    const bool ALLOW_NO_TEXTURE_POINTS = false;
    const bool SYNC_FRAMES             = false;
    const bool ORDERED_POINTCLOUD      = false;

    const bool PUBLISH_TF        = true;
    const double TF_PUBLISH_RATE = 0; // Static transform
    const double DIAGNOSTICS_PERIOD = 0; // Static transform

    const int IMAGE_WIDTH             = 640;
    const int IMAGE_HEIGHT            = 480;
    const double IMAGE_FPS            = 30;

    const std::string IMAGE_QOS       = "SYSTEM_DEFAULT";
    const std::string DEFAULT_QOS     = "DEFAULT";
    const std::string HID_QOS         = "HID_DEFAULT";
    const std::string EXTRINSICS_QOS  = "EXTRINSICS_DEFAULT";

    const double IMU_FPS      = 0;


    const bool ENABLE_DEPTH   = true;
    const bool ENABLE_INFRA1  = true;
    const bool ENABLE_INFRA2  = true;
    const bool ENABLE_COLOR   = true;
    const bool ENABLE_FISHEYE = true;
    const bool ENABLE_IMU     = true;
    const bool HOLD_BACK_IMU_FOR_FRAMES = false;
    const bool PUBLISH_ODOM_TF = true;


    const std::string DEFAULT_BASE_FRAME_ID            = "camera_link";
    const std::string DEFAULT_ODOM_FRAME_ID            = "odom_frame";
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

    const std::string DEFAULT_UNITE_IMU_METHOD         = "";
    const std::string DEFAULT_FILTERS                  = "";
    const std::string DEFAULT_TOPIC_ODOM_IN            = "";

    const float ROS_DEPTH_SCALE = 0.001;
}  // namespace realsense2_camera
