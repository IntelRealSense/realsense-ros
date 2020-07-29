// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once
#ifndef ___REALSENSE_NODE_FACTORY_HEADER___
#define ___REALSENSE_NODE_FACTORY_HEADER___

#define ROS_DEBUG(...) RCLCPP_DEBUG(_logger, __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(_logger, __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(_logger, __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(_logger, __VA_ARGS__)
#define ROS_DEBUG_STREAM(msg) RCLCPP_DEBUG_STREAM(_logger, msg)
#define ROS_INFO_STREAM(msg) RCLCPP_INFO_STREAM(_logger, msg)
#define ROS_WARN_STREAM(msg) RCLCPP_WARN_STREAM(_logger, msg)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(_logger, msg)
#define ROS_FATAL_STREAM(msg) RCLCPP_FATAL_STREAM(_logger, msg)

#define ROS_WARN_COND(cond, ...) RCLCPP_WARN_EXPRESSION(_logger, cond, __VA_ARGS__)
#define ROS_WARN_STREAM_COND(cond, msg) RCLCPP_WARN_STREAM_EXPRESSION(_logger, cond, msg)


// cpplint: c system headers
#include <eigen3/Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <console_bridge/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
// cpplint: c++ system headers
#include <algorithm>
#include <csignal>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
// cpplint: other headers
#include "constants.h"
#include "realsense_camera_msgs/msg/imu_info.hpp"
#include "realsense_camera_msgs/msg/extrinsics.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>

// #include <pluginlib/class_list_macros.h>
// #include <nodelet/nodelet.h>
// #include <image_transport/image_transport.h>
// #include <ros/ros.h>
// #include <ros/package.h>
// #include <librealsense2/rs.hpp>
// #include <librealsense2/rsutil.h>
// #include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <constants.h>
// #include <realsense2_camera/Extrinsics.h>
// #include <realsense2_camera/IMUInfo.h>
// #include <csignal>
// #include <eigen3/Eigen/Geometry>
#include <fstream>
#include <thread>

namespace realsense2_camera
{
    const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
    const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
    const stream_index_pair INFRA0{RS2_STREAM_INFRARED, 0};
    const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
    const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
    const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
    const stream_index_pair FISHEYE1{RS2_STREAM_FISHEYE, 1};
    const stream_index_pair FISHEYE2{RS2_STREAM_FISHEYE, 2};
    const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
    const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};
    const stream_index_pair POSE{RS2_STREAM_POSE, 0};
    

    const std::vector<stream_index_pair> IMAGE_STREAMS = {DEPTH, INFRA0, INFRA1, INFRA2,
                                                          COLOR,
                                                          FISHEYE,
                                                          FISHEYE1, FISHEYE2};

    const std::vector<stream_index_pair> HID_STREAMS = {GYRO, ACCEL, POSE};

    class InterfaceRealSenseNode
    {
    public:
        // virtual void publishTopics() = 0;
        // virtual void registerDynamicReconfigCb(ros::NodeHandle& nh) = 0;
        virtual ~InterfaceRealSenseNode() = default;
    };

    class RealSenseNodeFactory
    {
    public:
        RealSenseNodeFactory(rclcpp::Node::SharedPtr node);
        void onInit();
        virtual ~RealSenseNodeFactory();

    private:
        void closeDevice();
        void StartDevice();
        void change_device_callback(rs2::event_information& info);
        void getDevice(rs2::device_list list);
        void tryGetLogSeverity(rs2_log_severity& severity) const;
        static std::string parse_usb_port(std::string line);

        rclcpp::Node::SharedPtr _node;
        rs2::device _device;
        std::unique_ptr<InterfaceRealSenseNode> _realSenseNode;
        rs2::context _ctx;
        std::string _serial_no;
        std::string _usb_port_id;
        std::string _device_type;
        bool _initial_reset;
        std::thread _query_thread;
        bool _is_alive;
        rclcpp::Logger _logger;
        std::shared_ptr<diagnostic_updater::Updater> _diagnostic_updater;
    };
}//end namespace
#endif //___REALSENSE_NODE_FACTORY_HEADER___
