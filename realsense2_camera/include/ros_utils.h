// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once


#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>


namespace realsense2_camera
{
    typedef std::pair<rs2_stream, int> stream_index_pair;

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

    bool isValidCharInName(char c);

    std::string rs2_to_ros(std::string rs2_name);
    std::string ros_stream_to_string(rs2_stream stream);
    std::string create_graph_resource_name(const std::string &original_name);
    const rmw_qos_profile_t qos_string_to_qos(std::string str);
    const std::string list_available_qos_strings();

    rs2_stream rs2_string_to_stream(std::string str);

    stream_index_pair rs2_string_to_sip(const std::string& str);
}

