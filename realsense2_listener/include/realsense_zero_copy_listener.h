// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"

namespace realsense2_listener
{
    class ImageViewNode : public rclcpp::Node
    {
    public:
        explicit ImageViewNode(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions().use_intra_process_comms(true));

        ImageViewNode(const std::string &node_name, const std::string & ns, const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions().use_intra_process_comms(true));

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
        // cv::VideoCapture cap_;
        // cv::Mat frame_;
        rclcpp::Logger _logger;
    };
}
