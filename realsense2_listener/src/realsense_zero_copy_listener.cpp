// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#include <sstream>
#include <string>
#include <realsense_zero_copy_listener.h>
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"
#include "constants.h"
// Node which receives sensor_msgs/Image messages and prints the image latency.

using namespace realsense2_listener;

ImageViewNode::ImageViewNode(const std::string & node_name, const std::string & ns,
										   const rclcpp::NodeOptions & node_options) : 
	Node(node_name, ns, node_options),
	_logger(this->get_logger())
{
}


ImageViewNode::ImageViewNode(const rclcpp::NodeOptions & node_options)
    : Node("ImageViewNode", "/", node_options),
      _logger(this->get_logger())
{
    ROS_INFO_STREAM("ImageViewNode node is UP!");
    std::cout << "ImageViewNode node is UP!" << std::endl;
    // Create a subscription on the input topic.
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/color/image_raw",
        rclcpp::SensorDataQoS(),
        [&, this](const sensor_msgs::msg::Image::SharedPtr msg)
        {
            rclcpp::Time curr_time = this->get_clock()->now();
            auto latency = (curr_time - msg->header.stamp).seconds();
            std::cout << "Got msg with address " << std::hex << reinterpret_cast<std::uintptr_t>(msg.get()) << std::dec << " with latency of " << latency << std::endl;

            char key = cv::waitKey(1); // Look for key presses.
            if (key == 27 /* ESC */ || key == 'q')
            {
                rclcpp::shutdown();
            }
            if (key == ' ')
            { // If <space> then pause until another <space>.
                key = '\0';
                while (key != ' ')
                {
                    key = cv::waitKey(1);
                    if (key == 27 /* ESC */ || key == 'q')
                    {
                        rclcpp::shutdown();
                    }
                    if (!rclcpp::ok())
                    {
                        break;
                    }
                }
            }
        });
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(realsense2_listener::ImageViewNode)