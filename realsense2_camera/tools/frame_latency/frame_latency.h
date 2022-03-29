// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"

namespace rs2_ros {
namespace tools {
namespace frame_latency {
class FrameLatencyNode : public rclcpp::Node
{
public:
    explicit FrameLatencyNode( const rclcpp::NodeOptions & node_options
                               = rclcpp::NodeOptions().use_intra_process_comms( true ) );

    FrameLatencyNode( const std::string & node_name,
                      const std::string & ns,
                      const rclcpp::NodeOptions & node_options
                      = rclcpp::NodeOptions().use_intra_process_comms( true ) );

private:
    rclcpp::Subscription< sensor_msgs::msg::Image >::SharedPtr _sub;
    rclcpp::Logger _logger;
};
}  // namespace frame_latency
}  // namespace tools
}  // namespace rs2_ros
