// Copyright 2023 Intel Corporation. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
