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

#include <sstream>
#include <string>
#include <frame_latency/frame_latency.h>
#include <constants.h>
// Node which receives sensor_msgs/Image messages and prints the image latency.

using namespace rs2_ros::tools::frame_latency;

FrameLatencyNode::FrameLatencyNode( const std::string & node_name,
                                    const std::string & ns,
                                    const rclcpp::NodeOptions & node_options )
    : Node( node_name, ns, node_options )
    , _logger( this->get_logger() )
{
}


FrameLatencyNode::FrameLatencyNode( const rclcpp::NodeOptions & node_options )
    : Node( "frame_latency", "/", node_options )
    , _logger( this->get_logger() )
{
    ROS_INFO_STREAM( "frame_latency node is UP!" );
    ROS_INFO_STREAM( "Intra-Process is "
                     << ( this->get_node_options().use_intra_process_comms() ? "ON" : "OFF" ) );
    // Create a subscription on the input topic.
    _sub = this->create_subscription< sensor_msgs::msg::Image >(
        "/color/image_raw",  // TODO Currently color only, we can declare and accept the required
                             // streams as ros parameters
        rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( rmw_qos_profile_default ),
                     rmw_qos_profile_default ),
        [&, this]( const sensor_msgs::msg::Image::SharedPtr msg ) {
            rclcpp::Time curr_time = this->get_clock()->now();
            auto latency = ( curr_time - msg->header.stamp ).seconds();
            ROS_INFO_STREAM( "Got msg with address 0x"
                             << std::hex << reinterpret_cast< std::uintptr_t >( msg.get() )
                             << std::dec << " with latency of " << latency << " [sec]" );
        } );
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( rs2_ros::tools::frame_latency::FrameLatencyNode )
