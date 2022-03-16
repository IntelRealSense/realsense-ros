/* License: Apache 2.0. See LICENSE file in root directory. */
/* Copyright(c) 2022 Intel Corporation. All Rights Reserved. */
#pragma once

#include <rclcpp/rclcpp.hpp>

namespace realsense2_camera
{
    class ParametersBackend
    {
        public:
            ParametersBackend(rclcpp::Node& node) : 
                _node(node),
                _logger(node.get_logger())
                {};
            ~ParametersBackend();
            void add_on_set_parameters_callback(rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType callback);


        private:
            rclcpp::Node& _node;
            rclcpp::Logger _logger;
            std::shared_ptr<void> _ros_callback;
    };
}
