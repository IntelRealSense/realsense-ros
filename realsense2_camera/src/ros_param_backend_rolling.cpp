// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#include "ros_param_backend.h"

namespace realsense2_camera
{
    void ParametersBackend::add_on_set_parameters_callback(ros2_param_callback_type callback)
    {
        _ros_callback = _node.add_on_set_parameters_callback(callback);
    }

    ParametersBackend::~ParametersBackend()
    {
        if (_ros_callback)
        {
            _node.remove_on_set_parameters_callback((rclcpp::node_interfaces::OnSetParametersCallbackHandle*)(_ros_callback.get()));
            _ros_callback.reset();
        }
    }
}
