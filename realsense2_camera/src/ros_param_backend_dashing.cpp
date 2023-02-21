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

#include "ros_param_backend.h"

namespace realsense2_camera
{
    void ParametersBackend::add_on_set_parameters_callback(ros2_param_callback_type callback)
    {
        rclcpp::Node::OnParametersSetCallbackType prev_callback = _node.set_on_parameters_set_callback(callback);
        if (prev_callback)
        {
            rclcpp::Node::OnParametersSetCallbackType prev_callback = _node.set_on_parameters_set_callback(prev_callback);
            std::stringstream msg;
            msg << "Cannot set another callback to current node: " << _node.get_name();
            throw std::runtime_error(msg.str());
        }
    }

    ParametersBackend::~ParametersBackend()
    {
    }
}
