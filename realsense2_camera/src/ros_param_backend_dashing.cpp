#include "ros_param_backend.h"

namespace realsense2_camera
{
    void ParametersBackend::add_on_set_parameters_callback(rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType callback)
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