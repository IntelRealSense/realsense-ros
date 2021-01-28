#pragma once
#include <ros_utils.h>
#include "constants.h"

namespace realsense2_camera
{
    class Parameters
    {
        public:
            Parameters(rclcpp::Node& node);
            rclcpp::ParameterValue setParam(std::string param_name, rclcpp::ParameterValue initial_value, 
                                            std::function<void(const rclcpp::Parameter&)> func,
                                            rcl_interfaces::msg::ParameterDescriptor descriptor=rcl_interfaces::msg::ParameterDescriptor());

            template <class T>
            void setParamT(std::string param_name, rclcpp::ParameterValue initial_value, 
                           T& param,
                           rcl_interfaces::msg::ParameterDescriptor descriptor=rcl_interfaces::msg::ParameterDescriptor());
            void removeParam(std::string param_name);

        private:

        private:
            rclcpp::Node& _node;
            rclcpp::Logger _logger;
            std::map<std::string, std::function<void(const rclcpp::Parameter&)> > _param_functions;
            rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _ros_callback;
    };
}
