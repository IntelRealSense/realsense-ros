#pragma once
#include "constants.h"
#include "ros_param_backend.h"

namespace realsense2_camera
{
    class Parameters
    {
        public:
            Parameters(rclcpp::Node& node);
            ~Parameters();
            rclcpp::ParameterValue setParam(std::string param_name, rclcpp::ParameterValue initial_value, 
                                            std::function<void(const rclcpp::Parameter&)> func = std::function<void(const rclcpp::Parameter&)>(),
                                            rcl_interfaces::msg::ParameterDescriptor descriptor=rcl_interfaces::msg::ParameterDescriptor());

            template <class T>
            void setParamT(std::string param_name, rclcpp::ParameterValue initial_value, 
                           T& param,
                           std::function<void(const rclcpp::Parameter&)> func = std::function<void(const rclcpp::Parameter&)>(),
                           rcl_interfaces::msg::ParameterDescriptor descriptor=rcl_interfaces::msg::ParameterDescriptor());
            template <class T>
            void setParamValue(T& param, const T& value); // function updates the parameter value both locally and in the parameters server
            void removeParam(std::string param_name);

        private:

        private:
            rclcpp::Node& _node;
            rclcpp::Logger _logger;
            std::map<std::string, std::vector<std::function<void(const rclcpp::Parameter&)> > > _param_functions;
            std::map<void*, std::string> _param_names;
            ParametersBackend _params_backend;
    };
}
