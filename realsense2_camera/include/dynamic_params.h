// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once
#include <ros_utils.h>
#include "constants.h"
#include <deque>
#include "ros_param_backend.h"

namespace realsense2_camera
{
    class Parameters
    {
        public:
            Parameters(rclcpp::Node& node);
            ~Parameters();
            template <class T>
            T setParam(std::string param_name, const T& initial_value, 
                                    std::function<void(const rclcpp::Parameter&)> func = std::function<void(const rclcpp::Parameter&)>(), 
                                    rcl_interfaces::msg::ParameterDescriptor descriptor=rcl_interfaces::msg::ParameterDescriptor());

            template <class T>
            T readAndDeleteParam(std::string param_name, const T& initial_value);

            template <class T>
            void setParamT(std::string param_name, T& param,
                           std::function<void(const rclcpp::Parameter&)> func = std::function<void(const rclcpp::Parameter&)>(),
                           rcl_interfaces::msg::ParameterDescriptor descriptor=rcl_interfaces::msg::ParameterDescriptor());
            template <class T>
            void setParamValue(T& param, const T& value); // function updates the parameter value both locally and in the parameters server

            void setRosParamValue(const std::string param_name, void const* const value); // function updates the parameters server
            void removeParam(std::string param_name);
            void pushUpdateFunctions(std::vector<std::function<void()> > funcs);

            template <class T>
            void queueSetRosValue(const std::string& param_name, const T value);
            
        private:
            void monitor_update_functions();

        private:
            rclcpp::Node& _node;
            rclcpp::Logger _logger;
            std::map<std::string, std::function<void(const rclcpp::Parameter&)> > _param_functions;
            std::map<void*, std::string> _param_names;
            ParametersBackend _params_backend;
            std::condition_variable _update_functions_cv;
            bool _is_running;
            std::shared_ptr<std::thread> _update_functions_t;
            std::deque<std::function<void()> > _update_functions_v;
            std::list<std::string> self_set_parameters;
    };
}
