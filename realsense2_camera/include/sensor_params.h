#pragma once
#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <dynamic_params.h>

namespace realsense2_camera
{
    class SensorParams
    {
        public:
            SensorParams(rclcpp::Node& node, rclcpp::Logger logger):
                _logger(logger),
                _node(node),
                _parameters(_node) {};
            void registerDynamicOptions(rs2::options sensor, const std::string& module_name);
            Parameters& getParameters() {return _parameters;};

        public:
            rclcpp::Logger _logger;

        private:
            template<class T>
            void set_parameter(rs2::options sensor, rs2_option option, const std::string& module_name, const std::string& description_addition="");

        private:
            rclcpp::Node& _node;
            Parameters _parameters;

    };
}