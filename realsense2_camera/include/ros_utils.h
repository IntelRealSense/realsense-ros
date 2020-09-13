#pragma once
#ifndef ___REALSENSE_ROS_UTILS_HEADER___
#define ___REALSENSE_ROS_UTILS_HEADER___



#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>


namespace realsense2_camera
{

    // std::string create_graph_resource_name(const std::string &original_name);

    /**
     * Same as ros::names::isValidCharInName, but re-implemented here because it's not exposed.
     */
    bool isValidCharInName(char c)
    {
        return std::isalnum(c) || c == '/' || c == '_';
    }

    /**
     * ROS Graph Resource names don't allow spaces and hyphens (see http://wiki.ros.org/Names),
     * so we replace them here with underscores.
     */

    std::string create_graph_resource_name(const std::string &original_name)
    {
        std::string fixed_name = original_name;
        std::transform(fixed_name.begin(), fixed_name.end(), fixed_name.begin(),
                        [](unsigned char c) { return std::tolower(c); });
        std::replace_if(fixed_name.begin(), fixed_name.end(), [](const char c) { return !(realsense2_camera::isValidCharInName(c)); },
                        '_');
        return fixed_name;
    }

    typedef std::pair<rs2_stream, int> stream_index_pair;

    const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
    const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
    const stream_index_pair INFRA0{RS2_STREAM_INFRARED, 0};
    const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
    const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
    const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
    const stream_index_pair FISHEYE1{RS2_STREAM_FISHEYE, 1};
    const stream_index_pair FISHEYE2{RS2_STREAM_FISHEYE, 2};
    const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
    const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};
    const stream_index_pair POSE{RS2_STREAM_POSE, 0};

    rs2_stream rs2_string_to_stream(std::string str);

    stream_index_pair rs2_string_to_sip(const std::string& str);
}
// template<class T>
// rcl_interfaces::msg::SetParametersResult set_param(T* variable, std::string option_name, const std::vector<rclcpp::Parameter> & parameters);
// { 
//     rcl_interfaces::msg::SetParametersResult result;
//     result.successful = true;
//     for (const auto & parameter : parameters) {
//         if (option_name == parameter.get_name())
//         {
//             // std::cout << "set_option: " << option_name << " = " << parameter.get_value<T>() << std::endl;
//             variable = parameter.get_value<T>();
//         }
//     }
//     return result;
// }
#endif //___REALSENSE_ROS_UTILS_HEADER___
