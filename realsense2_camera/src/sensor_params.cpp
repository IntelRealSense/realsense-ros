// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#include <sensor_params.h>
#include <constants.h>
#include <iomanip>

namespace realsense2_camera
{
bool is_checkbox(rs2::options sensor, rs2_option option)
{
    rs2::option_range op_range = sensor.get_option_range(option);
    return op_range.max == 1.0f &&
        op_range.min == 0.0f &&
        op_range.step == 1.0f;
}

bool is_enum_option(rs2::options sensor, rs2_option option)
{
    static const int MAX_ENUM_OPTION_VALUES(100);
    static const float EPSILON(0.05);
    
    rs2::option_range op_range = sensor.get_option_range(option);
    if (abs((op_range.step - 1)) > EPSILON || (op_range.max > MAX_ENUM_OPTION_VALUES)) return false;
    for (auto i = op_range.min; i <= op_range.max; i += op_range.step)
    {
        if (sensor.get_option_value_description(option, i) == nullptr)
            continue;
        return true;
    }
    return false;
}

bool is_int_option(rs2::options sensor, rs2_option option)
{
    rs2::option_range op_range = sensor.get_option_range(option);
    return (op_range.step == 1.0);
}

std::map<std::string, int> get_enum_method(rs2::options sensor, rs2_option option)
{
    std::map<std::string, int> dict; // An enum to set size
    if (is_enum_option(sensor, option))
    {
        rs2::option_range op_range = sensor.get_option_range(option);
        const auto op_range_min = int(op_range.min);
        const auto op_range_max = int(op_range.max);
        const auto op_range_step = int(op_range.step);
        for (auto val = op_range_min; val <= op_range_max; val += op_range_step)
        {
            if (sensor.get_option_value_description(option, val) == nullptr)
                continue;
            dict[sensor.get_option_value_description(option, val)] = val;
        }
    }
    return dict;
}

template<class T>
void param_set_option(rs2::options sensor, rs2_option option, const rclcpp::Parameter& parameter)
{ 
    try
    {
        sensor.set_option(option, parameter.get_value<T>());
    }
    catch(const std::exception& e)
    {
        std::cout << "Failed to set value: " << e.what() << std::endl;
    }
}

void SensorParams::clearParameters()
{
    while ( !_parameters_names.empty() )
    {
        auto name = _parameters_names.back();
        _parameters->removeParam(name);
        _parameters_names.pop_back();        
    }
}

SensorParams::~SensorParams()
{
    clearParameters();
}

template<class T>
void SensorParams::set_parameter(rs2::options sensor, rs2_option option, const std::string& module_name, const std::string& description_addition)
{
    const std::string option_name(module_name + "." + create_graph_resource_name(rs2_option_to_string(option)));
    T option_value;
    try
    {
        option_value = static_cast<T>(sensor.get_option(option));
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An error has occurred while calling sensor for: " << option_name << ":" << ex.what());
        return;
    }
    rs2::option_range op_range = sensor.get_option_range(option);
    rcl_interfaces::msg::ParameterDescriptor crnt_descriptor;
    std::stringstream desc;
    desc << sensor.get_option_description(option) << std::endl << description_addition;
    crnt_descriptor.description = desc.str();
    if (std::is_same<T, int>::value || std::is_same<T, bool>::value)
    {
        rcl_interfaces::msg::IntegerRange range;
        range.from_value = int(op_range.min);
        range.to_value = int(op_range.max);
        crnt_descriptor.integer_range.push_back(range);
        if (std::is_same<T, bool>::value)
            ROS_DEBUG_STREAM("Declare: BOOL::" << option_name << " = " << option_value << "[" << op_range.min << ", " << op_range.max << "]");
        else
            ROS_DEBUG_STREAM("Declare: INT::" << option_name << " = " << option_value << "[" << op_range.min << ", " << op_range.max << "]");
    }
    else
    {
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = double(op_range.min);
        range.to_value = double(op_range.max);
        crnt_descriptor.floating_point_range.push_back(range);
        ROS_DEBUG_STREAM("Declare: DOUBLE::" << option_name << " = " << option_value);
    }

    T new_val;
    try
    {
        new_val = (_parameters->setParam<T>(option_name, option_value, [option, sensor](const rclcpp::Parameter& parameter)
                    {
                        param_set_option<T>(sensor, option, parameter);
                    }, crnt_descriptor));
        _parameters_names.push_back(option_name);
    }
    catch(const rclcpp::exceptions::InvalidParameterValueException& e)
    {
        ROS_WARN_STREAM("Failed to set parameter:" << option_name << " = " << option_value << "[" << op_range.min << ", " << op_range.max << "]\n" << e.what());
        return;
    }
    
    if (new_val != option_value)
    {
        try
        {
            sensor.set_option(option, new_val);
        }
        catch(std::exception& e)
        {
            ROS_WARN_STREAM("Failed to set value to sensor: " << option_name << " = " << option_value << "[" << op_range.min << ", " << op_range.max << "]\n" << e.what());            
        }
    }
}

void SensorParams::registerDynamicOptions(rs2::options sensor, const std::string& module_name)
{
    for (auto i = 0; i < RS2_OPTION_COUNT; i++)
    {
        rs2_option option = static_cast<rs2_option>(i);
        const std::string option_name(module_name + "." + create_graph_resource_name(rs2_option_to_string(option)));
        if (!sensor.supports(option) || sensor.is_option_read_only(option))
        {
            continue;
        }
        if (is_checkbox(sensor, option))
        {
            set_parameter<bool>(sensor, option, module_name);
            continue;
        }
        const auto enum_dict = get_enum_method(sensor, option);
        if (enum_dict.empty())
        {
            if (is_int_option(sensor, option))
            {
                set_parameter<int>(sensor, option, module_name);
            }
            else
            {
                if (i == RS2_OPTION_DEPTH_UNITS)
                {
                    rs2::option_range op_range = sensor.get_option_range(option);
                    if (ROS_DEPTH_SCALE >= op_range.min && ROS_DEPTH_SCALE <= op_range.max)
                    {
                        try
                        {
                            sensor.set_option(option, ROS_DEPTH_SCALE);
                        }
                        catch(const std::exception& e)
                        {
                            std::cout << "Failed to set value: " << e.what() << std::endl;
                        }
                        op_range.min = ROS_DEPTH_SCALE;
                        op_range.max = ROS_DEPTH_SCALE;
                    }
                }
                else
                {
                    set_parameter<double>(sensor, option, module_name);
                }
            }
        }
        else
        {
            std::vector<std::pair<std::string, int> > enum_vec;
            size_t longest_desc(0);
            for (auto enum_iter : enum_dict)
            {
                enum_vec.push_back(std::make_pair(enum_iter.first, enum_iter.second));
                longest_desc = std::max(longest_desc, enum_iter.first.size());
            }
            sort(enum_vec.begin(), enum_vec.end(), [](std::pair<std::string, int> e1, std::pair<std::string, int> e2){return (e1.second < e2.second);});
            std::stringstream description;
            for (auto vec_iter : enum_vec)
            {
                description << std::setw(longest_desc+6) << std::left << vec_iter.first << " : " << vec_iter.second << std::endl;
            }
            set_parameter<int>(sensor, option, module_name, description.str());
        }
    }
}
}
