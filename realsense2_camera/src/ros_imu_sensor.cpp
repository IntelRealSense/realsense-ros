#include <ros_sensor.h>
#include <ros_utils.h>
#include <stdio.h>

using namespace realsense2_camera;
using namespace rs2;

ImuSensor::ImuSensor(rs2::sensor sensor, rclcpp::Node& node,
                     std::function<void(rs2::frame)> frame_callback,
                     std::function<void()> update_sensor_func): 
    RosSensor(sensor, node, frame_callback, update_sensor_func) 
{
}

void ImuSensor::getUpdatedSensorParameters()
{
}

void ImuSensor::getUpdatedProfileParameters(const rs2::stream_profile& profile)
{
    stream_index_pair stream(profile.stream_type(), profile.stream_index());
    const std::string stream_name(create_graph_resource_name(STREAM_NAME(stream)));

    std::string param_name = "enable_" + stream_name;
    ROS_INFO_STREAM("reading parameter:" << param_name);
    _enabled_profiles[stream] = (_node.has_parameter(param_name) ? _node.get_parameter(param_name).get_parameter_value().get<rclcpp::PARAMETER_BOOL>() : ENABLE_IMU);

    ROS_INFO_STREAM(param_name << "=" << _enabled_profiles[stream]);
    param_name = stream_name + "_fps";
    ROS_INFO_STREAM("reading parameter:" << param_name);
    _fps[stream] = (_node.has_parameter(param_name) ? _node.get_parameter(param_name).get_parameter_value().get<rclcpp::PARAMETER_DOUBLE>() : 0);
    ROS_INFO_STREAM(param_name << "=" << _fps[stream]);
}

bool ImuSensor::isWantedProfile(const rs2::stream_profile& profile)
{
    stream_index_pair stream(profile.stream_type(), profile.stream_index());
    return (profile.fps() == _fps[stream]);
}

void ImuSensor::registerSensorParameters()
{}

void ImuSensor::registerProfileParameters()
{
    registerSensorUpdateParam("enable_%s", true);
    registerSensorUpdateParam("%s_fps", 0.0);
}
