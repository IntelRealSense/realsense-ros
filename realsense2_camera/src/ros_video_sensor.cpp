#include <ros_sensor.h>
#include <cv_bridge/cv_bridge.h>

using namespace realsense2_camera;
using namespace rs2;

VideoSensor::VideoSensor(rs2::sensor sensor, rclcpp::Node& node,
                         std::function<void(rs2::frame)> frame_callback,
                         std::function<void()> update_sensor_func): 
    RosSensor(sensor, node, frame_callback, update_sensor_func) 
{
    registerProfileParameters();
    _allowed_formats[RS2_STREAM_DEPTH] = RS2_FORMAT_Z16;
    _allowed_formats[RS2_STREAM_INFRARED] = RS2_FORMAT_Y8;
}

void VideoSensor::getUpdatedSensorParameters()
{
    const std::string module_name(create_graph_resource_name(get_info(RS2_CAMERA_INFO_NAME)));

    std::string param_name(module_name + ".width");
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _width = _params.getParameters().setParam(param_name, rclcpp::ParameterValue(IMAGE_WIDTH), [this](const rclcpp::Parameter& )
            {
                ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
            }).get<rclcpp::PARAMETER_INTEGER>();
    param_name = module_name + ".height";
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _height = _params.getParameters().setParam(param_name, rclcpp::ParameterValue(IMAGE_HEIGHT), [this](const rclcpp::Parameter& )
            {
                ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
            }).get<rclcpp::PARAMETER_INTEGER>();
    param_name = module_name + ".fps";
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _fps = _params.getParameters().setParam(param_name, rclcpp::ParameterValue(IMAGE_FPS), [this](const rclcpp::Parameter& )
            {
                ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
            }).get<rclcpp::PARAMETER_DOUBLE>();
    ROS_DEBUG_STREAM(param_name << " = " << _fps);

}

void VideoSensor::getUpdatedProfileParameters(const rs2::stream_profile& profile)
{
    stream_index_pair stream(profile.stream_type(), profile.stream_index());
    std::string param_name = "enable_" + create_graph_resource_name(STREAM_NAME(stream));
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _enabled_profiles[stream] = (_node.has_parameter(param_name) ? _node.get_parameter(param_name).get_parameter_value().get<rclcpp::PARAMETER_BOOL>() : true);
    ROS_DEBUG_STREAM(param_name << "=" << _enabled_profiles[stream]);
}

bool VideoSensor::isWantedProfile(const rs2::stream_profile& profile)
{
    auto video_profile = profile.as<rs2::video_stream_profile>();
    ROS_DEBUG_STREAM("Sensor profile: " <<
                        "stream_type: " << rs2_stream_to_string(video_profile.stream_type()) << "(" << video_profile.stream_index() << ")" <<
                        "Format: " << video_profile.format() <<
                        ", Width: " << video_profile.width() <<
                        ", Height: " << video_profile.height() <<
                        ", FPS: " << video_profile.fps());

    return ((video_profile.width() == _width) &&
            (video_profile.height() == _height) &&
            (video_profile.fps() == _fps) &&
            (_allowed_formats.find(video_profile.stream_type()) == _allowed_formats.end() || video_profile.format() == _allowed_formats[video_profile.stream_type()] ));
}

void VideoSensor::registerProfileParameters()
{
    registerSensorUpdateParam("enable_%s", true);
}
