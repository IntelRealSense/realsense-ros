#include <ros_sensor.h>
#include <cv_bridge/cv_bridge.h>

using namespace realsense2_camera;
using namespace rs2;

VideoSensor::VideoSensor(rs2::sensor sensor, rclcpp::Node& node,
                         std::function<void(rs2::frame)> frame_callback): 
    RosSensor(sensor, node, frame_callback) 
{
    _allowed_formats[RS2_STREAM_INFRARED] = RS2_FORMAT_Y8;
}

void VideoSensor::getUpdatedSensorParameters()
{
    // std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
    const std::string module_name(create_graph_resource_name(get_info(RS2_CAMERA_INFO_NAME)));

    std::string param_name(module_name + ".width");
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _width  = (_node.has_parameter(param_name) ? _node.get_parameter(param_name).get_parameter_value() : _node.declare_parameter(param_name, rclcpp::ParameterValue(IMAGE_WIDTH))).get<rclcpp::PARAMETER_INTEGER>();
    param_name = module_name + ".height";
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _height = (_node.has_parameter(param_name) ? _node.get_parameter(param_name).get_parameter_value() : _node.declare_parameter(param_name, rclcpp::ParameterValue(IMAGE_HEIGHT))).get<rclcpp::PARAMETER_INTEGER>();        
    param_name = module_name + ".fps";
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _fps   = (_node.has_parameter(param_name) ? _node.get_parameter(param_name).get_parameter_value() : _node.declare_parameter(param_name, rclcpp::ParameterValue(IMAGE_FPS))).get<rclcpp::PARAMETER_DOUBLE>();
}

void VideoSensor::getUpdatedProfileParameters(const rs2::stream_profile& profile)
{
    stream_index_pair stream(profile.stream_type(), profile.stream_index());
    std::string param_name = "enable_" + STREAM_NAME(stream);
    ROS_INFO_STREAM("reading parameter:" << param_name);
    _enable[stream] = (_node.has_parameter(param_name) ? _node.get_parameter(param_name).get_parameter_value() : _node.declare_parameter(param_name, rclcpp::ParameterValue(true))).get<rclcpp::PARAMETER_BOOL>();
    ROS_INFO_STREAM(param_name << "=" << _enable[stream]);

    // if (profile.is<rs2::video_stream_profile>())
    // {
    //     std::string param_name = "enable_" + STREAM_NAME(stream);
    //     ROS_DEBUG_STREAM("reading parameter:" << param_name);
    //     _enable[stream] = (_node.has_parameter(param_name) ? _node.get_parameter(param_name).get_parameter_value() : _node.declare_parameter(param_name, rclcpp::ParameterValue(true))).get<rclcpp::PARAMETER_BOOL>();
    // }
    // else if (profile.is<rs2::motion_stream_profile>())
    // {
    //     std::string param_name = "enable_" + STREAM_NAME(stream);
    //     _enable[stream] = (_node.has_parameter(param_name) ? _node.get_parameter(param_name).get_parameter_value() : _node.declare_parameter(param_name, rclcpp::ParameterValue(ENABLE_IMU))).get<rclcpp::PARAMETER_BOOL>();
    // }
    // else
    // {
    //     throw std::runtime_error("Sensor type is not implemented");
    // }
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

// stream_profile VideoSensor::getWantedProfile(const stream_index_pair& sip)
// {
//     for (auto& profile : _sensor.get_stream_profiles())
//     {
//         auto video_profile = profile.as<rs2::video_stream_profile>();
//         ROS_DEBUG_STREAM("Sensor profile: " <<
//                             "stream_type: " << rs2_stream_to_string(video_profile.stream_type()) << "(" << video_profile.stream_index() << ")" <<
//                             "Format: " << video_profile.format() <<
//                             ", Width: " << video_profile.width() <<
//                             ", Height: " << video_profile.height() <<
//                             ", FPS: " << video_profile.fps());

//         if ((video_profile.stream_type() == sip.first) &&
//             (video_profile.stream_index() == sip.second) &&
//             (video_profile.width() == _width) &&
//             (video_profile.height() == _height) &&
//             (video_profile.fps() == _fps) &&
//             (_allowed_formats.find(sip.first) == _allowed_formats.end() || video_profile.format() == _allowed_formats[sip.first] ))
//         {
//             return profile;
//         }
//     }
//     const std::string module_name(_sensor.get_info(RS2_CAMERA_INFO_NAME));
//     ROS_WARN_STREAM("Given stream configuration is not supported by the device! " <<
//                     "stream_type: " << rs2_stream_to_string(sip.first) << "(" << sip.second << ")" <<
//                     "Format: " << (_allowed_formats.find(sip.first) == _allowed_formats.end() ? std::string("None") :  rs2_format_to_string(_allowed_formats[sip.first])) <<
//                     ", Width: " << _width <<
//                     ", Height: " << _height <<
//                     ", FPS: " << _fps);
//     return stream_profile();
// }

// void VideoSensor::start(const std::vector<stream_profile>& profiles)
// {
//     RosSensor::start(profiles);
//     // for (auto& profile : profiles)
//     // {
//     //     updateStreamCalibData(profile.as<rs2::video_stream_profile>());
//     // }
//     if (_sensor.is<rs2::depth_sensor>())
//     {
//         // _depth_scale_meters = _sensor.as<rs2::depth_sensor>().get_depth_scale();
//     }
//     // createStreamPublishers(sensor);
// }

// void VideoSensor::createPublishers()
// {
//     std::vector<stream_profile> profiles = _sensor.get_active_streams();
//     for (auto& profile : profiles)
//     {
//         std::stringstream image_raw, camera_info;
//         bool rectified_image = false;
//         if (sensor.is<rs2::depth_sensor>())
//             rectified_image = true;

//         std::string stream_name(STREAM_NAME(profile.stream_type()));
//         image_raw << stream_name << "/image_" << ((rectified_image)?"rect_":"") << "raw";
//         camera_info << stream_name << "/camera_info";

//         _rs_diagnostic_updater.Add(stream_name, diagnostic_updater::FrequencyStatusParam(&_fps[stream], &_fps[stream]));
        
//         _image_publishers[stream] = {image_transport::create_publisher(&_node, image_raw.str(), rmw_qos_profile_sensor_data), stream_name};
//         _info_publisher[stream] = _node.create_publisher<sensor_msgs::msg::CameraInfo>(camera_info.str(), 1);

//         if (_align_depth && (stream != DEPTH) && stream.second < 2)
//         {
//             std::stringstream aligned_image_raw, aligned_camera_info;
//             aligned_image_raw << "aligned_depth_to_" << stream_name << "/image_raw";
//             aligned_camera_info << "aligned_depth_to_" << stream_name << "/camera_info";

//             std::string aligned_stream_name = "aligned_depth_to_" + stream_name;
//             _rs_diagnostic_updater.Add(aligned_stream_name, diagnostic_updater::FrequencyStatusParam(&_fps[stream], &_fps[stream]));
//             _depth_aligned_image_publishers[stream] = {image_transport::create_publisher(&_node, aligned_image_raw.str(), rmw_qos_profile_sensor_data), aligned_stream_name};
//             _depth_aligned_info_publisher[stream] = _node.create_publisher<sensor_msgs::msg::CameraInfo>(aligned_camera_info.str(), 1);
//         }

//         if (stream == DEPTH && _pointcloud)
//         {
//             _pointcloud_publisher = _node.create_publisher<sensor_msgs::msg::PointCloud2>("depth/color/points", 1);
//         }
//     }
// }