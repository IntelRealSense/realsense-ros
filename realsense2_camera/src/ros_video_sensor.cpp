#include <ros_sensor.h>
#include <cv_bridge/cv_bridge.h>

using namespace realsense2_camera;
using namespace rs2;

VideoSensor::VideoSensor(rs2::sensor sensor, std::shared_ptr<Parameters> parameters,
                         std::function<void(rs2::frame)> frame_callback,
                         std::function<void()> update_sensor_func): 
    RosSensor(sensor, parameters, frame_callback, update_sensor_func) 
{
    registerSensorParameters();
    _allowed_formats[RS2_STREAM_DEPTH] = RS2_FORMAT_Z16;
    _allowed_formats[RS2_STREAM_INFRARED] = RS2_FORMAT_Y8;
}

bool VideoSensor::start(const std::vector<stream_profile>& profiles)
{
    if (!RosSensor::start(profiles))
        return false;
    _is_first_frame = true;
    if (this->rs2::sensor::is<rs2::roi_sensor>())
        _first_frame_functions_stack.push_back([this](){set_sensor_auto_exposure_roi();});
    return true;
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

void VideoSensor::registerSensorParameters()
{
    registerSensorUpdateParam("enable_%s", _enabled_profiles, true);

    const std::string module_name(create_graph_resource_name(get_info(RS2_CAMERA_INFO_NAME)));

    std::string param_name(module_name + ".width");
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _params.getParameters()->setParamT(param_name, rclcpp::ParameterValue(IMAGE_WIDTH), _width, [this](const rclcpp::Parameter& )
            {
                ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
            });
    param_name = module_name + ".height";
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _params.getParameters()->setParamT(param_name, rclcpp::ParameterValue(IMAGE_HEIGHT), _height, [this](const rclcpp::Parameter& )
            {
                ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
            });
    param_name = module_name + ".fps";
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _params.getParameters()->setParamT(param_name, rclcpp::ParameterValue(IMAGE_FPS), _fps, [this](const rclcpp::Parameter& )
            {
                ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
            });
    registerAutoExposureROIOptions();
}

void VideoSensor::set_sensor_auto_exposure_roi()
{
    try
    {
        bool update_roi_range(false);
        if (_auto_exposure_roi.max_x > _width)
        {
            _params.getParameters()->setParamValue(_auto_exposure_roi.max_x, _width-1);
            update_roi_range = true;
        }
        if (_auto_exposure_roi.max_y > _height)
        {
            _params.getParameters()->setParamValue(_auto_exposure_roi.max_y, _height-1);
            update_roi_range = true;
        }
        if (update_roi_range)
        {
            registerAutoExposureROIOptions();
        }
        this->as<rs2::roi_sensor>().set_region_of_interest(_auto_exposure_roi);
    }
    catch(const std::runtime_error& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
}

void VideoSensor::registerAutoExposureROIOptions()
{
    std::string module_base_name(get_info(RS2_CAMERA_INFO_NAME));

    if (this->rs2::sensor::is<rs2::roi_sensor>())
    {
        int max_x(_width-1);
        int max_y(_height-1);

        std::string module_name = create_graph_resource_name(module_base_name) +".auto_exposure_roi";
        _auto_exposure_roi = {0, 0, max_x, max_y};

        ROS_DEBUG_STREAM("Publish roi for " << module_name);
        _params.getParameters()->setParamT(module_name + ".min_x", rclcpp::ParameterValue(0),     _auto_exposure_roi.min_x, [this](const rclcpp::Parameter&){set_sensor_auto_exposure_roi();});
        _params.getParameters()->setParamT(module_name + ".max_x", rclcpp::ParameterValue(max_x), _auto_exposure_roi.max_x, [this](const rclcpp::Parameter&){set_sensor_auto_exposure_roi();});
        _params.getParameters()->setParamT(module_name + ".min_y", rclcpp::ParameterValue(0),     _auto_exposure_roi.min_y, [this](const rclcpp::Parameter&){set_sensor_auto_exposure_roi();});
        _params.getParameters()->setParamT(module_name + ".max_y", rclcpp::ParameterValue(max_y), _auto_exposure_roi.max_y, [this](const rclcpp::Parameter&){set_sensor_auto_exposure_roi();});
    }
}
