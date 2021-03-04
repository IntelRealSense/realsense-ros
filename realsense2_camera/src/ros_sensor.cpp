#include <ros_sensor.h>

using namespace realsense2_camera;
using namespace rs2;

void RosSensor::setupErrorCallback()
{
    set_notifications_callback([&](const rs2::notification& n)
    {
        std::vector<std::string> error_strings({"RT IC2 Config error",
                                                "Left IC2 Config error"});
        if (n.get_severity() >= RS2_LOG_SEVERITY_ERROR)
        {
            ROS_WARN_STREAM("Hardware Notification:" << n.get_description() << "," << n.get_timestamp() << "," << n.get_severity() << "," << n.get_category());
        }
        if (error_strings.end() != find_if(error_strings.begin(), error_strings.end(), [&n] (std::string err) 
                                    {return (n.get_description().find(err) != std::string::npos); }))
        {
            ROS_ERROR_STREAM("Performing Hardware Reset.");
            // _sensor.get().get()->parent.hardware_reset();
            // rs2_error* e = nullptr;

            // rs2_create_device_from_sensor(_sensor.get().get(), &e);
            // rs2::get_sensor_parent(_sensor).hardware_reset();
        }
    });
}

RosSensor::RosSensor(rs2::sensor sensor,
    std::shared_ptr<Parameters> parameters, 
    std::function<void(rs2::frame)> frame_callback,
    std::function<void()> update_sensor_func): 
    rs2::sensor(sensor),
    _logger(rclcpp::get_logger("RealSenseCameraNode")),
    _origin_frame_callback(frame_callback),
    _params(parameters, _logger),
    _update_sensor_func(update_sensor_func)
{
    _frame_callback = [this](rs2::frame frame)
        {
            runFirstFrameInitialization();
            _origin_frame_callback(frame);
        };
    setParameters();
}

void RosSensor::setParameters()
{
    std::string module_name = create_graph_resource_name(get_info(RS2_CAMERA_INFO_NAME));
    _params.registerDynamicOptions(*this, module_name);
}

void RosSensor::runFirstFrameInitialization()
{
    if (_is_first_frame)
    {
        ROS_DEBUG_STREAM("runFirstFrameInitialization: " << _first_frame_functions_stack.size());
        _is_first_frame = false;
        if (!_first_frame_functions_stack.empty())
        {
            std::thread t = std::thread([=]()
            {
                while (!_first_frame_functions_stack.empty())
                {
                    _first_frame_functions_stack.back()();
                    _first_frame_functions_stack.pop_back();
                }
            });
            t.detach();
        }
    }
}

bool RosSensor::start(const std::vector<stream_profile>& profiles)
{
    if (get_active_streams().size() > 0)
        return false;
    setupErrorCallback();
    open(profiles);

    for (auto& profile : profiles)
    if (profile.is<rs2::video_stream_profile>())
    {
        auto video_profile = profile.as<rs2::video_stream_profile>();
        ROS_INFO_STREAM("Open profile: " <<
                            "stream_type: " << rs2_stream_to_string(video_profile.stream_type()) << "(" << video_profile.stream_index() << ")" <<
                            "Format: " << video_profile.format() <<
                            ", Width: " << video_profile.width() <<
                            ", Height: " << video_profile.height() <<
                            ", FPS: " << video_profile.fps());
    }
    else
    {
        ROS_INFO_STREAM("Open profile: " <<
                            "stream_type: " << rs2_stream_to_string(profile.stream_type()) << "(" << profile.stream_index() << ")" <<
                            "Format: " << profile.format() <<
                            ", FPS: " << profile.fps());
    }

    rs2::sensor::start(_frame_callback);
    return true;
}

void RosSensor::stop()
{
    if (get_active_streams().size() == 0)
        return;
    ROS_INFO_STREAM("Stop Sensor: " << get_info(RS2_CAMERA_INFO_NAME));
    rs2::sensor::stop();
    close();
}

bool profiles_equal(const rs2::stream_profile& a, const rs2::stream_profile& b)
{
    if (a.is<rs2::video_stream_profile>() && b.is<rs2::video_stream_profile>())
    {
        auto va = a.as<rs2::video_stream_profile>();
        auto vb = b.as<rs2::video_stream_profile>();
        return (va == vb && va.width() == vb.width() && va.height() == vb.height());
    }
    return (a == b);
}

bool is_profiles_in_profiles(const std::vector<stream_profile>& sub_profiles, const std::vector<stream_profile>& all_profiles)
{
    for (auto& a : sub_profiles)
    {
        bool found_profile(false);
        for (auto& b : all_profiles)
        {
            if (profiles_equal(a, b))
            {
                found_profile = true;
                break;
            }
        }
        if (!found_profile)
        {
            return false;
        }
    }
    return true;
}

bool compare_profiles_lists(const std::vector<stream_profile>& active_profiles, const std::vector<stream_profile>& wanted_profiles)
{
    return (is_profiles_in_profiles(active_profiles, wanted_profiles) && is_profiles_in_profiles(wanted_profiles, active_profiles));
}

bool RosSensor::getUpdatedProfiles(std::vector<stream_profile>& wanted_profiles)
{
    wanted_profiles.clear();
    std::vector<stream_profile> active_profiles = get_active_streams();
    std::set<stream_index_pair> found_sips;
    for (auto& profile : get_stream_profiles())
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (!_enabled_profiles[sip])
            continue;
        if (found_sips.find(sip) == found_sips.end() && isWantedProfile(profile))
        {
            wanted_profiles.push_back(profile);
            found_sips.insert(sip);
            ROS_DEBUG_STREAM("Found profile for " << rs2_stream_to_string(sip.first) << ":" << sip.second);
        }
    }

    ROS_DEBUG_STREAM(get_info(RS2_CAMERA_INFO_NAME) << ":" << "active_profiles.size() = " << active_profiles.size());
    for (auto& profile : active_profiles)
    {
        auto video_profile = profile.as<rs2::video_stream_profile>();
        ROS_DEBUG_STREAM("Sensor profile: " <<
                            "stream_type: " << rs2_stream_to_string(video_profile.stream_type()) << "(" << video_profile.stream_index() << ")" <<
                            "Format: " << video_profile.format() <<
                            ", Width: " << video_profile.width() <<
                            ", Height: " << video_profile.height() <<
                            ", unique_id: " << video_profile.unique_id() <<
                            ", FPS: " << video_profile.fps());
    }

    ROS_DEBUG_STREAM(get_info(RS2_CAMERA_INFO_NAME) << ":" << "wanted_profiles");
    for (auto& profile : wanted_profiles)
    {
        auto video_profile = profile.as<rs2::video_stream_profile>();
        ROS_DEBUG_STREAM("Sensor profile: " <<
                            "stream_type: " << rs2_stream_to_string(video_profile.stream_type()) << "(" << video_profile.stream_index() << ")" <<
                            "Format: " << video_profile.format() <<
                            ", Width: " << video_profile.width() <<
                            ", Height: " << video_profile.height() <<
                            ", unique_id: " << video_profile.unique_id() <<
                            ", FPS: " << video_profile.fps());
    }
    if (compare_profiles_lists(active_profiles, wanted_profiles))
    {
        return false;
    }
    return true;
}

template<class T>
void RosSensor::registerSensorUpdateParam(std::string template_name, std::map<stream_index_pair, T>& params, T value)
{
    // For each pair of stream-index, Function add a parameter to <params> and advertise it by <template_name>.
    // parameters in <params> are dynamically being updated.
    std::set<stream_index_pair> checked_sips, found_sips;
    for (auto& profile : get_stream_profiles())
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (checked_sips.insert(sip).second == false)
            continue;
        const std::string stream_name(create_graph_resource_name(STREAM_NAME(sip)));
        char* param_name = new char[template_name.size() + stream_name.size()];
        sprintf(param_name, template_name.c_str(), stream_name.c_str());

        _params.getParameters()->setParamT(std::string(param_name), rclcpp::ParameterValue(value), params[sip], [this](const rclcpp::Parameter& parameter)
                {
                    ROS_DEBUG_STREAM("callback for: " << parameter.get_name());
                    _update_sensor_func();                    
                });
    }
}
template void RosSensor::registerSensorUpdateParam<bool>(std::string template_name, std::map<stream_index_pair, bool>& params, bool value);
template void RosSensor::registerSensorUpdateParam<double>(std::string template_name, std::map<stream_index_pair, double>& params, double value);
