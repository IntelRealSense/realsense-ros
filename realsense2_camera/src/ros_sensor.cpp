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

void RosSensor::setParameters()
{
    std::string module_name = create_graph_resource_name(get_info(RS2_CAMERA_INFO_NAME));
    _params.registerDynamicOptions(*this, module_name);
}

bool RosSensor::start(const std::vector<stream_profile>& profiles)
{
    if (get_active_streams().size() > 0)
        return false;
    setupErrorCallback();
    open(profiles);
    rs2::sensor::start(_frame_callback);
    return true;
}

void RosSensor::stop()
{
    if (get_active_streams().size() == 0)
        return;
    rs2::sensor::stop();
    close();
}

bool compare_profile(stream_profile a, stream_profile b)
{
    return (a.unique_id() < b.unique_id());
    // if ((a.stream_type() < b.stream_type()) ||
    //     (a.stream_index() < b.stream_index()) ||
    //     (a.fps() < b.fps())
    //     return true;
    // if (a.is<rs2::video_stream_profile>) && (b.is<rs2::video_stream_profile>)
    // {
    //     auto va = a.as<rs2::video_stream_profile>();
    //     auto vb = b.as<rs2::video_stream_profile>();
    //     if ((a.format() < b.format()) ||
    //         (a.width() < b.width()) ||
    //         (a.height() < b.height())
    //         return true;
    // }
    // else if (a.is<rs2::motion_stream_profile>) && (b.is<rs2::video_stream_profile>)
    // else
    // {
    //     return true;
    // }
    
    //     (a.stream_index() < b.stream_index()) ||
    //     (a.stream_index() < b.stream_index()) ||
    //     (a.stream_index() < b.stream_index()) ||

    //    ()
    //     return true;
    // if ()
}

bool RosSensor::getUpdatedProfiles(std::vector<stream_profile>& wanted_profiles)
{
    wanted_profiles.clear();
    getUpdatedSensorParameters();   // read user parameters (_width, _height, _fps)
    std::vector<stream_profile> active_profiles = get_active_streams();
    std::set<stream_index_pair> checked_sips, found_sips;
    for (auto& profile : get_stream_profiles())
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (checked_sips.insert(sip).second == true)
            getUpdatedProfileParameters(profile);
        if (!_enabled_profiles[sip])
            continue;
        ROS_DEBUG_STREAM_ONCE("Looking for wanted profile: " << rs2_stream_to_string(sip.first) << ":" << sip.second);
        if (found_sips.find(sip) == found_sips.end() && isWantedProfile(profile))
        {
            wanted_profiles.push_back(profile);
            found_sips.insert(sip);
            ROS_DEBUG_STREAM("Found profile for " << rs2_stream_to_string(sip.first) << ":" << sip.second);
        }
    }

    std::set<stream_profile, bool(*)(stream_profile,stream_profile)> set_active_profiles(active_profiles.begin(), active_profiles.end(), compare_profile);
    ROS_DEBUG_STREAM("set_active_profiles.size() = " << set_active_profiles.size());
    for (auto& profile : set_active_profiles)
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

    std::set<stream_profile, bool(*)(stream_profile,stream_profile)> set_wanted_profiles(wanted_profiles.begin(), wanted_profiles.end(), compare_profile);
    ROS_DEBUG_STREAM("wanted_profiles");
    for (auto& profile : set_wanted_profiles)
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

    if (set_active_profiles == set_wanted_profiles)
    {
        return false;
    }
    return true;
}

template<class T>
void RosSensor::registerSensorUpdateParam(std::string template_name, T value)
{
    std::set<stream_index_pair> checked_sips, found_sips;
    for (auto& profile : get_stream_profiles())
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (checked_sips.insert(sip).second == false)
            continue;
        const std::string stream_name(create_graph_resource_name(STREAM_NAME(sip)));
        char* param_name = new char[template_name.size() + stream_name.size()];
        sprintf(param_name, template_name.c_str(), stream_name.c_str());

        _params.getParameters().setParam(std::string(param_name), rclcpp::ParameterValue(value), [this](const rclcpp::Parameter& parameter)
                {
                    ROS_INFO_STREAM("callback for: " << parameter.get_name() << " with value: ");
                    _update_sensor_func();                    
                });
    }
}
template void RosSensor::registerSensorUpdateParam<bool>(std::string template_name, bool value);
template void RosSensor::registerSensorUpdateParam<double>(std::string template_name, double value);

// void RosSensor::startWaitingUpdates()
// {
//     int time_interval(1000);
//     std::function<void()> func = [this, time_interval](){
//         std::mutex mu;
//         std::unique_lock<std::mutex> lock(mu);
//         while(_is_running) {
//             _cv.wait_for(lock, std::chrono::milliseconds(time_interval), [&]{return !_is_running;});
//             if (_is_running)
//             {
//                 publish_temperature();
//             }
//         }
//     };
//     _monitoring_t = std::make_shared<std::thread>(func);
// }

