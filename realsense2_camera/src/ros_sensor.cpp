#include <ros_sensor.h>
#include <boost/algorithm/string.hpp>

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
    getUpdatedSensorParameters();
    std::vector<stream_profile> active_profiles = get_active_streams();
    std::set<stream_index_pair> checked_sips, found_sips;
    for (auto& profile : get_stream_profiles())
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (checked_sips.insert(sip).second == true)
            getUpdatedProfileParameters(profile);
        if (!_enabled_profiles[sip])
            continue;
        ROS_INFO_STREAM_ONCE("Looking for " << rs2_stream_to_string(sip.first) << ":" << sip.second);
        if (found_sips.find(sip) == found_sips.end() && isWantedProfile(profile))
        {
            wanted_profiles.push_back(profile);
            found_sips.insert(sip);
            ROS_INFO_STREAM("Found profile for " << rs2_stream_to_string(sip.first) << ":" << sip.second);
            ROS_INFO_STREAM("wanted_profiles.size() = " << wanted_profiles.size());
        }
    }

    // std::set<stream_profile> set_active_profiles(active_profiles.begin(), active_profiles.end(), [](stream_profile a, stream_profile b){ return (if (a.stream_type()< ) });
    std::set<stream_profile, bool(*)(stream_profile,stream_profile)> set_active_profiles(active_profiles.begin(), active_profiles.end(), compare_profile);
    ROS_WARN("set_active_profiles");
    ROS_WARN_STREAM("set_active_profiles.size() = " << set_active_profiles.size());
    for (auto& profile : set_active_profiles)
    {
        auto video_profile = profile.as<rs2::video_stream_profile>();
        ROS_WARN_STREAM("Sensor profile: " <<
                            "stream_type: " << rs2_stream_to_string(video_profile.stream_type()) << "(" << video_profile.stream_index() << ")" <<
                            "Format: " << video_profile.format() <<
                            ", Width: " << video_profile.width() <<
                            ", Height: " << video_profile.height() <<
                            ", unique_id: " << video_profile.unique_id() <<
                            ", FPS: " << video_profile.fps());
    }

    std::set<stream_profile, bool(*)(stream_profile,stream_profile)> set_wanted_profiles(wanted_profiles.begin(), wanted_profiles.end(), compare_profile);
    ROS_WARN("wanted_profiles");
    ROS_WARN_STREAM("set_wanted_profiles.size() = " << set_wanted_profiles.size());
    for (auto& profile : set_wanted_profiles)
    {
        auto video_profile = profile.as<rs2::video_stream_profile>();
        ROS_WARN_STREAM("Sensor profile: " <<
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

rcl_interfaces::msg::SetParametersResult RosSensor::set_sensor_enable_param(std::string option_name, const std::vector<rclcpp::Parameter> & parameters)
{ 
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
        if (option_name == parameter.get_name())
        {
            std::vector<std::string> option_parts;
            boost::split(option_parts, option_name, [](char c){return c == '_';});
            if (option_parts[0] == "enable")
            {
                ROS_WARN_STREAM("****enable FOUND:" << option_parts[1]);
                stream_index_pair sip(rs2_string_to_sip(option_parts[1]));
                _enabled_profiles[sip] = parameter.get_value<bool>();
                ROS_WARN_STREAM("_enabled_profiles[" << option_parts[1] << "] = " << _enabled_profiles[sip]);
            }
        }
    }
    return result;
}

// void RosSensor::registerEnableProfileParams()
// {
//     std::string module_name = create_graph_resource_name(get_info(RS2_CAMERA_INFO_NAME));
//     std::set<stream_index_pair> checked_sips, found_sips;
//     for (auto& profile : get_stream_profiles())
//     {
//         stream_index_pair sip(profile.stream_type(), profile.stream_index());
//         if (checked_sips.insert(sip).second == false)
//             continue;
//         std::string param_name = "enable_" + STREAM_NAME(sip);
//         _node.declare_parameter(param_name, rclcpp::ParameterValue(true));
//         _callback_handlers.push_back(
//             _node.add_on_set_parameters_callback(
//                 [param_name, this](const std::vector<rclcpp::Parameter> & parameters) 
//                     { 
//                         rcl_interfaces::msg::SetParametersResult result;
//                         result.successful = true;
//                         for (const auto & parameter : parameters) {
//                             ROS_INFO_STREAM("set_option: " << param_name << " == " << parameter.get_name());
//                             if (param_name == parameter.get_name())
//                             {
//                                 ROS_INFO_STREAM("Check sensors change: ");
//                                 try
//                                 {
//                                     ROS_INFO_STREAM("set_option: " << param_name << " = " << parameter.get_value<bool>());
//                                     _update_sensor_func();
//                                 }
//                                 catch(const rclcpp::ParameterTypeException& e)
//                                 {
//                                     ROS_WARN_STREAM("Error handling profile enabling: " << e.what());
//                                 }
//                             }
//                         }
//                         return result;
//                     }));
//     }
// }

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

