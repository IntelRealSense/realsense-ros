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

std::vector<stream_profile> RosSensor::getUpdatedProfiles()
{
    getUpdatedSensorParameters();
    std::vector<stream_profile> active_profiles = get_active_streams();
    std::vector<stream_profile> wanted_profiles;
    std::set<stream_index_pair> checked_sips, found_sips;
    for (auto& profile : get_stream_profiles())
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (checked_sips.insert(sip).second == true)
            getUpdatedProfileParameters(profile);
        if (!_enable[sip])
            continue;
        ROS_INFO_STREAM_ONCE("Looking for " << rs2_stream_to_string(sip.first) << ":" << sip.second);
        if (found_sips.find(sip) == found_sips.end() && isWantedProfile(profile))
        {
            wanted_profiles.push_back(profile);
            found_sips.insert(sip);
            ROS_INFO_STREAM("Found profile for " << rs2_stream_to_string(sip.first) << ":" << sip.second);
        }
    }
    if (std::set<stream_profile>(active_profiles.begin(), active_profiles.end()) == std::set<stream_profile>(wanted_profiles.begin(), wanted_profiles.end()))
    {
        return std::vector<stream_profile>();
    }
    return wanted_profiles;
}
