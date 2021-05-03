#include <profile_manager.h>

using namespace realsense2_camera;
using namespace rs2;

ProfilesManager::ProfilesManager(std::shared_ptr<Parameters> parameters):
    _logger(rclcpp::get_logger("RealSenseCameraNode")),
    _params(parameters, _logger)
     {
     }

void ProfilesManager::clearParameters()
{
    while ( !_parameters_names.empty() )
    {
        auto name = _parameters_names.back();
        _params.getParameters()->removeParam(name);
        _parameters_names.pop_back();        
    }
}

template<class T>
void ProfilesManager::registerSensorUpdateParam(std::string template_name, 
                                                std::set<stream_index_pair> unique_sips, 
                                                std::map<stream_index_pair, std::shared_ptr<T> >& params, 
                                                T value, 
                                                std::function<void()> update_sensor_func)
{
    // This function registers parameters that their modification requires a sensor update.
    // For each pair of stream-index, Function add a parameter to <params> and advertise it by <template_name>.
    // parameters in <params> are dynamically being updated.
    for (auto& sip : unique_sips)
    {
        const std::string stream_name(create_graph_resource_name(STREAM_NAME(sip)));
        char* param_name = new char[template_name.size() + stream_name.size()];
        sprintf(param_name, template_name.c_str(), stream_name.c_str());

        params[sip] = std::make_shared<T>(value);
        ROS_WARN_STREAM("Set: " << param_name << ":" << value << " == " << *(params[sip]));
        std::shared_ptr<T> param = params[sip];
        // Calling setParam and modify (*param) and not setParamT that modifies param.
        rclcpp::ParameterValue aa = _params.getParameters()->setParam(std::string(param_name), rclcpp::ParameterValue(value), [param, update_sensor_func](const rclcpp::Parameter& parameter)
                {
                    *param = parameter.get_value<T>();
                    update_sensor_func();
                });
        _parameters_names.push_back(std::string(param_name));
    }
}

template void ProfilesManager::registerSensorUpdateParam<bool>(std::string template_name, std::set<stream_index_pair> unique_sips, std::map<stream_index_pair, std::shared_ptr<bool> >& params, bool value, std::function<void()> update_sensor_func);
template void ProfilesManager::registerSensorUpdateParam<double>(std::string template_name, std::set<stream_index_pair> unique_sips, std::map<stream_index_pair, std::shared_ptr<double> >& params, double value, std::function<void()> update_sensor_func);


bool ProfilesManager::isTypeExist()
{
    return (!_enabled_profiles.empty());
}

void ProfilesManager::addWantedProfiles(std::vector<rs2::stream_profile>& wanted_profiles)
{    
    std::map<stream_index_pair, bool> found_sips;
    std::map<stream_index_pair, rs2::stream_profile> default_profiles;
    ROS_INFO_STREAM("_all_profiles.size()" << _all_profiles.size());
    for (auto profile : _all_profiles)
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (!(*_enabled_profiles[sip])) continue;
        if (found_sips.find(sip) == found_sips.end())
        {
            found_sips[sip] = false;
        }
        else
        {
            if (found_sips.at(sip) == true) continue;
        }
        if (profile.is_default())
        {
            default_profiles[sip] = profile;
        }
        if (isWantedProfile(profile))
        {
            wanted_profiles.push_back(profile);
            found_sips[sip] = true;
            ROS_DEBUG_STREAM("Found profile for " << ros_stream_to_string(sip.first) << ":" << sip.second);
        }
    }
    // Print warning if any enabled profile found no match:
    for (auto found_sip : found_sips)
    {
        if (!found_sip.second)
        {
            std::stringstream msg;
            msg << "Could not find a match for profile: " << wanted_profile_string(found_sip.first);
            if (default_profiles.find(found_sip.first) != default_profiles.end())
            {
                msg << " : Using Default: " << profile_string(default_profiles[found_sip.first]);
                if (set_to_defaults(default_profiles))
                {
                    wanted_profiles.clear();
                    ROS_WARN_STREAM(msg.str());
                    return addWantedProfiles(wanted_profiles);
                }
                else
                {
                    wanted_profiles.push_back(default_profiles[found_sip.first]);
                }
            }
            else
            {
                msg << " : No default.";
            }
            ROS_WARN_STREAM(msg.str());
        }
    }
}

std::string ProfilesManager::profile_string(const rs2::stream_profile& profile)
{
    std::stringstream profile_str;
    if (profile.is<rs2::video_stream_profile>())
    {
        auto video_profile = profile.as<rs2::video_stream_profile>();
        profile_str << "stream_type: " << ros_stream_to_string(video_profile.stream_type()) << "(" << video_profile.stream_index() << ")" <<
                       ", Format: " << video_profile.format() <<
                       ", Width: " << video_profile.width() <<
                       ", Height: " << video_profile.height() <<
                       ", FPS: " << video_profile.fps();
    }
    else
    {
        profile_str << "stream_type: " << ros_stream_to_string(profile.stream_type()) << "(" << profile.stream_index() << ")" <<
                       "Format: " << profile.format() <<
                       ", FPS: " << profile.fps();
    }
    return profile_str.str();
}

VideoProfilesManager::VideoProfilesManager(std::shared_ptr<Parameters> parameters,
                                           const std::string& module_name):
    ProfilesManager(parameters),
    _module_name(module_name)
{
    _allowed_formats[RS2_STREAM_DEPTH] = RS2_FORMAT_Z16;
    _allowed_formats[RS2_STREAM_INFRARED] = RS2_FORMAT_Y8;
}

// bool VideoProfilesManager::isTypeExist()
// {
//     return _is_profile_exist;
// }

bool VideoProfilesManager::set_to_defaults(std::map<stream_index_pair, rs2::stream_profile>& default_profiles)
{
    rs2::stream_profile default_profile = default_profiles.begin()->second;
    auto video_profile = default_profile.as<rs2::video_stream_profile>();

    _params.getParameters()->setParamValue(_width, video_profile.width());
    _params.getParameters()->setParamValue(_height, video_profile.height());
    _params.getParameters()->setParamValue(_fps, video_profile.fps());
    return true;
}


std::string VideoProfilesManager::wanted_profile_string(stream_index_pair sip)
{
    std::stringstream str;
    str << STREAM_NAME(sip) << " with width: " << _width << ", " << "height: " << _height << ", fps: " << _fps;
    return str.str();
}

bool VideoProfilesManager::isWantedProfile(const rs2::stream_profile& profile)
{
    if (!profile.is<rs2::video_stream_profile>())
        return false;
    auto video_profile = profile.as<rs2::video_stream_profile>();
    ROS_DEBUG_STREAM("Sensor profile: " << profile_string(profile));

    return ((video_profile.width() == _width) &&
            (video_profile.height() == _height) &&
            (video_profile.fps() == _fps) &&
            (_allowed_formats.find(video_profile.stream_type()) == _allowed_formats.end() || video_profile.format() == _allowed_formats[video_profile.stream_type()] ));
}

void VideoProfilesManager::registerProfileParameters(std::vector<stream_profile> all_profiles, std::function<void()> update_sensor_func)
{
    std::set<stream_index_pair> checked_sips;
    for (auto& profile : all_profiles)
    {
        if (!profile.is<video_stream_profile>()) continue;
        ROS_DEBUG_STREAM("Register profile: " << profile_string(profile));
        _all_profiles.push_back(profile);
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        checked_sips.insert(sip);
    }
    if (!checked_sips.empty())
    {
        ROS_WARN_STREAM(__LINE__ << ": _enabled_profiles.size(): " << _enabled_profiles.size());
        registerSensorUpdateParam("enable_%s", checked_sips, _enabled_profiles, true, update_sensor_func);
        
        for (auto& sip : checked_sips)
        {
            ROS_WARN_STREAM(__LINE__ << ": _enabled_profiles[" << ros_stream_to_string(sip.first) << ":" << sip.second << "]: " << *(_enabled_profiles[sip]));
        }

        registerVideoSensorParams();
    }
}

void VideoProfilesManager::registerVideoSensorParams()
{

    std::string param_name(_module_name + ".width");
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _params.getParameters()->setParamT(param_name, rclcpp::ParameterValue(IMAGE_WIDTH), _width, [this](const rclcpp::Parameter& )
            {
                ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
            });
    _parameters_names.push_back(param_name);
    param_name = _module_name + ".height";
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _params.getParameters()->setParamT(param_name, rclcpp::ParameterValue(IMAGE_HEIGHT), _height, [this](const rclcpp::Parameter& )
            {
                ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
            });
    _parameters_names.push_back(param_name);
    param_name = _module_name + ".fps";
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _params.getParameters()->setParamT(param_name, rclcpp::ParameterValue(IMAGE_FPS), _fps, [this](const rclcpp::Parameter& )
            {
                ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
            });
    _parameters_names.push_back(param_name);
}

///////////////////////////////////////////////////////////////////////////////////////

bool MotionProfilesManager::isWantedProfile(const rs2::stream_profile& profile)
{
    stream_index_pair stream(profile.stream_type(), profile.stream_index());
    return (profile.fps() == *(_fps[stream]));
}

void MotionProfilesManager::registerProfileParameters(std::vector<stream_profile> all_profiles, std::function<void()> update_sensor_func)
{
    std::set<stream_index_pair> checked_sips;
    for (auto& profile : all_profiles)
    {
        if (!profile.is<motion_stream_profile>()) continue;
        ROS_DEBUG_STREAM("Register profile: " << profile_string(profile));
        _all_profiles.push_back(profile);
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        checked_sips.insert(sip);
    }
    registerSensorUpdateParam("enable_%s", checked_sips, _enabled_profiles, true, update_sensor_func);
    registerSensorUpdateParam("%s_fps", checked_sips, _fps, 0.0, update_sensor_func);
}

std::string MotionProfilesManager::wanted_profile_string(stream_index_pair sip)
{
    std::stringstream str;
    str << STREAM_NAME(sip) << " with fps: " << _fps[sip];
    return str.str();
}

bool MotionProfilesManager::set_to_defaults(std::map<stream_index_pair, rs2::stream_profile>&)
{
    return false;
}
///////////////////////////////////////////////////////////////////////////////////////

void PoseProfilesManager::registerProfileParameters(std::vector<stream_profile> all_profiles, std::function<void()> update_sensor_func)
{
    std::set<stream_index_pair> checked_sips;
    for (auto& profile : all_profiles)
    {
        if (!profile.is<pose_stream_profile>()) continue;
        _all_profiles.push_back(profile);
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        checked_sips.insert(sip);
    }
    registerSensorUpdateParam("enable_%s", checked_sips, _enabled_profiles, true, update_sensor_func);
    registerSensorUpdateParam("%s_fps", checked_sips, _fps, 0.0, update_sensor_func);
}
