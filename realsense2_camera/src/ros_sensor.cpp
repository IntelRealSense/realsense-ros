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
    registerSensorParameters();
}

RosSensor::~RosSensor()
{
    stop();
}

void RosSensor::setParameters()
{
    std::string module_name = create_graph_resource_name(rs2_to_ros(get_info(RS2_CAMERA_INFO_NAME)));
    _params.registerDynamicOptions(*this, module_name);
}

void RosSensor::registerSensorParameters()
{
    std::vector<stream_profile> all_profiles = get_stream_profiles();
    const std::string module_name(create_graph_resource_name(rs2_to_ros(get_info(RS2_CAMERA_INFO_NAME))));

    std::shared_ptr<ProfilesManager> profile_manager = std::make_shared<VideoProfilesManager>(_params.getParameters(), module_name);
    profile_manager->registerProfileParameters(all_profiles, _update_sensor_func);
    if (profile_manager->isTypeExist())
    {
        _profile_managers.push_back(profile_manager);
        registerAutoExposureROIOptions();
    }
    profile_manager = std::make_shared<MotionProfilesManager>(_params.getParameters());
    profile_manager->registerProfileParameters(all_profiles, _update_sensor_func);
    if (profile_manager->isTypeExist())
    {
        _profile_managers.push_back(profile_manager);
    }
    profile_manager = std::make_shared<PoseProfilesManager>(_params.getParameters());
    profile_manager->registerProfileParameters(all_profiles, _update_sensor_func);
    if (profile_manager->isTypeExist())
    {
        _profile_managers.push_back(profile_manager);
    }
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
    ROS_INFO_STREAM("Open profile: " << ProfilesManager::profile_string(profile));

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
    for (auto profile_manager : _profile_managers)
    {
        profile_manager->addWantedProfiles(wanted_profiles);        
    }

    ROS_DEBUG_STREAM(get_info(RS2_CAMERA_INFO_NAME) << ":" << "active_profiles.size() = " << active_profiles.size());
    for (auto& profile : active_profiles)
    {
        ROS_DEBUG_STREAM("Sensor profile: " << ProfilesManager::profile_string(profile));
    }

    ROS_DEBUG_STREAM(get_info(RS2_CAMERA_INFO_NAME) << ":" << "wanted_profiles");
    for (auto& profile : wanted_profiles)
    {
        ROS_DEBUG_STREAM("Sensor profile: " << ProfilesManager::profile_string(profile));
    }
    if (compare_profiles_lists(active_profiles, wanted_profiles))
    {
        return false;
    }
    return true;
}

void RosSensor::set_sensor_auto_exposure_roi()
{
    try
    {
        int width = std::dynamic_pointer_cast<VideoProfilesManager>(_profile_managers[0])->getWidth();
        int height = std::dynamic_pointer_cast<VideoProfilesManager>(_profile_managers[0])->getWidth();

        bool update_roi_range(false);
        if (_auto_exposure_roi.max_x > width)
        {
            _params.getParameters()->setParamValue(_auto_exposure_roi.max_x, width-1);
            update_roi_range = true;
        }
        if (_auto_exposure_roi.max_y > height)
        {
            _params.getParameters()->setParamValue(_auto_exposure_roi.max_y, height-1);
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

void RosSensor::registerAutoExposureROIOptions()
{
    std::string module_base_name(rs2_to_ros(get_info(RS2_CAMERA_INFO_NAME)));

    if (this->rs2::sensor::is<rs2::roi_sensor>())
    {
        int width = std::dynamic_pointer_cast<VideoProfilesManager>(_profile_managers[0])->getWidth();
        int height = std::dynamic_pointer_cast<VideoProfilesManager>(_profile_managers[0])->getWidth();

        int max_x(width-1);
        int max_y(height-1);

        std::string module_name = create_graph_resource_name(module_base_name) +".auto_exposure_roi";
        _auto_exposure_roi = {0, 0, max_x, max_y};

        ROS_DEBUG_STREAM("Publish roi for " << module_name);
        _params.getParameters()->setParamT(module_name + ".min_x", rclcpp::ParameterValue(0),     _auto_exposure_roi.min_x, [this](const rclcpp::Parameter&){set_sensor_auto_exposure_roi();});
        _params.getParameters()->setParamT(module_name + ".max_x", rclcpp::ParameterValue(max_x), _auto_exposure_roi.max_x, [this](const rclcpp::Parameter&){set_sensor_auto_exposure_roi();});
        _params.getParameters()->setParamT(module_name + ".min_y", rclcpp::ParameterValue(0),     _auto_exposure_roi.min_y, [this](const rclcpp::Parameter&){set_sensor_auto_exposure_roi();});
        _params.getParameters()->setParamT(module_name + ".max_y", rclcpp::ParameterValue(max_y), _auto_exposure_roi.max_y, [this](const rclcpp::Parameter&){set_sensor_auto_exposure_roi();});
    }
}
