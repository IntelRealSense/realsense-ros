#include "../include/base_realsense_node.h"
#include <fstream>

using namespace realsense2_camera;
using namespace rs2;

void BaseRealSenseNode::setup()
{
    // getParameters();
    setAvailableSensors();
    SetBaseStream();
    setupFiltersPublishers();
    setCallbackFunctions();
    startMonitoring();
    monitoringProfileChanges();
    updateSensors();
}

void BaseRealSenseNode::setupFiltersPublishers()
{
    _synced_imu_publisher = std::make_shared<SyncedImuPublisher>(_node.create_publisher<sensor_msgs::msg::Imu>("imu", 5));
    //_synced_imu_publisher = std::make_shared<SyncedImuPublisher>();
    // for(auto&& sensor : _available_ros_sensors)
    // {
    //     if (sensor->is<VideoSensor>() && !_pointcloud_publisher)
    //     {
    //         ROS_INFO("Start pointcloud publisher.");
    //         _pointcloud_publisher = _node.create_publisher<sensor_msgs::msg::PointCloud2>("depth/color/points", 1);
    //     }
    //     else if (sensor->is<ImuSensor>())
    //     {
    //         ROS_INFO("Start imu publisher.");
    //         _synced_imu_publisher = std::make_shared<SyncedImuPublisher>(_node.create_publisher<sensor_msgs::msg::Imu>("imu", 5));
    //         _synced_imu_publisher->Enable(_hold_back_imu_for_frames);
    //     }
    // }
}

void BaseRealSenseNode::monitoringProfileChanges()
{
    int time_interval(10000);
    std::function<void()> func = [this, time_interval](){
        std::mutex mu;
        std::unique_lock<std::mutex> lock(mu);
        while(_is_running) {
            _cv_mpc.wait_for(lock, std::chrono::milliseconds(time_interval), [&]{return (!_is_running || _is_profile_changed);});
            if (_is_running && _is_profile_changed)
            {
                ROS_DEBUG("Profile has changed");
                updateSensors();
                _is_profile_changed = false;
            }
        }
    };
    _monitoring_pc = std::make_shared<std::thread>(func);
}

void BaseRealSenseNode::setAvailableSensors()
{
    if (!_json_file_path.empty())
    {
        if (_dev.is<rs400::advanced_mode>())
        {
            std::stringstream ss;
            std::ifstream in(_json_file_path);
            if (in.is_open())
            {
                ss << in.rdbuf();
                std::string json_file_content = ss.str();

                auto adv = _dev.as<rs400::advanced_mode>();
                adv.load_json(json_file_content);
                ROS_INFO_STREAM("JSON file is loaded! (" << _json_file_path << ")");
            }
            else
                ROS_WARN_STREAM("JSON file provided doesn't exist! (" << _json_file_path << ")");
        }
        else
            ROS_WARN("Device does not support advanced settings!");
    }
    else
        ROS_INFO("JSON file is not provided");

    auto device_name = _dev.get_info(RS2_CAMERA_INFO_NAME);
    ROS_INFO_STREAM("Device Name: " << device_name);

    auto serial_no = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    ROS_INFO_STREAM("Device Serial No: " << serial_no);

    auto device_port_id = _dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);

    ROS_INFO_STREAM("Device physical port: " << device_port_id);

    auto fw_ver = _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
    ROS_INFO_STREAM("Device FW version: " << fw_ver);

    auto pid = _dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
    ROS_INFO_STREAM("Device Product ID: 0x" << pid);

    ROS_INFO_STREAM("Align Depth: " << ((_align_depth)?"On":"Off"));
    ROS_INFO_STREAM("Sync Mode: " << ((_sync_frames)?"On":"Off"));

    std::function<void(rs2::frame)> frame_callback_function = [this](rs2::frame frame){
        bool is_filter(_filters.end() != find_if(_filters.begin(), _filters.end(), [](std::shared_ptr<NamedFilter> f){return (f->_is_enabled); }));
        if (_sync_frames || _align_depth || is_filter)
            this->_asyncer.invoke(frame);
        else 
            frame_callback(frame);
    };

    std::function<void(rs2::frame)> imu_callback_function = [this](rs2::frame frame){
        if (_imu_sync_method == imu_sync_method::NONE)
            imu_callback(frame);
        else 
            imu_callback_sync(frame);
    };

    std::function<void(rs2::frame)> multiple_message_callback_function = [this](rs2::frame frame){multiple_message_callback(frame, _imu_sync_method);};

    std::function<void()> update_sensor_func = [this](){_is_profile_changed = true; _cv_mpc.notify_one();};

    _dev_sensors = _dev.query_sensors();
    for(auto&& sensor : _dev_sensors)
    {
        const std::string module_name(sensor.get_info(RS2_CAMERA_INFO_NAME));
        std::shared_ptr<RosSensor> rosSensor;
        if (sensor.is<rs2::depth_sensor>() || 
            sensor.is<rs2::color_sensor>() ||
            sensor.is<rs2::fisheye_sensor>())
        {
            ROS_DEBUG_STREAM("Set " << module_name << " as VideoSensor.");
            rosSensor = std::make_shared<RosSensor>(sensor, _parameters, frame_callback_function, update_sensor_func);
        }
        else if (sensor.is<rs2::motion_sensor>())
        {
            ROS_DEBUG_STREAM("Set " << module_name << " as ImuSensor.");
            rosSensor = std::make_shared<RosSensor>(sensor, _parameters, imu_callback_function, update_sensor_func);
        }
        else if (sensor.is<rs2::pose_sensor>())
        {
            ROS_DEBUG_STREAM("Set " << module_name << " as PoseSensor.");
            rosSensor = std::make_shared<RosSensor>(sensor, _parameters, multiple_message_callback_function, update_sensor_func);
        }
        else
        {
            ROS_ERROR_STREAM("Module Name \"" << module_name << "\" does not define a callback.");
            throw("Error: Module not supported");
        }

        _available_ros_sensors.push_back(rosSensor);
    }

}

void BaseRealSenseNode::setCallbackFunctions()
{
    _asyncer.start([this](rs2::frame f)
    {
        frame_callback(f);
    });
}

void BaseRealSenseNode::stopPublishers(const std::vector<stream_profile>& profiles)
{
    for (auto& profile : profiles)
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (profile.is<rs2::video_stream_profile>())
        {
            std::string name = _image_publishers[sip].second;
            // _rs_diagnostic_updater.Remove(name);
            _image_publishers.erase(sip);
            _info_publisher.erase(sip);

            name = _depth_aligned_image_publishers[sip].second;
            // _rs_diagnostic_updater.Remove(name);
            _depth_aligned_image_publishers.erase(sip);
            _depth_aligned_info_publisher.erase(sip);
        }
        else if (profile.is<rs2::motion_stream_profile>())
        {
            _imu_publishers.erase(sip);
            _imu_info_publisher.erase(sip);
        }
    }
}

void BaseRealSenseNode::startPublishers(const std::vector<stream_profile>& profiles, const RosSensor& sensor)
{
    const std::string module_name(create_graph_resource_name(rs2_to_ros(sensor.get_info(RS2_CAMERA_INFO_NAME))));
    for (auto& profile : profiles)
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (profile.is<rs2::video_stream_profile>())
        {
            std::stringstream image_raw, camera_info;
            bool rectified_image = false;
            if (sensor.rs2::sensor::is<rs2::depth_sensor>())
                rectified_image = true;

            std::string stream_name(STREAM_NAME(sip));
            image_raw << stream_name << "/image_" << ((rectified_image)?"rect_":"") << "raw";
            camera_info << stream_name << "/camera_info";

            // double fps(profile.fps());
            // _rs_diagnostic_updater.Add(stream_name, diagnostic_updater::FrequencyStatusParam(&fps, &fps));
            
            std::string param_name(module_name + "." + create_graph_resource_name(ros_stream_to_string(sip.first)) + ".qos");
            std::string qos_str = _parameters->setParam(param_name, rclcpp::ParameterValue(IMAGE_QOS), [this](const rclcpp::Parameter& )
                    {
                        ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
                    }).get<rclcpp::PARAMETER_STRING>();
            
            _image_publishers[sip] = {image_transport::create_publisher(&_node, image_raw.str(), qos_string_to_qos(qos_str)), stream_name}; // TODO: remove "stream_name"
            _info_publisher[sip] = _node.create_publisher<sensor_msgs::msg::CameraInfo>(camera_info.str(), 1);

            if ((sip != DEPTH) && sip.second < 2)
            {
                std::stringstream aligned_image_raw, aligned_camera_info;
                aligned_image_raw << "aligned_depth_to_" << stream_name << "/image_raw";
                aligned_camera_info << "aligned_depth_to_" << stream_name << "/camera_info";

                std::string aligned_stream_name = "aligned_depth_to_" + stream_name;
                // _rs_diagnostic_updater.Add(aligned_stream_name, diagnostic_updater::FrequencyStatusParam(&fps, &fps));
                _depth_aligned_image_publishers[sip] = {image_transport::create_publisher(&_node, aligned_image_raw.str(), qos_string_to_qos(qos_str)), aligned_stream_name};
                _depth_aligned_info_publisher[sip] = _node.create_publisher<sensor_msgs::msg::CameraInfo>(aligned_camera_info.str(), 1);
            }
            continue;
        }
        if (profile.is<rs2::motion_stream_profile>())
        {
            std::stringstream data_topic_name, info_topic_name;
            std::string stream_name(STREAM_NAME(sip));
            data_topic_name << stream_name << "/sample";
            _imu_publishers[sip] = _node.create_publisher<sensor_msgs::msg::Imu>(data_topic_name.str(), 100);
            // Publish Intrinsics:
            info_topic_name << stream_name << "/imu_info";
            _imu_info_publisher[sip] = _node.create_publisher<IMUInfo>(info_topic_name.str(), 1);
            IMUInfo info_msg = getImuInfo(profile);
            _imu_info_publisher[sip]->publish(info_msg);
            continue;
        }
        if (profile.is<rs2::pose_stream_profile>())
        {
            std::stringstream data_topic_name, info_topic_name;
            std::string stream_name(STREAM_NAME(sip));
            data_topic_name << stream_name << "/sample";
            _odom_publisher = _node.create_publisher<nav_msgs::msg::Odometry>(data_topic_name.str(), 100);
            continue;
        }
    }
}

void BaseRealSenseNode::updateSensors()
{
    try{
        for(auto&& sensor : _available_ros_sensors)
        {
            std::string module_name(sensor->get_info(RS2_CAMERA_INFO_NAME));
            // if active_profiles != wanted_profiles: stop sensor.
            std::vector<stream_profile> wanted_profiles;

            bool is_changed(sensor->getUpdatedProfiles(wanted_profiles));
            if (is_changed)
            {
                std::vector<stream_profile> active_profiles = sensor->get_active_streams();
                sensor->stop();
                stopPublishers(active_profiles);

                if (!wanted_profiles.empty())
                {
                    ROS_INFO_STREAM("Start Sensor: " << module_name);
                    startPublishers(wanted_profiles, *sensor);
                    updateProfilesStreamCalibData(wanted_profiles);
                    {
                        std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);
                        _static_tf_msgs.clear();
                        publishStaticTransforms(wanted_profiles);
                    }

                    sensor->start(wanted_profiles);
                    if (sensor->rs2::sensor::is<rs2::depth_sensor>())
                    {
                        _depth_scale_meters = sensor->as<rs2::depth_sensor>().get_depth_scale();
                    }
                }
            }
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Unknown exception has occured!");
        throw;
    }
}
