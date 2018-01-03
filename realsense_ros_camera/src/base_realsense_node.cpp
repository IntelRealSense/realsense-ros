#include "../include/base_realsense_node.h"

using namespace realsense_ros_camera;

BaseRealSenseNode::BaseRealSenseNode(ros::NodeHandle& nodeHandle,
                                     ros::NodeHandle& privateNodeHandle,
                                     rs2::device dev,
                                     const std::string& serial_no) :
    _dev(dev),  _node_handle(nodeHandle),
    _pnh(privateNodeHandle), _json_file_path(""),
    _serial_no(serial_no), _base_frame_id(""),
    _intialize_time_base(false)
{
    // Types for depth stream
    _format[DEPTH] = RS2_FORMAT_Z16;   // libRS type
    _image_format[DEPTH] = CV_16UC1;    // CVBridge type
    _encoding[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
    _unit_step_size[DEPTH] = sizeof(uint16_t); // sensor_msgs::ImagePtr row step size
    _stream_name[DEPTH] = "depth";

    // Infrared stream - Left
    _format[INFRA1] = RS2_FORMAT_Y8;   // libRS type
    _image_format[INFRA1] = CV_8UC1;    // CVBridge type
    _encoding[INFRA1] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[INFRA1] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[INFRA1] = "infra1";

    // Infrared stream - Right
    _format[INFRA2] = RS2_FORMAT_Y8;   // libRS type
    _image_format[INFRA2] = CV_8UC1;    // CVBridge type
    _encoding[INFRA2] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[INFRA2] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[INFRA2] = "infra2";

    // Types for color stream
    _format[COLOR] = RS2_FORMAT_RGB8;   // libRS type
    _image_format[COLOR] = CV_8UC3;    // CVBridge type
    _encoding[COLOR] = sensor_msgs::image_encodings::RGB8; // ROS message type
    _unit_step_size[COLOR] = 3; // sensor_msgs::ImagePtr row step size
    _stream_name[COLOR] = "color";

    // Types for fisheye stream
    _format[FISHEYE] = RS2_FORMAT_RAW8;   // libRS type
    _image_format[FISHEYE] = CV_8UC1;    // CVBridge type
    _encoding[FISHEYE] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[FISHEYE] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[FISHEYE] = "fisheye";

    // Types for Motion-Module streams
    _format[GYRO] = RS2_FORMAT_MOTION_XYZ32F;   // libRS type
    _image_format[GYRO] = CV_8UC1;    // CVBridge type
    _encoding[GYRO] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[GYRO] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[GYRO] = "gyro";

    _format[ACCEL] = RS2_FORMAT_MOTION_XYZ32F;   // libRS type
    _image_format[ACCEL] = CV_8UC1;    // CVBridge type
    _encoding[ACCEL] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[ACCEL] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[ACCEL] = "accel";
}

void BaseRealSenseNode::publishTopics()
{
    getParameters();
    setupDevice();
    setupPublishers();
    setupStreams();
    publishStaticTransforms();
    ROS_INFO_STREAM("RealSense Node Is Up!");
}

void BaseRealSenseNode::registerDynamicReconfigCb()
{
    ROS_INFO("Dynamic reconfig parameters is not implemented in the base node.");
}

void BaseRealSenseNode::getParameters()
{
    ROS_INFO("getParameters...");

    _pnh.param("enable_pointcloud", _pointcloud, POINTCLOUD);
    _pnh.param("enable_sync", _sync_frames, SYNC_FRAMES);
    if (_pointcloud)
        _sync_frames = true;

    _pnh.param("json_file_path", _json_file_path, std::string(""));

    _pnh.param("depth_width", _width[DEPTH], DEPTH_WIDTH);
    _pnh.param("depth_height", _height[DEPTH], DEPTH_HEIGHT);
    _pnh.param("depth_fps", _fps[DEPTH], DEPTH_FPS);
    _pnh.param("enable_depth", _enable[DEPTH], ENABLE_DEPTH);

    _pnh.param("infra1_width", _width[INFRA1], INFRA1_WIDTH);
    _pnh.param("infra1_height", _height[INFRA1], INFRA1_HEIGHT);
    _pnh.param("infra1_fps", _fps[INFRA1], INFRA1_FPS);
    _pnh.param("enable_infra1", _enable[INFRA1], ENABLE_INFRA1);

    _pnh.param("infra2_width", _width[INFRA2], INFRA2_WIDTH);
    _pnh.param("infra2_height", _height[INFRA2], INFRA2_HEIGHT);
    _pnh.param("infra2_fps", _fps[INFRA2], INFRA2_FPS);
    _pnh.param("enable_infra2", _enable[INFRA2], ENABLE_INFRA2);

    _pnh.param("color_width", _width[COLOR], COLOR_WIDTH);
    _pnh.param("color_height", _height[COLOR], COLOR_HEIGHT);
    _pnh.param("color_fps", _fps[COLOR], COLOR_FPS);
    _pnh.param("enable_color", _enable[COLOR], ENABLE_COLOR);

    _pnh.param("fisheye_width", _width[FISHEYE], FISHEYE_WIDTH);
    _pnh.param("fisheye_height", _height[FISHEYE], FISHEYE_HEIGHT);
    _pnh.param("fisheye_fps", _fps[FISHEYE], FISHEYE_FPS);
    _pnh.param("enable_fisheye", _enable[FISHEYE], ENABLE_FISHEYE);

    _pnh.param("gyro_fps", _fps[GYRO], GYRO_FPS);
    _pnh.param("accel_fps", _fps[ACCEL], ACCEL_FPS);
    _pnh.param("enable_imu", _enable[GYRO], ENABLE_IMU);
    _pnh.param("enable_imu", _enable[ACCEL], ENABLE_IMU);

    _pnh.param("base_frame_id", _base_frame_id, DEFAULT_BASE_FRAME_ID);
    _pnh.param("depth_frame_id", _frame_id[DEPTH], DEFAULT_DEPTH_FRAME_ID);
    _pnh.param("infra1_frame_id", _frame_id[INFRA1], DEFAULT_INFRA1_FRAME_ID);
    _pnh.param("infra2_frame_id", _frame_id[INFRA2], DEFAULT_INFRA2_FRAME_ID);
    _pnh.param("color_frame_id", _frame_id[COLOR], DEFAULT_COLOR_FRAME_ID);
    _pnh.param("fisheye_frame_id", _frame_id[FISHEYE], DEFAULT_FISHEYE_FRAME_ID);
    _pnh.param("imu_gyro_frame_id", _frame_id[GYRO], DEFAULT_IMU_FRAME_ID);
    _pnh.param("imu_accel_frame_id", _frame_id[ACCEL], DEFAULT_IMU_FRAME_ID);

    _pnh.param("depth_optical_frame_id", _optical_frame_id[DEPTH], DEFAULT_DEPTH_OPTICAL_FRAME_ID);
    _pnh.param("infra1_optical_frame_id", _optical_frame_id[INFRA1], DEFAULT_INFRA1_OPTICAL_FRAME_ID);
    _pnh.param("infra2_optical_frame_id", _optical_frame_id[INFRA2], DEFAULT_INFRA2_OPTICAL_FRAME_ID);
    _pnh.param("color_optical_frame_id", _optical_frame_id[COLOR], DEFAULT_COLOR_OPTICAL_FRAME_ID);
    _pnh.param("fisheye_optical_frame_id", _optical_frame_id[FISHEYE], DEFAULT_FISHEYE_OPTICAL_FRAME_ID);
    _pnh.param("gyro_optical_frame_id", _optical_frame_id[GYRO], DEFAULT_GYRO_OPTICAL_FRAME_ID);
    _pnh.param("accel_optical_frame_id", _optical_frame_id[ACCEL], DEFAULT_ACCEL_OPTICAL_FRAME_ID);
}

void BaseRealSenseNode::setupDevice()
{
    ROS_INFO("setupDevice...");
    try{
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

        auto camera_name = _dev.get_info(RS2_CAMERA_INFO_NAME);
        ROS_INFO_STREAM("Device Name: " << camera_name);

        ROS_INFO_STREAM("Device Serial No: " << _serial_no);

        auto fw_ver = _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
        ROS_INFO_STREAM("Device FW version: " << fw_ver);

        auto pid = _dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
        ROS_INFO_STREAM("Device Product ID: " << pid);

        ROS_INFO_STREAM("Sync Mode: " << ((_sync_frames)?"On":"Off"));

        auto dev_sensors = _dev.query_sensors();

        ROS_INFO_STREAM("Device Sensors: ");
        for(auto&& elem : dev_sensors)
        {
            std::string module_name = elem.get_info(RS2_CAMERA_INFO_NAME);
            if ("Stereo Module" == module_name || "Coded-Light Depth Sensor" == module_name)
            {
                _sensors[DEPTH] = elem;
                _sensors[INFRA1] = elem;
                _sensors[INFRA2] = elem;
            }
            else if ("RGB Camera" == module_name)
            {
                _sensors[COLOR] = elem;
            }
            else if ("Wide FOV Camera" == module_name)
            {
                _sensors[FISHEYE] = elem;
            }
            else if ("Motion Module" == module_name)
            {
                _sensors[GYRO] = elem;
                _sensors[ACCEL] = elem;
            }
            else
            {
                ROS_ERROR_STREAM("Module Name \"" << module_name << "\" isn't supported by LibRealSense! Terminate RealSense Node...");
                ros::shutdown();
                exit(1);
            }
            ROS_INFO_STREAM(std::string(elem.get_info(RS2_CAMERA_INFO_NAME)) << " was found.");
        }

        // Update "enable" map
        std::vector<std::vector<stream_index_pair>> streams(IMAGE_STREAMS);
        streams.insert(streams.end(), HID_STREAMS.begin(), HID_STREAMS.end());
        for (auto& elem : streams)
        {
            for (auto& stream_index : elem)
            {
                if (_enable[stream_index] && _sensors.find(stream_index) == _sensors.end()) // check if device supports the enabled stream
                {
                    ROS_INFO_STREAM(rs2_stream_to_string(stream_index.first) << " sensor isn't supported by current device! -- Skipping...");
                    _enable[stream_index] = false;
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

void BaseRealSenseNode::setupPublishers()
{
    ROS_INFO("setupPublishers...");
    image_transport::ImageTransport image_transport(_node_handle);

    std::vector<stream_index_pair> image_stream_types;
    for (auto& stream_vec : IMAGE_STREAMS)
    {
        for (auto& stream : stream_vec)
        {
            image_stream_types.push_back(stream);
        }
    }

    for (auto& stream : image_stream_types)
    {
        if (_enable[stream])
        {
            std::stringstream image_raw, camera_info;
            image_raw << _stream_name[stream] << "/image_raw";
            camera_info << _stream_name[stream] << "/camera_info";

            _image_publishers[stream] = image_transport.advertise(image_raw.str(), 1);
            _info_publisher[stream] = _node_handle.advertise<sensor_msgs::CameraInfo>(camera_info.str(), 1);

            if (stream == DEPTH && _pointcloud)
            {
                _pointcloud_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("depth/color/points", 1);
            }
        }
    }

    if (_enable[FISHEYE] &&
        _enable[DEPTH])
    {
        _fe_to_depth_publisher = _node_handle.advertise<Extrinsics>("extrinsics/fisheye2depth", 1, true);
    }

    if (_enable[GYRO])
    {
        _imu_publishers[GYRO] = _node_handle.advertise<sensor_msgs::Imu>("gyro/sample", 100);
        _info_publisher[GYRO] = _node_handle.advertise<IMUInfo>("gyro/imu_info", 1, true);
    }

    if (_enable[ACCEL])
    {
        _imu_publishers[ACCEL] = _node_handle.advertise<sensor_msgs::Imu>("accel/sample", 100);
        _info_publisher[ACCEL] = _node_handle.advertise<IMUInfo>("accel/imu_info", 1, true);
    }

    if (_enable[FISHEYE] &&
        (_enable[GYRO] ||
         _enable[ACCEL]))
    {
        _fe_to_imu_publisher = _node_handle.advertise<Extrinsics>("extrinsics/fisheye2imu", 1, true);
    }
}

void BaseRealSenseNode::setupStreams()
{
    ROS_INFO("setupStreams...");
    try{
        for (auto& streams : IMAGE_STREAMS)
        {
            for (auto& elem : streams)
            {
                if (_enable[elem])
                {
                    auto& sens = _sensors[elem];
                    auto profiles = sens.get_stream_profiles();
                    for (auto& profile : profiles)
                    {
                        auto video_profile = profile.as<rs2::video_stream_profile>();
                        if (video_profile.format() == _format[elem] &&
                            video_profile.width()  == _width[elem] &&
                            video_profile.height() == _height[elem] &&
                            video_profile.fps()    == _fps[elem] &&
                            video_profile.stream_index() == elem.second)
                        {
                            _enabled_profiles[elem].push_back(profile);

                            _image[elem] = cv::Mat(_width[elem], _height[elem], _image_format[elem], cv::Scalar(0, 0, 0));
                            ROS_INFO_STREAM(_stream_name[elem] << " stream is enabled - width: " << _width[elem] << ", height: " << _height[elem] << ", fps: " << _fps[elem]);
                            break;
                        }
                    }
                    if (_enabled_profiles.find(elem) == _enabled_profiles.end())
                    {
                        ROS_WARN_STREAM("Given stream configuration is not supported by the device! " <<
                                        " Stream: " << rs2_stream_to_string(elem.first) <<
                                        ", Format: " << _format[elem] <<
                                        ", Width: " << _width[elem] <<
                                        ", Height: " << _height[elem] <<
                                        ", FPS: " << _fps[elem]);
                        _enable[elem] = false;
                    }
                }
            }
        }

        // Publish image stream info
        for (auto& profiles : _enabled_profiles)
        {
            for (auto& profile : profiles.second)
            {
                auto video_profile = profile.as<rs2::video_stream_profile>();
                updateStreamCalibData(video_profile);
            }
        }

        auto frame_callback = [this](rs2::frame frame)
        {
            // We compute a ROS timestamp which is based on an initial ROS time at point of first frame,
            // and the incremental timestamp from the camera.
            // In sync mode the timestamp is based on ROS time
            if (false == _intialize_time_base)
            {
                _intialize_time_base = true;
                _ros_time_base = ros::Time::now();
                _camera_time_base = frame.get_timestamp();
            }

            ros::Time t;
            if (_sync_frames)
                t = ros::Time::now();
            else
                t = ros::Time(_ros_time_base.toSec()+ (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base) / /*ms to seconds*/ 1000);

            auto is_color_frame_arrived = false;
            auto is_depth_frame_arrived = false;
            if (frame.is<rs2::frameset>())
            {
                ROS_DEBUG("Frameset arrived");
                auto frameset = frame.as<rs2::frameset>();
                for (auto it = frameset.begin(); it != frameset.end(); ++it)
                {
                    auto f = (*it);
                    auto stream_type = f.get_profile().stream_type();
                    if (RS2_STREAM_COLOR == stream_type)
                        is_color_frame_arrived = true;
                    else if (RS2_STREAM_DEPTH == stream_type)
                        is_depth_frame_arrived = true;

                    ROS_DEBUG("Frameset contain %s frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                              rs2_stream_to_string(stream_type), frame.get_frame_number(), frame.get_timestamp(), t.toNSec());
                    publishFrame(f, t);
                }
            }
            else
            {
                auto stream_type = frame.get_profile().stream_type();
                ROS_DEBUG("%s video frame arrived. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                          rs2_stream_to_string(stream_type), frame.get_frame_number(), frame.get_timestamp(), t.toNSec());
                publishFrame(frame, t);
            }

            if(_pointcloud && is_depth_frame_arrived && is_color_frame_arrived &&
               (0 != _pointcloud_publisher.getNumSubscribers()))
            {
                ROS_DEBUG("publishPCTopic(...)");
                publishPCTopic(t);
            }
        };

        // Streaming IMAGES
        for (auto& streams : IMAGE_STREAMS)
        {
            std::vector<rs2::stream_profile> profiles;
            for (auto& elem : streams)
            {
                if (!_enabled_profiles[elem].empty())
                {
                    profiles.insert(profiles.begin(),
                                    _enabled_profiles[elem].begin(),
                                    _enabled_profiles[elem].end());
                }
            }

            if (!profiles.empty())
            {
                auto stream = streams.front();
                auto& sens = _sensors[stream];
                sens.open(profiles);

                if (DEPTH == stream)
                {
                    auto depth_sensor = sens.as<rs2::depth_sensor>();
                    _depth_scale_meters = depth_sensor.get_depth_scale();
                }

                if (_sync_frames)
                {
                    sens.start(_syncer);
                }
                else
                {
                    sens.start(frame_callback);
                }
            }
        }//end for

        if (_sync_frames)
        {
            _syncer.start(frame_callback);
        }

        // Streaming HID
        for (const auto streams : HID_STREAMS)
        {
            for (auto& elem : streams)
            {
                if (_enable[elem])
                {
                    auto& sens = _sensors[elem];
                    auto profiles = sens.get_stream_profiles();
                    for (rs2::stream_profile& profile : profiles)
                    {
                        if (profile.fps() == _fps[elem] &&
                            profile.format() == _format[elem])
                        {
                            _enabled_profiles[elem].push_back(profile);
                            break;
                        }
                    }
                }
            }
        }

        auto gyro_profile = _enabled_profiles.find(GYRO);
        auto accel_profile = _enabled_profiles.find(ACCEL);

        if (gyro_profile != _enabled_profiles.end() &&
            accel_profile != _enabled_profiles.end())
        {
            std::vector<rs2::stream_profile> profiles;
            profiles.insert(profiles.begin(), gyro_profile->second.begin(), gyro_profile->second.end());
            profiles.insert(profiles.begin(), accel_profile->second.begin(), accel_profile->second.end());
            auto& sens = _sensors[GYRO];
            sens.open(profiles);

            sens.start([this](rs2::frame frame){
                auto stream = frame.get_profile().stream_type();
                if (false == _intialize_time_base)
                    return;

                ROS_DEBUG("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
                          rs2_stream_to_string(frame.get_profile().stream_type()),
                          frame.get_profile().stream_index(),
                          rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));

                auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
                if (0 != _info_publisher[stream_index].getNumSubscribers() ||
                    0 != _imu_publishers[stream_index].getNumSubscribers())
                {
                    double elapsed_camera_ms = (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base) / /*ms to seconds*/ 1000;
                    ros::Time t(_ros_time_base.toSec() + elapsed_camera_ms);

                    auto imu_msg = sensor_msgs::Imu();
                    imu_msg.header.frame_id = _optical_frame_id[stream_index];
                    imu_msg.orientation.x = 0.0;
                    imu_msg.orientation.y = 0.0;
                    imu_msg.orientation.z = 0.0;
                    imu_msg.orientation.w = 0.0;
                    imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    auto axes = *(reinterpret_cast<const float3*>(frame.get_data()));
                    if (GYRO == stream_index)
                    {
                        imu_msg.angular_velocity.x = axes.x;
                        imu_msg.angular_velocity.y = axes.y;
                        imu_msg.angular_velocity.z = axes.z;
                    }
                    else if (ACCEL == stream_index)
                    {
                        imu_msg.linear_acceleration.x = axes.x;
                        imu_msg.linear_acceleration.y = axes.y;
                        imu_msg.linear_acceleration.z = axes.z;
                    }
                    _seq[stream_index] += 1;
                    imu_msg.header.seq = _seq[stream_index];
                    imu_msg.header.stamp = t;
                    _imu_publishers[stream_index].publish(imu_msg);
                    ROS_DEBUG("Publish %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
                }
            });

            if (_enable[GYRO])
            {
                ROS_INFO_STREAM(_stream_name[GYRO] << " stream is enabled - " << "fps: " << _fps[GYRO]);
                auto gyroInfo = getImuInfo(GYRO);
                _info_publisher[GYRO].publish(gyroInfo);
            }

            if (_enable[ACCEL])
            {
                ROS_INFO_STREAM(_stream_name[ACCEL] << " stream is enabled - " << "fps: " << _fps[ACCEL]);
                auto accelInfo = getImuInfo(ACCEL);
                _info_publisher[ACCEL].publish(accelInfo);
            }
        }


        if (_enable[DEPTH] &&
            _enable[FISHEYE])
        {
            auto ex = getFisheye2DepthExtrinsicsMsg();
            _fe_to_depth_publisher.publish(ex);
        }

        if (_enable[FISHEYE] &&
            (_enable[GYRO] || _enable[ACCEL]))
        {
            auto ex = getFisheye2ImuExtrinsicsMsg();
            _fe_to_imu_publisher.publish(ex);
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

void BaseRealSenseNode::updateStreamCalibData(const rs2::video_stream_profile& video_profile)
{
    stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};
    auto intrinsic = video_profile.get_intrinsics();
    _stream_intrinsics[stream_index] = intrinsic;

    _camera_info[stream_index].width = intrinsic.width;
    _camera_info[stream_index].height = intrinsic.height;
    _camera_info[stream_index].header.frame_id = _optical_frame_id[stream_index];

    _camera_info[stream_index].K.at(0) = intrinsic.fx;
    _camera_info[stream_index].K.at(2) = intrinsic.ppx;
    _camera_info[stream_index].K.at(4) = intrinsic.fy;
    _camera_info[stream_index].K.at(5) = intrinsic.ppy;
    _camera_info[stream_index].K.at(8) = 1;

    _camera_info[stream_index].P.at(0) = _camera_info[stream_index].K.at(0);
    _camera_info[stream_index].P.at(1) = 0;
    _camera_info[stream_index].P.at(2) = _camera_info[stream_index].K.at(2);
    _camera_info[stream_index].P.at(3) = 0;
    _camera_info[stream_index].P.at(4) = 0;
    _camera_info[stream_index].P.at(5) = _camera_info[stream_index].K.at(4);
    _camera_info[stream_index].P.at(6) = _camera_info[stream_index].K.at(5);
    _camera_info[stream_index].P.at(7) = 0;
    _camera_info[stream_index].P.at(8) = 0;
    _camera_info[stream_index].P.at(9) = 0;
    _camera_info[stream_index].P.at(10) = 1;
    _camera_info[stream_index].P.at(11) = 0;

    rs2::stream_profile depth_profile;
    if (!getEnabledProfile(DEPTH, depth_profile))
    {
        ROS_ERROR_STREAM("Given depth profile is not supported by current device!");
        ros::shutdown();
        exit(1);
    }

    // TODO: Why Depth to Color?
    if (stream_index == DEPTH && _enable[DEPTH] && _enable[COLOR])
    {
        _depth2color_extrinsics = depth_profile.get_extrinsics_to(_enabled_profiles[COLOR].front());
        // set depth to color translation values in Projection matrix (P)
        _camera_info[stream_index].P.at(3) = _depth2color_extrinsics.translation[0];     // Tx
        _camera_info[stream_index].P.at(7) = _depth2color_extrinsics.translation[1];     // Ty
        _camera_info[stream_index].P.at(11) = _depth2color_extrinsics.translation[2];    // Tz
    }

    _camera_info[stream_index].distortion_model = "plumb_bob";

    // set R (rotation matrix) values to identity matrix
    _camera_info[stream_index].R.at(0) = 1.0;
    _camera_info[stream_index].R.at(1) = 0.0;
    _camera_info[stream_index].R.at(2) = 0.0;
    _camera_info[stream_index].R.at(3) = 0.0;
    _camera_info[stream_index].R.at(4) = 1.0;
    _camera_info[stream_index].R.at(5) = 0.0;
    _camera_info[stream_index].R.at(6) = 0.0;
    _camera_info[stream_index].R.at(7) = 0.0;
    _camera_info[stream_index].R.at(8) = 1.0;

    for (int i = 0; i < 5; i++)
    {
        _camera_info[stream_index].D.push_back(intrinsic.coeffs[i]);
    }
}

Eigen::Quaternionf BaseRealSenseNode::rotationMatrixToQuaternion(float rotation[3]) const
{
    Eigen::Matrix3f m;
    m << rotation[0], rotation[1], rotation[2],
         rotation[3], rotation[4], rotation[5],
         rotation[6], rotation[7], rotation[8];
    Eigen::Quaternionf q(m);
    return q;
}

void BaseRealSenseNode::publishStaticTransforms()
{
    ROS_INFO("publishStaticTransforms...");
    // Publish transforms for the cameras
    tf::Quaternion q_c2co;
    geometry_msgs::TransformStamped b2c_msg; // Base to Color
    geometry_msgs::TransformStamped c2co_msg; // Color to Color_Optical

    tf::Quaternion q_d2do;
    geometry_msgs::TransformStamped b2d_msg; // Base to Depth
    geometry_msgs::TransformStamped d2do_msg; // Depth to Depth_Optical

    tf::Quaternion ir1_2_ir1o;
    geometry_msgs::TransformStamped b2ir1_msg; // Base to IR1
    geometry_msgs::TransformStamped ir1_2_ir1o_msg; // IR1 to IR1_Optical

    tf::Quaternion ir2_2_ir2o;
    geometry_msgs::TransformStamped b2ir2_msg; // Base to IR2
    geometry_msgs::TransformStamped ir2_2_ir2o_msg; // IR2 to IR2_Optical

    // Get the current timestamp for all static transforms
    ros::Time transform_ts_ = ros::Time::now();

    // The depth frame is used as the base frame.
    // Hence no additional transformation is done from base frame to depth frame.
    // Transform base frame to depth frame
    b2d_msg.header.stamp = transform_ts_;
    b2d_msg.header.frame_id = _base_frame_id;
    b2d_msg.child_frame_id = _frame_id[DEPTH];
    b2d_msg.transform.translation.x = 0;
    b2d_msg.transform.translation.y = 0;
    b2d_msg.transform.translation.z = 0;
    b2d_msg.transform.rotation.x = 0;
    b2d_msg.transform.rotation.y = 0;
    b2d_msg.transform.rotation.z = 0;
    b2d_msg.transform.rotation.w = 1;
    _static_tf_broadcaster.sendTransform(b2d_msg);

    // Transform depth frame to depth optical frame
    q_d2do.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    d2do_msg.header.stamp = transform_ts_;
    d2do_msg.header.frame_id = _frame_id[DEPTH];
    d2do_msg.child_frame_id = _optical_frame_id[DEPTH];
    d2do_msg.transform.translation.x = 0;
    d2do_msg.transform.translation.y = 0;
    d2do_msg.transform.translation.z = 0;
    d2do_msg.transform.rotation.x = q_d2do.getX();
    d2do_msg.transform.rotation.y = q_d2do.getY();
    d2do_msg.transform.rotation.z = q_d2do.getZ();
    d2do_msg.transform.rotation.w = q_d2do.getW();
    _static_tf_broadcaster.sendTransform(d2do_msg);

    rs2::stream_profile depth_profile;
    if (!getEnabledProfile(DEPTH, depth_profile))
    {
        ROS_ERROR_STREAM("Given depth profile is not supported by current device!");
        ros::shutdown();
        exit(1);
    }

    if (_enable[COLOR])
    {
        // Transform base frame to color frame
        auto q = rotationMatrixToQuaternion(_depth2color_extrinsics.rotation);

        b2c_msg.header.stamp = transform_ts_;
        b2c_msg.header.frame_id = _base_frame_id;
        b2c_msg.child_frame_id = _frame_id[COLOR];
        b2c_msg.transform.translation.x = _depth2color_extrinsics.translation[2];
        b2c_msg.transform.translation.y = -_depth2color_extrinsics.translation[0];
        b2c_msg.transform.translation.z = -_depth2color_extrinsics.translation[1];
        b2c_msg.transform.rotation.x = q.x();
        b2c_msg.transform.rotation.y = q.y();
        b2c_msg.transform.rotation.z = q.z();
        b2c_msg.transform.rotation.w = q.w();
        _static_tf_broadcaster.sendTransform(b2c_msg);

        // Transform color frame to color optical frame
        q_c2co.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
        c2co_msg.header.stamp = transform_ts_;
        c2co_msg.header.frame_id = _frame_id[COLOR];
        c2co_msg.child_frame_id = _optical_frame_id[COLOR];
        c2co_msg.transform.translation.x = 0;
        c2co_msg.transform.translation.y = 0;
        c2co_msg.transform.translation.z = 0;
        c2co_msg.transform.rotation.x = q_c2co.getX();
        c2co_msg.transform.rotation.y = q_c2co.getY();
        c2co_msg.transform.rotation.z = q_c2co.getZ();
        c2co_msg.transform.rotation.w = q_c2co.getW();
        _static_tf_broadcaster.sendTransform(c2co_msg);
    }

    if (_enable[INFRA1])
    {
        auto d2ir1_extrinsics = depth_profile.get_extrinsics_to(_enabled_profiles[INFRA1].front());
        auto q = rotationMatrixToQuaternion(d2ir1_extrinsics.rotation);

        // Transform base frame to infra1
        b2ir1_msg.header.stamp = transform_ts_;
        b2ir1_msg.header.frame_id = _base_frame_id;
        b2ir1_msg.child_frame_id = _frame_id[INFRA1];
        b2ir1_msg.transform.translation.x = d2ir1_extrinsics.translation[2];
        b2ir1_msg.transform.translation.y = -d2ir1_extrinsics.translation[0];
        b2ir1_msg.transform.translation.z = -d2ir1_extrinsics.translation[1];

        b2ir1_msg.transform.rotation.x = q.x();
        b2ir1_msg.transform.rotation.y = q.y();
        b2ir1_msg.transform.rotation.z = q.z();
        b2ir1_msg.transform.rotation.w = q.w();
        _static_tf_broadcaster.sendTransform(b2ir1_msg);

        // Transform infra1 frame to infra1 optical frame
        ir1_2_ir1o.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
        ir1_2_ir1o_msg.header.stamp = transform_ts_;
        ir1_2_ir1o_msg.header.frame_id = _frame_id[INFRA1];
        ir1_2_ir1o_msg.child_frame_id = _optical_frame_id[INFRA1];
        ir1_2_ir1o_msg.transform.translation.x = 0;
        ir1_2_ir1o_msg.transform.translation.y = 0;
        ir1_2_ir1o_msg.transform.translation.z = 0;
        ir1_2_ir1o_msg.transform.rotation.x = ir1_2_ir1o.getX();
        ir1_2_ir1o_msg.transform.rotation.y = ir1_2_ir1o.getY();
        ir1_2_ir1o_msg.transform.rotation.z = ir1_2_ir1o.getZ();
        ir1_2_ir1o_msg.transform.rotation.w = ir1_2_ir1o.getW();
        _static_tf_broadcaster.sendTransform(ir1_2_ir1o_msg);
    }

    if (_enable[INFRA2])
    {
        auto d2ir2_extrinsics = depth_profile.get_extrinsics_to(_enabled_profiles[INFRA2].front());
        auto q = rotationMatrixToQuaternion(d2ir2_extrinsics.rotation);

        // Transform base frame to infra2
        b2ir2_msg.header.stamp = transform_ts_;
        b2ir2_msg.header.frame_id = _base_frame_id;
        b2ir2_msg.child_frame_id = _frame_id[INFRA2];
        b2ir2_msg.transform.translation.x = d2ir2_extrinsics.translation[2];
        b2ir2_msg.transform.translation.y = -d2ir2_extrinsics.translation[0];
        b2ir2_msg.transform.translation.z = -d2ir2_extrinsics.translation[1];
        b2ir2_msg.transform.rotation.x = q.x();
        b2ir2_msg.transform.rotation.y = q.y();
        b2ir2_msg.transform.rotation.z = q.z();
        b2ir2_msg.transform.rotation.w = q.w();
        _static_tf_broadcaster.sendTransform(b2ir2_msg);

        // Transform infra2 frame to infra1 optical frame
        ir2_2_ir2o.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
        ir2_2_ir2o_msg.header.stamp = transform_ts_;
        ir2_2_ir2o_msg.header.frame_id = _frame_id[INFRA2];
        ir2_2_ir2o_msg.child_frame_id = _optical_frame_id[INFRA2];
        ir2_2_ir2o_msg.transform.translation.x = 0;
        ir2_2_ir2o_msg.transform.translation.y = 0;
        ir2_2_ir2o_msg.transform.translation.z = 0;
        ir2_2_ir2o_msg.transform.rotation.x = ir2_2_ir2o.getX();
        ir2_2_ir2o_msg.transform.rotation.y = ir2_2_ir2o.getY();
        ir2_2_ir2o_msg.transform.rotation.z = ir2_2_ir2o.getZ();
        ir2_2_ir2o_msg.transform.rotation.w = ir2_2_ir2o.getW();
        _static_tf_broadcaster.sendTransform(ir2_2_ir2o_msg);
    }
    // TODO: Publish Fisheye TF
}

void BaseRealSenseNode::publishPCTopic(const ros::Time& t)
{
    auto color_intrinsics = _stream_intrinsics[COLOR];
    auto image_depth16 = reinterpret_cast<const uint16_t*>(_image[DEPTH].data);
    auto depth_intrinsics = _stream_intrinsics[DEPTH];
    sensor_msgs::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = t;
    msg_pointcloud.header.frame_id = _optical_frame_id[DEPTH];
    msg_pointcloud.width = depth_intrinsics.width;
    msg_pointcloud.height = depth_intrinsics.height;
    msg_pointcloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);

    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "rgb", 1, sensor_msgs::PointField::FLOAT32);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");

    sensor_msgs::PointCloud2Iterator<uint8_t>iter_r(msg_pointcloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t>iter_g(msg_pointcloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t>iter_b(msg_pointcloud, "b");

    float depth_point[3], color_point[3], color_pixel[2], scaled_depth;
    unsigned char* color_data = _image[COLOR].data;

    // Fill the PointCloud2 fields
    for (int y = 0; y < depth_intrinsics.height; ++y)
    {
        for (int x = 0; x < depth_intrinsics.width; ++x)
        {
            scaled_depth = static_cast<float>(*image_depth16) * _depth_scale_meters;
            float depth_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
            rs2_deproject_pixel_to_point(depth_point, &depth_intrinsics, depth_pixel, scaled_depth);

            if (depth_point[2] <= 0.f || depth_point[2] > 5.f)
            {
                depth_point[0] = 0.f;
                depth_point[1] = 0.f;
                depth_point[2] = 0.f;
            }

            *iter_x = depth_point[0];
            *iter_y = depth_point[1];
            *iter_z = depth_point[2];

            rs2_transform_point_to_point(color_point, &_depth2color_extrinsics, depth_point);
            rs2_project_point_to_pixel(color_pixel, &color_intrinsics, color_point);

            if (color_pixel[1] < 0.f || color_pixel[1] > color_intrinsics.height
                || color_pixel[0] < 0.f || color_pixel[0] > color_intrinsics.width)
            {
                // For out of bounds color data, default to a shade of blue in order to visually distinguish holes.
                // This color value is same as the librealsense out of bounds color value.
                *iter_r = static_cast<uint8_t>(96);
                *iter_g = static_cast<uint8_t>(157);
                *iter_b = static_cast<uint8_t>(198);
            }
            else
            {
                auto i = static_cast<int>(color_pixel[0]);
                auto j = static_cast<int>(color_pixel[1]);

                auto offset = i * 3 + j * color_intrinsics.width * 3;
                *iter_r = static_cast<uint8_t>(color_data[offset]);
                *iter_g = static_cast<uint8_t>(color_data[offset + 1]);
                *iter_b = static_cast<uint8_t>(color_data[offset + 2]);
            }

            ++image_depth16;
            ++iter_x; ++iter_y; ++iter_z;
            ++iter_r; ++iter_g; ++iter_b;
        }
    }
    _pointcloud_publisher.publish(msg_pointcloud);
}

Extrinsics BaseRealSenseNode::rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics) const
{
    Extrinsics extrinsicsMsg;
    for (int i = 0; i < 9; ++i)
    {
        extrinsicsMsg.rotation[i] = extrinsics.rotation[i];
        if (i < 3)
            extrinsicsMsg.translation[i] = extrinsics.translation[i];
    }

    return extrinsicsMsg;
}

Extrinsics BaseRealSenseNode::getFisheye2ImuExtrinsicsMsg()
{
    auto& fisheye = _enabled_profiles[FISHEYE].front();
    auto& hid = _enabled_profiles[GYRO].front();
    Extrinsics extrinsicsMsg = rsExtrinsicsToMsg(fisheye.get_extrinsics_to(hid));
    extrinsicsMsg.header.frame_id = "fisheye2imu_extrinsics";
    return extrinsicsMsg;
}

Extrinsics BaseRealSenseNode::getFisheye2DepthExtrinsicsMsg()
{
    auto& fisheye = _enabled_profiles[FISHEYE].front();
    auto& depth = _enabled_profiles[DEPTH].front();
    Extrinsics extrinsicsMsg = rsExtrinsicsToMsg(fisheye.get_extrinsics_to(depth));
    extrinsicsMsg.header.frame_id = "fisheye2depth_extrinsics";
    return extrinsicsMsg;
}

IMUInfo BaseRealSenseNode::getImuInfo(const stream_index_pair& stream_index)
{
    IMUInfo info{};
    auto imuIntrinsics = _sensors[stream_index].get_motion_intrinsics(stream_index.first);
    if (GYRO == stream_index)
    {
        info.header.frame_id = "imu_gyro";
    }
    else if (ACCEL == stream_index)
    {
        info.header.frame_id = "imu_accel";
    }

    auto index = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            info.data[index] = imuIntrinsics.data[i][j];
            ++index;
        }
        info.noise_variances[i] =  imuIntrinsics.noise_variances[i];
        info.bias_variances[i] = imuIntrinsics.bias_variances[i];
    }
    return info;
}

void BaseRealSenseNode::publishFrame(rs2::frame f, const ros::Time& t)
{
    ROS_DEBUG("publishFrame(...)");
    stream_index_pair stream{f.get_profile().stream_type(), f.get_profile().stream_index()};
    auto& image = _image[stream];
    image.data = (uint8_t*)f.get_data();
    ++(_seq[stream]);
    auto& info_publisher = _info_publisher[stream];
    auto& image_publisher = _image_publishers[stream];
    if(0 != info_publisher.getNumSubscribers() ||
       0 != image_publisher.getNumSubscribers())
    {
        auto width = 0;
        auto height = 0;
        auto bpp = 1;
        if (f.is<rs2::video_frame>())
        {
            auto image = f.as<rs2::video_frame>();
            width = image.get_width();
            height = image.get_height();
            bpp = image.get_bytes_per_pixel();
        }

        sensor_msgs::ImagePtr img;
        img = cv_bridge::CvImage(std_msgs::Header(), _encoding[stream], image).toImageMsg();
        img->width = width;
        img->height = height;
        img->is_bigendian = false;
        img->step = width * bpp;
        img->header.frame_id = _optical_frame_id[stream];
        img->header.stamp = t;
        img->header.seq = _seq[stream];

        auto& cam_info = _camera_info[stream];
        cam_info.header.stamp = t;
        cam_info.header.seq = _seq[stream];
        info_publisher.publish(cam_info);

        image_publisher.publish(img);
        ROS_DEBUG("%s stream published", rs2_stream_to_string(f.get_profile().stream_type()));
    }
}

bool BaseRealSenseNode::getEnabledProfile(const stream_index_pair& stream_index, rs2::stream_profile& profile)
    {
        // Assuming that all D400 SKUs have depth sensor
        auto profiles = _enabled_profiles[stream_index];
        auto it = std::find_if(profiles.begin(), profiles.end(),
                               [&](const rs2::stream_profile& profile)
                               { return (profile.stream_type() == stream_index.first); });
        if (it == profiles.end())
            return false;

        profile =  *it;
        return true;
    }


BaseD400Node::BaseD400Node(ros::NodeHandle& nodeHandle,
                           ros::NodeHandle& privateNodeHandle,
                           rs2::device dev, const std::string& serial_no)
    : BaseRealSenseNode(nodeHandle,
                        privateNodeHandle,
                        dev, serial_no)
{}

void BaseD400Node::callback(base_d400_paramsConfig &config, uint32_t level)
{
    ROS_DEBUG_STREAM("D400 - Level: " << level);

    if (set_default_dynamic_reconfig_values == level)
    {
        for (int i = 1 ; i < base_depth_count ; ++i)
            setParam(config ,(base_depth_param)i);
    }
    else
    {
        setParam(config, (base_depth_param)level);
    }
}

void BaseD400Node::setOption(stream_index_pair sip, rs2_option opt, float val)
{
    _sensors[sip].set_option(opt, val);
}

void BaseD400Node::setParam(rs435_paramsConfig &config, base_depth_param param)
{
    base_d400_paramsConfig base_config;
    base_config.base_depth_gain = config.rs435_depth_gain;
    base_config.base_depth_enable_auto_exposure = config.rs435_depth_enable_auto_exposure;
    base_config.base_depth_visual_preset = config.rs435_depth_visual_preset;
    base_config.base_depth_frames_queue_size = config.rs435_depth_frames_queue_size;
    base_config.base_depth_error_polling_enabled = config.rs435_depth_error_polling_enabled;
    base_config.base_depth_output_trigger_enabled = config.rs435_depth_output_trigger_enabled;
    base_config.base_depth_units = config.rs435_depth_units;
    base_config.base_JSON_file_path = config.rs435_JSON_file_path;
    setParam(base_config, param);
}

void BaseD400Node::setParam(rs415_paramsConfig &config, base_depth_param param)
{
    base_d400_paramsConfig base_config;
    base_config.base_depth_gain = config.rs415_depth_gain;
    base_config.base_depth_enable_auto_exposure = config.rs415_depth_enable_auto_exposure;
    base_config.base_depth_visual_preset = config.rs415_depth_visual_preset;
    base_config.base_depth_frames_queue_size = config.rs415_depth_frames_queue_size;
    base_config.base_depth_error_polling_enabled = config.rs415_depth_error_polling_enabled;
    base_config.base_depth_output_trigger_enabled = config.rs415_depth_output_trigger_enabled;
    base_config.base_depth_units = config.rs415_depth_units;
    base_config.base_JSON_file_path = config.rs415_JSON_file_path;
    setParam(base_config, param);
}

void BaseD400Node::setParam(base_d400_paramsConfig &config, base_depth_param param)
{
    // W/O for zero param
    if (0 == param)
        return;

    switch (param) {
    case base_depth_gain:
        setOption(DEPTH, RS2_OPTION_GAIN, config.base_depth_gain);
        break;
    case base_depth_enable_auto_exposure:
        setOption(DEPTH, RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.base_depth_enable_auto_exposure);
        break;
    case base_depth_visual_preset:
        setOption(DEPTH, RS2_OPTION_VISUAL_PRESET, config.base_depth_visual_preset);
        break;
    case base_depth_frames_queue_size:
        setOption(DEPTH, RS2_OPTION_FRAMES_QUEUE_SIZE, config.base_depth_frames_queue_size);
        break;
    case base_depth_error_polling_enabled:
        setOption(DEPTH, RS2_OPTION_ERROR_POLLING_ENABLED, config.base_depth_error_polling_enabled);
        break;
    case base_depth_output_trigger_enabled:
        setOption(DEPTH, RS2_OPTION_OUTPUT_TRIGGER_ENABLED, config.base_depth_output_trigger_enabled);
        break;
    case base_depth_units:
        break;
    case base_JSON_file_path:
    {
        auto adv_dev = _dev.as<rs400::advanced_mode>();
        if (!adv_dev)
        {
            ROS_WARN_STREAM("Device doesn't support Advanced Mode!");
            return;
        }
        if (!config.base_JSON_file_path.empty())
        {
            std::ifstream in(config.base_JSON_file_path);
            if (!in.is_open())
            {
                ROS_WARN_STREAM("JSON file provided doesn't exist!");
                return;
            }

            adv_dev.load_json(config.base_JSON_file_path);
        }
        break;
    }
    default:
        ROS_WARN_STREAM("Unrecognized D400 param (" << param << ")");
        break;
    }
}

void BaseD400Node::registerDynamicReconfigCb()
{
    _server = std::make_shared<dynamic_reconfigure::Server<base_d400_paramsConfig>>();
    _f = boost::bind(&BaseD400Node::callback, this, _1, _2);
    _server->setCallback(_f);
}
