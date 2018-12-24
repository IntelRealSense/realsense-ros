#include "../include/base_realsense_node.h"
#include "assert.h"
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <mutex>
#include <sys/time.h>

using namespace realsense2_camera;
using namespace ddynamic_reconfigure;

SyncedImuPublisher::SyncedImuPublisher(ros::Publisher imu_publisher, std::size_t waiting_list_size):
            _publisher(imu_publisher),
            _waiting_list_size(waiting_list_size)
            {}

SyncedImuPublisher::~SyncedImuPublisher()
{
    PublishPendingMessages();
}

void SyncedImuPublisher::Publish(sensor_msgs::Imu imu_msg)
{
    std::lock_guard<std::mutex> lock_guard(_mutex);
    if (_pause_mode)
    {
        if (_pendeing_messages.size() >= _waiting_list_size)
        {
            throw std::runtime_error("SyncedImuPublisher inner list reached maximum size of " + std::to_string(_pendeing_messages.size()));
        }
        _pendeing_messages.push(imu_msg);
    }
    else
    {
        _publisher.publish(imu_msg);
        // ROS_INFO_STREAM("iid1:" << imu_msg.header.seq << ", time: " << std::setprecision (20) << imu_msg.header.stamp.toSec());
    }
    return;
}

void SyncedImuPublisher::Pause()
{
    if (!_is_enabled) return;
    std::lock_guard<std::mutex> lock_guard(_mutex);
    _pause_mode = true;
}

void SyncedImuPublisher::Resume()
{
    std::lock_guard<std::mutex> lock_guard(_mutex);
    PublishPendingMessages();
    _pause_mode = false;
}

void SyncedImuPublisher::PublishPendingMessages()
{
    // ROS_INFO_STREAM("publish imu: " << _pendeing_messages.size());
    while (!_pendeing_messages.empty())
    {
        const sensor_msgs::Imu &imu_msg = _pendeing_messages.front();
        _publisher.publish(imu_msg);
        // ROS_INFO_STREAM("iid2:" << imu_msg.header.seq << ", time: " << std::setprecision (20) << imu_msg.header.stamp.toSec());
        _pendeing_messages.pop();
    }
}

std::string BaseRealSenseNode::getNamespaceStr()
{
    auto ns = ros::this_node::getNamespace();
    ns.erase(std::remove(ns.begin(), ns.end(), '/'), ns.end());
    return ns;
}

BaseRealSenseNode::BaseRealSenseNode(ros::NodeHandle& nodeHandle,
                                     ros::NodeHandle& privateNodeHandle,
                                     rs2::device dev,
                                     const std::string& serial_no) :
    _dev(dev),  _node_handle(nodeHandle),
    _pnh(privateNodeHandle), _json_file_path(""),
    _serial_no(serial_no), _base_frame_id(""),
    _intialize_time_base(false),
    _namespace(getNamespaceStr())
{
    // Types for depth stream
    _is_frame_arrived[DEPTH] = false;
    _format[DEPTH] = RS2_FORMAT_Z16;   // libRS type
    _image_format[DEPTH] = CV_16UC1;    // CVBridge type
    _encoding[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
    _unit_step_size[DEPTH] = sizeof(uint16_t); // sensor_msgs::ImagePtr row step size
    _stream_name[DEPTH] = "depth";
    _depth_aligned_encoding[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Infrared stream - Left
    _is_frame_arrived[INFRA1] = false;
    _format[INFRA1] = RS2_FORMAT_Y8;   // libRS type
    _image_format[INFRA1] = CV_8UC1;    // CVBridge type
    _encoding[INFRA1] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[INFRA1] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[INFRA1] = "infra1";
    _depth_aligned_encoding[INFRA1] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Infrared stream - Right
    _is_frame_arrived[INFRA2] = false;
    _format[INFRA2] = RS2_FORMAT_Y8;   // libRS type
    _image_format[INFRA2] = CV_8UC1;    // CVBridge type
    _encoding[INFRA2] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[INFRA2] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[INFRA2] = "infra2";
    _depth_aligned_encoding[INFRA2] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Types for color stream
    _is_frame_arrived[COLOR] = false;
    _format[COLOR] = RS2_FORMAT_RGB8;   // libRS type
    _image_format[COLOR] = CV_8UC3;    // CVBridge type
    _encoding[COLOR] = sensor_msgs::image_encodings::RGB8; // ROS message type
    _unit_step_size[COLOR] = 3; // sensor_msgs::ImagePtr row step size
    _stream_name[COLOR] = "color";
    _depth_aligned_encoding[COLOR] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Types for fisheye stream
    _is_frame_arrived[FISHEYE] = false;
    _format[FISHEYE] = RS2_FORMAT_RAW8;   // libRS type
    _image_format[FISHEYE] = CV_8UC1;    // CVBridge type
    _encoding[FISHEYE] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[FISHEYE] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[FISHEYE] = "fisheye";
    _depth_aligned_encoding[FISHEYE] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Types for Motion-Module streams
    _is_frame_arrived[GYRO] = false;
    _format[GYRO] = RS2_FORMAT_MOTION_XYZ32F;   // libRS type
    _image_format[GYRO] = CV_8UC1;    // CVBridge type
    _encoding[GYRO] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[GYRO] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[GYRO] = "gyro";

    _is_frame_arrived[ACCEL] = false;
    _format[ACCEL] = RS2_FORMAT_MOTION_XYZ32F;   // libRS type
    _image_format[ACCEL] = CV_8UC1;    // CVBridge type
    _encoding[ACCEL] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[ACCEL] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[ACCEL] = "accel";
}

void BaseRealSenseNode::toggleSensors(bool enabled)
{
    for (auto it=_sensors.begin(); it != _sensors.end(); it++)
    {
        auto& sens = _sensors[it->first];
        try
        {
            if (enabled)
                sens.start(_syncer);
            else
                sens.stop();
        }
        catch(const rs2::wrong_api_call_sequence_error& ex)
        {
            ROS_DEBUG_STREAM("toggleSensors: " << ex.what());
        }
    }
}

void BaseRealSenseNode::setupErrorCallback()
{
    for (auto&& s : _dev.query_sensors())
    {
        s.set_notifications_callback([&](const rs2::notification& n)
        {
            if (n.get_severity() >= RS2_LOG_SEVERITY_ERROR)
            {
                ROS_ERROR_STREAM("Hardware Notification:" << n.get_description() << "," << n.get_timestamp() << "," << n.get_severity() << "," << n.get_category());
            }
            if (n.get_description().find("RT IC2 Config error") != std::string::npos)
            {
                ROS_ERROR_STREAM("Hardware Reset is needed.");
                // _dev.hardware_reset();
            }
        });
    }
}

void BaseRealSenseNode::publishTopics()
{
    getParameters();
    setupDevice();
    setupErrorCallback();
    setupPublishers();
    setupStreams();
    setupFilters();
    publishStaticTransforms();
    ROS_INFO_STREAM("RealSense Node Is Up!");
}

bool is_checkbox(rs2::options sensor, rs2_option option)
{
    rs2::option_range op_range = sensor.get_option_range(option);
    return op_range.max == 1.0f &&
        op_range.min == 0.0f &&
        op_range.step == 1.0f;
}

bool is_enum_option(rs2::options sensor, rs2_option option)
{
    rs2::option_range op_range = sensor.get_option_range(option);
    if (op_range.step < 0.001f) return false;
    for (auto i = op_range.min; i <= op_range.max; i += op_range.step)
    {
        if (sensor.get_option_value_description(option, i) == nullptr)
            return false;
    }
    return true;
}

bool is_int_option(rs2::options sensor, rs2_option option)
{
    rs2::option_range op_range = sensor.get_option_range(option);
    return (op_range.step == 1.0);
}

std::map<std::string, int> get_enum_method(rs2::options sensor, rs2_option option)
{
    std::map<std::string, int> dict; // An enum to set size
    if (is_enum_option(sensor, option))
    {
        rs2::option_range op_range = sensor.get_option_range(option);
        for (auto val = op_range.min; val <= op_range.max; val += op_range.step)
        {
            dict[sensor.get_option_value_description(option, val)] = int(val);
        }
    }
    return dict;
}

void BaseRealSenseNode::registerDynamicOption(ros::NodeHandle& nh, rs2::options sensor, std::string& module_name)
{
    ros::NodeHandle nh1(module_name);
    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh1);
    for (auto i = 0; i < RS2_OPTION_COUNT; i++)
    {
        rs2_option option = static_cast<rs2_option>(i);
        if (!sensor.supports(option) || sensor.is_option_read_only(option))
        {
            continue;
        }
        if (is_checkbox(sensor, option))
        {
            ddynrec->add(new DDBool(rs2_option_to_string(option), i, sensor.get_option_description(option), bool(sensor.get_option(option))));
            continue;
        }
        std::map<std::string, int> enum_dict = get_enum_method(sensor, option);
        if (enum_dict.empty())
        {
            rs2::option_range op_range = sensor.get_option_range(option);
            if (is_int_option(sensor, option))
            {
                ddynrec->add(new DDInt(rs2_option_to_string(option), i, sensor.get_option_description(option), sensor.get_option(option), op_range.min, op_range.max));
            }
            else
            {
                ddynrec->add(new DDDouble(rs2_option_to_string(option), i, sensor.get_option_description(option), sensor.get_option(option), op_range.min, op_range.max));
            }
        }
        else
        {
            ROS_DEBUG_STREAM("Add enum: " << rs2_option_to_string(option) << ". value=" << int(sensor.get_option(option)));
            ddynrec->add(new DDEnum(rs2_option_to_string(option), i, sensor.get_option_description(option), int(sensor.get_option(option)), enum_dict));
        }
    }
    ddynrec->start(boost::bind(callback, _1, _2, sensor));
    _ddynrec.push_back(ddynrec);
}

void BaseRealSenseNode::registerDynamicReconfigCb(ros::NodeHandle& nh)
{
    ROS_INFO("Setting Dynamic reconfig parameters.");

    for(rs2::sensor sensor : _dev_sensors)
    {
        std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
        std::replace( module_name.begin(), module_name.end(), '-', '_');
        std::replace( module_name.begin(), module_name.end(), ' ', '_'); // replace all ' ' to '_'
        ROS_DEBUG_STREAM("module_name:" << module_name);
        registerDynamicOption(nh, sensor, module_name);
    }

    for (NamedFilter nfilter : _filters)
    {
        std::string module_name = nfilter._name;
        auto sensor = *(nfilter._filter);
        ROS_DEBUG_STREAM("module_name:" << module_name);
        registerDynamicOption(nh, sensor, module_name);
    }
    ROS_INFO("Done Setting Dynamic reconfig parameters.");
}

void BaseRealSenseNode::callback(const ddynamic_reconfigure::DDMap& map, int level, rs2::options sensor) {
    rs2_option option = static_cast<rs2_option>(level);
    double value = get(map, rs2_option_to_string(option)).toDouble();
    ROS_DEBUG_STREAM("option: " << rs2_option_to_string(option) << ". value: " << value);
    sensor.set_option(option, value);
}

rs2_stream BaseRealSenseNode::rs2_string_to_stream(std::string str)
{
    if (str == "RS2_STREAM_ANY")
        return RS2_STREAM_ANY;
    if (str == "RS2_STREAM_COLOR")
        return RS2_STREAM_COLOR;
    if (str == "RS2_STREAM_INFRARED")
        return RS2_STREAM_INFRARED;
    if (str == "RS2_STREAM_FISHEYE")
        return RS2_STREAM_FISHEYE;
    throw std::runtime_error("Unknown stream string " + str);
}

void BaseRealSenseNode::getParameters()
{
    ROS_INFO("getParameters...");

    _pnh.param("align_depth", _align_depth, ALIGN_DEPTH);
    _pnh.param("enable_pointcloud", _pointcloud, POINTCLOUD);
    std::string pc_texture_stream("");
    int pc_texture_idx;
    _pnh.param("pointcloud_texture_stream", pc_texture_stream, std::string("RS2_STREAM_COLOR"));
    _pnh.param("pointcloud_texture_index", pc_texture_idx, 0);
    _pointcloud_texture = stream_index_pair{rs2_string_to_stream(pc_texture_stream), pc_texture_idx};

    _pnh.param("filters", _filters_str, DEFAULT_FILTERS);
    _pointcloud |= (_filters_str.find("pointcloud") != std::string::npos);

    _pnh.param("enable_sync", _sync_frames, SYNC_FRAMES);
    if (_pointcloud || _align_depth || _filters_str.size() > 0)
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
    _pnh.param("unite_imu", _unite_imu, UNITE_IMU);

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
    if (_unite_imu)
    {
        _pnh.param("imu_optical_frame_id", _optical_frame_id[GYRO], DEFAULT_IMU_OPTICAL_FRAME_ID);
    }
    else
    {
        _pnh.param("gyro_optical_frame_id", _optical_frame_id[GYRO], DEFAULT_GYRO_OPTICAL_FRAME_ID);
    }
    _pnh.param("accel_optical_frame_id", _optical_frame_id[ACCEL], DEFAULT_ACCEL_OPTICAL_FRAME_ID);

    _pnh.param("aligned_depth_to_color_frame_id",   _depth_aligned_frame_id[COLOR],   DEFAULT_ALIGNED_DEPTH_TO_COLOR_FRAME_ID);
    _pnh.param("aligned_depth_to_infra1_frame_id",  _depth_aligned_frame_id[INFRA1],  DEFAULT_ALIGNED_DEPTH_TO_INFRA1_FRAME_ID);
    _pnh.param("aligned_depth_to_infra2_frame_id",  _depth_aligned_frame_id[INFRA2],  DEFAULT_ALIGNED_DEPTH_TO_INFRA2_FRAME_ID);
    _pnh.param("aligned_depth_to_fisheye_frame_id", _depth_aligned_frame_id[FISHEYE], DEFAULT_ALIGNED_DEPTH_TO_FISHEYE_FRAME_ID);
    _pnh.param("clip_distance", _clipping_distance, static_cast<float>(-1.0));
    _pnh.param("linear_accel_cov", _linear_accel_cov, static_cast<float>(0.01));
    _pnh.param("hold_back_imu_for_frames", _hold_back_imu_for_frames, HOLD_BACK_IMU_FOR_FRAMES);
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

        ROS_INFO_STREAM("ROS Node Namespace: " << _namespace);

        auto camera_name = _dev.get_info(RS2_CAMERA_INFO_NAME);
        ROS_INFO_STREAM("Device Name: " << camera_name);

        ROS_INFO_STREAM("Device Serial No: " << _serial_no);

        auto fw_ver = _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
        ROS_INFO_STREAM("Device FW version: " << fw_ver);

        auto pid = _dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
        ROS_INFO_STREAM("Device Product ID: 0x" << pid);

        ROS_INFO_STREAM("Enable PointCloud: " << ((_pointcloud)?"On":"Off"));
        ROS_INFO_STREAM("Align Depth: " << ((_align_depth)?"On":"Off"));
        ROS_INFO_STREAM("Sync Mode: " << ((_sync_frames)?"On":"Off"));

        _dev_sensors = _dev.query_sensors();

        ROS_INFO_STREAM("Device Sensors: ");
        for(auto&& elem : _dev_sensors)
        {
            std::string module_name = elem.get_info(RS2_CAMERA_INFO_NAME);
            if ("Stereo Module" == module_name)
            {
                _sensors[DEPTH] = elem;
                _sensors[INFRA1] = elem;
                _sensors[INFRA2] = elem;
            }
            else if ("Coded-Light Depth Sensor" == module_name)
            {
                _sensors[DEPTH] = elem;
                _sensors[INFRA1] = elem;
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
                ROS_ERROR_STREAM("Module Name \"" << module_name << "\" isn't supported by LibRealSense! Terminating RealSense Node...");
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
                    ROS_INFO_STREAM("(" << rs2_stream_to_string(stream_index.first) << ", " << stream_index.second << ") sensor isn't supported by current device! -- Skipping...");
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
            bool rectified_image = false;
            if (stream == DEPTH || stream == INFRA1 || stream == INFRA2)
                rectified_image = true;

            image_raw << _stream_name[stream] << "/image_" << ((rectified_image)?"rect_":"") << "raw";
            camera_info << _stream_name[stream] << "/camera_info";

            std::shared_ptr<FrequencyDiagnostics> frequency_diagnostics(new FrequencyDiagnostics(_fps[stream], _stream_name[stream], _serial_no));
            _image_publishers[stream] = {image_transport.advertise(image_raw.str(), 1), frequency_diagnostics};
            _info_publisher[stream] = _node_handle.advertise<sensor_msgs::CameraInfo>(camera_info.str(), 1);

            if (_align_depth && (stream != DEPTH))
            {
                std::stringstream aligned_image_raw, aligned_camera_info;
                aligned_image_raw << "aligned_depth_to_" << _stream_name[stream] << "/image_raw";
                aligned_camera_info << "aligned_depth_to_" << _stream_name[stream] << "/camera_info";

                std::string aligned_stream_name = "aligned_depth_to_" + _stream_name[stream];
                std::shared_ptr<FrequencyDiagnostics> frequency_diagnostics(new FrequencyDiagnostics(_fps[stream], aligned_stream_name, _serial_no));
                _depth_aligned_image_publishers[stream] = {image_transport.advertise(aligned_image_raw.str(), 1), frequency_diagnostics};
                _depth_aligned_info_publisher[stream] = _node_handle.advertise<sensor_msgs::CameraInfo>(aligned_camera_info.str(), 1);
            }

            if (stream == DEPTH && _pointcloud)
            {
                _pointcloud_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("depth/color/points", 1);
            }
        }
    }

    if (_enable[FISHEYE] &&
        _enable[DEPTH])
    {
        _depth_to_other_extrinsics_publishers[FISHEYE] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_fisheye", 1, true);
    }

    if (_enable[COLOR] &&
        _enable[DEPTH])
    {
        _depth_to_other_extrinsics_publishers[COLOR] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_color", 1, true);
    }

    if (_enable[INFRA1] &&
        _enable[DEPTH])
    {
        _depth_to_other_extrinsics_publishers[INFRA1] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_infra1", 1, true);
    }

    if (_enable[INFRA2] &&
        _enable[DEPTH])
    {
        _depth_to_other_extrinsics_publishers[INFRA2] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_infra2", 1, true);
    }

    _synced_imu_publisher = std::make_shared<SyncedImuPublisher>();
    if (_unite_imu && _enable[GYRO] && _enable[ACCEL])
    {
        ROS_INFO("Start publisher IMU");
        _synced_imu_publisher = std::make_shared<SyncedImuPublisher>(_node_handle.advertise<sensor_msgs::Imu>("imu", 1));
        _synced_imu_publisher->Enable(_hold_back_imu_for_frames);

        _info_publisher[GYRO] = _node_handle.advertise<IMUInfo>("imu_info", 1, true);
    }
    else
    {
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
    }
}

void BaseRealSenseNode::updateIsFrameArrived(std::map<stream_index_pair, bool>& is_frame_arrived,
                                             rs2_stream stream_type, int stream_index)
{
    try
    {
        is_frame_arrived.at({stream_type, stream_index}) = true;
    }
    catch (std::out_of_range)
    {
        ROS_ERROR_STREAM("Stream type is not supported! (" << stream_type << ", " << stream_index << ")");
    }
}

void BaseRealSenseNode::publishAlignedDepthToOthers(rs2::frameset frames, const ros::Time& t)
{
    for (auto it = frames.begin(); it != frames.end(); ++it)
    {
        auto frame = (*it);
        auto stream_type = frame.get_profile().stream_type();

        if (RS2_STREAM_DEPTH == stream_type)
            continue;

        auto stream_index = frame.get_profile().stream_index();
        stream_index_pair sip{stream_type, stream_index};
        auto& info_publisher = _depth_aligned_info_publisher.at(sip);
        auto& image_publisher = _depth_aligned_image_publishers.at(sip);

        if(0 != info_publisher.getNumSubscribers() ||
           0 != image_publisher.first.getNumSubscribers())
        {
            rs2::align align(stream_type);
            rs2::frameset processed = frames.apply_filter(align);
            rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

            publishFrame(aligned_depth_frame, t, sip,
                         _depth_aligned_image,
                         _depth_aligned_info_publisher,
                         _depth_aligned_image_publishers, _depth_aligned_seq,
                         _depth_aligned_camera_info, _optical_frame_id,
                         _depth_aligned_encoding);
        }
    }
}

void BaseRealSenseNode::enable_devices()
{
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
					ROS_DEBUG_STREAM("Sensor profile: " <<
									 "Format: " << video_profile.format() <<
									 ", Width: " << video_profile.width() <<
									 ", Height: " << video_profile.height() <<
									 ", FPS: " << video_profile.fps());

					if (video_profile.format() == _format[elem] &&
						(_width[elem] == 0 || video_profile.width() == _width[elem]) &&
						(_height[elem] == 0 || video_profile.height() == _height[elem]) &&
						(_fps[elem] == 0 || video_profile.fps() == _fps[elem]) &&
						video_profile.stream_index() == elem.second)
					{
						_width[elem] = video_profile.width();
						_height[elem] = video_profile.height();
						_fps[elem] = video_profile.fps();

						_enabled_profiles[elem].push_back(profile);

						_image[elem] = cv::Mat(_height[elem], _width[elem], _image_format[elem], cv::Scalar(0, 0, 0));

						ROS_INFO_STREAM(_stream_name[elem] << " stream is enabled - width: " << _width[elem] << ", height: " << _height[elem] << ", fps: " << _fps[elem]);
						break;
					}
				}
				if (_enabled_profiles.find(elem) == _enabled_profiles.end())
				{
					ROS_WARN_STREAM("Given stream configuration is not supported by the device! " <<
						" Stream: " << rs2_stream_to_string(elem.first) <<
						", Stream Index: " << elem.second <<
						", Format: " << _format[elem] <<
						", Width: " << _width[elem] <<
						", Height: " << _height[elem] <<
						", FPS: " << _fps[elem]);
					_enable[elem] = false;
				}
			}
		}
	}
	if (_align_depth)
	{
		for (auto& profiles : _enabled_profiles)
		{
			_depth_aligned_image[profiles.first] = cv::Mat(_height[DEPTH], _width[DEPTH], _image_format[DEPTH], cv::Scalar(0, 0, 0));
		}
	}
}

void BaseRealSenseNode::setupFilters()
{
    std::vector<std::string> filters_str;
    boost::split(filters_str, _filters_str, [](char c){return c == ',';});
    bool use_disparity_filter(false);
    bool use_colorizer_filter(false);
    for (std::vector<std::string>::const_iterator s_iter=filters_str.begin(); s_iter!=filters_str.end(); s_iter++)
    {
        if ((*s_iter) == "colorizer")
        {
            use_colorizer_filter = true;
        }
        else if ((*s_iter) == "disparity")
        {
            use_disparity_filter = true;
        }
        else if ((*s_iter) == "spatial")
        {
            ROS_INFO("Add Filter: spatial");
            _filters.push_back(NamedFilter("spatial", std::make_shared<rs2::spatial_filter>()));
        }
        else if ((*s_iter) == "temporal")
        {
            ROS_INFO("Add Filter: temporal");
            _filters.push_back(NamedFilter("temporal", std::make_shared<rs2::temporal_filter>()));
        }
        else if ((*s_iter) == "decimation")
        {
            ROS_INFO("Add Filter: decimation");
            _filters.push_back(NamedFilter("decimation", std::make_shared<rs2::decimation_filter>()));
        }
        else if ((*s_iter) == "pointcloud")
        {
            assert(_pointcloud); // For now, it is set in getParameters()..
        }
        else if ((*s_iter).size() > 0)
        {
            ROS_ERROR_STREAM("Unknown Filter: " << (*s_iter));
            throw;
        }
    }
    if (use_disparity_filter)
    {
        ROS_INFO("Add Filter: disparity");
        _filters.insert(_filters.begin(), NamedFilter("disparity_start", std::make_shared<rs2::disparity_transform>()));
        _filters.push_back(NamedFilter("disparity_end", std::make_shared<rs2::disparity_transform>(false)));
        ROS_INFO("Done Add Filter: disparity");
    }
    if (use_colorizer_filter)
    {
        ROS_INFO("Add Filter: colorizer");
        _filters.push_back(NamedFilter("colorizer", std::make_shared<rs2::colorizer>()));

        // Types for depth stream
        _format[DEPTH] = _format[COLOR];   // libRS type
        _image_format[DEPTH] = _image_format[COLOR];    // CVBridge type
        _encoding[DEPTH] = _encoding[COLOR]; // ROS message type
        _unit_step_size[DEPTH] = _unit_step_size[COLOR]; // sensor_msgs::ImagePtr row step size

        _width[DEPTH] = _width[COLOR];
        _height[DEPTH] = _height[COLOR];
        _image[DEPTH] = cv::Mat(_height[DEPTH], _width[DEPTH], _image_format[DEPTH], cv::Scalar(0, 0, 0));
    }
    if (_pointcloud)
    {
    	ROS_INFO("Add Filter: pointcloud");
        _filters.push_back(NamedFilter("pointcloud", std::make_shared<rs2::pointcloud>(_pointcloud_texture.first, _pointcloud_texture.second)));
    }
    ROS_INFO("num_filters: %d", static_cast<int>(_filters.size()));
}

void BaseRealSenseNode::clip_depth(rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist)
{
    uint16_t* p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));

    int width = depth_frame.get_width();
    int height = depth_frame.get_height();

    #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Get the depth value of the current pixel
            auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

            // Check if the depth value is greater than the threashold
            if (pixels_distance > clipping_dist)
            {
                p_depth_frame[depth_pixel_index] = -1; //Set to invalid (<=0) value.
            }
        }
    }
}

void BaseRealSenseNode::setupStreams()
{
	ROS_INFO("setupStreams...");
	enable_devices();
    try{
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
            _synced_imu_publisher->Pause();
            try{
                double frame_time = frame.get_timestamp();

                // We compute a ROS timestamp which is based on an initial ROS time at point of first frame,
                // and the incremental timestamp from the camera.
                // In sync mode the timestamp is based on ROS time
                if (false == _intialize_time_base)
                {
                    if (RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME == frame.get_frame_timestamp_domain())
                        ROS_WARN("Frame metadata isn't available! (frame_timestamp_domain = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME)");

                    _intialize_time_base = true;
                    _ros_time_base = ros::Time::now();
                    _camera_time_base = frame_time;
                }

                ros::Time t;
                if (_sync_frames)
                {
                    t = ros::Time::now();
                }
                else
                {
                    t = ros::Time(_ros_time_base.toSec()+ (/*ms*/ frame_time - /*ms*/ _camera_time_base) / /*ms to seconds*/ 1000);
                }

                std::map<stream_index_pair, bool> is_frame_arrived(_is_frame_arrived);
                if (frame.is<rs2::frameset>())
                {
                    ROS_DEBUG("Frameset arrived.");
                    bool is_depth_arrived = false;
                    rs2::frame depth_frame;
                    auto frameset = frame.as<rs2::frameset>();
                    ROS_DEBUG("List of frameset before applying filters: size: %d", static_cast<int>(frameset.size()));
                    for (auto it = frameset.begin(); it != frameset.end(); ++it)
                    {
                        auto f = (*it);
                        auto stream_type = f.get_profile().stream_type();
                        auto stream_index = f.get_profile().stream_index();
                        auto stream_format = f.get_profile().format();
                        auto stream_unique_id = f.get_profile().unique_id();
                        updateIsFrameArrived(is_frame_arrived, stream_type, stream_index);

                        ROS_DEBUG("Frameset contain (%s, %d, %s %d) frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                                  rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), stream_unique_id, frame.get_frame_number(), frame_time, t.toNSec());
                    }
                    // Clip depth_frame for max range:
                    if (_clipping_distance > 0)
                    {
                        rs2::depth_frame depth_frame = frameset.get_depth_frame();
                        if (depth_frame)
                            this->clip_depth(depth_frame,_depth_scale_meters, _clipping_distance);
                    }


                    ROS_DEBUG("num_filters: %d", static_cast<int>(_filters.size()));
                    for (std::vector<NamedFilter>::const_iterator filter_it = _filters.begin(); filter_it != _filters.end(); filter_it++)
                    {
                        ROS_DEBUG("Applying filter: %s", filter_it->_name.c_str());
                        frameset = filter_it->_filter->process(frameset);
                    }

                    ROS_DEBUG("List of frameset after applying filters: size: %d", static_cast<int>(frameset.size()));
                    for (auto it = frameset.begin(); it != frameset.end(); ++it)
                    {
                        auto f = (*it);
                        auto stream_type = f.get_profile().stream_type();
                        auto stream_index = f.get_profile().stream_index();
                        auto stream_format = f.get_profile().format();
                        auto stream_unique_id = f.get_profile().unique_id();

                        ROS_DEBUG("Frameset contain (%s, %d, %s %d) frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                                  rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), stream_unique_id, frame.get_frame_number(), frame_time, t.toNSec());
                    }
                    ROS_DEBUG("END OF LIST");
                    ROS_DEBUG_STREAM("Remove streams with same type and index:");
                    // TODO - Fix the following issue:
                    // Currently publishers are set using a map of stream type and index only.
                    // It means that colorized depth image <DEPTH, 0, Z16> and colorized depth image <DEPTH, 0, RGB>
                    // use the same publisher.
                    // As a workaround we remove the earlier one, the original one, assuming that if colorizer filter is
                    // set it means that that's what the client wants.
                    //
                    bool points_in_set(false);
                    std::vector<rs2::frame> frames_to_publish;
                    std::vector<stream_index_pair> is_in_set;
                    for (auto it = frameset.begin(); it != frameset.end(); ++it)
                    {
                        auto f = (*it);
                        auto stream_type = f.get_profile().stream_type();
                        auto stream_index = f.get_profile().stream_index();
                        auto stream_format = f.get_profile().format();
                        if (f.is<rs2::points>())
                        {
                            if (!points_in_set)
                            {
                                points_in_set = true;
                                frames_to_publish.push_back(f);
                            }
                            continue;
                        }
                        stream_index_pair sip{stream_type,stream_index};
                        if (std::find(is_in_set.begin(), is_in_set.end(), sip) == is_in_set.end())
                        {
                            is_in_set.push_back(sip);
                            frames_to_publish.push_back(f);
                        }
                        if (_align_depth && stream_type == RS2_STREAM_DEPTH && stream_format == RS2_FORMAT_Z16)
                        {
                            depth_frame = f;
                            is_depth_arrived = true;
                        }
                    }

                    for (auto it = frames_to_publish.begin(); it != frames_to_publish.end(); ++it)
                    {
                        auto f = (*it);
                        auto stream_type = f.get_profile().stream_type();
                        auto stream_index = f.get_profile().stream_index();
                        auto stream_format = f.get_profile().format();

                        ROS_DEBUG("Frameset contain (%s, %d, %s) frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                                  rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), frame.get_frame_number(), frame_time, t.toNSec());

                        if (f.is<rs2::points>())
                        {
                            if (0 != _pointcloud_publisher.getNumSubscribers())
                            {
                                ROS_DEBUG("Publish pointscloud");
                                publishPointCloud(f.as<rs2::points>(), t, frameset);
                            }
                            continue;
                        }
                        else
                        {
                            ROS_DEBUG("Not points");
                        }
                        stream_index_pair sip{stream_type,stream_index};
                        publishFrame(f, t,
                                     sip,
                                     _image,
                                     _info_publisher,
                                     _image_publishers, _seq,
                                     _camera_info, _optical_frame_id,
                                     _encoding);
                    }

                    if (_align_depth && is_depth_arrived)
                    {
                        ROS_DEBUG("publishAlignedDepthToOthers(...)");
                        publishAlignedDepthToOthers(frameset, t);
                    }
                }
                else
                {
                    auto stream_type = frame.get_profile().stream_type();
                    auto stream_index = frame.get_profile().stream_index();
                    updateIsFrameArrived(is_frame_arrived, stream_type, stream_index);
                    ROS_DEBUG("Single video frame arrived (%s, %d). frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                              rs2_stream_to_string(stream_type), stream_index, frame.get_frame_number(), frame_time, t.toNSec());

                    stream_index_pair sip{stream_type,stream_index};
                    publishFrame(frame, t,
                                 sip,
                                 _image,
                                 _info_publisher,
                                 _image_publishers, _seq,
                                 _camera_info, _optical_frame_id,
                                 _encoding);
                }
            }
            catch(const std::exception& ex)
            {
                ROS_ERROR_STREAM("An error has occurred during frame callback: " << ex.what());
            }
            _synced_imu_publisher->Resume();
        }; // frame_callback

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
                        if (profile.stream_type() == elem.first &&
                            profile.fps() == _fps[elem] &&
                            profile.format() == _format[elem])
                        {
                            _enabled_profiles[elem].push_back(profile);
                            break;
                        }
                    }
                    if (_enabled_profiles[elem].size() == 0)
                    {
                        ROS_WARN_STREAM("No mathcing profile found for " << _stream_name[elem] << " with fps=" << _fps[elem] << " and format=" << _format[elem]);
                        ROS_WARN_STREAM("profiles found for " << _stream_name[elem] << ":");
                        for (rs2::stream_profile& profile : profiles)
                        {
                            if (profile.stream_type() != elem.first) continue;
                            ROS_WARN_STREAM("fps: " << profile.fps() << ". format: " << profile.format());
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
            ROS_INFO("starting imu...");
            std::vector<rs2::stream_profile> profiles;
            profiles.insert(profiles.begin(), gyro_profile->second.begin(), gyro_profile->second.end());
            profiles.insert(profiles.begin(), accel_profile->second.begin(), accel_profile->second.end());
            auto& sens = _sensors[GYRO];
            sens.open(profiles);

            auto imu_callback = [this](rs2::frame frame){
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
                    // double elapsed_camera_ms = (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base) / /*ms to seconds*/ 1000;
                    double elapsed_camera_ms = (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base);
                    ros::Time t(_ros_time_base.toSec() + elapsed_camera_ms);

                    auto imu_msg = sensor_msgs::Imu();
                    imu_msg.header.frame_id = _optical_frame_id[stream_index];
                    imu_msg.orientation.x = 0.0;
                    imu_msg.orientation.y = 0.0;
                    imu_msg.orientation.z = 0.0;
                    imu_msg.orientation.w = 0.0;
                    imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    auto axis = *(reinterpret_cast<const float3*>(frame.get_data()));
                    if (GYRO == stream_index)
                    {
                        imu_msg.angular_velocity.x = axis.x;
                        imu_msg.angular_velocity.y = axis.y;
                        imu_msg.angular_velocity.z = axis.z;
                    }
                    else if (ACCEL == stream_index)
                    {
                        imu_msg.linear_acceleration.x = axis.x;
                        imu_msg.linear_acceleration.y = axis.y;
                        imu_msg.linear_acceleration.z = axis.z;
                    }
                    _seq[stream_index] += 1;
                    imu_msg.header.seq = _seq[stream_index];
                    imu_msg.header.stamp = t;
                    _imu_publishers[stream_index].publish(imu_msg);
                    ROS_DEBUG("Publish %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
                }
            };

            auto imu_callback_sync = [this](rs2::frame frame){
                static std::mutex m_mutex;
                static const stream_index_pair stream_imu = GYRO;
                static sensor_msgs::Imu imu_msg = sensor_msgs::Imu();
                static int seq = 0;
                static bool init_gyro(false), init_accel(false);
                static double accel_factor(0);
                imu_msg.header.frame_id = _frame_id[stream_imu];
                imu_msg.orientation.x = 0.0;
                imu_msg.orientation.y = 0.0;
                imu_msg.orientation.z = 0.0;
                imu_msg.orientation.w = 0.0;

                imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                imu_msg.linear_acceleration_covariance = { _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov};
                imu_msg.angular_velocity_covariance = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

                m_mutex.lock();

                while (true)
                {
                    auto stream = frame.get_profile().stream_type();
                    auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
                    double frame_time = frame.get_timestamp();

                    if (false == _intialize_time_base)
                        break;
                
                    double elapsed_camera_ms = (/*ms*/ frame_time - /*ms*/ _camera_time_base) / 1000.0;
                    ros::Time t(_ros_time_base.toSec() + elapsed_camera_ms);
                    seq += 1;

                    if (0 != _synced_imu_publisher->getNumSubscribers())
                    {
                        auto axis = *(reinterpret_cast<const float3*>(frame.get_data()));
                        if (true)
                        {
                            // Convert from optical frame to frame:
                            float3 temp;
                            temp.x = axis.z;
                            temp.y = -axis.x;
                            temp.z = -axis.y;

                            axis.x = temp.x;
                            axis.y = temp.y;
                            axis.z = temp.z;
                            imu_msg.header.frame_id = _frame_id[stream_index];
                        }
                        if (GYRO == stream_index)
                        {
                            init_gyro = true;
                            imu_msg.angular_velocity.x = axis.x;
                            imu_msg.angular_velocity.y = axis.y;
                            imu_msg.angular_velocity.z = axis.z;
                        }
                        else if (ACCEL == stream_index)
                        {
                            if (!init_accel)
                            {
                                // Init accel_factor:
                                Eigen::Vector3d v(axis.x, axis.y, axis.z);
                                accel_factor = 9.81 / v.norm();
                                ROS_INFO_STREAM("accel_factor set to: " << accel_factor);
                            }
                            init_accel = true;
                            if (true)
                            {
                                Eigen::Vector3d v(axis.x, axis.y, axis.z);
                                v*=accel_factor;
                                axis.x = v.x();
                                axis.y = v.y();
                                axis.z = v.z();
                            }
                            imu_msg.linear_acceleration.x = axis.x;
                            imu_msg.linear_acceleration.y = axis.y;
                            imu_msg.linear_acceleration.z = axis.z;
                        }
                        imu_msg.header.seq = seq;
                        imu_msg.header.stamp = t;
                        if (!(init_gyro && init_accel))
                            break;
                        _synced_imu_publisher->Publish(imu_msg);
                        ROS_DEBUG("Publish united %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
                    }
                    break;
                }
                m_mutex.unlock();
            };
            if (_unite_imu)
            {
                ROS_INFO_STREAM("Gyro and accelometer are enabled and combined to IMU message at " << (_fps[GYRO] + _fps[ACCEL]) << " fps: ");
                sens.start(imu_callback_sync);
            }
            else
            {
                sens.start(imu_callback);

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
        }

        if (_enable[DEPTH] &&
            _enable[FISHEYE])
        {
            static const char* frame_id = "depth_to_fisheye_extrinsics";
            auto ex = getRsExtrinsics(DEPTH, FISHEYE);
            _depth_to_other_extrinsics[FISHEYE] = ex;
            _depth_to_other_extrinsics_publishers[FISHEYE].publish(rsExtrinsicsToMsg(ex, frame_id));
        }

        if (_enable[DEPTH] &&
            _enable[COLOR])
        {
            static const char* frame_id = "depth_to_color_extrinsics";
            auto ex = getRsExtrinsics(DEPTH, COLOR);
            _depth_to_other_extrinsics[COLOR] = ex;
            _depth_to_other_extrinsics_publishers[COLOR].publish(rsExtrinsicsToMsg(ex, frame_id));
        }

        if (_enable[DEPTH] &&
            _enable[INFRA1])
        {
            static const char* frame_id = "depth_to_infra1_extrinsics";
            auto ex = getRsExtrinsics(DEPTH, INFRA1);
            _depth_to_other_extrinsics[INFRA1] = ex;
            _depth_to_other_extrinsics_publishers[INFRA1].publish(rsExtrinsicsToMsg(ex, frame_id));
        }

        if (_enable[DEPTH] &&
            _enable[INFRA2])
        {
            static const char* frame_id = "depth_to_infra2_extrinsics";
            auto ex = getRsExtrinsics(DEPTH, INFRA2);
            _depth_to_other_extrinsics[INFRA2] = ex;
            _depth_to_other_extrinsics_publishers[INFRA2].publish(rsExtrinsicsToMsg(ex, frame_id));
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

    if (stream_index == DEPTH && _enable[DEPTH] && _enable[COLOR])
    {
        _camera_info[stream_index].P.at(3) = 0;     // Tx
        _camera_info[stream_index].P.at(7) = 0;     // Ty
    }

    if (_align_depth)
    {
        for (auto& profiles : _enabled_profiles)
        {
            for (auto& profile : profiles.second)
            {
                auto video_profile = profile.as<rs2::video_stream_profile>();
                stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};
                _depth_aligned_camera_info[stream_index] = _camera_info[stream_index];
            }
        }
    }
}

tf::Quaternion BaseRealSenseNode::rotationMatrixToQuaternion(const float rotation[9]) const
{
    Eigen::Matrix3f m;
    // We need to be careful about the order, as RS2 rotation matrix is
    // column-major, while Eigen::Matrix3f expects row-major.
    m << rotation[0], rotation[3], rotation[6],
         rotation[1], rotation[4], rotation[7],
         rotation[2], rotation[5], rotation[8];
    Eigen::Quaternionf q(m);
    return tf::Quaternion(q.x(), q.y(), q.z(), q.w());
}

void BaseRealSenseNode::publish_static_tf(const ros::Time& t,
                                          const float3& trans,
                                          const quaternion& q,
                                          const std::string& from,
                                          const std::string& to)
{
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = from;
    msg.child_frame_id = to;
    msg.transform.translation.x = trans.z;
    msg.transform.translation.y = -trans.x;
    msg.transform.translation.z = -trans.y;
    msg.transform.rotation.x = q.x;
    msg.transform.rotation.y = q.y;
    msg.transform.rotation.z = q.z;
    msg.transform.rotation.w = q.w;
    _static_tf_broadcaster.sendTransform(msg);
}

void BaseRealSenseNode::publishStaticTransforms()
{
    // Publish static transforms
    tf::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);

    // Get the current timestamp for all static transforms
    ros::Time transform_ts_ = ros::Time::now();

    // The depth frame is used as the base link.
    // Hence no additional transformation is done from base link to depth frame.
    // Transform base link to depth frame
    float3 zero_trans{0, 0, 0};
    publish_static_tf(transform_ts_, zero_trans, quaternion{0, 0, 0, 1}, _base_frame_id, _frame_id[DEPTH]);

    // Transform depth frame to depth optical frame
    quaternion q{quaternion_optical.getX(), quaternion_optical.getY(), quaternion_optical.getZ(), quaternion_optical.getW()};
    publish_static_tf(transform_ts_, zero_trans, q, _frame_id[DEPTH], _optical_frame_id[DEPTH]);

    rs2::stream_profile depth_profile;
    if (!getEnabledProfile(DEPTH, depth_profile))
    {
        ROS_ERROR_STREAM("Given depth profile is not supported by current device!");
        ros::shutdown();
        exit(1);
    }

    if (_enable[COLOR])
    {
        // Transform base to color
        const auto& ex = getRsExtrinsics(COLOR, DEPTH);
        auto Q = rotationMatrixToQuaternion(ex.rotation);
        Q = quaternion_optical * Q * quaternion_optical.inverse();

        float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
        quaternion q1{Q.getX(), Q.getY(), Q.getZ(), Q.getW()};
        publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _frame_id[COLOR]);

        // Transform color frame to color optical frame
        quaternion q2{quaternion_optical.getX(), quaternion_optical.getY(), quaternion_optical.getZ(), quaternion_optical.getW()};
        publish_static_tf(transform_ts_, zero_trans, q2, _frame_id[COLOR], _optical_frame_id[COLOR]);

        if (_align_depth)
        {
            publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _depth_aligned_frame_id[COLOR]);
            publish_static_tf(transform_ts_, zero_trans, q2, _depth_aligned_frame_id[COLOR], _optical_frame_id[COLOR]);
        }
    }

    if (_enable[INFRA1])
    {
        const auto& ex = getRsExtrinsics(INFRA1, DEPTH);
        auto Q = rotationMatrixToQuaternion(ex.rotation);
        Q = quaternion_optical * Q * quaternion_optical.inverse();

        // Transform base to infra1
        float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
        quaternion q1{Q.getX(), Q.getY(), Q.getZ(), Q.getW()};
        publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _frame_id[INFRA1]);

        // Transform infra1 frame to infra1 optical frame
        quaternion q2{quaternion_optical.getX(), quaternion_optical.getY(), quaternion_optical.getZ(), quaternion_optical.getW()};
        publish_static_tf(transform_ts_, zero_trans, q2, _frame_id[INFRA1], _optical_frame_id[INFRA1]);

        if (_align_depth)
        {
            publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _depth_aligned_frame_id[INFRA1]);
            publish_static_tf(transform_ts_, zero_trans, q2, _depth_aligned_frame_id[INFRA1], _optical_frame_id[INFRA1]);
        }
    }

    if (_enable[INFRA2])
    {
        const auto& ex = getRsExtrinsics(INFRA2, DEPTH);
        auto Q = rotationMatrixToQuaternion(ex.rotation);
        Q = quaternion_optical * Q * quaternion_optical.inverse();

        // Transform base to infra2
        float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
        quaternion q1{Q.getX(), Q.getY(), Q.getZ(), Q.getW()};
        publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _frame_id[INFRA2]);

        // Transform infra2 frame to infra1 optical frame
        quaternion q2{quaternion_optical.getX(), quaternion_optical.getY(), quaternion_optical.getZ(), quaternion_optical.getW()};
        publish_static_tf(transform_ts_, zero_trans, q2, _frame_id[INFRA2], _optical_frame_id[INFRA2]);

        if (_align_depth)
        {
            publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _depth_aligned_frame_id[INFRA2]);
            publish_static_tf(transform_ts_, zero_trans, q2, _depth_aligned_frame_id[INFRA2], _optical_frame_id[INFRA2]);
        }
    }

    if (_enable[FISHEYE])
    {
        const auto& ex = getRsExtrinsics(FISHEYE, DEPTH);
        auto Q = rotationMatrixToQuaternion(ex.rotation);
        Q = quaternion_optical * Q * quaternion_optical.inverse();

        // Transform base to infra2
        float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
        quaternion q1{Q.getX(), Q.getY(), Q.getZ(), Q.getW()};
        publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _frame_id[FISHEYE]);

        // Transform infra2 frame to infra1 optical frame
        quaternion q2{quaternion_optical.getX(), quaternion_optical.getY(), quaternion_optical.getZ(), quaternion_optical.getW()};
        publish_static_tf(transform_ts_, zero_trans, q2, _frame_id[FISHEYE], _optical_frame_id[FISHEYE]);

        if (_align_depth)
        {
            publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _depth_aligned_frame_id[FISHEYE]);
            publish_static_tf(transform_ts_, zero_trans, q2, _depth_aligned_frame_id[FISHEYE], _optical_frame_id[FISHEYE]);
        }
    }
    if (_enable[GYRO])
    {
        // Transform base to Gyro
        // const auto& ex = getRsExtrinsics(Gyro, DEPTH);
        // Currently, No extrinsics available:
        const rs2_extrinsics ex = {{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}};
        
        auto Q = rotationMatrixToQuaternion(ex.rotation);
        Q = quaternion_optical * Q * quaternion_optical.inverse();

        float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
        quaternion q1{Q.getX(), Q.getY(), Q.getZ(), Q.getW()};
        publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _frame_id[GYRO]);

        // Transform gyro frame to gyro optical frame
        quaternion q2{quaternion_optical.getX(), quaternion_optical.getY(), quaternion_optical.getZ(), quaternion_optical.getW()};
        publish_static_tf(transform_ts_, zero_trans, q2, _frame_id[GYRO], _optical_frame_id[GYRO]);
    }
    if (_enable[ACCEL] and !_unite_imu)
    {
        // Transform base to Accel
        // const auto& ex = getRsExtrinsics(Accel, DEPTH);
        // Currently, No extrinsics available:
        const rs2_extrinsics ex = {{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}};
        
        auto Q = rotationMatrixToQuaternion(ex.rotation);
        Q = quaternion_optical * Q * quaternion_optical.inverse();

        float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
        quaternion q1{Q.getX(), Q.getY(), Q.getZ(), Q.getW()};
        publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _frame_id[ACCEL]);

        // Transform gyro frame to gyro optical frame
        quaternion q2{quaternion_optical.getX(), quaternion_optical.getY(), quaternion_optical.getZ(), quaternion_optical.getW()};
        publish_static_tf(transform_ts_, zero_trans, q2, _frame_id[ACCEL], _optical_frame_id[ACCEL]);
    }

}

void reverse_memcpy(unsigned char* dst, const unsigned char* src, size_t n)
{
    size_t i;

    for (i=0; i < n; ++i)
        dst[n-1-i] = src[i];

}

void BaseRealSenseNode::publishPointCloud(rs2::points pc, const ros::Time& t, const rs2::frameset& frameset)
{
    std::vector<NamedFilter>::iterator pc_filter = find_if(_filters.begin(), _filters.end(), [] (NamedFilter s) { return s._name == "pointcloud"; } );
    rs2_stream texture_source_id = static_cast<rs2_stream>(pc_filter->_filter->get_option(rs2_option::RS2_OPTION_STREAM_FILTER));
    bool use_texture = texture_source_id != RS2_STREAM_ANY;
    rs2::frameset::iterator texture_frame_itr = frameset.end();
    if (use_texture)
    {
        std::set<rs2_format> available_formats{ rs2_format::RS2_FORMAT_RGB8, rs2_format::RS2_FORMAT_Y8 };
        
        for (texture_frame_itr = frameset.begin();  
             texture_frame_itr != frameset.end() &&
             ((*texture_frame_itr).get_profile().stream_type() != texture_source_id ||
              available_formats.find((*texture_frame_itr).get_profile().format()) == available_formats.end()); 
              ++texture_frame_itr);
              
        if (texture_frame_itr == frameset.end())
        {
            std::string texture_source_name = pc_filter->_filter->get_option_value_description(rs2_option::RS2_OPTION_STREAM_FILTER, static_cast<float>(texture_source_id));
            ROS_WARN_STREAM("No stream match for pointcloud chosen texture " << texture_source_name);
            return;
        }
    }

    int texture_width(0), texture_height(0);
    int num_colors(0);

    const rs2::vertex* vertex = pc.get_vertices();
    const rs2::texture_coordinate* color_point = pc.get_texture_coordinates();
    int num_valid_points(0);
    if (use_texture)
    {
        for (size_t point_idx=0; point_idx < pc.size(); point_idx++, color_point++)
        {
            float i = static_cast<float>(color_point->u);
            float j = static_cast<float>(color_point->v);

            if (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f)
            {
                num_valid_points++;
            }
        }
    }
    else
    {
        for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++)
        {
            if (static_cast<float>(vertex->z) > 0)
            {
                num_valid_points++;
            }
        }
    }

    sensor_msgs::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = t;
    msg_pointcloud.header.frame_id = _optical_frame_id[DEPTH];
    msg_pointcloud.width = num_valid_points;
    msg_pointcloud.height = 1;
    msg_pointcloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");    

    vertex = pc.get_vertices();
    if (use_texture)
    {
        rs2::video_frame texture_frame = (*texture_frame_itr).as<rs2::video_frame>();
        texture_width = texture_frame.get_width();
        texture_height = texture_frame.get_height();
        num_colors = texture_frame.get_bytes_per_pixel();
        uint8_t* color_data = (uint8_t*)texture_frame.get_data();
        std::string format_str;
        switch(texture_frame.get_profile().format())
        {
            case RS2_FORMAT_RGB8:
                format_str = "rgb";
                break;
            case RS2_FORMAT_Y8:
                format_str = "intensity";
                break;
            default:
                throw std::runtime_error("Unhandled texture format passed in pointcloud " + std::to_string(texture_frame.get_profile().format()));
        }
        msg_pointcloud.point_step = addPointField(msg_pointcloud, format_str.c_str(), 1, sensor_msgs::PointField::UINT32, msg_pointcloud.point_step);
        msg_pointcloud.row_step = msg_pointcloud.width * msg_pointcloud.point_step;
        msg_pointcloud.data.resize(msg_pointcloud.height * msg_pointcloud.row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t>iter_color(msg_pointcloud, format_str);
        color_point = pc.get_texture_coordinates();

        float color_pixel[2];
        for (size_t point_idx=0; point_idx < pc.size(); vertex++, point_idx++, color_point++)
        {
            float i = static_cast<float>(color_point->u);
            float j = static_cast<float>(color_point->v);
            if (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;

                color_pixel[0] = i * texture_width;
                color_pixel[1] = j * texture_height;

                int pixx = static_cast<int>(color_pixel[0]);
                int pixy = static_cast<int>(color_pixel[1]);
                int offset = (pixy * texture_width + pixx) * num_colors;
                reverse_memcpy(&(*iter_color), color_data+offset, num_colors);  // PointCloud2 order of rgb is bgr.

                ++iter_x; ++iter_y; ++iter_z;
                ++iter_color;
            }
        }
    }
    else
    {
        sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");
        for (size_t point_idx=0; point_idx < pc.size(); vertex++, point_idx++)
        {
            if (static_cast<float>(vertex->z) > 0)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;

                ++iter_x; ++iter_y; ++iter_z;
            }
        }
    }
    _pointcloud_publisher.publish(msg_pointcloud);
}


Extrinsics BaseRealSenseNode::rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics, const std::string& frame_id) const
{
    Extrinsics extrinsicsMsg;
    for (int i = 0; i < 9; ++i)
    {
        extrinsicsMsg.rotation[i] = extrinsics.rotation[i];
        if (i < 3)
            extrinsicsMsg.translation[i] = extrinsics.translation[i];
    }

    extrinsicsMsg.header.frame_id = frame_id;
    return extrinsicsMsg;
}

rs2_extrinsics BaseRealSenseNode::getRsExtrinsics(const stream_index_pair& from_stream, const stream_index_pair& to_stream)
{
    auto& from = _enabled_profiles[from_stream].front();
    auto& to = _enabled_profiles[to_stream].front();
    return from.get_extrinsics_to(to);
}

IMUInfo BaseRealSenseNode::getImuInfo(const stream_index_pair& stream_index)
{
    IMUInfo info{};
    auto sp = _enabled_profiles[stream_index].front().as<rs2::motion_stream_profile>();
    rs2_motion_device_intrinsic imuIntrinsics;
    try
    {
        imuIntrinsics = sp.get_motion_intrinsics();
    }
    catch(const runtime_error &ex)
    {
        ROS_DEBUG_STREAM("No Motion Intrinsics available.");
        imuIntrinsics = {{{1,0,0,0},{0,1,0,0},{0,0,1,0}}, {0,0,0}, {0,0,0}};
    }
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

void BaseRealSenseNode::publishFrame(rs2::frame f, const ros::Time& t,
                                     const stream_index_pair& stream,
                                     std::map<stream_index_pair, cv::Mat>& images,
                                     const std::map<stream_index_pair, ros::Publisher>& info_publishers,
                                     const std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics>& image_publishers,
                                     std::map<stream_index_pair, int>& seq,
                                     std::map<stream_index_pair, sensor_msgs::CameraInfo>& camera_info,
                                     const std::map<stream_index_pair, std::string>& optical_frame_id,
                                     const std::map<stream_index_pair, std::string>& encoding,
                                     bool copy_data_from_frame)
{
    ROS_DEBUG("publishFrame(...)");
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
    auto& image = images[stream];

    if (copy_data_from_frame)
    {
        if (images[stream].size() != cv::Size(width, height))
        {
            image.create(height, width, _image_format[stream]);
        }
        image.data = (uint8_t*)f.get_data();
    }

    ++(seq[stream]);
    auto& info_publisher = info_publishers.at(stream);
    auto& image_publisher = image_publishers.at(stream);
    if(0 != info_publisher.getNumSubscribers() ||
       0 != image_publisher.first.getNumSubscribers())
    {
        sensor_msgs::ImagePtr img;
        img = cv_bridge::CvImage(std_msgs::Header(), encoding.at(stream), image).toImageMsg();
        img->width = width;
        img->height = height;
        img->is_bigendian = false;
        img->step = width * bpp;
        img->header.frame_id = optical_frame_id.at(stream);
        img->header.stamp = t;
        img->header.seq = seq[stream];

        auto& cam_info = camera_info.at(stream);
        cam_info.header.stamp = t;
        cam_info.header.seq = seq[stream];
        info_publisher.publish(cam_info);

        image_publisher.first.publish(img);
        image_publisher.second->update();
        // ROS_INFO_STREAM("fid: " << cam_info.header.seq << ", time: " << std::setprecision (20) << t.toSec());
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

