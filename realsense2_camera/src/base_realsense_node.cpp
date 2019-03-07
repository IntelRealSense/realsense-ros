#include "../include/base_realsense_node.h"
#include "assert.h"
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <mutex>
#include <tf/transform_broadcaster.h>

using namespace realsense2_camera;
using namespace ddynamic_reconfigure;

// stream_index_pair sip{stream_type, stream_index};
#define STREAM_NAME(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << _stream_name[sip.first] << ((sip.second>0) ? std::to_string(sip.second) : ""))).str()
#define FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << "camera_" << STREAM_NAME(sip) << "_frame")).str()
#define OPTICAL_FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << "camera_" << STREAM_NAME(sip) << "_optical_frame")).str()
#define ALIGNED_DEPTH_TO_FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << "camera_aligned_depth_to_" << STREAM_NAME(sip) << "_frame")).str()

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
        if (_pending_messages.size() >= _waiting_list_size)
        {
            throw std::runtime_error("SyncedImuPublisher inner list reached maximum size of " + std::to_string(_pending_messages.size()));
        }
        _pending_messages.push(imu_msg);
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
    // ROS_INFO_STREAM("publish imu: " << _pending_messages.size());
    while (!_pending_messages.empty())
    {
        const sensor_msgs::Imu &imu_msg = _pending_messages.front();
        _publisher.publish(imu_msg);
        // ROS_INFO_STREAM("iid2:" << imu_msg.header.seq << ", time: " << std::setprecision (20) << imu_msg.header.stamp.toSec());
        _pending_messages.pop();
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
    _base_frame_id(""), _dev(dev),  _node_handle(nodeHandle),
    _pnh(privateNodeHandle), _json_file_path(""),
    _serial_no(serial_no),
    _is_initialized_time_base(false),
    _namespace(getNamespaceStr())
{
    // Types for depth stream
    _image_format[RS2_STREAM_DEPTH] = CV_16UC1;    // CVBridge type
    _encoding[RS2_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
    _unit_step_size[RS2_STREAM_DEPTH] = sizeof(uint16_t); // sensor_msgs::ImagePtr row step size
    _stream_name[RS2_STREAM_DEPTH] = "depth";
    _depth_aligned_encoding[RS2_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Infrared stream
    _image_format[RS2_STREAM_INFRARED] = CV_8UC1;    // CVBridge type
    _encoding[RS2_STREAM_INFRARED] = sensor_msgs::image_encodings::MONO8; // ROS message type
    _unit_step_size[RS2_STREAM_INFRARED] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[RS2_STREAM_INFRARED] = "infra";
    _depth_aligned_encoding[RS2_STREAM_INFRARED] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Types for color stream
    _image_format[RS2_STREAM_COLOR] = CV_8UC3;    // CVBridge type
    _encoding[RS2_STREAM_COLOR] = sensor_msgs::image_encodings::RGB8; // ROS message type
    _unit_step_size[RS2_STREAM_COLOR] = 3; // sensor_msgs::ImagePtr row step size
    _stream_name[RS2_STREAM_COLOR] = "color";
    _depth_aligned_encoding[RS2_STREAM_COLOR] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Types for fisheye stream
    _image_format[RS2_STREAM_FISHEYE] = CV_8UC1;    // CVBridge type
    _encoding[RS2_STREAM_FISHEYE] = sensor_msgs::image_encodings::MONO8; // ROS message type
    _unit_step_size[RS2_STREAM_FISHEYE] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[RS2_STREAM_FISHEYE] = "fisheye";
    _depth_aligned_encoding[RS2_STREAM_FISHEYE] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Types for Motion-Module streams
    _stream_name[RS2_STREAM_GYRO] = "gyro";

    _stream_name[RS2_STREAM_ACCEL] = "accel";

    _stream_name[RS2_STREAM_POSE] = "pose";
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
            std::vector<std::string> error_strings({"RT IC2 Config error", 
                                                    "Motion Module force pause",
                                                    "stream start failure"});
            if (n.get_severity() >= RS2_LOG_SEVERITY_ERROR)
            {
                ROS_WARN_STREAM("Hardware Notification:" << n.get_description() << "," << n.get_timestamp() << "," << n.get_severity() << "," << n.get_category());
            }
            if (error_strings.end() != find_if(error_strings.begin(), error_strings.end(), [&n] (std::string err) 
                                        {return (n.get_description().find(err) != std::string::npos); }))
            {
                ROS_ERROR_STREAM("Hardware Reset is needed. use option: initial_reset:=true");
                _dev.hardware_reset();
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

/**
 * ROS Graph Resource names don't allow spaces and hyphens (see http://wiki.ros.org/Names),
 * so we replace them here with underscores.
 */
std::string create_graph_resource_name(const std::string &original_name)
{
  std::string fixed_name = original_name;
  std::replace(fixed_name.begin(), fixed_name.end(), '-', '_');
  std::replace(fixed_name.begin(), fixed_name.end(), ' ', '_');
  return fixed_name;
}

void BaseRealSenseNode::registerDynamicOption(ros::NodeHandle& nh, rs2::options sensor, std::string& module_name)
{
    ros::NodeHandle nh1(module_name);
    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh1);
    for (auto i = 0; i < RS2_OPTION_COUNT; i++)
    {
        rs2_option option = static_cast<rs2_option>(i);
        std::string option_name = create_graph_resource_name(rs2_option_to_string(option));
        if (!sensor.supports(option) || sensor.is_option_read_only(option))
        {
            continue;
        }
        if (is_checkbox(sensor, option))
        {
            ddynrec->add(new DDBool(option_name, i, sensor.get_option_description(option), bool(sensor.get_option(option))));
            continue;
        }
        std::map<std::string, int> enum_dict = get_enum_method(sensor, option);
        if (enum_dict.empty())
        {
            rs2::option_range op_range = sensor.get_option_range(option);
            if (is_int_option(sensor, option))
            {
                ddynrec->add(new DDInt(option_name, i, sensor.get_option_description(option), sensor.get_option(option), op_range.min, op_range.max));
            }
            else
            {
                ddynrec->add(new DDDouble(option_name, i, sensor.get_option_description(option), sensor.get_option(option), op_range.min, op_range.max));
            }
        }
        else
        {
            if (int(sensor.get_option(option)) > (int)enum_dict.size())
            {
                ROS_WARN_STREAM("Option " << option_name << 
                                " has a value: " << int(sensor.get_option(option)) << 
                                " which is beyond it's scope: " << enum_dict.size());
            ROS_DEBUG_STREAM("Add enum: " << rs2_option_to_string(option) << ". value=" << int(sensor.get_option(option)));
                for (auto item: enum_dict)
                {
                    ROS_INFO_STREAM("Add item: " << item.first << ":" << item.second); // << ":" << sensor.get_option_description(static_cast<rs2_option>(item.second)));
                }
            }
            ddynrec->add(new DDEnum(option_name, i, sensor.get_option_description(option), int(sensor.get_option(option)), enum_dict));
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
        std::string module_name = create_graph_resource_name(sensor.get_info(RS2_CAMERA_INFO_NAME));
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
    std::string option_name = create_graph_resource_name(rs2_option_to_string(option));
    double value = get(map, option_name.c_str()).toDouble();
    ROS_DEBUG_STREAM("option: " << option_name << ". value: " << value);
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

    for (auto& stream : IMAGE_STREAMS)
    {
        string param_name(_stream_name[stream.first] + "_width");
        ROS_DEBUG_STREAM("reading parameter:" << param_name);
        _pnh.param(param_name, _width[stream], IMAGE_WIDTH);
        param_name = _stream_name[stream.first] + "_height";
        ROS_DEBUG_STREAM("reading parameter:" << param_name);
        _pnh.param(param_name, _height[stream], IMAGE_HEIGHT);
        param_name = _stream_name[stream.first] + "_fps";
        ROS_DEBUG_STREAM("reading parameter:" << param_name);
        _pnh.param(param_name, _fps[stream], IMAGE_FPS);
        param_name = "enable_" + STREAM_NAME(stream);
        ROS_DEBUG_STREAM("reading parameter:" << param_name);
        _pnh.param(param_name, _enable[stream], true);
    }

    for (auto& stream : HID_STREAMS)
    {
        string param_name(_stream_name[stream.first] + "_fps");
        ROS_DEBUG_STREAM("reading parameter:" << param_name);
        _pnh.param(param_name, _fps[stream], IMU_FPS);
        param_name = "enable_" + STREAM_NAME(stream);
        _pnh.param(param_name, _enable[stream], ENABLE_IMU);
        ROS_DEBUG_STREAM("_enable[" << _stream_name[stream.first] << "]:" << _enable[stream]);
    }
    _pnh.param("base_frame_id", _base_frame_id, DEFAULT_BASE_FRAME_ID);
    _pnh.param("odom_frame_id", _odom_frame_id, DEFAULT_ODOM_FRAME_ID);

    std::vector<stream_index_pair> streams(IMAGE_STREAMS);
    streams.insert(streams.end(), HID_STREAMS.begin(), HID_STREAMS.end());
    for (auto& stream : streams)
    {
        string param_name(static_cast<std::ostringstream&&>(std::ostringstream() << STREAM_NAME(stream) << "_frame_id").str());
        _pnh.param(param_name, _frame_id[stream], FRAME_ID(stream));
        ROS_DEBUG_STREAM("frame_id: reading parameter:" << param_name << " : " << _frame_id[stream]);
        param_name = static_cast<std::ostringstream&&>(std::ostringstream() << STREAM_NAME(stream) << "_optical_frame_id").str();
        _pnh.param(param_name, _optical_frame_id[stream], OPTICAL_FRAME_ID(stream));
        ROS_DEBUG_STREAM("optical: reading parameter:" << param_name << " : " << _optical_frame_id[stream]);
    }

    std::string unite_imu_method_str("");
    _pnh.param("unite_imu_method", unite_imu_method_str, DEFAULT_UNITE_IMU_METHOD);
    if (unite_imu_method_str == "linear_interpolation")
        _imu_sync_method = imu_sync_method::LINEAR_INTERPOLATION;
    else if (unite_imu_method_str == "copy")
        _imu_sync_method = imu_sync_method::COPY;
    else
        _imu_sync_method = imu_sync_method::NONE;

    if (_imu_sync_method > imu_sync_method::NONE)
    {
        _pnh.param("imu_optical_frame_id", _optical_frame_id[GYRO], DEFAULT_IMU_OPTICAL_FRAME_ID);
    }

    for (auto& stream : IMAGE_STREAMS)
    {
        if (stream == DEPTH) continue;
        if (stream.second > 1) continue;
        string param_name(static_cast<std::ostringstream&&>(std::ostringstream() << "aligned_depth_to_" << STREAM_NAME(stream) << "_frame_id").str());
        _pnh.param(param_name, _depth_aligned_frame_id[stream], ALIGNED_DEPTH_TO_FRAME_ID(stream));
    }

    _pnh.param("clip_distance", _clipping_distance, static_cast<float>(-1.0));
    _pnh.param("linear_accel_cov", _linear_accel_cov, static_cast<double>(0.01));
    _pnh.param("angular_velocity_cov", _angular_velocity_cov, static_cast<double>(0.01));
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


        std::function<void(rs2::frame)> frame_callback_function, imu_callback_function;
        if (_sync_frames)
        {
            frame_callback_function = _syncer;

            auto frame_callback_inner = [this](rs2::frame frame){
                frame_callback(frame);
            };
            _syncer.start(frame_callback_inner);
        }
        else
        {
            frame_callback_function = [this](rs2::frame frame){frame_callback(frame);};
        }

        if (_imu_sync_method == imu_sync_method::NONE)
        {
            imu_callback_function = [this](rs2::frame frame){imu_callback(frame);};
        }
        else
        {
            imu_callback_function = [this](rs2::frame frame){imu_callback_sync(frame, _imu_sync_method);};
        }
        std::function<void(rs2::frame)> multiple_message_callback_function = [this](rs2::frame frame){multiple_message_callback(frame, _imu_sync_method);};
        std::function<void(rs2::frame)> pose_callback_function = [this](rs2::frame frame){pose_callback(frame);};

        ROS_INFO_STREAM("Device Sensors: ");
        for(auto&& elem : _dev_sensors)
        {
            std::string module_name = elem.get_info(RS2_CAMERA_INFO_NAME);

            if ("Stereo Module" == module_name)
            {
                _sensors[DEPTH] = elem;
                _sensors[INFRA1] = elem;
                _sensors[INFRA2] = elem;
                _sensors_callback[module_name] = frame_callback_function;
            }
            else if ("Coded-Light Depth Sensor" == module_name)
            {
                _sensors[DEPTH] = elem;
                _sensors[INFRA1] = elem;
                _sensors_callback[module_name] = frame_callback_function;
            }
            else if ("RGB Camera" == module_name)
            {
                _sensors[COLOR] = elem;
                _sensors_callback[module_name] = frame_callback_function;
            }
            else if ("Wide FOV Camera" == module_name)
            {
                _sensors[FISHEYE] = elem;
                _sensors_callback[module_name] = frame_callback_function;
            }
            else if ("Motion Module" == module_name)
            {
                _sensors[GYRO] = elem;
                _sensors[ACCEL] = elem;
                _sensors_callback[module_name] = imu_callback_function;
            }
            else if ("Tracking Module" == module_name)
            {
                _sensors[GYRO] = elem;
                _sensors[ACCEL] = elem;
                _sensors[POSE] = elem;
                _sensors[FISHEYE1] = elem;
                _sensors[FISHEYE2] = elem;
                _sensors_callback[module_name] = multiple_message_callback_function;
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
        for (std::pair<stream_index_pair, bool> const& enable : _enable )
        {
            const stream_index_pair& stream_index(enable.first);
            if (enable.second && _sensors.find(stream_index) == _sensors.end())
            {
                ROS_INFO_STREAM("(" << rs2_stream_to_string(stream_index.first) << ", " << stream_index.second << ") sensor isn't supported by current device! -- Skipping...");
                _enable[enable.first] = false;
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

    for (auto& stream : IMAGE_STREAMS)
    {
        if (_enable[stream])
        {
            std::stringstream image_raw, camera_info;
            bool rectified_image = false;
            if (stream == DEPTH || stream == INFRA1 || stream == INFRA2)
                rectified_image = true;

            std::string stream_name(STREAM_NAME(stream));
            image_raw << stream_name << "/image_" << ((rectified_image)?"rect_":"") << "raw";
            camera_info << stream_name << "/camera_info";

            std::shared_ptr<FrequencyDiagnostics> frequency_diagnostics(new FrequencyDiagnostics(_fps[stream], stream_name, _serial_no));
            _image_publishers[stream] = {image_transport.advertise(image_raw.str(), 1), frequency_diagnostics};
            _info_publisher[stream] = _node_handle.advertise<sensor_msgs::CameraInfo>(camera_info.str(), 1);

            if (_align_depth && (stream != DEPTH) && stream.second < 2)
            {
                std::stringstream aligned_image_raw, aligned_camera_info;
                aligned_image_raw << "aligned_depth_to_" << stream_name << "/image_raw";
                aligned_camera_info << "aligned_depth_to_" << stream_name << "/camera_info";

                std::string aligned_stream_name = "aligned_depth_to_" + stream_name;
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

    _synced_imu_publisher = std::make_shared<SyncedImuPublisher>();
    if (_imu_sync_method > imu_sync_method::NONE && _enable[GYRO] && _enable[ACCEL])
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
    if (_enable[POSE])
    {
        _imu_publishers[POSE] = _node_handle.advertise<nav_msgs::Odometry>("odom/sample", 100);
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
        if (stream_index > 1)
        {
            continue;
        }
        stream_index_pair sip{stream_type, stream_index};
        auto& info_publisher = _depth_aligned_info_publisher.at(sip);
        auto& image_publisher = _depth_aligned_image_publishers.at(sip);

        if(0 != info_publisher.getNumSubscribers() ||
           0 != image_publisher.first.getNumSubscribers())
        {
            std::shared_ptr<rs2::align> align;
            try{
                align = _align.at(stream_type);
            }
            catch(const std::out_of_range& e)
            {
                ROS_DEBUG_STREAM("Allocate align filter for:" << rs2_stream_to_string(sip.first) << sip.second);
                align = (_align[stream_type] = std::make_shared<rs2::align>(stream_type));
            }
            rs2::frameset processed = frames.apply_filter(*align);
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
    for (auto& elem : IMAGE_STREAMS)
    {
        if (_enable[elem])
        {
            auto& sens = _sensors[elem];
            auto profiles = sens.get_stream_profiles();
            for (auto& profile : profiles)
            {
                auto video_profile = profile.as<rs2::video_stream_profile>();
                ROS_DEBUG_STREAM("Sensor profile: " <<
                                    "stream_type: " << rs2_stream_to_string(elem.first) << "(" << elem.second << ")" <<
                                    "Format: " << video_profile.format() <<
                                    ", Width: " << video_profile.width() <<
                                    ", Height: " << video_profile.height() <<
                                    ", FPS: " << video_profile.fps());

                if ((video_profile.stream_type() == elem.first) &&
                    (_width[elem] == 0 || video_profile.width() == _width[elem]) &&
                    (_height[elem] == 0 || video_profile.height() == _height[elem]) &&
                    (_fps[elem] == 0 || video_profile.fps() == _fps[elem]) &&
                    video_profile.stream_index() == elem.second)
                {
                    _width[elem] = video_profile.width();
                    _height[elem] = video_profile.height();
                    _fps[elem] = video_profile.fps();

                    _enabled_profiles[elem].push_back(profile);

                    _image[elem] = cv::Mat(_height[elem], _width[elem], _image_format[elem.first], cv::Scalar(0, 0, 0));

                    ROS_INFO_STREAM(STREAM_NAME(elem) << " stream is enabled - width: " << _width[elem] << ", height: " << _height[elem] << ", fps: " << _fps[elem]);
                    break;
                }
            }
            if (_enabled_profiles.find(elem) == _enabled_profiles.end())
            {
                ROS_WARN_STREAM("Given stream configuration is not supported by the device! " <<
                    " Stream: " << rs2_stream_to_string(elem.first) <<
                    ", Stream Index: " << elem.second <<
                    ", Width: " << _width[elem] <<
                    ", Height: " << _height[elem] <<
                    ", FPS: " << _fps[elem]);
                _enable[elem] = false;
            }
        }
    }
	if (_align_depth)
	{
		for (auto& profiles : _enabled_profiles)
		{
			_depth_aligned_image[profiles.first] = cv::Mat(_height[DEPTH], _width[DEPTH], _image_format[DEPTH.first], cv::Scalar(0, 0, 0));
		}
	}

    // Streaming HID
    for (auto& elem : HID_STREAMS)
    {
        if (_enable[elem])
        {
            auto& sens = _sensors[elem];
            auto profiles = sens.get_stream_profiles();
            ROS_DEBUG_STREAM("Available profiles:");
            for (rs2::stream_profile& profile : profiles)
            {
                ROS_DEBUG_STREAM("type:" << rs2_stream_to_string(profile.stream_type()) <<
                                " fps: " << profile.fps() << ". format: " << profile.format());
            }
            for (rs2::stream_profile& profile : profiles)
            {
                if (profile.stream_type() == elem.first &&
                   (_fps[elem] == 0 || profile.fps() == _fps[elem]))
                {
                    _fps[elem] = profile.fps();
                    _enabled_profiles[elem].push_back(profile);
                    break;
                }
            }
            if (_enabled_profiles.find(elem) == _enabled_profiles.end())
            {
                std::string stream_name(STREAM_NAME(elem));
                ROS_WARN_STREAM("No mathcing profile found for " << stream_name << " with fps=" << _fps[elem]);
                ROS_WARN_STREAM("profiles found for " <<stream_name << ":");
                for (rs2::stream_profile& profile : profiles)
                {
                    if (profile.stream_type() != elem.first) continue;
                    ROS_WARN_STREAM("fps: " << profile.fps() << ". format: " << profile.format());
                }
            }
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
        _image_format[DEPTH.first] = _image_format[COLOR.first];    // CVBridge type
        _encoding[DEPTH.first] = _encoding[COLOR.first]; // ROS message type
        _unit_step_size[DEPTH.first] = _unit_step_size[COLOR.first]; // sensor_msgs::ImagePtr row step size

        _width[DEPTH] = _width[COLOR];
        _height[DEPTH] = _height[COLOR];
        _image[DEPTH] = cv::Mat(_height[DEPTH], _width[DEPTH], _image_format[DEPTH.first], cv::Scalar(0, 0, 0));
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

    #ifdef _OPENMP
    #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    #endif
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

BaseRealSenseNode::CIMUHistory::CIMUHistory(size_t size)
{
    m_max_size = size;
}
void BaseRealSenseNode::CIMUHistory::add_data(sensor_name module, BaseRealSenseNode::CIMUHistory::imuData data)
{
    m_map[module].push_front(data);
    if (m_map[module].size() > m_max_size)
        m_map[module].pop_back();
}
bool BaseRealSenseNode::CIMUHistory::is_all_data(sensor_name module)
{
    return m_map[module].size() == m_max_size;
}
bool BaseRealSenseNode::CIMUHistory::is_data(sensor_name module)
{
    return m_map[module].size() > 0;
}
const std::list<BaseRealSenseNode::CIMUHistory::imuData>& BaseRealSenseNode::CIMUHistory::get_data(sensor_name module)
{
    return m_map[module];
}
BaseRealSenseNode::CIMUHistory::imuData BaseRealSenseNode::CIMUHistory::last_data(sensor_name module)
{
    return m_map[module].front();
}
BaseRealSenseNode::CIMUHistory::imuData BaseRealSenseNode::CIMUHistory::imuData::operator*(const double factor)
{
    BaseRealSenseNode::CIMUHistory::imuData new_data(*this);
    new_data.m_reading *= factor;
    new_data.m_time *= factor;
    return new_data;
}

BaseRealSenseNode::CIMUHistory::imuData BaseRealSenseNode::CIMUHistory::imuData::operator+(const BaseRealSenseNode::CIMUHistory::imuData& other)
{
    BaseRealSenseNode::CIMUHistory::imuData new_data(*this);
    new_data.m_reading += other.m_reading;
    new_data.m_time += other.m_time;
    return new_data;
}

double BaseRealSenseNode::FillImuData_LinearInterpolation(const stream_index_pair stream_index, const BaseRealSenseNode::CIMUHistory::imuData imu_data, sensor_msgs::Imu& imu_msg)
{
    static CIMUHistory _imu_history(2);
    CIMUHistory::sensor_name this_sensor(static_cast<CIMUHistory::sensor_name>(ACCEL == stream_index));
    CIMUHistory::sensor_name that_sensor(static_cast<CIMUHistory::sensor_name>(!this_sensor));
    _imu_history.add_data(this_sensor, imu_data);

    if (!_imu_history.is_all_data(this_sensor) || !_imu_history.is_data(that_sensor) )
        return -1;
    const std::list<CIMUHistory::imuData> this_data = _imu_history.get_data(this_sensor);
    CIMUHistory::imuData that_last_data = _imu_history.last_data(that_sensor);
    std::list<CIMUHistory::imuData>::const_iterator this_data_iter = this_data.begin();
    CIMUHistory::imuData this_last_data(*this_data_iter);
    this_data_iter++;
    CIMUHistory::imuData this_prev_data(*this_data_iter);
    if (this_prev_data.m_time > that_last_data.m_time)
        return -1;  // "that" data was already sent.
    double factor( (that_last_data.m_time - this_prev_data.m_time) / (this_last_data.m_time - this_prev_data.m_time) );
    CIMUHistory::imuData interp_data = this_prev_data*(1-factor) + this_last_data*factor;

    CIMUHistory::imuData accel_data = that_last_data;
    CIMUHistory::imuData gyro_data = interp_data;
    if (this_sensor == CIMUHistory::sensor_name::mACCEL)
    {
        std::swap(accel_data, gyro_data);
    }
    imu_msg.angular_velocity.x = gyro_data.m_reading.x;
    imu_msg.angular_velocity.y = gyro_data.m_reading.y;
    imu_msg.angular_velocity.z = gyro_data.m_reading.z;

    imu_msg.linear_acceleration.x = accel_data.m_reading.x;
    imu_msg.linear_acceleration.y = accel_data.m_reading.y;
    imu_msg.linear_acceleration.z = accel_data.m_reading.z;
    return that_last_data.m_time;
}


double BaseRealSenseNode::FillImuData_Copy(const stream_index_pair stream_index, const BaseRealSenseNode::CIMUHistory::imuData imu_data, sensor_msgs::Imu& imu_msg)
{
    if (GYRO == stream_index)
    {
        imu_msg.angular_velocity.x = imu_data.m_reading.x;
        imu_msg.angular_velocity.y = imu_data.m_reading.y;
        imu_msg.angular_velocity.z = imu_data.m_reading.z;
    }
    else if (ACCEL == stream_index)
    {
        imu_msg.linear_acceleration.x = imu_data.m_reading.x;
        imu_msg.linear_acceleration.y = imu_data.m_reading.y;
        imu_msg.linear_acceleration.z = imu_data.m_reading.z;
    }
    return imu_data.m_time;
}

void BaseRealSenseNode::ConvertFromOpticalFrameToFrame(float3& data)
{
    float3 temp;
    temp.x = data.z;
    temp.y = -data.x;
    temp.z = -data.y;

    data.x = temp.x;
    data.y = temp.y;
    data.z = temp.z;
}

void BaseRealSenseNode::imu_callback_sync(rs2::frame frame, imu_sync_method sync_method)
{
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
    imu_msg.angular_velocity_covariance = { _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov};

    m_mutex.lock();

    while (true)
    {
        auto stream = frame.get_profile().stream_type();
        auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
        double frame_time = frame.get_timestamp();

        bool placeholder_false(false);
        if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
        {
            setBaseTime(frame_time, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME == frame.get_frame_timestamp_domain());
        }

        seq += 1;
        double elapsed_camera_ms = (/*ms*/ frame_time - /*ms*/ _camera_time_base) / 1000.0;

        if (0 != _synced_imu_publisher->getNumSubscribers())
        {
            auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
            if (true)
            {
                // Convert from optical frame to frame:
                ConvertFromOpticalFrameToFrame(crnt_reading);
                imu_msg.header.frame_id = _frame_id[stream_index];
            }
            if (GYRO == stream_index)
            {
                init_gyro = true;
            }
            if (ACCEL == stream_index)
            {
                if (!init_accel)
                {
                    // Init accel_factor:
                    Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
                    accel_factor = 9.81 / v.norm();
                    ROS_INFO_STREAM("accel_factor set to: " << accel_factor);
                }
                init_accel = true;
                if (true)
                {
                    Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
                    v*=accel_factor;
                    crnt_reading.x = v.x();
                    crnt_reading.y = v.y();
                    crnt_reading.z = v.z();
                }
            }
            CIMUHistory::imuData imu_data(crnt_reading, elapsed_camera_ms);
            switch (sync_method)
            {
                case NONE: //Cannot really be NONE. Just to avoid compilation warning.
                case COPY:
                    elapsed_camera_ms = FillImuData_Copy(stream_index, imu_data, imu_msg);
                    break;
                case LINEAR_INTERPOLATION:
                    elapsed_camera_ms = FillImuData_LinearInterpolation(stream_index, imu_data, imu_msg);
                    break;
            }
            if (elapsed_camera_ms < 0)
                break;
            ros::Time t(_ros_time_base.toSec() + elapsed_camera_ms);
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

void BaseRealSenseNode::imu_callback(rs2::frame frame)
{
    auto stream = frame.get_profile().stream_type();
    double frame_time = frame.get_timestamp();
    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
        setBaseTime(frame_time, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME == frame.get_frame_timestamp_domain());
    }

    ROS_DEBUG("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
                rs2_stream_to_string(frame.get_profile().stream_type()),
                frame.get_profile().stream_index(),
                rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));

    auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
    if (0 != _info_publisher[stream_index].getNumSubscribers() ||
        0 != _imu_publishers[stream_index].getNumSubscribers())
    {
        double elapsed_camera_ms = (/*ms*/ frame_time - /*ms*/ _camera_time_base) / 1000.0;
        ros::Time t(_ros_time_base.toSec() + elapsed_camera_ms);

        auto imu_msg = sensor_msgs::Imu();
        imu_msg.header.frame_id = _optical_frame_id[stream_index];
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 0.0;
        imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        imu_msg.linear_acceleration_covariance = { _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov};
        imu_msg.angular_velocity_covariance = { _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov};

        auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        ConvertFromOpticalFrameToFrame(crnt_reading);
        if (GYRO == stream_index)
        {
            imu_msg.angular_velocity.x = crnt_reading.x;
            imu_msg.angular_velocity.y = crnt_reading.y;
            imu_msg.angular_velocity.z = crnt_reading.z;
        }
        else if (ACCEL == stream_index)
        {
            imu_msg.linear_acceleration.x = crnt_reading.x;
            imu_msg.linear_acceleration.y = crnt_reading.y;
            imu_msg.linear_acceleration.z = crnt_reading.z;
        }
        _seq[stream_index] += 1;
        imu_msg.header.seq = _seq[stream_index];
        imu_msg.header.stamp = t;
        _imu_publishers[stream_index].publish(imu_msg);
        ROS_DEBUG("Publish %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
    }
}

void BaseRealSenseNode::pose_callback(rs2::frame frame)
{
    double frame_time = frame.get_timestamp();
    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
        setBaseTime(frame_time, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME == frame.get_frame_timestamp_domain());
    }

    ROS_DEBUG("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
                rs2_stream_to_string(frame.get_profile().stream_type()),
                frame.get_profile().stream_index(),
                rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));
    const auto& stream_index(POSE);
    rs2_pose pose = frame.as<rs2::pose_frame>().get_pose_data();
    double elapsed_camera_ms = (/*ms*/ frame_time - /*ms*/ _camera_time_base) / 1000.0;
    ros::Time t(_ros_time_base.toSec() + elapsed_camera_ms);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = -pose.translation.z;
    pose_msg.pose.position.y = -pose.translation.x;
    pose_msg.pose.position.z = pose.translation.y;
    pose_msg.pose.orientation.x = -pose.rotation.z;
    pose_msg.pose.orientation.y = -pose.rotation.x;
    pose_msg.pose.orientation.z = pose.rotation.y;
    pose_msg.pose.orientation.w = pose.rotation.w;

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = _odom_frame_id;
    msg.child_frame_id = _frame_id[POSE];
    msg.transform.translation.x = pose_msg.pose.position.x;
    msg.transform.translation.y = pose_msg.pose.position.y;
    msg.transform.translation.z = pose_msg.pose.position.z;
    msg.transform.rotation.x = pose_msg.pose.orientation.x;
    msg.transform.rotation.y = pose_msg.pose.orientation.y;
    msg.transform.rotation.z = pose_msg.pose.orientation.z;
    msg.transform.rotation.w = pose_msg.pose.orientation.w;

    br.sendTransform(msg);

    if (0 != _imu_publishers[stream_index].getNumSubscribers())
    {
        double cov_pose(_linear_accel_cov * pow(10, 3-pose.tracker_confidence));
        double cov_twist(_angular_velocity_cov * pow(10, 1-pose.tracker_confidence));

        geometry_msgs::Vector3Stamped v_msg;
        v_msg.vector.x = -pose.velocity.z;
        v_msg.vector.y = -pose.velocity.x;
        v_msg.vector.z = pose.velocity.y;

        geometry_msgs::Vector3Stamped om_msg;
        om_msg.vector.x = -pose.angular_velocity.z;
        om_msg.vector.y = -pose.angular_velocity.x;
        om_msg.vector.z = pose.angular_velocity.y;

        nav_msgs::Odometry odom_msg;
        _seq[stream_index] += 1;

        odom_msg.header.frame_id = _odom_frame_id;
        odom_msg.child_frame_id = _frame_id[POSE];
        odom_msg.header.stamp = t;
        odom_msg.header.seq = _seq[stream_index];
        odom_msg.pose.pose = pose_msg.pose;
        odom_msg.pose.covariance = {cov_pose, 0, 0, 0, 0, 0,
                                    0, cov_pose, 0, 0, 0, 0,
                                    0, 0, cov_pose, 0, 0, 0,
                                    0, 0, 0, cov_pose, 0, 0,
                                    0, 0, 0, 0, cov_pose, 0,
                                    0, 0, 0, 0, 0, cov_pose};
        odom_msg.twist.twist.linear = v_msg.vector;
        odom_msg.twist.twist.angular = om_msg.vector;
        odom_msg.twist.covariance ={cov_twist, 0, 0, 0, 0, 0,
                                    0, cov_twist, 0, 0, 0, 0,
                                    0, 0, cov_twist, 0, 0, 0,
                                    0, 0, 0, cov_twist, 0, 0,
                                    0, 0, 0, 0, cov_twist, 0,
                                    0, 0, 0, 0, 0, cov_twist};
        _imu_publishers[stream_index].publish(odom_msg);
        ROS_DEBUG("Publish %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
    }
}

void BaseRealSenseNode::frame_callback(rs2::frame frame)
{
    _synced_imu_publisher->Pause();
    
    try{
        double frame_time = frame.get_timestamp();

        // We compute a ROS timestamp which is based on an initial ROS time at point of first frame,
        // and the incremental timestamp from the camera.
        // In sync mode the timestamp is based on ROS time
        bool placeholder_false(false);
        if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
        {
            setBaseTime(frame_time, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME == frame.get_frame_timestamp_domain());
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
        else if (frame.is<rs2::video_frame>())
        {
            auto stream_type = frame.get_profile().stream_type();
            auto stream_index = frame.get_profile().stream_index();
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

void BaseRealSenseNode::multiple_message_callback(rs2::frame frame, imu_sync_method sync_method)
{
    auto stream = frame.get_profile().stream_type();
    switch (stream)
    {
        case RS2_STREAM_GYRO:
        case RS2_STREAM_ACCEL:
            if (sync_method > imu_sync_method::NONE) imu_callback_sync(frame, sync_method);
            else imu_callback(frame);
            break;
        case RS2_STREAM_POSE:
            pose_callback(frame);
            break;
        default:
            frame_callback(frame);
    }
}

void BaseRealSenseNode::setBaseTime(double frame_time, bool warn_no_metadata)
{
    ROS_WARN_COND(warn_no_metadata, "Frame metadata isn't available! (frame_timestamp_domain = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME)");

    _ros_time_base = ros::Time::now();
    _camera_time_base = frame_time;
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
                if (profile.is<rs2::video_stream_profile>())
                {
                    auto video_profile = profile.as<rs2::video_stream_profile>();
                    updateStreamCalibData(video_profile);
                }
            }
        }

        // Streaming IMAGES
        std::map<std::string, std::vector<rs2::stream_profile> > profiles;
        std::map<std::string, rs2::sensor> active_sensors;
        for (const std::pair<stream_index_pair, std::vector<rs2::stream_profile>>& profile : _enabled_profiles)
        {
            std::string module_name = _sensors[profile.first].get_info(RS2_CAMERA_INFO_NAME);
            ROS_INFO_STREAM("insert " << rs2_stream_to_string(profile.second.begin()->stream_type())
              << " to " << module_name);
            profiles[module_name].insert(profiles[module_name].begin(),
                                            profile.second.begin(),
                                            profile.second.end());
            active_sensors[module_name] = _sensors[profile.first];
        }

        for (const std::pair<std::string, std::vector<rs2::stream_profile> >& sensor_profile : profiles)
        {
            std::string module_name = sensor_profile.first;
            // for (const rs2::stream_profile& profile : sensor_profile.second)
            // {
            //     ROS_WARN_STREAM("type:" << rs2_stream_to_string(profile.stream_type()) <<
            //                     " fps: " << profile.fps() << ". format: " << profile.format());
            // }
            rs2::sensor sensor = active_sensors[module_name];
            sensor.open(sensor_profile.second);
            sensor.start(_sensors_callback[module_name]);
            if (sensor.is<rs2::depth_sensor>())
            {
                _depth_scale_meters = sensor.as<rs2::depth_sensor>().get_depth_scale();
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
                                          const tf::Quaternion& q,
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
    msg.transform.rotation.x = q.getX();
    msg.transform.rotation.y = q.getY();
    msg.transform.rotation.z = q.getZ();
    msg.transform.rotation.w = q.getW();
    _static_tf_broadcaster.sendTransform(msg);
}

void BaseRealSenseNode::calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile)
{
    // Transform base to stream
    tf::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    float3 zero_trans{0, 0, 0};

    ros::Time transform_ts_ = ros::Time::now();

    rs2_extrinsics ex;
    try
    {
        ex = getAProfile(stream).get_extrinsics_to(base_profile);
    }
    catch (std::exception& e)
    {
        if (!strcmp(e.what(), "Requested extrinsics are not available!"))
        {
            ROS_WARN_STREAM(e.what() << " : using unity as default.");
            ex = rs2_extrinsics({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}});
        }
        else
        {
            throw e;
        }
    }

    auto Q = rotationMatrixToQuaternion(ex.rotation);
    Q = quaternion_optical * Q * quaternion_optical.inverse();

    float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
    publish_static_tf(transform_ts_, trans, Q, _base_frame_id, _frame_id[stream]);

    // Transform stream frame to stream optical frame
    publish_static_tf(transform_ts_, zero_trans, quaternion_optical, _frame_id[stream], _optical_frame_id[stream]);

    if (_align_depth && _depth_aligned_frame_id.find(stream) != _depth_aligned_frame_id.end())
    {
        publish_static_tf(transform_ts_, trans, Q, _base_frame_id, _depth_aligned_frame_id[stream]);
        publish_static_tf(transform_ts_, zero_trans, quaternion_optical, _depth_aligned_frame_id[stream], _optical_frame_id[stream]);
    }
}

void BaseRealSenseNode::publishStaticTransforms()
{
    // Publish static transforms
    const std::vector<stream_index_pair> base_stream_priority = {DEPTH, GYRO};

    std::vector<stream_index_pair>::const_iterator base_stream(base_stream_priority.begin());
    while( (_sensors.find(*base_stream) == _sensors.end()) && (base_stream != base_stream_priority.end()))
    {
        base_stream++;
    }
    if (base_stream == base_stream_priority.end())
    {
        throw std::runtime_error("No known base_stream found for transformations.");
    }
    ROS_INFO_STREAM("SELECTED BASE:" << base_stream->first << ", " << base_stream->second);

    rs2::stream_profile base_profile = getAProfile(*base_stream);
    for (std::pair<stream_index_pair, bool> ienable : _enable)
    {
        if (ienable.second)
        {
            calcAndPublishStaticTransform(ienable.first, base_profile);
        }
    }

    // Publish Extinsics Topics:
    if (_enable[DEPTH] &&
        _enable[FISHEYE])
    {
        static const char* frame_id = "depth_to_fisheye_extrinsics";
        const auto& ex = base_profile.get_extrinsics_to(getAProfile(FISHEYE));

        _depth_to_other_extrinsics[FISHEYE] = ex;
        _depth_to_other_extrinsics_publishers[FISHEYE].publish(rsExtrinsicsToMsg(ex, frame_id));
    }

    if (_enable[DEPTH] &&
        _enable[COLOR])
    {
        static const char* frame_id = "depth_to_color_extrinsics";
        const auto& ex = base_profile.get_extrinsics_to(getAProfile(COLOR));
        _depth_to_other_extrinsics[COLOR] = ex;
        _depth_to_other_extrinsics_publishers[COLOR].publish(rsExtrinsicsToMsg(ex, frame_id));
    }

    if (_enable[DEPTH] &&
        _enable[INFRA1])
    {
        static const char* frame_id = "depth_to_infra1_extrinsics";
        const auto& ex = base_profile.get_extrinsics_to(getAProfile(INFRA1));
        _depth_to_other_extrinsics[INFRA1] = ex;
        _depth_to_other_extrinsics_publishers[INFRA1].publish(rsExtrinsicsToMsg(ex, frame_id));
    }

    if (_enable[DEPTH] &&
        _enable[INFRA2])
    {
        static const char* frame_id = "depth_to_infra2_extrinsics";
        const auto& ex = base_profile.get_extrinsics_to(getAProfile(INFRA2));
        _depth_to_other_extrinsics[INFRA2] = ex;
        _depth_to_other_extrinsics_publishers[INFRA2].publish(rsExtrinsicsToMsg(ex, frame_id));
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
    static int warn_count(0);
    static const int DISPLAY_WARN_NUMBER(5);
    rs2::frameset::iterator texture_frame_itr = frameset.end();
    if (use_texture)
    {
        std::set<rs2_format> available_formats{ rs2_format::RS2_FORMAT_RGB8, rs2_format::RS2_FORMAT_Y8 };
        
        texture_frame_itr = find_if(frameset.begin(), frameset.end(), [&texture_source_id, &available_formats] (rs2::frame f) 
                                {return (rs2_stream(f.get_profile().stream_type()) == texture_source_id) &&
                                            (available_formats.find(f.get_profile().format()) != available_formats.end()); });
        if (texture_frame_itr == frameset.end())
        {
            warn_count++;
            std::string texture_source_name = pc_filter->_filter->get_option_value_description(rs2_option::RS2_OPTION_STREAM_FILTER, static_cast<float>(texture_source_id));
            ROS_WARN_STREAM_COND(warn_count == DISPLAY_WARN_NUMBER, "No stream match for pointcloud chosen texture " << texture_source_name);
            return;
        }
        warn_count = 0;
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

rs2::stream_profile BaseRealSenseNode::getAProfile(const stream_index_pair& stream)
{
    const std::vector<rs2::stream_profile> profiles = _sensors[stream].get_stream_profiles();
    return *(std::find_if(profiles.begin(), profiles.end(),
                                            [&stream] (const rs2::stream_profile& profile) { 
                                                return ((profile.stream_type() == stream.first) && (profile.stream_index() == stream.second)); 
                                            }));
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
                                     const std::map<rs2_stream, std::string>& encoding,
                                     bool copy_data_from_frame)
{
    ROS_DEBUG("publishFrame(...)");
    unsigned int width = 0;
    unsigned int height = 0;
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
            image.create(height, width, image.type());
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
        img = cv_bridge::CvImage(std_msgs::Header(), encoding.at(stream.first), image).toImageMsg();
        img->width = width;
        img->height = height;
        img->is_bigendian = false;
        img->step = width * bpp;
        img->header.frame_id = optical_frame_id.at(stream);
        img->header.stamp = t;
        img->header.seq = seq[stream];

        auto& cam_info = camera_info.at(stream);
        if (cam_info.width != width)
        {
            updateStreamCalibData(f.get_profile().as<rs2::video_stream_profile>());
        }
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

