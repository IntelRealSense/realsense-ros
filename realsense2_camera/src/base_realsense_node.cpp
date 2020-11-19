#include "../include/base_realsense_node.h"
#include "assert.h"
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <algorithm>
#include <mutex>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <fstream>

using namespace realsense2_camera;

SyncedImuPublisher::SyncedImuPublisher(rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher, 
                                       std::size_t waiting_list_size):
            _publisher(imu_publisher), _pause_mode(false),
            _waiting_list_size(waiting_list_size)
            {}

SyncedImuPublisher::~SyncedImuPublisher()
{
    PublishPendingMessages();
}

void SyncedImuPublisher::Publish(sensor_msgs::msg::Imu imu_msg)
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
        _publisher->publish(imu_msg);
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
    while (!_pending_messages.empty())
    {
        const sensor_msgs::msg::Imu &imu_msg = _pending_messages.front();
        _publisher->publish(imu_msg);
        _pending_messages.pop();
    }
}
size_t SyncedImuPublisher::getNumSubscribers()
{ 
    if (!_publisher) return 0;
    return _publisher->get_subscription_count();
}

void PointcloudFilter::set(const bool is_enabled)
{
    if ((is_enabled) && (!_pointcloud_publisher))
    {
        _pointcloud_publisher = _node.create_publisher<sensor_msgs::msg::PointCloud2>("depth/color/points", 1);
    }
    else if ((!is_enabled) && (_pointcloud_publisher))
    {
        _pointcloud_publisher.reset();        
    }
    NamedFilter::set(is_enabled);
}

BaseRealSenseNode::BaseRealSenseNode(rclcpp::Node& node,
                                    rs2::device dev, const std::string& serial_no,
                                    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater) :
    _ros_clock(RCL_ROS_TIME),
    _is_running(true),
    _node(node),
    _logger(rclcpp::get_logger("RealSenseCameraNode")),
    // _rs_diagnostic_updater(diagnostic_updater, serial_no),
    _dev(dev),
    _json_file_path(""),
    _serial_no(serial_no),
    _static_tf_broadcaster(node),
    _is_initialized_time_base(false),
    _is_profile_changed(false),
    _parameters(node)
{
    _image_format[1] = CV_8UC1;    // CVBridge type
    _image_format[2] = CV_16UC1;    // CVBridge type
    _image_format[3] = CV_8UC3;    // CVBridge type
    _encoding[1] = sensor_msgs::image_encodings::MONO8; // ROS message type
    _encoding[2] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
    _encoding[3] = sensor_msgs::image_encodings::RGB8; // ROS message type
    
    // Infrared stream
    _format[RS2_STREAM_INFRARED] = RS2_FORMAT_Y8;

    _monitor_options = {RS2_OPTION_ASIC_TEMPERATURE, RS2_OPTION_PROJECTOR_TEMPERATURE};

    publishTopics();
}

BaseRealSenseNode::~BaseRealSenseNode()
{
    // Kill dynamic transform thread
    if (_tf_t)
        _tf_t->join();

    _is_running = false;
    _cv_temp.notify_one();
    _cv_mpc.notify_one();
    if (_monitoring_t && _monitoring_t->joinable())
    {
        _monitoring_t->join();
    }
    if (_monitoring_pc && _monitoring_pc->joinable())
    {
        _monitoring_pc->join();
    }

    std::set<std::string> module_names;
    for (const std::pair<stream_index_pair, std::vector<rs2::stream_profile>>& profile : _enabled_profiles)
    {
        std::string module_name = _sensors[profile.first].get_info(RS2_CAMERA_INFO_NAME);
        std::pair< std::set<std::string>::iterator, bool> res = module_names.insert(module_name);
        if (res.second)
        {
            _sensors[profile.first].stop();
            _sensors[profile.first].close();
        }
    }
}

void BaseRealSenseNode::setupErrorCallback(const rs2::sensor& sensor)
{
    sensor.set_notifications_callback([&](const rs2::notification& n)
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
            _dev.hardware_reset();
        }
    });
}

void BaseRealSenseNode::publishTopics()
{
    getParameters();
    setupFilters();
    setup();
    // setupDevice();
    // registerDynamicReconfigCb();
    // setupErrorCallback();
    // enable_devices();
    // setupPublishers();
    // setupStreams();
    // SetBaseStream();
//    registerAutoExposureROIOptions(_node_handle);
    // publishStaticTransforms();
    // publishIntrinsics();
    // startMonitoring();   TODO: do
    ROS_INFO_STREAM("RealSense Node Is Up!");
}

void BaseRealSenseNode::runFirstFrameInitialization(rs2_stream stream_type)
{
    if (_is_first_frame[stream_type])
    {
        ROS_DEBUG_STREAM("runFirstFrameInitialization: " << _video_functions_stack.size() << ", " << rs2_stream_to_string(stream_type));
        _is_first_frame[stream_type] = false;
        if (!_video_functions_stack[stream_type].empty())
        {
            std::thread t = std::thread([=]()
            {
                while (!_video_functions_stack[stream_type].empty())
                {
                    _video_functions_stack[stream_type].back()();
                    _video_functions_stack[stream_type].pop_back();
                }
            });
            t.detach();
        }
    }
}
// bool is_checkbox(rs2::options sensor, rs2_option option)
// {
//     rs2::option_range op_range = sensor.get_option_range(option);
//     return op_range.max == 1.0f &&
//         op_range.min == 0.0f &&
//         op_range.step == 1.0f;
// }

// bool is_enum_option(rs2::options sensor, rs2_option option)
// {
//     static const int MAX_ENUM_OPTION_VALUES(100);
//     static const float EPSILON(0.05);
    
//     rs2::option_range op_range = sensor.get_option_range(option);
//     if (abs((op_range.step - 1)) > EPSILON || (op_range.max > MAX_ENUM_OPTION_VALUES)) return false;
//     for (auto i = op_range.min; i <= op_range.max; i += op_range.step)
//     {
//         if (sensor.get_option_value_description(option, i) == nullptr)
//             continue;
//         return true;
//     }
//     return false;
// }

// bool is_int_option(rs2::options sensor, rs2_option option)
// {
//     rs2::option_range op_range = sensor.get_option_range(option);
//     return (op_range.step == 1.0);
// }

// std::map<std::string, int> get_enum_method(rs2::options sensor, rs2_option option)
// {
//     std::map<std::string, int> dict; // An enum to set size
//     if (is_enum_option(sensor, option))
//     {
//         rs2::option_range op_range = sensor.get_option_range(option);
//         const auto op_range_min = int(op_range.min);
//         const auto op_range_max = int(op_range.max);
//         const auto op_range_step = int(op_range.step);
//         for (auto val = op_range_min; val <= op_range_max; val += op_range_step)
//         {
//             if (sensor.get_option_value_description(option, val) == nullptr)
//                 continue;
//             dict[sensor.get_option_value_description(option, val)] = val;
//         }
//     }
//     return dict;
// }


// void BaseRealSenseNode::set_auto_exposure_roi(const std::string option_name, rs2::sensor sensor, int new_value)
// {
//     rs2::region_of_interest& auto_exposure_roi(_auto_exposure_roi[sensor.get_info(RS2_CAMERA_INFO_NAME)]);
//     if (option_name == "left")
//         auto_exposure_roi.min_x = new_value;
//     else if (option_name == "right")
//         auto_exposure_roi.max_x = new_value;
//     else if (option_name == "top")
//         auto_exposure_roi.min_y = new_value;
//     else if (option_name == "bottom")
//         auto_exposure_roi.max_y = new_value;
//     else
//     {
//         ROS_WARN_STREAM("Invalid option_name: " << option_name << " while setting auto exposure ROI.");
//         return;
//     }
//     set_sensor_auto_exposure_roi(sensor);
// }

// void BaseRealSenseNode::set_sensor_auto_exposure_roi(rs2::sensor sensor)
// {
//     const rs2::region_of_interest& auto_exposure_roi(_auto_exposure_roi[sensor.get_info(RS2_CAMERA_INFO_NAME)]);
//     try
//     {
//         sensor.as<rs2::roi_sensor>().set_region_of_interest(auto_exposure_roi);
//     }
//     catch(const std::runtime_error& e)
//     {
//         ROS_ERROR_STREAM(e.what());
//     }
// }

// #ifdef false
// void BaseRealSenseNode::readAndSetDynamicParam(ros::NodeHandle& nh1, std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec, 
//                                                const std::string option_name, const int min_val, const int max_val, rs2::sensor sensor, 
//                                                int* option_value)
// {
//     nh1.param(option_name, *option_value, *option_value); //param (const std::string &param_name, T &param_val, const T &default_val) const
//     if (*option_value < min_val) *option_value = min_val;
//     if (*option_value > max_val) *option_value = max_val;
    
//     ddynrec->registerVariable<int>(
//         option_name, *option_value, [this, sensor, option_name](int new_value){set_auto_exposure_roi(option_name, sensor, new_value);},
//         "auto-exposure " + option_name + " coordinate", min_val, max_val);
// }

// void BaseRealSenseNode::registerAutoExposureROIOptions(ros::NodeHandle& nh)
// {
//     for (const std::pair<stream_index_pair, std::vector<rs2::stream_profile>>& profile : _enabled_profiles)
//     {
//         rs2::sensor sensor = _sensors[profile.first];
//         std::string module_base_name(sensor.get_info(RS2_CAMERA_INFO_NAME));
//         if (sensor.is<rs2::roi_sensor>() && _auto_exposure_roi.find(module_base_name) == _auto_exposure_roi.end())
//         {
//             int max_x(_width[profile.first]-1);
//             int max_y(_height[profile.first]-1);

//             std::string module_name = create_graph_resource_name(module_base_name) +"/auto_exposure_roi";
//             ros::NodeHandle nh1(nh, module_name);
//             std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh1);

//             _auto_exposure_roi[module_base_name] = {0, 0, max_x, max_y};
//             rs2::region_of_interest& auto_exposure_roi(_auto_exposure_roi[module_base_name]);
//             readAndSetDynamicParam(nh1, ddynrec, "left", 0, max_x, sensor, &(auto_exposure_roi.min_x));
//             readAndSetDynamicParam(nh1, ddynrec, "right", 0, max_x, sensor, &(auto_exposure_roi.max_x));
//             readAndSetDynamicParam(nh1, ddynrec, "top", 0, max_y, sensor, &(auto_exposure_roi.min_y));
//             readAndSetDynamicParam(nh1, ddynrec, "bottom", 0, max_y, sensor, &(auto_exposure_roi.max_y));

//             ddynrec->publishServicesTopics();
//             _ddynrec.push_back(ddynrec);

//             // Initiate the call to set_sensor_auto_exposure_roi, after the first frame arrive.
//             rs2_stream stream_type = profile.first.first;
//             _video_functions_stack[stream_type].push_back([this, sensor](){set_sensor_auto_exposure_roi(sensor);});
//             _is_first_frame[stream_type] = true;
//         }
//     }
// }
// #endif //#ifdef false

// void BaseRealSenseNode::registerDynamicOption(rs2::options sensor, std::string& module_name)
// {
//     rclcpp::Parameter node_param;
//     for (auto i = 0; i < RS2_OPTION_COUNT; i++)
//     {
//         rs2_option option = static_cast<rs2_option>(i);
//         const std::string option_name(module_name + "." + create_graph_resource_name(rs2_option_to_string(option)));
//         if (!sensor.supports(option) || sensor.is_option_read_only(option))
//         {
//             continue;
//         }
//         if (is_checkbox(sensor, option))
//         {
//             auto option_value = bool(sensor.get_option(option));
//             // if (nh1.param(option_name, option_value, option_value))
//             // if (_node.get_parameter(option_name, node_param))
//             // {
//             //     option_value = node_param.as_bool();
//             //     sensor.set_option(option, option_value);
//             // }
//             rcl_interfaces::msg::IntegerRange range;
//             range.from_value = 0;
//             range.to_value = 1;
//             rcl_interfaces::msg::ParameterDescriptor crnt_descriptor;
//             crnt_descriptor.description = sensor.get_option_description(option);
//             crnt_descriptor.integer_range.push_back(range);
//             ROS_DEBUG_STREAM("Declare: " << option_name);
//             bool new_val = _node.declare_parameter(option_name, rclcpp::ParameterValue(option_value), crnt_descriptor).get<rclcpp::PARAMETER_BOOL>();
//             if (new_val != option_value)
//             {
//                 sensor.set_option(option, new_val);
//             }
//             continue;
//         }
// #ifdef false        
//         const auto enum_dict = get_enum_method(sensor, option);
//         if (enum_dict.empty())
//         {
//             rs2::option_range op_range = sensor.get_option_range(option);
//             const auto sensor_option_value = sensor.get_option(option);
//             auto option_value = sensor_option_value;
//             if (nh1.param(option_name, option_value, option_value))
//             {
//                 if (option_value < op_range.min || op_range.max < option_value)
//                 {
//                     ROS_WARN_STREAM("Param '" << nh1.resolveName(option_name) << "' has value " << option_value
//                             << " outside the range [" << op_range.min << ", " << op_range.max
//                             << "]. Using current sensor value " << sensor_option_value << " instead.");
//                     option_value = sensor_option_value;
//                 }
//                 else
//                 {
//                     sensor.set_option(option, option_value);
//                 }
//             }
//             if (is_int_option(sensor, option))
//             {
//               ddynrec->registerVariable<int>(
//                   option_name, int(option_value),
//                   [option, sensor](int new_value) { sensor.set_option(option, new_value); },
//                   sensor.get_option_description(option), int(op_range.min), int(op_range.max));
//             }
//             else
//             {
//                 if (i == RS2_OPTION_DEPTH_UNITS)
//                 {
//                     if (ROS_DEPTH_SCALE >= op_range.min && ROS_DEPTH_SCALE <= op_range.max)
//                     {
//                         sensor.set_option(option, ROS_DEPTH_SCALE);
//                         op_range.min = ROS_DEPTH_SCALE;
//                         op_range.max = ROS_DEPTH_SCALE;

//                         _depth_scale_meters = ROS_DEPTH_SCALE;
//                     }
//                 }
//                 else
//                 {
//                   ddynrec->registerVariable<double>(
//                       option_name, option_value,
//                       [option, sensor](double new_value) { sensor.set_option(option, new_value); },
//                       sensor.get_option_description(option), double(op_range.min), double(op_range.max));
//                 }
//             }
//         }
//         else
//         {
//             const auto sensor_option_value = sensor.get_option(option);
//             auto option_value = int(sensor_option_value);
//             if (nh1.param(option_name, option_value, option_value))
//             {
//                 if (std::find_if(enum_dict.cbegin(), enum_dict.cend(),
//                                  [&option_value](const std::pair<std::string, int>& kv) {
//                                      return kv.second == option_value;
//                                  }) == enum_dict.cend())
//                 {
//                     ROS_WARN_STREAM("Param '" << nh1.resolveName(option_name) << "' has value " << option_value
//                                               << " that is not in the enum " << enum_dict
//                                               << ". Using current sensor value " << sensor_option_value << " instead.");
//                     option_value = sensor_option_value;
//                 }
//                 else
//                 {
//                     sensor.set_option(option, option_value);
//                 }
//             }
//             ddynrec->registerEnumVariable<int>(
//                 option_name, option_value,
//                 [option, sensor](int new_value) { sensor.set_option(option, new_value); },
//                 sensor.get_option_description(option), enum_dict);
//         }
// #endif // false        
//     }
//     // ddynrec->publishServicesTopics();
//     // _ddynrec.push_back(ddynrec);
// }

// void BaseRealSenseNode::registerDynamicReconfigCb()
// {
//     ROS_INFO("Setting Dynamic reconfig parameters.");

//     for(rs2::sensor sensor : _dev_sensors)
//     {
//         std::string module_name = create_graph_resource_name(sensor.get_info(RS2_CAMERA_INFO_NAME));
//         ROS_DEBUG_STREAM("module_name:" << module_name);
//         registerDynamicOption(sensor, module_name);
//     }

//     for (NamedFilter nfilter : _filters)
//     {
//         std::string module_name = nfilter._name;
//         auto sensor = *(nfilter._filter);
//         ROS_DEBUG_STREAM("module_name:" << module_name);
//         registerDynamicOption(sensor, module_name);
//     }
//     ROS_INFO("Done Setting Dynamic reconfig parameters.");
// }

void BaseRealSenseNode::publishAlignedDepthToOthers(rs2::frameset frames, const rclcpp::Time& t)
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

        if(0 != info_publisher->get_subscription_count() ||
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
                         _depth_aligned_image_publishers);
        }
    }
}

void BaseRealSenseNode::setupFilters()
{
    _filters.push_back(std::make_shared<NamedFilter>("decimation", std::make_shared<rs2::decimation_filter>()));
    _filters.push_back(std::make_shared<NamedFilter>("disparity_start", std::make_shared<rs2::disparity_transform>()));
    _filters.push_back(std::make_shared<NamedFilter>("spatial", std::make_shared<rs2::spatial_filter>()));
    _filters.push_back(std::make_shared<NamedFilter>("temporal", std::make_shared<rs2::temporal_filter>()));
    _filters.push_back(std::make_shared<NamedFilter>("hole_filling", std::make_shared<rs2::hole_filling_filter>()));
    _filters.push_back(std::make_shared<NamedFilter>("disparity_end", std::make_shared<rs2::disparity_transform>(false)));
    _filters.push_back(std::make_shared<NamedFilter>("colorizer", std::make_shared<rs2::colorizer>())); // TODO: Callback must take care of depth image_format, encoding etc.
    _filters.push_back(std::make_shared<PointcloudFilter>("pointcloud", std::make_shared<rs2::pointcloud>(_pointcloud_texture.first, _pointcloud_texture.second), _node, false));

    for (auto nfilter : _filters)
    {
        if (nfilter->_name == "disparity_end")
            continue;
        std::stringstream param_name;
        auto sensor = *(nfilter->_filter);
        std::string module_name = create_graph_resource_name(sensor.get_info(RS2_CAMERA_INFO_NAME));
        param_name << "post_processing_block." << module_name;
        ROS_DEBUG_STREAM("module_name:" << param_name.str());
        registerDynamicOptions(sensor, param_name.str());
        param_name << ".enable";
        _parameters.setParam(param_name.str(), rclcpp::ParameterValue(nfilter->_is_enabled), [this, nfilter](const rclcpp::Parameter& parameter)
                {
                    nfilter->set(parameter.get_value<bool>());
                });
    }
}

// void BaseRealSenseNode::setupFilters()
// {
//     ROS_WARN_STREAM("READING parameters:");
//     std::vector<std::string> parameter_prefixes({"post_processing_block"});
//     rcl_interfaces::msg::ListParametersResult pp_params = _node.list_parameters(parameter_prefixes, 10);
//     bool use_disparity_filter(false);
//     bool use_colorizer_filter(false);
//     bool use_decimation_filter(false);
//     for (auto & param_name : pp_params.names)
//     {
//         std::string filter_name;
//         ROS_WARN_STREAM("parameter: " << param_name);
//         std::vector<std::string> param_parts;
//         boost::split(param_parts, param_name, [](char c){return c == '.';});
//         for (int i=0; i<param_parts.size(); i++)
//         {
//             if (param_parts[i] == "post_processing_block")
//             {
//                 filter_name = param_parts[i+1];
//             }
//         }
//         if (filter_name == "colorizer")
//         {
//             use_colorizer_filter = true;
//         }
//         else if (filter_name == "disparity")
//         {
//             use_disparity_filter = true;
//         }
//         else if (filter_name == "decimation")
//         {
//             use_decimation_filter = true;
//         }
//         else
//         {
//             bool filter_enabled = _node.get_parameter(param_name).get_parameter_value().get<rclcpp::PARAMETER_BOOL>();
//             std::vector<NamedFilter>::const_iterator filter_iter = find_if(_filters.begin(), _filters.end(), [&filter_name] (NamedFilter filter) 
//                                         {return (filter._name == filter_name); });

//             if (!filter_enabled && _filters.end() != filter_iter)
//             {
//                 ROS_WARN_STREAM("filter: " << filter_name << " exists.");
//                 _filters.erase(filter_iter);
//             }
//             else if (filter_enabled && _filters.end() == filter_iter)
//             {
//                 ROS_WARN_STREAM("filter: " << filter_name << " NOT exists.");
//                 if (filter_name == "spatial")
//                 {
//                     ROS_INFO("Add Filter: spatial");
//                     _filters.push_back(NamedFilter("spatial", std::make_shared<rs2::spatial_filter>()));
//                 }
//                 else if (filter_name == "temporal")
//                 {
//                     ROS_INFO("Add Filter: temporal");
//                     _filters.push_back(NamedFilter("temporal", std::make_shared<rs2::temporal_filter>()));
//                 }
//                 else if (filter_name == "hole_filling")
//                 {
//                     ROS_INFO("Add Filter: hole_filling");
//                     _filters.push_back(NamedFilter("hole_filling", std::make_shared<rs2::hole_filling_filter>()));
//                 }
//             }
//         }
        
//         // else if ((*s_iter) == "pointcloud")
//         // {
//         //     assert(_pointcloud); // For now, it is set in getParameters()..
//         // }
//         // if (_node.get_parameter(param_name).get_parameter_value().get<rclcpp::PARAMETER_BOOL>())
//         // {

//         // }

//     }
// }

// void BaseRealSenseNode::setupFilters()
// {
//     std::vector<std::string> filters_str;
//     boost::split(filters_str, _filters_str, [](char c){return c == ',';});
//     bool use_disparity_filter(false);
//     bool use_colorizer_filter(false);
//     bool use_decimation_filter(false);
//     for (std::vector<std::string>::const_iterator s_iter=filters_str.begin(); s_iter!=filters_str.end(); s_iter++)
//     {
//         if ((*s_iter) == "colorizer")
//         {
//             use_colorizer_filter = true;
//         }
//         else if ((*s_iter) == "disparity")
//         {
//             use_disparity_filter = true;
//         }
//         else if ((*s_iter) == "spatial")
//         {
//             ROS_INFO("Add Filter: spatial");
//             _filters.push_back(NamedFilter("spatial", std::make_shared<rs2::spatial_filter>()));
//         }
//         else if ((*s_iter) == "temporal")
//         {
//             ROS_INFO("Add Filter: temporal");
//             _filters.push_back(NamedFilter("temporal", std::make_shared<rs2::temporal_filter>()));
//         }
//         else if ((*s_iter) == "hole_filling")
//         {
//             ROS_INFO("Add Filter: hole_filling");
//             _filters.push_back(NamedFilter("hole_filling", std::make_shared<rs2::hole_filling_filter>()));
//         }
//         else if ((*s_iter) == "decimation")
//         {
//             use_decimation_filter = true;
//         }
//         else if ((*s_iter) == "pointcloud")
//         {
//             assert(_pointcloud); // For now, it is set in getParameters()..
//         }
//         else if ((*s_iter).size() > 0)
//         {
//             ROS_ERROR_STREAM("Unknown Filter: " << (*s_iter));
//             throw;
//         }
//     }
//     if (use_disparity_filter)
//     {
//         ROS_INFO("Add Filter: disparity");
//         _filters.insert(_filters.begin(), NamedFilter("disparity_start", std::make_shared<rs2::disparity_transform>()));
//         _filters.push_back(NamedFilter("disparity_end", std::make_shared<rs2::disparity_transform>(false)));
//         ROS_INFO("Done Add Filter: disparity");
//     }
//     if (use_decimation_filter)
//     {
//       ROS_INFO("Add Filter: decimation");
//       _filters.insert(_filters.begin(),NamedFilter("decimation", std::make_shared<rs2::decimation_filter>()));
//     }
//     if (use_colorizer_filter)
//     {
//         ROS_INFO("Add Filter: colorizer");
//         _filters.push_back(NamedFilter("colorizer", std::make_shared<rs2::colorizer>()));
//     }
//     if (_pointcloud)
//     {
//     	ROS_INFO("Add Filter: pointcloud");
//         // Get PointCloud parameters:
//         std::string pc_texture_stream = _node.declare_parameter("pointcloud_texture_stream", rclcpp::ParameterValue("RS2_STREAM_COLOR")).get<rclcpp::PARAMETER_STRING>();
//         int pc_texture_idx = _node.declare_parameter("pointcloud_texture_index", rclcpp::ParameterValue(0)).get<rclcpp::PARAMETER_INTEGER>();
//         _pointcloud_texture = stream_index_pair{rs2_string_to_stream(pc_texture_stream), pc_texture_idx};
//         if (!_pointcloud_publisher)
//         {
//             ROS_INFO("Start pointcloud publisher.");
//             _pointcloud_publisher = _node.create_publisher<sensor_msgs::msg::PointCloud2>("depth/color/points", 1);
//         }
//         _filters.push_back(NamedFilter("pointcloud", std::make_shared<rs2::pointcloud>(_pointcloud_texture.first, _pointcloud_texture.second)));
//     }
//     ROS_INFO("num_filters: %d", static_cast<int>(_filters.size()));
// }



cv::Mat& BaseRealSenseNode::fix_depth_scale(const cv::Mat& from_image, cv::Mat& to_image)
{
    static const float meter_to_mm = 0.001f;
    if (fabs(_depth_scale_meters - meter_to_mm) < 1e-6)
    {
        to_image = from_image;
        return to_image;
    }

    if (to_image.size() != from_image.size())
    {
        to_image.create(from_image.rows, from_image.cols, from_image.type());
    }

    CV_Assert(from_image.depth() == _image_format[2]);

    int nRows = from_image.rows;
    int nCols = from_image.cols;

    if (from_image.isContinuous())
    {
        nCols *= nRows;
        nRows = 1;
    }

    int i,j;
    const uint16_t* p_from;
    uint16_t* p_to;
    for( i = 0; i < nRows; ++i)
    {
        p_from = from_image.ptr<uint16_t>(i);
        p_to = to_image.ptr<uint16_t>(i);
        for ( j = 0; j < nCols; ++j)
        {
            p_to[j] = p_from[j] * _depth_scale_meters / meter_to_mm;
        }
    }
    return to_image;
}

void BaseRealSenseNode::clip_depth(rs2::depth_frame depth_frame, float clipping_dist)
{
    uint16_t* p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));
    uint16_t clipping_value = static_cast<uint16_t>(clipping_dist / _depth_scale_meters);

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
            // Check if the depth value is greater than the threashold
            if (p_depth_frame[depth_pixel_index] > clipping_value)
            {
                p_depth_frame[depth_pixel_index] = 0; //Set to invalid (<=0) value.
            }
        }
    }
}

sensor_msgs::msg::Imu BaseRealSenseNode::CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data)
{
    sensor_msgs::msg::Imu imu_msg;
    rclcpp::Time t(gyro_data.m_time_ns);  //rclcpp::Time(uint64_t nanoseconds)
    imu_msg.header.stamp = t;

    imu_msg.angular_velocity.x = gyro_data.m_data.x();
    imu_msg.angular_velocity.y = gyro_data.m_data.y();
    imu_msg.angular_velocity.z = gyro_data.m_data.z();

    imu_msg.linear_acceleration.x = accel_data.m_data.x();
    imu_msg.linear_acceleration.y = accel_data.m_data.y();
    imu_msg.linear_acceleration.z = accel_data.m_data.z();
    return imu_msg;
}

template <typename T> T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

void BaseRealSenseNode::FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs)
{
    static std::deque<CimuData> _imu_history;
    _imu_history.push_back(imu_data);
    stream_index_pair type(imu_data.m_type);
    imu_msgs.clear();

    if ((type != ACCEL) || _imu_history.size() < 3)
        return;
    
    std::deque<CimuData> gyros_data;
    CimuData accel0, accel1, crnt_imu;

    while (_imu_history.size()) 
    {
        crnt_imu = _imu_history.front();
        _imu_history.pop_front();
        if (!accel0.is_set() && crnt_imu.m_type == ACCEL) 
        {
            accel0 = crnt_imu;
        } 
        else if (accel0.is_set() && crnt_imu.m_type == ACCEL) 
        {
            accel1 = crnt_imu;
            const double dt = accel1.m_time_ns - accel0.m_time_ns;

            while (gyros_data.size())
            {
                CimuData crnt_gyro = gyros_data.front();
                gyros_data.pop_front();
                const double alpha = (crnt_gyro.m_time_ns - accel0.m_time_ns) / dt;
                CimuData crnt_accel(ACCEL, lerp(accel0.m_data, accel1.m_data, alpha), crnt_gyro.m_time_ns);
                imu_msgs.push_back(CreateUnitedMessage(crnt_accel, crnt_gyro));
            }
            accel0 = accel1;
        } 
        else if (accel0.is_set() && crnt_imu.m_time_ns >= accel0.m_time_ns && crnt_imu.m_type == GYRO)
        {
            gyros_data.push_back(crnt_imu);
        }
    }
    _imu_history.push_back(crnt_imu);
    return;
}

void BaseRealSenseNode::FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs)
{
    stream_index_pair type(imu_data.m_type);

    static CimuData _accel_data(ACCEL, {0,0,0}, -1.0);
    if (ACCEL == type)
    {
        _accel_data = imu_data;
        return;
    }
    if (!_accel_data.is_set())
        return;

    imu_msgs.push_back(CreateUnitedMessage(_accel_data, imu_data));
}

void BaseRealSenseNode::ImuMessage_AddDefaultValues(sensor_msgs::msg::Imu& imu_msg)
{
    imu_msg.header.frame_id = DEFAULT_IMU_OPTICAL_FRAME_ID;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 0.0;

    imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    imu_msg.linear_acceleration_covariance = { _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov};
    imu_msg.angular_velocity_covariance = { _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov};
}

void BaseRealSenseNode::imu_callback_sync(rs2::frame frame, imu_sync_method sync_method)
{
    static std::mutex m_mutex;

    m_mutex.lock();

    auto stream = frame.get_profile().stream_type();
    auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
    double frame_time = frame.get_timestamp();

    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
        setBaseTime(frame_time, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME == frame.get_frame_timestamp_domain());
    }

    double elapsed_camera_ns = (/*ms*/ frame_time - /*ms*/ _camera_time_base) * 1000.0;

    if (0 != _synced_imu_publisher->getNumSubscribers())
    {
        auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
        CimuData imu_data(stream_index, v, elapsed_camera_ns);
        std::deque<sensor_msgs::msg::Imu> imu_msgs;
        switch (sync_method)
        {
            case NONE: //Cannot really be NONE. Just to avoid compilation warning.
            case COPY:
                FillImuData_Copy(imu_data, imu_msgs);
                break;
            case LINEAR_INTERPOLATION:
                FillImuData_LinearInterpolation(imu_data, imu_msgs);
                break;
        }
        while (imu_msgs.size())
        {
            sensor_msgs::msg::Imu imu_msg = imu_msgs.front();
            rclcpp::Time t(_ros_time_base + rclcpp::Duration(imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec));
            imu_msg.header.stamp = t;
            ImuMessage_AddDefaultValues(imu_msg);
            _synced_imu_publisher->Publish(imu_msg);
            ROS_DEBUG("Publish united %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
            imu_msgs.pop_front();
        }
    }
    m_mutex.unlock();
}

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
    if (0 != _imu_publishers[stream_index]->get_subscription_count())
    {
        double elapsed_camera_ns = (/*ms*/ frame_time - /*ms*/ _camera_time_base) * 1000.0;
        rclcpp::Time t(_ros_time_base + rclcpp::Duration(elapsed_camera_ns));

        auto imu_msg = sensor_msgs::msg::Imu();
        ImuMessage_AddDefaultValues(imu_msg);
        imu_msg.header.frame_id = OPTICAL_FRAME_ID(stream_index);

        auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
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
        imu_msg.header.stamp = t;
        _imu_publishers[stream_index]->publish(imu_msg);
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
    rs2_pose pose = frame.as<rs2::pose_frame>().get_pose_data();
    double elapsed_camera_ns = (/*ms*/ frame_time - /*ms*/ _camera_time_base) * 1000.0;
    rclcpp::Time t(_ros_time_base + rclcpp::Duration(elapsed_camera_ns));

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose.position.x = -pose.translation.z;
    pose_msg.pose.position.y = -pose.translation.x;
    pose_msg.pose.position.z = pose.translation.y;
    pose_msg.pose.orientation.x = -pose.rotation.z;
    pose_msg.pose.orientation.y = -pose.rotation.x;
    pose_msg.pose.orientation.z = pose.rotation.y;
    pose_msg.pose.orientation.w = pose.rotation.w;

    static tf2_ros::TransformBroadcaster br(_node);
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = DEFAULT_ODOM_FRAME_ID;
    msg.child_frame_id = FRAME_ID(POSE);
    msg.transform.translation.x = pose_msg.pose.position.x;
    msg.transform.translation.y = pose_msg.pose.position.y;
    msg.transform.translation.z = pose_msg.pose.position.z;
    msg.transform.rotation.x = pose_msg.pose.orientation.x;
    msg.transform.rotation.y = pose_msg.pose.orientation.y;
    msg.transform.rotation.z = pose_msg.pose.orientation.z;
    msg.transform.rotation.w = pose_msg.pose.orientation.w;

    if (_publish_odom_tf) br.sendTransform(msg);

    if (0 != _odom_publisher->get_subscription_count())
    {
        double cov_pose(_linear_accel_cov * pow(10, 3-(int)pose.tracker_confidence));
        double cov_twist(_angular_velocity_cov * pow(10, 1-(int)pose.tracker_confidence));

        geometry_msgs::msg::Vector3Stamped v_msg;
        tf2::Vector3 tfv(-pose.velocity.z, -pose.velocity.x, pose.velocity.y);
        tf2::Quaternion q(-msg.transform.rotation.x,-msg.transform.rotation.y,-msg.transform.rotation.z,msg.transform.rotation.w);
        tfv=tf2::quatRotate(q,tfv);
        v_msg.vector.x = tfv.x();
        v_msg.vector.y = tfv.y();
        v_msg.vector.z = tfv.z();
	
        tfv = tf2::Vector3(-pose.angular_velocity.z, -pose.angular_velocity.x, pose.angular_velocity.y);
        tfv=tf2::quatRotate(q,tfv);
        geometry_msgs::msg::Vector3Stamped om_msg;
        om_msg.vector.x = tfv.x();
        om_msg.vector.y = tfv.y();
        om_msg.vector.z = tfv.z();	

        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.frame_id = DEFAULT_ODOM_FRAME_ID;
        odom_msg.child_frame_id = FRAME_ID(POSE);
        odom_msg.header.stamp = t;
        odom_msg.pose.pose = pose_msg.pose;
        odom_msg.pose.covariance = {cov_pose, 0, 0, 0, 0, 0,
                                    0, cov_pose, 0, 0, 0, 0,
                                    0, 0, cov_pose, 0, 0, 0,
                                    0, 0, 0, cov_twist, 0, 0,
                                    0, 0, 0, 0, cov_twist, 0,
                                    0, 0, 0, 0, 0, cov_twist};
        odom_msg.twist.twist.linear = v_msg.vector;
        odom_msg.twist.twist.angular = om_msg.vector;
        odom_msg.twist.covariance ={cov_pose, 0, 0, 0, 0, 0,
                                    0, cov_pose, 0, 0, 0, 0,
                                    0, 0, cov_pose, 0, 0, 0,
                                    0, 0, 0, cov_twist, 0, 0,
                                    0, 0, 0, 0, cov_twist, 0,
                                    0, 0, 0, 0, 0, cov_twist};
        _odom_publisher->publish(odom_msg);
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

        rclcpp::Time t;
        if (_sync_frames)
        {
            t = _ros_clock.now();
        }
        else
        {
            double elapsed_camera_ns = (/*ms*/ frame_time - /*ms*/ _camera_time_base) * 1000.0;
            t = rclcpp::Time(_ros_time_base + rclcpp::Duration(elapsed_camera_ns));
        }

        if (frame.is<rs2::frameset>())
        {
            ROS_DEBUG("Frameset arrived.");
            bool is_depth_arrived = false;
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
                            rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), stream_unique_id, frame.get_frame_number(), frame_time, t.nanoseconds());
                runFirstFrameInitialization(stream_type);
            }
            // Clip depth_frame for max range:
            rs2::depth_frame depth_frame = frameset.get_depth_frame();
            if (depth_frame && _clipping_distance > 0)
            {
                clip_depth(depth_frame, _clipping_distance);
            }

            ROS_DEBUG("num_filters: %d", static_cast<int>(_filters.size()));
            for (auto filter_it : _filters)
            {
                if (filter_it->_is_enabled)
                {
                    ROS_DEBUG("Applying filter: %s", filter_it->_name.c_str());
                    frameset = filter_it->_filter->process(frameset);
                }
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
                            rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), stream_unique_id, frame.get_frame_number(), frame_time, t.nanoseconds());
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
                            rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), frame.get_frame_number(), frame_time, t.nanoseconds());

                if (f.is<rs2::points>())
                {
                    publishPointCloud(f.as<rs2::points>(), t, frameset);
                    continue;
                }
                stream_index_pair sip{stream_type,stream_index};
                publishFrame(f, t,
                                sip,
                                _image,
                                _info_publisher,
                                _image_publishers);
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
                        rs2_stream_to_string(stream_type), stream_index, frame.get_frame_number(), frame_time, t.nanoseconds());
            runFirstFrameInitialization(stream_type);

            stream_index_pair sip{stream_type,stream_index};
            if (frame.is<rs2::depth_frame>())
            {
                if (_clipping_distance > 0)
                {
                    clip_depth(frame, _clipping_distance);
                }
            }
            publishFrame(frame, t,
                            sip,
                            _image,
                            _info_publisher,
                            _image_publishers);
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An error has occurred during frame callback: " << ex.what());
    }
    _synced_imu_publisher->Resume();
} // frame_callback

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

    _ros_time_base = _ros_clock.now();
    _camera_time_base = frame_time;
}

void BaseRealSenseNode::updateProfilesStreamCalibData(const std::vector<rs2::stream_profile>& profiles)
{
    for (auto& profile : profiles)
    {
        if (profile.is<rs2::video_stream_profile>())
        {
            updateStreamCalibData(profile.as<rs2::video_stream_profile>());
        }
    }
}

void BaseRealSenseNode::updateStreamCalibData(const rs2::video_stream_profile& video_profile)
{
    stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};
    auto intrinsic = video_profile.get_intrinsics();
    _stream_intrinsics[stream_index] = intrinsic;
    _camera_info[stream_index].width = intrinsic.width;
    _camera_info[stream_index].height = intrinsic.height;
    _camera_info[stream_index].header.frame_id = OPTICAL_FRAME_ID(stream_index);

    _camera_info[stream_index].k.at(0) = intrinsic.fx;
    _camera_info[stream_index].k.at(2) = intrinsic.ppx;
    _camera_info[stream_index].k.at(4) = intrinsic.fy;
    _camera_info[stream_index].k.at(5) = intrinsic.ppy;
    _camera_info[stream_index].k.at(8) = 1;

    _camera_info[stream_index].p.at(0) = _camera_info[stream_index].k.at(0);
    _camera_info[stream_index].p.at(1) = 0;
    _camera_info[stream_index].p.at(2) = _camera_info[stream_index].k.at(2);
    _camera_info[stream_index].p.at(3) = 0;
    _camera_info[stream_index].p.at(4) = 0;
    _camera_info[stream_index].p.at(5) = _camera_info[stream_index].k.at(4);
    _camera_info[stream_index].p.at(6) = _camera_info[stream_index].k.at(5);
    _camera_info[stream_index].p.at(7) = 0;
    _camera_info[stream_index].p.at(8) = 0;
    _camera_info[stream_index].p.at(9) = 0;
    _camera_info[stream_index].p.at(10) = 1;
    _camera_info[stream_index].p.at(11) = 0;

    if (intrinsic.model == RS2_DISTORTION_KANNALA_BRANDT4)
    {
        _camera_info[stream_index].distortion_model = "equidistant";
    } else {
        _camera_info[stream_index].distortion_model = "plumb_bob";
    }

    // set R (rotation matrix) values to identity matrix
    _camera_info[stream_index].r.at(0) = 1.0;
    _camera_info[stream_index].r.at(1) = 0.0;
    _camera_info[stream_index].r.at(2) = 0.0;
    _camera_info[stream_index].r.at(3) = 0.0;
    _camera_info[stream_index].r.at(4) = 1.0;
    _camera_info[stream_index].r.at(5) = 0.0;
    _camera_info[stream_index].r.at(6) = 0.0;
    _camera_info[stream_index].r.at(7) = 0.0;
    _camera_info[stream_index].r.at(8) = 1.0;

    _camera_info[stream_index].d.resize(5);
    for (int i = 0; i < 5; i++)
    {
        _camera_info[stream_index].d.at(i) = intrinsic.coeffs[i];
    }

    if (stream_index == DEPTH && _enable[DEPTH] && _enable[COLOR])
    {
        _camera_info[stream_index].p.at(3) = 0;     // Tx
        _camera_info[stream_index].p.at(7) = 0;     // Ty
    }
}

tf2::Quaternion BaseRealSenseNode::rotationMatrixToQuaternion(const float rotation[9]) const
{
    Eigen::Matrix3f m;
    // We need to be careful about the order, as RS2 rotation matrix is
    // column-major, while Eigen::Matrix3f expects row-major.
    m << rotation[0], rotation[3], rotation[6],
         rotation[1], rotation[4], rotation[7],
         rotation[2], rotation[5], rotation[8];
    Eigen::Quaternionf q(m);
    return tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
}

void BaseRealSenseNode::publish_static_tf(const rclcpp::Time& t,
                                          const float3& trans,
                                          const tf2::Quaternion& q,
                                          const std::string& from,
                                          const std::string& to)
{
    geometry_msgs::msg::TransformStamped msg;
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
    _static_tf_msgs.push_back(msg);
}

void BaseRealSenseNode::calcAndPublishStaticTransform(const rs2::stream_profile& profile, const rs2::stream_profile& base_profile)
{
    // Transform base to stream
    stream_index_pair sip(profile.stream_type(), profile.stream_index());
    stream_index_pair base_sip(base_profile.stream_type(), base_profile.stream_index());
    tf2::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    float3 zero_trans{0, 0, 0};

    rclcpp::Time transform_ts_ = _ros_clock.now();

    rs2_extrinsics ex;
    try
    {
        ex = profile.get_extrinsics_to(base_profile);
    }
    catch (std::exception& e)
    {
        if (!strcmp(e.what(), "Requested extrinsics are not available!"))
        {
            ROS_WARN_STREAM("(" << rs2_stream_to_string(profile.stream_type()) << ", " << profile.stream_index() << ") -> (" << rs2_stream_to_string(base_profile.stream_type()) << ", " << base_profile.stream_index() << "): " << e.what() << " : using unity as default.");
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
    std::string base_frame_id = FRAME_ID(base_sip);
    publish_static_tf(transform_ts_, trans, Q, base_frame_id, FRAME_ID(sip));

    // Transform stream frame to stream optical frame
    publish_static_tf(transform_ts_, zero_trans, quaternion_optical, FRAME_ID(sip), OPTICAL_FRAME_ID(sip));

    if (profile.is<rs2::video_stream_profile>() && profile.stream_type() != RS2_STREAM_DEPTH && profile.stream_index() == 1)
    {
        publish_static_tf(transform_ts_, trans, Q, base_frame_id, ALIGNED_DEPTH_TO_FRAME_ID(sip));
        publish_static_tf(transform_ts_, zero_trans, quaternion_optical, ALIGNED_DEPTH_TO_FRAME_ID(sip), OPTICAL_FRAME_ID(sip));
    }
}

void BaseRealSenseNode::SetBaseStream()
{
    const std::vector<stream_index_pair> base_stream_priority = {DEPTH, POSE};
    std::set<stream_index_pair> checked_sips;
    std::map<stream_index_pair, rs2::stream_profile> available_profiles;
    for(auto&& sensor : _available_ros_sensors)
    {
        for (auto& profile : sensor->get_stream_profiles())
        {
            stream_index_pair sip(profile.stream_type(), profile.stream_index());
            if (available_profiles.find(sip) != available_profiles.end())
                continue;
            available_profiles[sip] = profile;
        }
    }
    
    std::vector<stream_index_pair>::const_iterator base_stream(base_stream_priority.begin());
    while( (available_profiles.find(*base_stream) == available_profiles.end()) && (base_stream != base_stream_priority.end()))
    {
        base_stream++;
    }
    if (base_stream == base_stream_priority.end())
    {
        throw std::runtime_error("No known base_stream found for transformations.");
    }
    ROS_INFO_STREAM("SELECTED BASE:" << base_stream->first << ", " << base_stream->second);

    _base_profile = available_profiles[*base_stream];
}

void BaseRealSenseNode::publishStaticTransforms(std::vector<rs2::stream_profile> profiles)
{
    // Publish static transforms
    if (_publish_tf)
    {
        for (auto& profile : profiles)
        {
            calcAndPublishStaticTransform(profile, _base_profile);
        }
        // Static transform for non-positive values
        if (_tf_publish_rate > 0)
            _tf_t = std::shared_ptr<std::thread>(new std::thread(boost::bind(&BaseRealSenseNode::publishDynamicTransforms, this)));
        else
            _static_tf_broadcaster.sendTransform(_static_tf_msgs);
    }
}

void BaseRealSenseNode::publishDynamicTransforms()
{
    // Publish transforms for the cameras
    ROS_WARN("Publishing dynamic camera transforms (/tf) at %g Hz", _tf_publish_rate);

    rclcpp::Rate loop_rate(_tf_publish_rate);

    while (rclcpp::ok())
    {
        // Update the time stamp for publication
        {
            std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);
            rclcpp::Time t = _ros_clock.now();        
            for(auto& msg : _static_tf_msgs)
                msg.header.stamp = t;
            _dynamic_tf_broadcaster->sendTransform(_static_tf_msgs);
        }

        loop_rate.sleep();
    }
}

// void BaseRealSenseNode::publishIntrinsics()
// {
//     if (_enable[GYRO])
//     {
//         _imu_info_publisher[GYRO] = _node.create_publisher<IMUInfo>("gyro/imu_info", 1);
//         IMUInfo info_msg = getImuInfo(GYRO);
//         _imu_info_publisher[GYRO]->publish(info_msg);
//     }

//     if (_enable[ACCEL])
//     {
//         _imu_info_publisher[ACCEL] = _node.create_publisher<IMUInfo>("accel/imu_info", 1);
//         IMUInfo info_msg = getImuInfo(ACCEL);
//         _imu_info_publisher[ACCEL]->publish(info_msg);
//     }
// }

void reverse_memcpy(unsigned char* dst, const unsigned char* src, size_t n)
{
    size_t i;

    for (i=0; i < n; ++i)
        dst[n-1-i] = src[i];

}

void BaseRealSenseNode::publishPointCloud(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset)
{
    std::vector<std::shared_ptr<NamedFilter>>::iterator pc_filter_ = find_if(_filters.begin(), _filters.end(), [] (std::shared_ptr<NamedFilter> s) { return s->is<PointcloudFilter>(); } );
    std::shared_ptr<PointcloudFilter> pc_filter = std::static_pointer_cast<PointcloudFilter>(*pc_filter_);

    if (!pc_filter->getPublisher()->get_subscription_count())
        return;

    ROS_DEBUG("Publish pointscloud");
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

    _valid_pc_indices.clear();
    for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++, color_point++)
    {
        if (static_cast<float>(vertex->z) > 0)
        {
            float i = static_cast<float>(color_point->u);
            float j = static_cast<float>(color_point->v);
            if (_allow_no_texture_points || (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f))
            {
                _valid_pc_indices.push_back(point_idx);
            }
        }
    }

    _msg_pointcloud.header.stamp = t;
    _msg_pointcloud.header.frame_id = OPTICAL_FRAME_ID(DEPTH);
    _msg_pointcloud.width = _valid_pc_indices.size();
    _msg_pointcloud.height = 1;
    _msg_pointcloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(_msg_pointcloud);
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
        _msg_pointcloud.point_step = addPointField(_msg_pointcloud, format_str.c_str(), 1, sensor_msgs::msg::PointField::FLOAT32, _msg_pointcloud.point_step);
        _msg_pointcloud.row_step = _msg_pointcloud.width * _msg_pointcloud.point_step;
        _msg_pointcloud.data.resize(_msg_pointcloud.height * _msg_pointcloud.row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(_msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(_msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(_msg_pointcloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t>iter_color(_msg_pointcloud, format_str);
        color_point = pc.get_texture_coordinates();

        float color_pixel[2];
        unsigned int prev_idx(0);
        for (auto idx=_valid_pc_indices.begin(); idx != _valid_pc_indices.end(); idx++)
        {
            unsigned int idx_jump(*idx-prev_idx);
            prev_idx = *idx;
            vertex+=idx_jump;
            color_point+=idx_jump;

            *iter_x = vertex->x;
            *iter_y = vertex->y;
            *iter_z = vertex->z;

            float i(color_point->u);
            float j(color_point->v);
            if (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f)
            {
                color_pixel[0] = i * texture_width;
                color_pixel[1] = j * texture_height;
                int pixx = static_cast<int>(color_pixel[0]);
                int pixy = static_cast<int>(color_pixel[1]);
                int offset = (pixy * texture_width + pixx) * num_colors;
                reverse_memcpy(&(*iter_color), color_data+offset, num_colors);  // PointCloud2 order of rgb is bgr.
            }

            ++iter_x; ++iter_y; ++iter_z;
            ++iter_color;
        }
    }
    else
    {
        sensor_msgs::PointCloud2Iterator<float>iter_x(_msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(_msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(_msg_pointcloud, "z");
        unsigned int prev_idx(0);
        for (auto idx=_valid_pc_indices.begin(); idx != _valid_pc_indices.end(); idx++)
        {
            unsigned int idx_jump(*idx-prev_idx);
            prev_idx = *idx;
            vertex+=idx_jump;

            *iter_x = vertex->x;
            *iter_y = vertex->y;
            *iter_z = vertex->z;

            ++iter_x; ++iter_y; ++iter_z;
        }
    }
    pc_filter->getPublisher()->publish(_msg_pointcloud);
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

IMUInfo BaseRealSenseNode::getImuInfo(const rs2::stream_profile& profile)
{
    IMUInfo info{};
    auto sp = profile.as<rs2::motion_stream_profile>();
    rs2_motion_device_intrinsic imuIntrinsics;
    try
    {
        imuIntrinsics = sp.get_motion_intrinsics();
    }
    catch(const std::runtime_error &ex)
    {
        ROS_DEBUG_STREAM("No Motion Intrinsics available.");
        imuIntrinsics = {{{1,0,0,0},{0,1,0,0},{0,0,1,0}}, {0,0,0}, {0,0,0}};
    }

    auto index = 0;
    stream_index_pair sip(profile.stream_type(), profile.stream_index());
    info.header.frame_id = OPTICAL_FRAME_ID(sip);
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

void BaseRealSenseNode::publishFrame(rs2::frame f, const rclcpp::Time& t,
                                     const stream_index_pair& stream,
                                     std::map<stream_index_pair, cv::Mat>& images,
                                     const std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>& info_publishers,
                                     const std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics>& image_publishers)
{
    ROS_DEBUG("publishFrame(...)");
    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int bpp = 1;
    if (f.is<rs2::video_frame>())
    {
        auto timage = f.as<rs2::video_frame>();
        width = timage.get_width();
        height = timage.get_height();
        bpp = timage.get_bytes_per_pixel();
    }
    auto& image = images[stream];

    if (image.size() != cv::Size(width, height) || image.depth() != _image_format[bpp])
    {
        image.create(height, width, _image_format[bpp]);
    }
    image.data = (uint8_t*)f.get_data();

    if (f.is<rs2::depth_frame>())
    {
        image = fix_depth_scale(image, _depth_scaled_image[stream]);
    }

    auto& info_publisher = info_publishers.at(stream);
    auto& image_publisher = image_publishers.at(stream);
    if(0 != info_publisher->get_subscription_count() ||
       0 != image_publisher.first.getNumSubscribers())
    {
        sensor_msgs::msg::Image::SharedPtr img;
        img = cv_bridge::CvImage(std_msgs::msg::Header(), _encoding.at(bpp), image).toImageMsg();
        img->width = width;
        img->height = height;
        img->is_bigendian = false;
        img->step = width * bpp;
        img->header.frame_id = OPTICAL_FRAME_ID(stream);
        img->header.stamp = t;

        auto& cam_info = _camera_info.at(stream);
        if (cam_info.width != width)
        {
            updateStreamCalibData(f.get_profile().as<rs2::video_stream_profile>());
        }
        cam_info.header.stamp = t;
        info_publisher->publish(cam_info);

        image_publisher.first.publish(img);
        // _rs_diagnostic_updater.Tick(image_publisher.second);
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

void BaseRealSenseNode::startMonitoring()
{
    int time_interval(10000);
    std::function<void()> func = [this, time_interval](){
        std::mutex mu;
        std::unique_lock<std::mutex> lock(mu);
        while(_is_running) {
            _cv_temp.wait_for(lock, std::chrono::milliseconds(time_interval), [&]{return !_is_running;});
            if (_is_running)
            {
                publish_temperature();
            }
        }
    };
    _monitoring_t = std::make_shared<std::thread>(func);
}

void BaseRealSenseNode::publish_temperature()
{
    rs2::options sensor(*_available_ros_sensors[0]);
    for (rs2_option option : _monitor_options)
    {
        if (sensor.supports(option))
        {
            std::string name(rs2_option_to_string(option));
            try
            {
                auto option_value = sensor.get_option(option);
                // _rs_diagnostic_updater.update_temperatue(name, option_value);
            }
            catch(const std::exception& e)
            {
                ROS_DEBUG_STREAM("Failed checking for temperature - " << name << std::endl << e.what());
            }
        }
    }
}
