// Copyright 2023 Intel Corporation. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../include/base_realsense_node.h"
#include "assert.h"
#include <algorithm>
#include <mutex>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <fstream>
#include <image_publisher.h>

// Header files for disabling intra-process comms for static broadcaster.
#include <rclcpp/publisher_options.hpp>
// This header file is not available in ROS 2 Dashing.
#ifndef DASHING
#include <tf2_ros/qos.hpp>
#endif

using namespace realsense2_camera;

SyncedImuPublisher::SyncedImuPublisher(rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher, 
                                       std::size_t waiting_list_size):
            _publisher(imu_publisher), _pause_mode(false),
            _waiting_list_size(waiting_list_size), _is_enabled(false)
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

BaseRealSenseNode::BaseRealSenseNode(rclcpp::Node& node,
                                     rs2::device dev,
                                     std::shared_ptr<Parameters> parameters,
                                     bool use_intra_process) :
    _is_running(true),
    _node(node),
    _logger(node.get_logger()),
    _parameters(parameters),
    _dev(dev),
    _json_file_path(""),
    _depth_scale_meters(0),
    _clipping_distance(0),
    _linear_accel_cov(0),
    _angular_velocity_cov(0),
    _hold_back_imu_for_frames(false),
    _publish_tf(false),
    _tf_publish_rate(TF_PUBLISH_RATE),
    _diagnostics_period(0),
    _use_intra_process(use_intra_process),
    _is_initialized_time_base(false),
    _camera_time_base(0),
    _sync_frames(SYNC_FRAMES),
    _pointcloud(false),
    _publish_odom_tf(false),
    _imu_sync_method(imu_sync_method::NONE),
    _is_profile_changed(false),
    _is_align_depth_changed(false)
{
    if ( use_intra_process )
    {
        ROS_INFO("Intra-Process communication enabled");
    }

    // intra-process do not support latched QoS, so we need to disable intra-process for this topic
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    #ifndef DASHING
    _static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node, tf2_ros::StaticBroadcasterQoS(), std::move(options));
    #else
    _static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node, rclcpp::QoS(100), std::move(options));
    #endif

    _image_format[1] = CV_8UC1;    // CVBridge type
    _image_format[2] = CV_16UC1;    // CVBridge type
    _image_format[3] = CV_8UC3;    // CVBridge type
    _encoding[1] = sensor_msgs::image_encodings::MONO8; // ROS message type
    _encoding[2] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
    _encoding[3] = sensor_msgs::image_encodings::RGB8; // ROS message type
    
    // Infrared stream
    _format[RS2_STREAM_INFRARED] = RS2_FORMAT_Y8;

    _monitor_options = {RS2_OPTION_ASIC_TEMPERATURE, RS2_OPTION_PROJECTOR_TEMPERATURE};
}

BaseRealSenseNode::~BaseRealSenseNode()
{
    // Kill dynamic transform thread
    _is_running = false;
    _cv_tf.notify_one();
    if (_tf_t && _tf_t->joinable())
        _tf_t->join();

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
    clearParameters();
    for(auto&& sensor : _available_ros_sensors)
    {
        sensor->stop();
    }
}

void BaseRealSenseNode::hardwareResetRequest()
{
    ROS_ERROR_STREAM("Performing Hardware Reset.");
    _dev.hardware_reset();
}

void BaseRealSenseNode::publishTopics()
{
    getParameters();
    setup();
    ROS_INFO_STREAM("RealSense Node Is Up!");
}

void BaseRealSenseNode::setupFilters()
{
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::decimation_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::hdr_merge>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::sequence_id_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::disparity_transform>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::spatial_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::temporal_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::hole_filling_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::disparity_transform>(false), _parameters, _logger));

    /* 
    update_align_depth_func is being used in the align depth filter for triggiring the thread that monitors profile
    changes (_monitoring_pc) on every disable/enable of the align depth filter. This filter enablement/disablement affects
    several topics creation/destruction, therefore, refreshing the topics is required similarly to what is done when turning on/off a sensor.
    See BaseRealSenseNode::monitoringProfileChanges() as reference.
    */ 
    std::function<void(const rclcpp::Parameter&)> update_align_depth_func = [this](const rclcpp::Parameter&){
        {
            std::lock_guard<std::mutex> lock_guard(_profile_changes_mutex);
            _is_align_depth_changed = true;
        }
        _cv_mpc.notify_one();
    };
    _align_depth_filter = std::make_shared<AlignDepthFilter>(std::make_shared<rs2::align>(RS2_STREAM_COLOR), update_align_depth_func, _parameters, _logger);
    _filters.push_back(_align_depth_filter);

    _colorizer_filter = std::make_shared<NamedFilter>(std::make_shared<rs2::colorizer>(), _parameters, _logger); 
    _filters.push_back(_colorizer_filter);

    _pc_filter = std::make_shared<PointcloudFilter>(std::make_shared<rs2::pointcloud>(), _node, _parameters, _logger);
    _filters.push_back(_pc_filter);
}

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
    imu_msg.header.frame_id = IMU_OPTICAL_FRAME_ID;
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
        _is_initialized_time_base = setBaseTime(frame_time, frame.get_frame_timestamp_domain());
    }

    if (0 != _synced_imu_publisher->getNumSubscribers())
    {
        auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
        CimuData imu_data(stream_index, v, frameSystemTimeSec(frame).nanoseconds());
        std::deque<sensor_msgs::msg::Imu> imu_msgs;
        switch (sync_method)
        {
            case imu_sync_method::COPY:
                FillImuData_Copy(imu_data, imu_msgs);
                break;
            case imu_sync_method::LINEAR_INTERPOLATION:
                FillImuData_LinearInterpolation(imu_data, imu_msgs);
                break;
            case imu_sync_method::NONE: //Cannot really be NONE. Just to avoid compilation warning.
                throw std::runtime_error("sync_method in this section can be either COPY or LINEAR_INTERPOLATION");
                break;
        }
        while (imu_msgs.size())
        {
            sensor_msgs::msg::Imu imu_msg = imu_msgs.front();
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
        _is_initialized_time_base = setBaseTime(frame_time, frame.get_frame_timestamp_domain());
    }

    ROS_DEBUG("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
                ros_stream_to_string(frame.get_profile().stream_type()).c_str(),
                frame.get_profile().stream_index(),
                rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));

    auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
    rclcpp::Time t(frameSystemTimeSec(frame));

    if(_imu_publishers.find(stream_index) == _imu_publishers.end())
    {
        ROS_DEBUG("Received IMU callback while topic does not exist");
        return;
    }

    if (0 != _imu_publishers[stream_index]->get_subscription_count())
    {
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
        ROS_DEBUG("Publish %s stream", ros_stream_to_string(frame.get_profile().stream_type()).c_str());
    }
    publishMetadata(frame, t, OPTICAL_FRAME_ID(stream_index));
}

void BaseRealSenseNode::pose_callback(rs2::frame frame)
{
    double frame_time = frame.get_timestamp();
    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
        _is_initialized_time_base = setBaseTime(frame_time, frame.get_frame_timestamp_domain());
    }

    ROS_DEBUG("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
                rs2_stream_to_string(frame.get_profile().stream_type()),
                frame.get_profile().stream_index(),
                rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));
    rs2_pose pose = frame.as<rs2::pose_frame>().get_pose_data();
    rclcpp::Time t(frameSystemTimeSec(frame));

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
    double frame_time = frame.get_timestamp();

    // We compute a ROS timestamp which is based on an initial ROS time at point of first frame,
    // and the incremental timestamp from the camera.
    // In sync mode the timestamp is based on ROS time
    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
        _is_initialized_time_base = setBaseTime(frame_time, frame.get_frame_timestamp_domain());
    }

    rclcpp::Time t(frameSystemTimeSec(frame));
    if (frame.is<rs2::frameset>())
    {
        ROS_DEBUG("Frameset arrived.");
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
        }
        // Clip depth_frame for max range:
        rs2::depth_frame original_depth_frame = frameset.get_depth_frame();
        if (original_depth_frame && _clipping_distance > 0)
        {
            clip_depth(original_depth_frame, _clipping_distance);
        }

        ROS_DEBUG("num_filters: %d", static_cast<int>(_filters.size()));
        for (auto filter_it : _filters)
        {
            frameset = filter_it->Process(frameset);
        }

        ROS_DEBUG("List of frameset after applying filters: size: %d", static_cast<int>(frameset.size()));
        bool sent_depth_frame(false);
        for (auto it = frameset.begin(); it != frameset.end(); ++it)
        {
            auto f = (*it);
            auto stream_type = f.get_profile().stream_type();
            auto stream_index = f.get_profile().stream_index();
            auto stream_format = f.get_profile().format();
            stream_index_pair sip{stream_type,stream_index};

            ROS_DEBUG("Frameset contain (%s, %d, %s) frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu", 
                rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), f.get_frame_number(), frame_time, t.nanoseconds());
            if (f.is<rs2::video_frame>())
                ROS_DEBUG_STREAM("frame: " << f.as<rs2::video_frame>().get_width() << " x " << f.as<rs2::video_frame>().get_height());

            if (f.is<rs2::points>())
            {
                publishPointCloud(f.as<rs2::points>(), t, frameset);
                continue;
            }
            if (stream_type == RS2_STREAM_DEPTH)
            {
                if (sent_depth_frame) continue;
                sent_depth_frame = true;
                if (_align_depth_filter->is_enabled())
                {
                    publishFrame(f, t, COLOR,
                            _depth_aligned_image,
                            _depth_aligned_info_publisher,
                            _depth_aligned_image_publishers,
                            false);
                    continue;
                }
            }
            publishFrame(f, t, sip,
                        _image,
                        _info_publisher,
                        _image_publishers);
        }
        if (original_depth_frame && _align_depth_filter->is_enabled())
        {
            rs2::frame frame_to_send;
            if (_colorizer_filter->is_enabled())
                frame_to_send = _colorizer_filter->Process(original_depth_frame);
            else
                frame_to_send = original_depth_frame;
                
            publishFrame(frame_to_send, t,
                        DEPTH,
                        _image,
                        _info_publisher,
                        _image_publishers);
        }
    }
    else if (frame.is<rs2::video_frame>())
    {
        auto stream_type = frame.get_profile().stream_type();
        auto stream_index = frame.get_profile().stream_index();
        ROS_DEBUG("Single video frame arrived (%s, %d). frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                    rs2_stream_to_string(stream_type), stream_index, frame.get_frame_number(), frame_time, t.nanoseconds());
            
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

bool BaseRealSenseNode::setBaseTime(double frame_time, rs2_timestamp_domain time_domain)
{
    ROS_WARN_ONCE(time_domain == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME ? "Frame metadata isn't available! (frame_timestamp_domain = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME)" : "");
    if (time_domain == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
    {
        ROS_WARN("frame's time domain is HARDWARE_CLOCK. Timestamps may reset periodically.");
        _ros_time_base = _node.now();
        _camera_time_base = frame_time;
        return true;
    }
    return false;
}

uint64_t BaseRealSenseNode::millisecondsToNanoseconds(double timestamp_ms)
{
        // modf breaks input into an integral and fractional part
        double int_part_ms, fract_part_ms;
        fract_part_ms = modf(timestamp_ms, &int_part_ms);

        //convert both parts to ns
        static constexpr uint64_t milli_to_nano = 1000000;
        uint64_t int_part_ns = static_cast<uint64_t>(int_part_ms) * milli_to_nano;
        uint64_t fract_part_ns = static_cast<uint64_t>(std::round(fract_part_ms * milli_to_nano));

        return int_part_ns + fract_part_ns;
}

rclcpp::Time BaseRealSenseNode::frameSystemTimeSec(rs2::frame frame)
{
    double timestamp_ms = frame.get_timestamp();
    if (frame.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
    {
        double elapsed_camera_ns = millisecondsToNanoseconds(timestamp_ms - _camera_time_base);

        /*
        Fixing deprecated-declarations compilation warning.
        Duration(rcl_duration_value_t) is deprecated in favor of 
        static Duration::from_nanoseconds(rcl_duration_value_t)
        starting from GALACTIC.
        */
#if defined(FOXY) || defined(ELOQUENT) || defined(DASHING)
        auto duration = rclcpp::Duration(elapsed_camera_ns);
#else
        auto duration = rclcpp::Duration::from_nanoseconds(elapsed_camera_ns);
#endif

        return rclcpp::Time(_ros_time_base + duration);
    }
    else
    {
        return rclcpp::Time(millisecondsToNanoseconds(timestamp_ms));
    }
}

void BaseRealSenseNode::updateProfilesStreamCalibData(const std::vector<rs2::stream_profile>& profiles)
{
    std::shared_ptr<rs2::stream_profile> left_profile;
    std::shared_ptr<rs2::stream_profile> right_profile;
    for (auto& profile : profiles)
    {
        if (profile.is<rs2::video_stream_profile>())
        {
            updateStreamCalibData(profile.as<rs2::video_stream_profile>());

            // stream index: 1=left, 2=right
            if (profile.stream_index() == 1) { left_profile = std::make_shared<rs2::stream_profile>(profile); }
            if (profile.stream_index() == 2) { right_profile = std::make_shared<rs2::stream_profile>(profile);  }
        }
    }
    if (left_profile && right_profile) {
        updateExtrinsicsCalibData(left_profile->as<rs2::video_stream_profile>(), right_profile->as<rs2::video_stream_profile>());
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

    int coeff_size(5);
    if (intrinsic.model == RS2_DISTORTION_KANNALA_BRANDT4)
    {
        _camera_info[stream_index].distortion_model = "equidistant";
        coeff_size = 4;
    } else {
        _camera_info[stream_index].distortion_model = "plumb_bob";
    }

    _camera_info[stream_index].d.resize(coeff_size);
    for (int i = 0; i < coeff_size; i++)
    {
        _camera_info[stream_index].d.at(i) = intrinsic.coeffs[i];
    }

    if (stream_index == DEPTH && _enable[DEPTH] && _enable[COLOR])
    {
        _camera_info[stream_index].p.at(3) = 0;     // Tx
        _camera_info[stream_index].p.at(7) = 0;     // Ty
    }
}

void BaseRealSenseNode::updateExtrinsicsCalibData(const rs2::video_stream_profile& left_video_profile, const rs2::video_stream_profile& right_video_profile)
{
    stream_index_pair left{left_video_profile.stream_type(), left_video_profile.stream_index()};
    stream_index_pair right{right_video_profile.stream_type(), right_video_profile.stream_index()};

    float fx = _camera_info[right].k.at(0);
    float fy = _camera_info[right].k.at(4);
    const auto& ex = right_video_profile.get_extrinsics_to(left_video_profile);
    _camera_info[right].header.frame_id = OPTICAL_FRAME_ID(left);
    _camera_info[right].p.at(3) = -fx * ex.translation[0] + 0.0; // Tx - avoid -0.0 values.
    _camera_info[right].p.at(7) = -fy * ex.translation[1] + 0.0; // Ty - avoid -0.0 values.
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

void BaseRealSenseNode::publishExtrinsicsTopic(const stream_index_pair& sip, const rs2_extrinsics& ex){
    // Publish extrinsics topic:
    Extrinsics msg = rsExtrinsicsToMsg(ex);
    if (_extrinsics_publishers.find(sip) != _extrinsics_publishers.end())
    {
        _extrinsics_publishers[sip]->publish(msg);
    }
}

void BaseRealSenseNode::calcAndPublishStaticTransform(const rs2::stream_profile& profile, const rs2::stream_profile& base_profile)
{
    // Transform base to stream
    stream_index_pair sip(profile.stream_type(), profile.stream_index());
    tf2::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    float3 zero_trans{0, 0, 0};
    tf2::Quaternion zero_rot_quaternions;
    zero_rot_quaternions.setRPY(0, 0, 0);

    rclcpp::Time transform_ts_ = _node.now();

    rs2_extrinsics ex;
    try
    {
        ex = base_profile.get_extrinsics_to(profile);
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
    publish_static_tf(transform_ts_, trans, Q, _base_frame_id, FRAME_ID(sip));

    // Transform stream frame to stream optical frame
    publish_static_tf(transform_ts_, zero_trans, quaternion_optical, FRAME_ID(sip), OPTICAL_FRAME_ID(sip));

    if (profile.is<rs2::video_stream_profile>() && profile.stream_type() != RS2_STREAM_DEPTH && profile.stream_index() == 1)
    {
        publish_static_tf(transform_ts_, trans, Q, _base_frame_id, ALIGNED_DEPTH_TO_FRAME_ID(sip));
        publish_static_tf(transform_ts_, zero_trans, quaternion_optical, ALIGNED_DEPTH_TO_FRAME_ID(sip), OPTICAL_FRAME_ID(sip));
    }

    if ((_imu_sync_method > imu_sync_method::NONE) && (profile.stream_type() == RS2_STREAM_GYRO))
    {
        publish_static_tf(transform_ts_, zero_trans, zero_rot_quaternions, FRAME_ID(sip), IMU_FRAME_ID);
        publish_static_tf(transform_ts_, zero_trans, quaternion_optical, IMU_FRAME_ID, IMU_OPTICAL_FRAME_ID);
    }

    publishExtrinsicsTopic(sip, ex);
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
    while((base_stream != base_stream_priority.end()) && (available_profiles.find(*base_stream) == available_profiles.end()))
    {
        base_stream++;
    }
    if (base_stream == base_stream_priority.end())
    {
        throw std::runtime_error("No known base_stream found for transformations.");
    }
    ROS_DEBUG_STREAM("SELECTED BASE:" << base_stream->first << ", " << base_stream->second);

    _base_profile = available_profiles[*base_stream];
}

void BaseRealSenseNode::publishStaticTransforms(std::vector<rs2::stream_profile> profiles)
{
    // Publish static transforms
    if (_publish_tf)
    {
        for (auto &profile : profiles)
        {
            calcAndPublishStaticTransform(profile, _base_profile);
        }
        if (_static_tf_broadcaster)
            _static_tf_broadcaster->sendTransform(_static_tf_msgs);
    }
}

void BaseRealSenseNode::startDynamicTf()
{
    if (_tf_publish_rate > 0)
    {
        ROS_WARN("Publishing dynamic camera transforms (/tf) at %g Hz", _tf_publish_rate);
        if (!_tf_t)
        {
            _dynamic_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(_node);
            _tf_t = std::make_shared<std::thread>([this]()
            {
                publishDynamicTransforms();
            });
        }
    }
    else
    {
        if (_tf_t && _tf_t->joinable())
        {
            _tf_t->join();
            _tf_t.reset();
            _dynamic_tf_broadcaster.reset();
        }
    }
}

void BaseRealSenseNode::publishDynamicTransforms()
{
    // Publish transforms for the cameras
    std::unique_lock<std::mutex> lock(_publish_dynamic_tf_mutex);
    while (rclcpp::ok() && _is_running && _tf_publish_rate > 0)
    {
        _cv_tf.wait_for(lock, std::chrono::milliseconds((int)(1000.0/_tf_publish_rate)), [&]{return (!(_is_running && _tf_publish_rate > 0));});
        {
            std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);
            rclcpp::Time t = _node.now();
            try
            {
                for(auto& msg : _static_tf_msgs)
                    msg.header.stamp = t;
                _dynamic_tf_broadcaster->sendTransform(_static_tf_msgs);
            }
            catch(const std::exception& e)
            {
                ROS_ERROR_STREAM("Error publishing dynamic transforms: " << e.what());
            }
        }
    }
}

void BaseRealSenseNode::publishPointCloud(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset)
{
    std::string frame_id = (_align_depth_filter->is_enabled() ? OPTICAL_FRAME_ID(COLOR) : OPTICAL_FRAME_ID(DEPTH));
    _pc_filter->Publish(pc, t, frameset, frame_id);
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
                                     const std::map<stream_index_pair, std::shared_ptr<image_publisher>>& image_publishers,
                                     const bool is_publishMetadata)
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

    if (info_publishers.find(stream) == info_publishers.end() ||
        image_publishers.find(stream) == image_publishers.end())
        {
            // Stream is already disabled.
            return;
        }
    auto& info_publisher = info_publishers.at(stream);
    auto& image_publisher = image_publishers.at(stream);
    if(0 != info_publisher->get_subscription_count() ||
       0 != image_publisher->get_subscription_count())
    {
        auto& cam_info = _camera_info.at(stream);
        if (cam_info.width != width)
        {
            updateStreamCalibData(f.get_profile().as<rs2::video_stream_profile>());
        }
        cam_info.header.stamp = t;
        info_publisher->publish(cam_info);

        // Prepare image topic to be published
        // We use UniquePtr for allow intra-process publish when subscribers of that type are available
        sensor_msgs::msg::Image::UniquePtr img(new sensor_msgs::msg::Image());

        if (!img)
        {
            ROS_ERROR("sensor image message allocation failed, frame was dropped");
            return;
        }

        // Convert the CV::Mat into a ROS image message (1 copy is done here)
        cv_bridge::CvImage(std_msgs::msg::Header(), _encoding.at(bpp), image).toImageMsg(*img);

        // Convert OpenCV Mat to ROS Image
        img->header.frame_id = OPTICAL_FRAME_ID(stream);
        img->header.stamp = t;
        img->height = height;
        img->width = width;
        img->is_bigendian = false;
        img->step = width * bpp;
        // Transfer the unique pointer ownership to the RMW
        sensor_msgs::msg::Image* msg_address = img.get();
        image_publisher->publish(std::move(img));

        ROS_DEBUG_STREAM(rs2_stream_to_string(f.get_profile().stream_type()) << " stream published, message address: " << std::hex << msg_address);
    }
    if (is_publishMetadata)
    {
        publishMetadata(f, t, OPTICAL_FRAME_ID(stream));
    }
}

void BaseRealSenseNode::publishMetadata(rs2::frame f, const rclcpp::Time& header_time, const std::string& frame_id)
{
    stream_index_pair stream = {f.get_profile().stream_type(), f.get_profile().stream_index()};    
    if (_metadata_publishers.find(stream) != _metadata_publishers.end())
    {
        auto& md_publisher = _metadata_publishers.at(stream);
        if (0 != md_publisher->get_subscription_count())
        {
            realsense2_camera_msgs::msg::Metadata msg;
            msg.header.frame_id = frame_id;
            msg.header.stamp = header_time;
            std::stringstream json_data;
            const char* separator = ",";
            json_data << "{";
            // Add additional fields:
            json_data << "\"" << "frame_number" << "\":" << f.get_frame_number();
            json_data << separator << "\"" << "clock_domain" << "\":" << "\"" << create_graph_resource_name(rs2_timestamp_domain_to_string(f.get_frame_timestamp_domain())) << "\"";
            json_data << separator << "\"" << "frame_timestamp" << "\":" << std::fixed << f.get_timestamp();

            for (auto i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
            {
                if (f.supports_frame_metadata((rs2_frame_metadata_value)i))
                {
                    rs2_frame_metadata_value mparam = (rs2_frame_metadata_value)i;
                    std::string name = create_graph_resource_name(rs2_frame_metadata_to_string(mparam));
                    if (RS2_FRAME_METADATA_FRAME_TIMESTAMP == i)
                    {
                        name = "hw_timestamp";
                    }
                    rs2_metadata_type val = f.get_frame_metadata(mparam);
                    json_data << separator << "\"" << name << "\":" << val;
                }
            }
            json_data << "}";
            msg.json_data = json_data.str();
            md_publisher->publish(msg);
        }
    }
}

void BaseRealSenseNode::startDiagnosticsUpdater()
{
    std::string serial_no = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    if (_diagnostics_period > 0)
    {
        ROS_INFO_STREAM("Publish diagnostics every " << _diagnostics_period << " seconds.");
        _diagnostics_updater = std::make_shared<diagnostic_updater::Updater>(&_node, _diagnostics_period);

        _diagnostics_updater->setHardwareID(serial_no);

        _diagnostics_updater->add("Temperatures", [this](diagnostic_updater::DiagnosticStatusWrapper& status)
        {
            bool got_temperature(false);
            for(auto&& sensor : _available_ros_sensors)
            {
                for (rs2_option option : _monitor_options)
                {
                    if (sensor->supports(option))
                    {
                        status.add(rs2_option_to_string(option), sensor->get_option(option));
                        got_temperature = true;
                    }
                }
                if (got_temperature) break;
            }
            status.summary(0, "OK");
        });
    }
}
