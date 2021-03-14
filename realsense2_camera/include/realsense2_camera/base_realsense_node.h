// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once
#ifndef ___BASE_REALSENSE_NODE_HEADER___
#define ___BASE_REALSENSE_NODE_HEADER___

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "realsense2_camera/constants.h"
#include <cv_bridge/cv_bridge.h>

// #include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.hpp>
#include "realsense2_camera_msgs/msg/imu_info.hpp"
#include "realsense2_camera_msgs/msg/extrinsics.hpp"
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <eigen3/Eigen/Geometry>
#include <condition_variable>

#include <queue>
#include <mutex>
#include <atomic>
#include <thread>

using realsense2_camera_msgs::msg::Extrinsics;
using realsense2_camera_msgs::msg::IMUInfo;

namespace realsense2_camera
{
    typedef std::pair<rs2_stream, int> stream_index_pair;

    const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
    const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
    const stream_index_pair INFRA0{RS2_STREAM_INFRARED, 0};
    const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
    const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
    const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
    const stream_index_pair FISHEYE1{RS2_STREAM_FISHEYE, 1};
    const stream_index_pair FISHEYE2{RS2_STREAM_FISHEYE, 2};
    const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
    const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};
    const stream_index_pair POSE{RS2_STREAM_POSE, 0};
    const stream_index_pair CONFIDENCE{RS2_STREAM_CONFIDENCE, 0};
    

    const std::vector<stream_index_pair> IMAGE_STREAMS = {DEPTH, INFRA0, INFRA1, INFRA2,
                                                          COLOR,
                                                          FISHEYE,
                                                          FISHEYE1, FISHEYE2, CONFIDENCE};

    const std::vector<stream_index_pair> HID_STREAMS = {GYRO, ACCEL, POSE};

    class NamedFilter
    {
        public:
            std::string _name;
            std::shared_ptr<rs2::filter> _filter;

        public:
            NamedFilter(std::string name, std::shared_ptr<rs2::filter> filter):
            _name(name), _filter(filter)
            {}
    };

	class PipelineSyncer : public rs2::asynchronous_syncer
	{
	public: 
		void operator()(rs2::frame f) const
		{
			invoke(std::move(f));
		}
	};

    class SyncedImuPublisher
    {
        public:
            SyncedImuPublisher() {_is_enabled=false;};
            SyncedImuPublisher(rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher, 
                               std::size_t waiting_list_size=1000);
            ~SyncedImuPublisher();
            void Pause();   // Pause sending messages. All messages from now on are saved in queue.
            void Resume();  // Send all pending messages and allow sending future messages.
            void Publish(sensor_msgs::msg::Imu msg);     //either send or hold message.
            size_t getNumSubscribers();
            void Enable(bool is_enabled) {_is_enabled=is_enabled;};
        
        private:
            void PublishPendingMessages();

        private:
            std::mutex                                          _mutex;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _publisher;
            bool                                                _pause_mode;
            std::queue<sensor_msgs::msg::Imu>                   _pending_messages;
            std::size_t                                         _waiting_list_size;
            bool                                                _is_enabled;
    };

    class BaseRealSenseNode
    {
    public:
        BaseRealSenseNode(rclcpp::Node& node,
                          rs2::device dev, const std::string& serial_no);
        ~BaseRealSenseNode();

    public:
        enum imu_sync_method{NONE, COPY, LINEAR_INTERPOLATION};

    protected:
        class float3
        {
            public:
                float x, y, z;

            public:
                float3& operator*=(const float& factor)
                {
                    x*=factor;
                    y*=factor;
                    z*=factor;
                    return (*this);
                }
                float3& operator+=(const float3& other)
                {
                    x+=other.x;
                    y+=other.y;
                    z+=other.z;
                    return (*this);
                }
        };

        std::string _base_frame_id;
        std::string _odom_frame_id;
        std::map<stream_index_pair, std::string> _frame_id;
        std::map<stream_index_pair, std::string> _optical_frame_id;
        std::map<stream_index_pair, std::string> _depth_aligned_frame_id;
        rclcpp::Node& _node;
        bool _align_depth;
        std::vector<rs2_option> _monitor_options;
        rclcpp::Logger _logger;

        virtual void calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile);
        void publishTopics();
        rs2::stream_profile getAProfile(const stream_index_pair& stream);
        tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) const;
        void publish_static_tf(const rclcpp::Time& t,
                               const float3& trans,
                               const tf2::Quaternion& q,
                               const std::string& from,
                               const std::string& to);
        const rclcpp::ParameterValue declareParameter(const std::string &name, const rclcpp::ParameterValue &default_value=rclcpp::ParameterValue(), const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor=rcl_interfaces::msg::ParameterDescriptor());
        template<class T>
        void setNgetNodeParameter(T& param, const std::string& param_name, const T& default_value, const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor=rcl_interfaces::msg::ParameterDescriptor());

    private:
        class CimuData
        {
            public:
                CimuData() : m_time_ns(-1) {};
                CimuData(const stream_index_pair type, Eigen::Vector3d data, double time):
                    m_type(type),
                    m_data(data),
                    m_time_ns(time){};
                bool is_set() {return m_time_ns > 0;};
            public:
                stream_index_pair m_type;
                Eigen::Vector3d m_data;
                double          m_time_ns;
        };

        void getParameters();
        void setupDevice();
        void setupErrorCallback();
        void setupPublishers();
        void enable_devices();
        void setupFilters();
        void setupStreams();
        bool setBaseTime(double frame_time, rs2_timestamp_domain time_domain);
        rclcpp::Time frameSystemTimeSec(rs2::frame frame);
        cv::Mat& fix_depth_scale(const cv::Mat& from_image, cv::Mat& to_image);
        void clip_depth(rs2::depth_frame depth_frame, float clipping_dist);
        void updateStreamCalibData(const rs2::video_stream_profile& video_profile);
        void updateExtrinsicsCalibData(const rs2::video_stream_profile& left_video_profile, const rs2::video_stream_profile& right_video_profile);
        void SetBaseStream();
        void publishStaticTransforms();
        void publishDynamicTransforms();
        void publishIntrinsics();
        void runFirstFrameInitialization(rs2_stream stream_type);
        void publishPointCloud(rs2::points f, const rclcpp::Time& t, const rs2::frameset& frameset);
        Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics, const std::string& frame_id) const;

        IMUInfo getImuInfo(const stream_index_pair& stream_index);
        void publishFrame(rs2::frame f, const rclcpp::Time& t,
                          const stream_index_pair& stream,
                          std::map<stream_index_pair, cv::Mat>& images,
                          const std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>& info_publishers,
                          const std::map<stream_index_pair, image_transport::Publisher>& image_publishers,
                          std::map<stream_index_pair, int>& seq,
                          std::map<stream_index_pair, sensor_msgs::msg::CameraInfo>& camera_info,
                          const std::map<stream_index_pair, std::string>& optical_frame_id,
                          const std::map<rs2_stream, std::string>& encoding);
        bool getEnabledProfile(const stream_index_pair& stream_index, rs2::stream_profile& profile);

        void publishAlignedDepthToOthers(rs2::frameset frames, const rclcpp::Time& t);
        sensor_msgs::msg::Imu CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data);

        void FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs);
        void ImuMessage_AddDefaultValues(sensor_msgs::msg::Imu& imu_msg);
        void FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs);
        void imu_callback(rs2::frame frame);
        void imu_callback_sync(rs2::frame frame, imu_sync_method sync_method=imu_sync_method::COPY);
        void pose_callback(rs2::frame frame);
        void multiple_message_callback(rs2::frame frame, imu_sync_method sync_method);
        void frame_callback(rs2::frame frame);

        template<class T>
        void set_parameter(rs2::options sensor, rs2_option option, const std::string& module_name, const std::string& description_addition="");

        void registerDynamicOption(rs2::options sensor, std::string& module_name);
        void registerDynamicReconfigCb();
        void registerAutoExposureROIOption(const std::string option_name, const int min_val, const int max_val, rs2::sensor sensor, int* option_value);
        void registerAutoExposureROIOptions();
        void set_auto_exposure_roi(const std::string variable_name, rs2::sensor sensor, const std::vector<rclcpp::Parameter> & parameters);
        void set_sensor_auto_exposure_roi(rs2::sensor sensor);
        const rmw_qos_profile_t qos_string_to_qos(std::string str);
        rs2_stream rs2_string_to_stream(std::string str);
        void clean();

        rs2::device _dev;
        std::map<stream_index_pair, rs2::sensor> _sensors;
        std::map<std::string, std::function<void(rs2::frame)>> _sensors_callback;

        std::string _json_file_path;
        std::string _serial_no;
        float _depth_scale_meters;
        float _clipping_distance;
        bool _allow_no_texture_points;
        bool _ordered_pc;

        double _linear_accel_cov;
        double _angular_velocity_cov;
        bool  _hold_back_imu_for_frames;

        std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;
        std::map<stream_index_pair, int> _width;
        std::map<stream_index_pair, int> _height;
        std::map<stream_index_pair, double> _fps;
        std::map<stream_index_pair, std::string> _qos;
        std::map<rs2_stream, rs2_format>  _format;
        std::map<stream_index_pair, bool> _enable;
        std::map<rs2_stream, std::string> _stream_name;
        bool _publish_tf;
        double _tf_publish_rate;
        tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;
        tf2_ros::TransformBroadcaster _dynamic_tf_broadcaster;
        std::vector<geometry_msgs::msg::TransformStamped> _static_tf_msgs;
        std::shared_ptr<std::thread> _tf_t;

        std::map<stream_index_pair, image_transport::Publisher> _image_publishers;
        
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> _imu_publishers;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> _odom_publisher;
        std::shared_ptr<SyncedImuPublisher> _synced_imu_publisher;
        std::map<rs2_stream, int> _image_format;
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> _info_publisher;
        std::map<stream_index_pair, rclcpp::Publisher<IMUInfo>::SharedPtr> _imu_info_publisher;
        std::map<stream_index_pair, cv::Mat> _image;
        std::map<rs2_stream, std::string> _encoding;

        std::map<stream_index_pair, int> _seq;
        std::map<rs2_stream, int> _unit_step_size;
        std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> _camera_info;
        std::atomic_bool _is_initialized_time_base;
        double _camera_time_base;
        std::map<stream_index_pair, std::vector<rs2::stream_profile>> _enabled_profiles;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_publisher;
        rclcpp::Time _ros_time_base;
        bool _sync_frames;
        bool _pointcloud;
        bool _publish_odom_tf;
        imu_sync_method _imu_sync_method;
        std::string _filters_str;
        stream_index_pair _pointcloud_texture;
        PipelineSyncer _syncer;
        std::vector<NamedFilter> _filters;
        std::vector<rs2::sensor> _dev_sensors;
        std::map<rs2_stream, std::shared_ptr<rs2::align>> _align;

        std::map<stream_index_pair, cv::Mat> _depth_aligned_image;
        std::map<stream_index_pair, cv::Mat> _depth_scaled_image;
        std::map<rs2_stream, std::string> _depth_aligned_encoding;
        std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> _depth_aligned_camera_info;
        std::map<stream_index_pair, int> _depth_aligned_seq;
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> _depth_aligned_info_publisher;
        std::map<stream_index_pair, image_transport::Publisher> _depth_aligned_image_publishers;
        std::map<stream_index_pair, rclcpp::Publisher<Extrinsics>::SharedPtr> _depth_to_other_extrinsics_publishers;
        std::map<stream_index_pair, rs2_extrinsics> _depth_to_other_extrinsics;
        std::map<std::string, rs2::region_of_interest> _auto_exposure_roi;
        std::map<rs2_stream, bool> _is_first_frame;
        std::map<rs2_stream, std::vector<std::function<void()> > > _video_functions_stack;

        stream_index_pair _base_stream;

        sensor_msgs::msg::PointCloud2 _msg_pointcloud;
        std::vector< unsigned int > _valid_pc_indices;
        std::vector<rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr> _callback_handlers;
        std::set<std::string> _variable_names;

    };//end class
}
#endif //___BASE_REALSENSE_NODE_HEADER___

