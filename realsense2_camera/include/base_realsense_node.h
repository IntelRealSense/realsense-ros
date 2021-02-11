// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once
#ifndef ___BASE_REALSENSE_NODE_HEADER___
#define ___BASE_REALSENSE_NODE_HEADER___

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "constants.h"
#include <cv_bridge/cv_bridge.h>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <diagnostic_updater/publisher.hpp>
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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <eigen3/Eigen/Geometry>
#include <condition_variable>

#include <ros_sensor.h>
#include <named_filter.h>

#include <queue>
#include <mutex>
#include <atomic>
#include <thread>

using realsense2_camera_msgs::msg::Extrinsics;
using realsense2_camera_msgs::msg::IMUInfo;

#define FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_" << STREAM_NAME(sip) << "_frame")).str()
#define OPTICAL_FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_" << STREAM_NAME(sip) << "_optical_frame")).str()
#define ALIGNED_DEPTH_TO_FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_" << "aligned_depth_to_" << STREAM_NAME(sip) << "_frame")).str()

namespace realsense2_camera
{
    typedef std::pair<rs2_stream, int> stream_index_pair;

    // const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
    // const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
    // const stream_index_pair INFRA0{RS2_STREAM_INFRARED, 0};
    // const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
    // const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
    // const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
    // const stream_index_pair FISHEYE1{RS2_STREAM_FISHEYE, 1};
    // const stream_index_pair FISHEYE2{RS2_STREAM_FISHEYE, 2};
    // const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
    // const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};
    // const stream_index_pair POSE{RS2_STREAM_POSE, 0};
    

    const std::vector<stream_index_pair> IMAGE_STREAMS = {DEPTH, INFRA0, INFRA1, INFRA2,
                                                          COLOR,
                                                          FISHEYE,
                                                          FISHEYE1, FISHEYE2};

    const std::vector<stream_index_pair> HID_STREAMS = {GYRO, ACCEL, POSE};


    class Diagnostics
    {
        public:
            Diagnostics(std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater, std::string serial_no):
                _updater(diagnostic_updater)
            {
                _updater->removeByName(serial_no);
                _updater->add(serial_no, this, &Diagnostics::update_temperatures);
            }
            void update_temperatures(diagnostic_updater::DiagnosticStatusWrapper& status)
            {
                for (auto entry : _crnt_temp)
                {
                    status.add(entry.first, entry.second);
                }
                status.summary(0, "OK");
            }
            void Add(const std::string& name, 
                const diagnostic_updater::FrequencyStatusParam& param)
            {
                frequency_status_[name] = std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(name, *_updater, param);
            }
            void Remove(const std::string& name)
            {
                frequency_status_.erase(name);
            }
            void Tick(const std::string& name)
            {
                frequency_status_[name]->tick();
                _updater->force_update();
            }
            void update_temperatue(const std::string& name, double crnt_temperaure)
            {
                _crnt_temp[name] = crnt_temperaure;
                _updater->force_update();
            }

        private:
            std::map<std::string, std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> > frequency_status_;
            std::map<std::string, double> _crnt_temp;
            std::shared_ptr<diagnostic_updater::Updater> _updater;
    };

    typedef std::pair<image_transport::Publisher, std::string> ImagePublisherWithFrequencyDiagnostics;

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

        bool _is_running;
        rclcpp::Node& _node;
        bool _align_depth;
        std::string _camera_name;
        std::vector<rs2_option> _monitor_options;
        rclcpp::Logger _logger;
        // Diagnostics _rs_diagnostic_updater;

        virtual void calcAndPublishStaticTransform(const rs2::stream_profile& profile, const rs2::stream_profile& base_profile);
        void publishTopics();
        tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) const;
        void publish_static_tf(const rclcpp::Time& t,
                               const float3& trans,
                               const tf2::Quaternion& q,
                               const std::string& from,
                               const std::string& to);
        void setup();

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
        void setupErrorCallback(const rs2::sensor& sensor);
        void setupPublishers();
        void enable_devices();
        void setupFilters();
        void setBaseTime(double frame_time, bool warn_no_metadata);
        cv::Mat& fix_depth_scale(const cv::Mat& from_image, cv::Mat& to_image);
        void clip_depth(rs2::depth_frame depth_frame, float clipping_dist);
        void updateProfilesStreamCalibData(const std::vector<rs2::stream_profile>& profiles);
        void updateExtrinsicsCalibData(const rs2::video_stream_profile& left_video_profile, const rs2::video_stream_profile& right_video_profile);
        void updateStreamCalibData(const rs2::video_stream_profile& video_profile);
        void SetBaseStream();
        void publishStaticTransforms(std::vector<rs2::stream_profile> profiles);
        void publishDynamicTransforms();
        void publishIntrinsics();
        void runFirstFrameInitialization(rs2_stream stream_type);
        void publishPointCloud(rs2::points f, const rclcpp::Time& t, const rs2::frameset& frameset);
        Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics, const std::string& frame_id) const;

        IMUInfo getImuInfo(const rs2::stream_profile& profile);
        void publishFrame(rs2::frame f, const rclcpp::Time& t,
                          const stream_index_pair& stream,
                          std::map<stream_index_pair, cv::Mat>& images,
                          const std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>& info_publishers,
                          const std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics>& image_publishers);
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
        
        // void readAndSetDynamicParam(ros::NodeHandle& nh1, std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec, const std::string option_name, const int min_val, const int max_val, rs2::sensor sensor, int* option_value);
        // void registerAutoExposureROIOptions(ros::NodeHandle& nh);
        void set_auto_exposure_roi(const std::string option_name, rs2::sensor sensor, int new_value);
        void set_sensor_auto_exposure_roi(rs2::sensor sensor);
        void startMonitoring();
        void monitoringProfileChanges();
        void publish_temperature();
        void setupFiltersPublishers();
        void setAvailableSensors();
        void setCallbackFunctions();
        void getUpdatedSensorParameters(const rs2::sensor& sensor);
        void updateSensors();
        void startPublishers(const std::vector<rs2::stream_profile>& profiles, const RosSensor& sensor);
        void stopPublishers(const std::vector<rs2::stream_profile>& profiles);

        rs2::device _dev;
        std::map<stream_index_pair, rs2::sensor> _sensors;
        std::map<std::string, std::function<void(rs2::frame)>> _sensors_callback;

        std::string _json_file_path;
        std::string _serial_no;
        float _depth_scale_meters;
        float _clipping_distance;

        double _linear_accel_cov;
        double _angular_velocity_cov;
        bool  _hold_back_imu_for_frames;

        std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;
        std::map<rs2_stream, rs2_format>  _format;
        std::map<stream_index_pair, bool> _enable;
        bool _publish_tf;
        double _tf_publish_rate;
        std::mutex _publish_tf_mutex;

        tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _dynamic_tf_broadcaster;
        std::vector<geometry_msgs::msg::TransformStamped> _static_tf_msgs;
        std::shared_ptr<std::thread> _tf_t;

        std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics> _image_publishers;
        
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> _imu_publishers;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> _odom_publisher;
        std::shared_ptr<SyncedImuPublisher> _synced_imu_publisher;
        std::map<unsigned int, int> _image_format;
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> _info_publisher;
        std::map<stream_index_pair, rclcpp::Publisher<IMUInfo>::SharedPtr> _imu_info_publisher;
        std::map<stream_index_pair, cv::Mat> _image;
        std::map<unsigned int, std::string> _encoding;

        std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> _camera_info;
        std::atomic_bool _is_initialized_time_base;
        double _camera_time_base;
        std::map<stream_index_pair, std::vector<rs2::stream_profile>> _enabled_profiles;

        rclcpp::Time _ros_time_base;
        bool _sync_frames;
        bool _pointcloud;
        bool _publish_odom_tf;
        imu_sync_method _imu_sync_method;
        stream_index_pair _pointcloud_texture;
        PipelineSyncer _syncer;
        rs2::asynchronous_syncer _asyncer;
        std::vector<std::shared_ptr<NamedFilter>> _filters;
        std::vector<rs2::sensor> _dev_sensors;
        std::vector<std::shared_ptr<RosSensor>> _available_ros_sensors;

        std::map<rs2_stream, std::shared_ptr<rs2::align>> _align;

        std::map<stream_index_pair, cv::Mat> _depth_aligned_image;
        std::map<stream_index_pair, cv::Mat> _depth_scaled_image;
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> _depth_aligned_info_publisher;
        std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics> _depth_aligned_image_publishers;
        // std::map<stream_index_pair, rclcpp::Publisher<Extrinsics>::SharedPtr> _depth_to_other_extrinsics_publishers;
        std::map<stream_index_pair, rs2_extrinsics> _depth_to_other_extrinsics;
        std::map<std::string, rs2::region_of_interest> _auto_exposure_roi;
        std::map<rs2_stream, bool> _is_first_frame;
        std::map<rs2_stream, std::vector<std::function<void()> > > _video_functions_stack;

        std::shared_ptr<std::thread> _monitoring_t;
        std::shared_ptr<std::thread> _monitoring_pc;   //pc = profile changes
        mutable std::condition_variable _cv_temp, _cv_mpc;
        bool _is_profile_changed;

        rs2::stream_profile _base_profile;

        Parameters _parameters;
    };//end class
}
#endif //___BASE_REALSENSE_NODE_HEADER___

