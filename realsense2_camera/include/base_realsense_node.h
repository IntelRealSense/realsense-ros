// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include "../include/realsense_node_factory.h"
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <ddynamic_reconfigure/param/dd_all_params.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <queue>
#include <mutex>
#include <atomic>

namespace realsense2_camera
{
    struct FrequencyDiagnostics
    {
      FrequencyDiagnostics(double expected_frequency, std::string name, std::string hardware_id) :
        expected_frequency_(expected_frequency),
        frequency_status_(diagnostic_updater::FrequencyStatusParam(&expected_frequency_, &expected_frequency_)),
        diagnostic_updater_(ros::NodeHandle(), ros::NodeHandle("~"), ros::this_node::getName() + "_" + name)
      {
        ROS_INFO("Expected frequency for %s = %.5f", name.c_str(), expected_frequency_);
        diagnostic_updater_.setHardwareID(hardware_id);
        diagnostic_updater_.add(frequency_status_);
      }

      void update()
      {
        frequency_status_.tick();
        diagnostic_updater_.update();
      }

      double expected_frequency_;
      diagnostic_updater::FrequencyStatus frequency_status_;
      diagnostic_updater::Updater diagnostic_updater_;
    };
    typedef std::pair<image_transport::Publisher, std::shared_ptr<FrequencyDiagnostics>> ImagePublisherWithFrequencyDiagnostics;

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
            SyncedImuPublisher(ros::Publisher imu_publisher, std::size_t waiting_list_size=1000);
            ~SyncedImuPublisher();
            void Pause();   // Pause sending messages. All messages from now on are saved in queue.
            void Resume();  // Send all pending messages and allow sending future messages.
            void Publish(sensor_msgs::Imu msg);     //either send or hold message.
            uint32_t getNumSubscribers() { return _publisher.getNumSubscribers();};
            void Enable(bool is_enabled) {_is_enabled=is_enabled;};
        
        private:
            void PublishPendingMessages();

        private:
            std::mutex                    _mutex;
            ros::Publisher                _publisher;
            bool                          _pause_mode;
            std::queue<sensor_msgs::Imu>  _pending_messages;
            std::size_t                     _waiting_list_size;
            bool                          _is_enabled;
    };

    class BaseRealSenseNode : public InterfaceRealSenseNode
    {
    public:
        BaseRealSenseNode(ros::NodeHandle& nodeHandle,
                          ros::NodeHandle& privateNodeHandle,
                          rs2::device dev,
                          const std::string& serial_no);

        void toggleSensors(bool enabled);
        virtual void publishTopics() override;
        virtual void registerDynamicReconfigCb(ros::NodeHandle& nh) override;
        virtual ~BaseRealSenseNode() {}

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
        bool _align_depth;

        virtual void calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile);
        rs2::stream_profile getAProfile(const stream_index_pair& stream);
        tf::Quaternion rotationMatrixToQuaternion(const float rotation[9]) const;
        void publish_static_tf(const ros::Time& t,
                               const float3& trans,
                               const tf::Quaternion& q,
                               const std::string& from,
                               const std::string& to);


    private:
        class CIMUHistory
        {
            public:
                enum sensor_name {mGYRO, mACCEL};
                class imuData
                {
                    public:
                        imuData(const imuData& other):
                            imuData(other.m_reading, other.m_time)
                            {};
                        imuData(const float3 reading, double time):
                            m_reading(reading),
                            m_time(time)
                            {};
                        imuData operator*(const double factor);
                        imuData operator+(const imuData& other);
                    public:
                        BaseRealSenseNode::float3 m_reading;
                        double                    m_time;
                };
                
            private:
                size_t m_max_size;
                std::map<sensor_name, std::list<imuData> > m_map;

            public:
                CIMUHistory(size_t size);
                void add_data(sensor_name module, imuData data);
                bool is_all_data(sensor_name);
                bool is_data(sensor_name);
                const std::list<imuData>& get_data(sensor_name module);
                imuData last_data(sensor_name module);
        };

        static std::string getNamespaceStr();
        void getParameters();
        void setupDevice();
        void setupErrorCallback();
        void setupPublishers();
        void enable_devices();
        void setupFilters();
        void setupStreams();
        void setBaseTime(double frame_time, bool warn_no_metadata);
        void clip_depth(rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist);
        void updateStreamCalibData(const rs2::video_stream_profile& video_profile);
        void publishStaticTransforms();
        void publishPointCloud(rs2::points f, const ros::Time& t, const rs2::frameset& frameset);
        Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics, const std::string& frame_id) const;

        IMUInfo getImuInfo(const stream_index_pair& stream_index);
        void publishFrame(rs2::frame f, const ros::Time& t,
                          const stream_index_pair& stream,
                          std::map<stream_index_pair, cv::Mat>& images,
                          const std::map<stream_index_pair, ros::Publisher>& info_publishers,
                          const std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics>& image_publishers,
                          std::map<stream_index_pair, int>& seq,
                          std::map<stream_index_pair, sensor_msgs::CameraInfo>& camera_info,
                          const std::map<stream_index_pair, std::string>& optical_frame_id,
                          const std::map<rs2_stream, std::string>& encoding,
                          bool copy_data_from_frame = true);
        bool getEnabledProfile(const stream_index_pair& stream_index, rs2::stream_profile& profile);

        void publishAlignedDepthToOthers(rs2::frameset frames, const ros::Time& t);
        static void callback(const ddynamic_reconfigure::DDMap& map, int, rs2::options sensor);
        double FillImuData_Copy(const stream_index_pair stream_index, const CIMUHistory::imuData imu_data, sensor_msgs::Imu& imu_msg);
        double FillImuData_LinearInterpolation(const stream_index_pair stream_index, const CIMUHistory::imuData imu_data, sensor_msgs::Imu& imu_msg);
        static void ConvertFromOpticalFrameToFrame(float3& data);
        void imu_callback(rs2::frame frame);
        void imu_callback_sync(rs2::frame frame, imu_sync_method sync_method=imu_sync_method::COPY);
        void pose_callback(rs2::frame frame);
        void multiple_message_callback(rs2::frame frame, imu_sync_method sync_method);
        void frame_callback(rs2::frame frame);
        void registerDynamicOption(ros::NodeHandle& nh, rs2::options sensor, std::string& module_name);
        rs2_stream rs2_string_to_stream(std::string str);

        rs2::device _dev;
        ros::NodeHandle& _node_handle, _pnh;
        std::map<stream_index_pair, rs2::sensor> _sensors;
        std::map<std::string, std::function<void(rs2::frame)>> _sensors_callback;
        std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>> _ddynrec;

        std::string _json_file_path;
        std::string _serial_no;
        float _depth_scale_meters;
        float _clipping_distance;
        double _linear_accel_cov;
        double _angular_velocity_cov;
        bool  _hold_back_imu_for_frames;

        std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;
        std::map<stream_index_pair, int> _width;
        std::map<stream_index_pair, int> _height;
        std::map<stream_index_pair, int> _fps;
        std::map<stream_index_pair, bool> _enable;
        std::map<rs2_stream, std::string> _stream_name;
        tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;

        std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics> _image_publishers;
        std::map<stream_index_pair, ros::Publisher> _imu_publishers;
        std::shared_ptr<SyncedImuPublisher> _synced_imu_publisher;
        std::map<rs2_stream, int> _image_format;
        std::map<stream_index_pair, ros::Publisher> _info_publisher;
        std::map<stream_index_pair, cv::Mat> _image;
        std::map<rs2_stream, std::string> _encoding;
        std::map<stream_index_pair, std::vector<uint8_t>> _aligned_depth_images;

        std::map<stream_index_pair, int> _seq;
        std::map<rs2_stream, int> _unit_step_size;
        std::map<stream_index_pair, sensor_msgs::CameraInfo> _camera_info;
        std::atomic_bool _is_initialized_time_base;
        double _camera_time_base;
        std::map<stream_index_pair, std::vector<rs2::stream_profile>> _enabled_profiles;

        ros::Publisher _pointcloud_publisher;
        ros::Time _ros_time_base;
        bool _sync_frames;
        bool _pointcloud;
        imu_sync_method _imu_sync_method;
        std::string _filters_str;
        stream_index_pair _pointcloud_texture;
        PipelineSyncer _syncer;
        std::vector<NamedFilter> _filters;
        std::vector<rs2::sensor> _dev_sensors;
        std::map<rs2_stream, std::shared_ptr<rs2::align>> _align;

        std::map<stream_index_pair, cv::Mat> _depth_aligned_image;
        std::map<rs2_stream, std::string> _depth_aligned_encoding;
        std::map<stream_index_pair, sensor_msgs::CameraInfo> _depth_aligned_camera_info;
        std::map<stream_index_pair, int> _depth_aligned_seq;
        std::map<stream_index_pair, ros::Publisher> _depth_aligned_info_publisher;
        std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics> _depth_aligned_image_publishers;
        std::map<stream_index_pair, ros::Publisher> _depth_to_other_extrinsics_publishers;
        std::map<stream_index_pair, rs2_extrinsics> _depth_to_other_extrinsics;

        const std::string _namespace;

    };//end class

}

