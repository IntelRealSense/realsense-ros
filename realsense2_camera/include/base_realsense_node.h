// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once

#include "../include/realsense_node_factory.h"
#include <dynamic_reconfigure/server.h>
#include <realsense2_camera/base_d400_paramsConfig.h>
#include <realsense2_camera/rs415_paramsConfig.h>
#include <realsense2_camera/rs435_paramsConfig.h>


namespace realsense2_camera
{
    enum base_depth_param{
        base_depth_gain = 1,
        base_depth_enable_auto_exposure,
        base_depth_visual_preset,
        base_depth_frames_queue_size,
        base_depth_error_polling_enabled,
        base_depth_output_trigger_enabled,
        base_depth_units,
        base_JSON_file_path,
        base_depth_count
    };

    class BaseRealSenseNode : public InterfaceRealSenseNode
    {
    public:
        BaseRealSenseNode(ros::NodeHandle& nodeHandle,
                          ros::NodeHandle& privateNodeHandle,
                          rs2::device dev,
                          const std::string& serial_no);

        virtual void publishTopics() override;
        virtual void registerDynamicReconfigCb() override;
        virtual ~BaseRealSenseNode() {}

    protected:

        const uint32_t set_default_dynamic_reconfig_values = 0xffffffff;
        rs2::device _dev;
        ros::NodeHandle& _node_handle, _pnh;
        std::map<stream_index_pair, rs2::sensor> _sensors;

    private:
        struct float3
        {
            float x, y, z;
        };

        struct quaternion
        {
            double x, y, z, w;
        };

        static std::string getNamespaceStr();
        void getParameters();
        void setupDevice();
        void setupPublishers();
        void setupStreams();
        void updateStreamCalibData(const rs2::video_stream_profile& video_profile);
        Eigen::Quaternionf rotationMatrixToQuaternion(const float rotation[3]) const;
        void publish_static_tf(const ros::Time& t,
                               const float3& trans,
                               const quaternion& q,
                               const std::string& from,
                               const std::string& to);
        void publishStaticTransforms();
        void publishRgbToDepthPCTopic(const ros::Time& t, const std::map<stream_index_pair, bool>& is_frame_arrived);
        Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics, const std::string& frame_id) const;
        rs2_extrinsics getRsExtrinsics(const stream_index_pair& from_stream, const stream_index_pair& to_stream);

        IMUInfo getImuInfo(const stream_index_pair& stream_index);
        void publishFrame(rs2::frame f, const ros::Time& t,
                          const stream_index_pair& stream,
                          std::map<stream_index_pair, cv::Mat>& images,
                          const std::map<stream_index_pair, ros::Publisher>& info_publishers,
                          const std::map<stream_index_pair, image_transport::Publisher>& image_publishers,
                          std::map<stream_index_pair, int>& seq,
                          std::map<stream_index_pair, sensor_msgs::CameraInfo>& camera_info,
                          const std::map<stream_index_pair, std::string>& optical_frame_id,
                          const std::map<stream_index_pair, std::string>& encoding,
                          bool copy_data_from_frame = true);
        bool getEnabledProfile(const stream_index_pair& stream_index, rs2::stream_profile& profile);

        void updateIsFrameArrived(std::map<stream_index_pair, bool>& is_frame_arrived,
                                  rs2_stream stream_type, int stream_index);

        void publishAlignedDepthToOthers(rs2::frame depth_frame, const std::vector<rs2::frame>& frames, const ros::Time& t);

        void alignFrame(const rs2_intrinsics& from_intrin,
                        const rs2_intrinsics& other_intrin,
                        rs2::frame from_image,
                        uint32_t output_image_bytes_per_pixel,
                        const rs2_extrinsics& from_to_other,
                        std::vector<uint8_t>& out_vec);

        const rs2_extrinsics _i_ex{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}};

        std::string _json_file_path;
        std::string _serial_no;
        float _depth_scale_meters;

        std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;
        std::map<stream_index_pair, int> _width;
        std::map<stream_index_pair, int> _height;
        std::map<stream_index_pair, int> _fps;
        std::map<stream_index_pair, bool> _enable;
        std::map<stream_index_pair, std::string> _stream_name;
        tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;

        std::map<stream_index_pair, image_transport::Publisher> _image_publishers;
        std::map<stream_index_pair, ros::Publisher> _imu_publishers;
        std::map<stream_index_pair, int> _image_format;
        std::map<stream_index_pair, rs2_format> _format;
        std::map<stream_index_pair, ros::Publisher> _info_publisher;
        std::map<stream_index_pair, cv::Mat> _image;
        std::map<stream_index_pair, std::string> _encoding;
        std::map<stream_index_pair, std::vector<uint8_t>> _aligned_depth_images;

        std::string _base_frame_id;
        std::map<stream_index_pair, std::string> _frame_id;
        std::map<stream_index_pair, std::string> _optical_frame_id;
        std::map<stream_index_pair, int> _seq;
        std::map<stream_index_pair, int> _unit_step_size;
        std::map<stream_index_pair, sensor_msgs::CameraInfo> _camera_info;
        bool _intialize_time_base;
        double _camera_time_base;
        std::map<stream_index_pair, std::vector<rs2::stream_profile>> _enabled_profiles;

        ros::Publisher _pointcloud_publisher;
        ros::Time _ros_time_base;
        bool _align_depth;
        bool _sync_frames;
        bool _pointcloud;
        rs2::asynchronous_syncer _syncer;

        std::map<stream_index_pair, cv::Mat> _depth_aligned_image;
        std::map<stream_index_pair, std::string> _depth_aligned_encoding;
        std::map<stream_index_pair, sensor_msgs::CameraInfo> _depth_aligned_camera_info;
        std::map<stream_index_pair, int> _depth_aligned_seq;
        std::map<stream_index_pair, ros::Publisher> _depth_aligned_info_publisher;
        std::map<stream_index_pair, image_transport::Publisher> _depth_aligned_image_publishers;
        std::map<stream_index_pair, std::string> _depth_aligned_frame_id;
        std::map<stream_index_pair, ros::Publisher> _depth_to_other_extrinsics_publishers;
        std::map<stream_index_pair, rs2_extrinsics> _depth_to_other_extrinsics;

        std::map<stream_index_pair, bool> _is_frame_arrived;
        const std::string _namespace;
    };//end class

    class BaseD400Node : public BaseRealSenseNode
    {
    public:
        BaseD400Node(ros::NodeHandle& nodeHandle,
                     ros::NodeHandle& privateNodeHandle,
                     rs2::device dev, const std::string& serial_no);
        virtual void registerDynamicReconfigCb() override;

    protected:
        void setParam(rs415_paramsConfig &config, base_depth_param param);
        void setParam(rs435_paramsConfig &config, base_depth_param param);

    private:
        void callback(base_d400_paramsConfig &config, uint32_t level);
        void setOption(stream_index_pair sip, rs2_option opt, float val);
        void setParam(base_d400_paramsConfig &config, base_depth_param param);

        std::shared_ptr<dynamic_reconfigure::Server<base_d400_paramsConfig>> _server;
        dynamic_reconfigure::Server<base_d400_paramsConfig>::CallbackType _f;
    };
}
