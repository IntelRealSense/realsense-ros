#pragma once

#include <base_realsense_node.h>
#include <realsense2_camera/MapPathString.h>

namespace realsense2_camera
{
    class T265RealsenseNode : public BaseRealSenseNode
    {
        public:
            T265RealsenseNode(ros::NodeHandle& nodeHandle,
                              ros::NodeHandle& privateNodeHandle,
                              rs2::device dev,
                              const std::string& serial_no);
            virtual void toggleSensors(bool enabled) override;
            virtual void publishTopics() override;

        protected:
            void calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile) override;

        private:
            void initializeOdometryInput();
            void setupMapReutilization();
            void setupSubscribers();
            void handleWarning();   
            void odom_in_callback(const nav_msgs::Odometry::ConstPtr& msg);
            void warningDiagnostic (diagnostic_updater::DiagnosticStatusWrapper &stat);
            bool importLocalizationMap(const std::string &localization_file);
            bool exportLocalizationMap(const std::string &localization_file);
            std::vector<uint8_t> bytesFromRawFile(const std::string &filename);
            void rawFileFromBytes(const std::string &filename, const std::vector<uint8_t> &bytes);
            void initMapFrame(bool relocalizing);
            ros::Timer _timer;
            bool relocalization_pose_initialized;
            tf2_ros::TransformBroadcaster _dynamic_tf_broadcaster;
            diagnostic_updater::Updater callback_updater;

            // Service callbacks
            bool LoadRelocalizationMapSrv(realsense2_camera::MapPathString::Request &req,
                                          realsense2_camera::MapPathString::Response &res);
            bool SaveRelocalizationMapSrv(realsense2_camera::MapPathString::Request &req,
                                          realsense2_camera::MapPathString::Response &res);

            ros::Subscriber _odom_subscriber;
            ros::ServiceServer _load_map_srv;
            ros::ServiceServer _save_map_srv;
            rs2::pose_sensor _pose_sensor;
            rs2::wheel_odometer _wo_snr;
            bool _use_odom_in;
            std::string  _T265_fault;
    };
}
