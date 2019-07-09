#pragma once

#include <base_realsense_node.h>

namespace realsense2_camera
{
    class T265RealsenseNode : public BaseRealSenseNode
    {
        public:
            T265RealsenseNode(ros::NodeHandle& nodeHandle,
                          ros::NodeHandle& privateNodeHandle,
                          rs2::device dev,
                          const std::string& serial_no);
            void publishTopics();

        protected:
            void calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile) override;

        private:
            void initializeOdometryInput();
            void setupSubscribers();
            void odom_in_callback(const nav_msgs::Odometry::ConstPtr& msg);

            ros::Subscriber _odom_subscriber;
            rs2::wheel_odometer _wo_snr;
            bool _use_odom_in;
    };
}
