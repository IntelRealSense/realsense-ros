#pragma once

#include <base_realsense_node.h>
#include "std_msgs/msg/string.hpp"

namespace realsense2_camera
{
    class T265RealsenseNode : public BaseRealSenseNode
    {
        public:
            T265RealsenseNode(rclcpp::Node& node,
                          rs2::device dev, std::shared_ptr<Parameters> parameters);
            virtual bool toggleSensors(bool enabled, std::string& msg) override;
            virtual void publishTopics() override;

        protected:
            void calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile) override;

        private:
            void initializeOdometryInput();
            void setupSubscribers();
            void odom_in_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscriber;
            rs2::wheel_odometer _wo_snr;
            bool _use_odom_in;
    };
}
