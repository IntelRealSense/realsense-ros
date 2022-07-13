// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once

#include <base_realsense_node.h>
#include "std_msgs/msg/string.hpp"

namespace realsense2_camera
{
    class T265RealsenseNode : public BaseRealSenseNode
    {
        public:
            T265RealsenseNode(rclcpp::Node& node,
                          rs2::device dev,
                          std::shared_ptr<Parameters> parameters,
                          bool use_intra_process = false);
            void publishTopics();

        protected:
            void calcAndPublishStaticTransform(const rs2::stream_profile& profile, const rs2::stream_profile& base_profile) override;

        private:
            void initializeOdometryInput();
            void setupSubscribers();
            void odom_in_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscriber;
            rs2::wheel_odometer _wo_snr;
            bool _use_odom_in;
    };
}
