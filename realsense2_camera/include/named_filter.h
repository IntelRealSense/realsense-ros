// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once

#include <string>
#include <memory>
#include <librealsense2/rs.hpp>
#include <sensor_params.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ros_sensor.h>

namespace realsense2_camera
{
    class NamedFilter
    {
        public:
            NamedFilter(std::shared_ptr<rs2::filter> filter, std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled=false, bool is_set_parameters=true);
            bool is_enabled() {return _is_enabled;};
            rs2::frameset Process(rs2::frameset frameset);
            rs2::frame Process(rs2::frame frame);

        protected:
            void setParameters(std::function<void(const rclcpp::Parameter&)> enable_param_func = std::function<void(const rclcpp::Parameter&)>());

        private:
            void clearParameters();

        public:
            std::shared_ptr<rs2::filter> _filter;

        protected:
            bool _is_enabled;
            SensorParams _params;
            std::vector<std::string> _parameters_names;
            rclcpp::Logger _logger;

    };

    class PointcloudFilter : public NamedFilter
    {
        public:
            PointcloudFilter(std::shared_ptr<rs2::filter> filter, rclcpp::Node& node, std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled=false);
        
            void setPublisher();
            void Publish(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset, const std::string& frame_id);

        private:
            void setParameters();

        private:
            bool _is_enabled_pc;
            rclcpp::Node& _node;
            bool _allow_no_texture_points;
            bool _ordered_pc;
            std::mutex _mutex_publisher;
            sensor_msgs::msg::PointCloud2 _msg_pointcloud;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_publisher;
            std::string _pointcloud_qos;
    };

    class AlignDepthFilter : public NamedFilter
    {
        public:
            AlignDepthFilter(std::shared_ptr<rs2::filter> filter, std::function<void(const rclcpp::Parameter&)> update_align_depth_func,
                std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled = false);
    };
}
