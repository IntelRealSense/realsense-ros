// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <constants.h>
#include <realsense2_camera/Extrinsics.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <realsense2_camera/IMUInfo.h>
#include <csignal>
#include <eigen3/Eigen/Geometry>
#include <fstream>


namespace realsense2_camera
{
    const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
    const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
    const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
    const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
    const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
    const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
    const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};

    const std::vector<std::vector<stream_index_pair>> IMAGE_STREAMS = {{{DEPTH, INFRA1, INFRA2},
                                                                        {COLOR},
                                                                        {FISHEYE}}};

    const std::vector<std::vector<stream_index_pair>> HID_STREAMS = {{GYRO, ACCEL}};

    inline void signalHandler(int signum)
    {
        ROS_INFO_STREAM(strsignal(signum) << " Signal is received! Terminating RealSense Node...");
        ros::shutdown();
        exit(signum);
    }

    class InterfaceRealSenseNode
    {
    public:
        virtual void publishTopics() = 0;
        virtual void registerDynamicReconfigCb() = 0;
        virtual ~InterfaceRealSenseNode() = default;
    };

    class RealSenseNodeFactory : public nodelet::Nodelet
    {
    public:
        RealSenseNodeFactory();
        virtual ~RealSenseNodeFactory() {}

    private:
        virtual void onInit() override;
        void tryGetLogSeverity(rs2_log_severity& severity) const;

        std::unique_ptr<InterfaceRealSenseNode> _realSenseNode;
        rs2::device _device;
        rs2::context _ctx;
    };
}//end namespace
