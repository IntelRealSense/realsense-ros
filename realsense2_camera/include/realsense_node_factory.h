// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once

// cpplint: c system headers
#include "constants.h"
#include "base_realsense_node.h"
#include <builtin_interfaces/msg/time.hpp>
#include <console_bridge/console.h>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include <algorithm>
#include <csignal>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>

namespace realsense2_camera
{
    class RealSenseNodeFactory : public rclcpp::Node
    {
    public:
        explicit RealSenseNodeFactory(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
        RealSenseNodeFactory(
            const std::string & node_name, const std::string & ns,
            const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
        virtual ~RealSenseNodeFactory();

    private:
        void init();
        void closeDevice();
        void startDevice();
        void changeDeviceCallback(rs2::event_information& info);
        void getDevice(rs2::device_list list);
        void tryGetLogSeverity(rs2_log_severity& severity) const;
        static std::string parseUsbPort(std::string line);

        rclcpp::Node::SharedPtr _node;
        rs2::device _device;
        std::unique_ptr<BaseRealSenseNode> _realSenseNode;
        rs2::context _ctx;
        std::string _serial_no;
        std::string _usb_port_id;
        std::string _device_type;
        double _wait_for_device_timeout;
        double _reconnect_timeout;
        bool _initial_reset;
        std::thread _query_thread;
        bool _is_alive;
        rclcpp::Logger _logger;
        std::shared_ptr<Parameters> _parameters;
    };
}//end namespace
