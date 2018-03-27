// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include "../include/base_realsense_node.h"
#include <realsense2_camera/rs415_paramsConfig.h>

namespace realsense2_camera
{
    enum rs415_param{
        rs415_depth_enable_auto_white_balance = 9,
        rs415_depth_exposure,
        rs415_depth_laser_power,
        rs415_depth_emitter_enabled,
        rs415_color_backlight_compensation,
        rs415_color_brightness,
        rs415_color_contrast,
        rs415_color_exposure,
        rs415_color_gain,
        rs415_color_gamma,
        rs415_color_hue,
        rs415_color_saturation,
        rs415_color_sharpness,
        rs415_color_white_balance,
        rs415_color_enable_auto_exposure,
        rs415_color_enable_auto_white_balance,
        rs415_color_frames_queue_size,
        rs415_color_power_line_frequency,
        rs415_color_auto_exposure_priority,
        rs415_param_count
    };

    class RS415Node : public BaseD400Node
    {
    public:
        RS415Node(ros::NodeHandle& nodeHandle,
                  ros::NodeHandle& privateNodeHandle,
                  rs2::device dev, const std::string& serial_no);

        virtual void registerDynamicReconfigCb() override;

    private:
        void callback(rs415_paramsConfig &config, uint32_t level);
        void setParam(rs415_paramsConfig &config, rs415_param param);

        dynamic_reconfigure::Server<rs415_paramsConfig> _server;
        dynamic_reconfigure::Server<rs415_paramsConfig>::CallbackType _f;
    };
}
