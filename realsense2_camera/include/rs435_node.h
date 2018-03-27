// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include "../include/base_realsense_node.h"
#include <realsense2_camera/rs435_paramsConfig.h>

namespace realsense2_camera
{
    enum rs435_param{
        rs435_depth_exposure = 9,
        rs435_depth_laser_power,
        rs435_depth_emitter_enabled,
        rs435_color_backlight_compensation,
        rs435_color_brightness,
        rs435_color_contrast,
        rs435_color_exposure,
        rs435_color_gain,
        rs435_color_gamma,
        rs435_color_hue,
        rs435_color_saturation,
        rs435_color_sharpness,
        rs435_color_white_balance,
        rs435_color_enable_auto_exposure,
        rs435_color_frames_queue_size,
        rs435_color_power_line_frequency,
        rs435_color_auto_exposure_priority,
        rs435_param_count
    };

    class RS435Node : public BaseD400Node
    {
    public:
        RS435Node(ros::NodeHandle& nodeHandle,
                  ros::NodeHandle& privateNodeHandle,
                  rs2::device dev, const std::string& serial_no);

        virtual void registerDynamicReconfigCb() override;

    private:
            void callback(rs435_paramsConfig &config, uint32_t level);
            void setParam(rs435_paramsConfig &config, rs435_param param);

            dynamic_reconfigure::Server<rs435_paramsConfig> _server;
            dynamic_reconfigure::Server<rs435_paramsConfig>::CallbackType _f;
    };
}
