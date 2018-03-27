// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include "../include/base_realsense_node.h"
#include <realsense2_camera/sr300_paramsConfig.h>

namespace realsense2_camera
{
    enum sr300_param{
        sr300_param_color_backlight_compensation = 1,
        sr300_param_color_brightness,
        sr300_param_color_contrast,
        sr300_param_color_gain,
        sr300_param_color_gamma,
        sr300_param_color_hue,
        sr300_param_color_saturation,
        sr300_param_color_sharpness,
        sr300_param_color_white_balance,
        sr300_param_color_enable_auto_white_balance,
        sr300_param_color_exposure,
        sr300_param_color_enable_auto_exposure,
        sr300_param_depth_visual_preset,
        sr300_param_depth_laser_power,
        sr300_param_depth_accuracy,
        sr300_param_depth_motion_range,
        sr300_param_depth_filter_option,
        sr300_param_depth_confidence_threshold,
        sr300_param_depth_frames_queue_size,
        sr300_param_depth_units,
        sr300_param_count
    };

    class SR300Node : public BaseRealSenseNode
    {
    public:
        SR300Node(ros::NodeHandle& nodeHandle,
                  ros::NodeHandle& privateNodeHandle,
                  rs2::device dev, const std::string& serial_no);

        virtual void registerDynamicReconfigCb() override;

    private:
        void callback(sr300_paramsConfig &config, uint32_t level);
        void setParam(sr300_paramsConfig &config, sr300_param param);

        dynamic_reconfigure::Server<sr300_paramsConfig> _server;
        dynamic_reconfigure::Server<sr300_paramsConfig>::CallbackType _f;
    };
}
