#include "../include/rs435_node.h"

using namespace realsense2_camera;


RS435Node::RS435Node(ros::NodeHandle& nodeHandle,
                     ros::NodeHandle& privateNodeHandle,
                     rs2::device dev, const std::string& serial_no)
    : BaseD400Node(nodeHandle, privateNodeHandle, dev, serial_no)
{}

void RS435Node::registerDynamicReconfigCb()
{
    _f = boost::bind(&RS435Node::callback, this, _1, _2);
    _server.setCallback(_f);
}

void RS435Node::setParam(rs435_paramsConfig &config, rs435_param param)
{
    // W/O for zero param
    if (0 == param)
        return;

    switch (param) {
    case rs435_color_backlight_compensation:
        ROS_DEBUG_STREAM("rs435_color_backlight_compensation: " << config.rs435_color_backlight_compensation);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_BACKLIGHT_COMPENSATION, config.rs435_color_backlight_compensation);
        break;
    case rs435_color_brightness:
        ROS_DEBUG_STREAM("rs435_color_brightness: " << config.rs435_color_brightness);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_BRIGHTNESS, config.rs435_color_brightness);
        break;
    case rs435_color_contrast:
        ROS_DEBUG_STREAM("rs435_color_contrast: " << config.rs435_color_contrast);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_CONTRAST, config.rs435_color_contrast);
        break;
    case rs435_color_gain:
        ROS_DEBUG_STREAM("rs435_color_gain: " << config.rs435_color_gain);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_GAIN, config.rs435_color_gain);
        break;
    case rs435_color_gamma:
        ROS_DEBUG_STREAM("rs435_color_gamma: " << config.rs435_color_gamma);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_GAMMA, config.rs435_color_gamma);
        break;
    case rs435_color_hue:
        ROS_DEBUG_STREAM("rs435_color_hue: " << config.rs435_color_hue);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_HUE, config.rs435_color_hue);
        break;
    case rs435_color_saturation:
        ROS_DEBUG_STREAM("rs435_color_saturation: " << config.rs435_color_saturation);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_SATURATION, config.rs435_color_saturation);
        break;
    case rs435_color_sharpness:
        ROS_DEBUG_STREAM("rs435_color_sharpness: " << config.rs435_color_sharpness);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_SHARPNESS, config.rs435_color_sharpness);
        break;
    case rs435_color_enable_auto_exposure:
        ROS_DEBUG_STREAM("rs435_color_enable_auto_exposure: " << config.rs435_color_enable_auto_exposure);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.rs435_color_enable_auto_exposure);
        break;
    case rs435_color_exposure:
        ROS_DEBUG_STREAM("rs435_color_exposure: " << config.rs435_color_exposure);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_EXPOSURE, config.rs435_color_exposure);
        break;
    case rs435_color_white_balance:
        ROS_DEBUG_STREAM("rs435_color_white_balance: " << config.rs435_color_white_balance * 10);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_WHITE_BALANCE, config.rs435_color_white_balance * 10);
        break;
    case rs435_color_frames_queue_size:
        ROS_DEBUG_STREAM("rs435_color_frames_queue_size: " << config.rs435_color_frames_queue_size);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_FRAMES_QUEUE_SIZE, config.rs435_color_frames_queue_size);
        break;
    case rs435_color_power_line_frequency:
        ROS_DEBUG_STREAM("rs435_color_power_line_frequency: " << config.rs435_color_power_line_frequency);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_POWER_LINE_FREQUENCY, config.rs435_color_power_line_frequency);
        break;
    case rs435_color_auto_exposure_priority:
        ROS_DEBUG_STREAM("rs435_color_auto_exposure_priority: " << config.rs435_color_auto_exposure_priority);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_AUTO_EXPOSURE_PRIORITY, config.rs435_color_auto_exposure_priority);
        break;
    case rs435_depth_exposure:
    {
        static const auto rs435_depth_exposure_factor = 20;
        ROS_DEBUG_STREAM("rs435_depth_exposure: " << config.rs435_depth_exposure * rs435_depth_exposure_factor);
        _sensors[DEPTH].set_option(rs2_option::RS2_OPTION_EXPOSURE, config.rs435_depth_exposure * rs435_depth_exposure_factor);
    }
        break;
    case rs435_depth_laser_power:
    {
        static const auto rs435_depth_laser_power_factor = 30;
        ROS_DEBUG_STREAM("rs435_depth_laser_power: " << config.rs435_depth_laser_power * rs435_depth_laser_power_factor);
        _sensors[DEPTH].set_option(rs2_option::RS2_OPTION_LASER_POWER, config.rs435_depth_laser_power * rs435_depth_laser_power_factor);
    }
        break;
    case rs435_depth_emitter_enabled:
        ROS_DEBUG_STREAM("rs435_depth_emitter_enabled: " << config.rs435_depth_emitter_enabled);
        _sensors[DEPTH].set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, config.rs435_depth_emitter_enabled);
        break;
    default:
        BaseD400Node::setParam(config, (base_depth_param)param);
        break;
    }
}

void RS435Node::callback(rs435_paramsConfig &config, uint32_t level)
{
    ROS_DEBUG_STREAM("RS435Node - Level: " << level);

    if (set_default_dynamic_reconfig_values == level)
    {
        for (int i = 1 ; i < rs435_param_count ; ++i)
        {
            ROS_DEBUG_STREAM("rs435_param = " << i);
            setParam(config ,(rs435_param)i);
        }
    }
    else
    {
        setParam(config, (rs435_param)level);
    }
}
