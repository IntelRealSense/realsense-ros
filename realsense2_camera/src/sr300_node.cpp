#include "../include/sr300_node.h"

using namespace realsense2_camera;

SR300Node::SR300Node(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle, rs2::device dev, const std::string& serial_no)
    : BaseRealSenseNode(nodeHandle, privateNodeHandle, dev, serial_no)
{}

void SR300Node::registerDynamicReconfigCb()
{
    _f = boost::bind(&SR300Node::callback, this, _1, _2);
    _server.setCallback(_f);
}

void SR300Node::setParam(sr300_paramsConfig &config, sr300_param param)
{
    // W/O for zero param
    if (0 == param)
        return;

    switch (param) {
    case sr300_param_color_backlight_compensation:
        ROS_DEBUG_STREAM("sr300_param_color_backlight_compensation: " << config.sr300_color_backlight_compensation);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_BACKLIGHT_COMPENSATION, config.sr300_color_backlight_compensation);
        break;
    case sr300_param_color_brightness:
        ROS_DEBUG_STREAM("sr300_color_backlight_compensation: " << config.sr300_color_backlight_compensation);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_BRIGHTNESS, config.sr300_color_brightness);
        break;
    case sr300_param_color_contrast:
        ROS_DEBUG_STREAM("sr300_param_color_contrast: " << config.sr300_color_contrast);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_CONTRAST, config.sr300_color_contrast);
        break;
    case sr300_param_color_gain:
        ROS_DEBUG_STREAM("sr300_param_color_gain: " << config.sr300_color_gain);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_GAIN, config.sr300_color_gain);
        break;
    case sr300_param_color_gamma:
        ROS_DEBUG_STREAM("sr300_param_color_gain: " << config.sr300_color_gamma);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_GAMMA, config.sr300_color_gamma);
        break;
    case sr300_param_color_hue:
        ROS_DEBUG_STREAM("sr300_param_color_hue: " << config.sr300_color_hue);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_HUE, config.sr300_color_hue);
        break;
    case sr300_param_color_saturation:
        ROS_DEBUG_STREAM("sr300_param_color_saturation: " << config.sr300_color_saturation);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_SATURATION, config.sr300_color_saturation);
        break;
    case sr300_param_color_sharpness:
        ROS_DEBUG_STREAM("sr300_param_color_sharpness: " << config.sr300_color_sharpness);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_SHARPNESS, config.sr300_color_sharpness);
        break;
    case sr300_param_color_white_balance:
        ROS_DEBUG_STREAM("sr300_param_color_white_balance: " << config.sr300_color_white_balance);
        if (_sensors[COLOR].get_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE))
            _sensors[COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);

        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_WHITE_BALANCE, config.sr300_color_white_balance);
        break;
    case sr300_param_color_enable_auto_white_balance:
        ROS_DEBUG_STREAM("rs435_depth_emitter_enabled: " << config.sr300_color_enable_auto_white_balance);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, config.sr300_color_enable_auto_white_balance);
        break;
    case sr300_param_color_exposure:
        ROS_DEBUG_STREAM("sr300_param_color_exposure: " << config.sr300_color_exposure);
        if (_sensors[COLOR].get_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE))
            _sensors[COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);

        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_EXPOSURE, config.sr300_color_exposure);
        break;
    case sr300_param_color_enable_auto_exposure:
        ROS_DEBUG_STREAM("sr300_param_color_enable_auto_exposure: " << config.sr300_color_enable_auto_white_balance);
        _sensors[COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.sr300_color_enable_auto_exposure);
        break;
    case sr300_param_depth_visual_preset:
        ROS_DEBUG_STREAM("sr300_param_color_enable_auto_exposure: " << config.sr300_depth_visual_preset);
        _sensors[DEPTH].set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, config.sr300_depth_visual_preset);
        break;
    case sr300_param_depth_laser_power:
        ROS_DEBUG_STREAM("sr300_param_color_enable_auto_exposure: " << config.sr300_depth_laser_power);
        _sensors[DEPTH].set_option(rs2_option::RS2_OPTION_LASER_POWER, config.sr300_depth_laser_power);
        break;
    case sr300_param_depth_accuracy:
        ROS_DEBUG_STREAM("sr300_param_depth_accuracy: " << config.sr300_depth_accuracy);
        _sensors[DEPTH].set_option(rs2_option::RS2_OPTION_ACCURACY, config.sr300_depth_accuracy);
        break;
    case sr300_param_depth_motion_range:
        ROS_DEBUG_STREAM("sr300_param_depth_motion_range: " << config.sr300_depth_motion_range);
        _sensors[DEPTH].set_option(rs2_option::RS2_OPTION_MOTION_RANGE, config.sr300_depth_motion_range);
        break;
    case sr300_param_depth_filter_option:
        ROS_DEBUG_STREAM("sr300_param_depth_filter_option: " << config.sr300_depth_filter_option);
        _sensors[DEPTH].set_option(rs2_option::RS2_OPTION_FILTER_OPTION, config.sr300_depth_filter_option);
        break;
    case sr300_param_depth_confidence_threshold:
        ROS_DEBUG_STREAM("sr300_param_depth_confidence_threshold: " << config.sr300_depth_confidence_threshold);
        _sensors[DEPTH].set_option(rs2_option::RS2_OPTION_CONFIDENCE_THRESHOLD, config.sr300_depth_confidence_threshold);
        break;
    case sr300_param_depth_frames_queue_size:
        ROS_DEBUG_STREAM("sr300_param_depth_frames_queue_size: " << config.sr300_depth_frames_queue_size);
        _sensors[DEPTH].set_option(rs2_option::RS2_OPTION_FRAMES_QUEUE_SIZE, config.sr300_depth_frames_queue_size);
        break;
    case sr300_param_depth_units:
        break;
    default:
            ROS_WARN_STREAM("Unrecognized sr300 param (" << param << ")");
        break;
    }

    // TODO
//    // Auto range options
//    "sr300_auto_range_enable_motion_versus_range"
//    "sr300_auto_range_enable_laser",

//    // Set only if sr300_auto_range_enable_motion_versus_range is set to 1
//    "sr300_auto_range_min_motion_versus_range",
//    "sr300_auto_range_max_motion_versus_range",
//    "sr300_auto_range_start_motion_versus_range",

//    // Set only if sr300_auto_range_enable_laser is enabled
//    "sr300_auto_range_min_laser"
//    "sr300_auto_range_max_laser"
//    "sr300_auto_range_start_laser"

//    "sr300_auto_range_upper_threshold"
//    "sr300_auto_range_lower_threshold"
}

void SR300Node::callback(sr300_paramsConfig &config, uint32_t level)
{
    ROS_DEBUG_STREAM("SR300Node - Level: " << level);

    if (set_default_dynamic_reconfig_values == level)
    {
        for (int i = 1 ; i < sr300_param_count ; ++i)
        {
            ROS_DEBUG_STREAM("sr300_param = " << i);
            setParam(config ,(sr300_param)i);
        }
    }
    else
    {
        setParam(config, (sr300_param)level);
    }
}
