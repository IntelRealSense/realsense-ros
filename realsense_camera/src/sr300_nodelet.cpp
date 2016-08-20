/******************************************************************************
 Copyright (c) 2016, Intel Corporation
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <realsense_camera/sr300_nodelet.h>

PLUGINLIB_EXPORT_CLASS (realsense_camera::SR300Nodelet, nodelet::Nodelet)

namespace realsense_camera
{
  /*
   * Initialize the nodelet.
   */
  void SR300Nodelet::onInit()
  {
    format_[RS_STREAM_COLOR] = RS_FORMAT_RGB8;
    encoding_[RS_STREAM_COLOR] = sensor_msgs::image_encodings::RGB8;
    cv_type_[RS_STREAM_COLOR] = CV_8UC3;
    unit_step_size_[RS_STREAM_COLOR] = sizeof(unsigned char) * 3;

    format_[RS_STREAM_DEPTH] = RS_FORMAT_Z16;
    encoding_[RS_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;
    cv_type_[RS_STREAM_DEPTH] = CV_16UC1;
    unit_step_size_[RS_STREAM_DEPTH] = sizeof(uint16_t);

    format_[RS_STREAM_INFRARED] = RS_FORMAT_Y16;
    encoding_[RS_STREAM_INFRARED] = sensor_msgs::image_encodings::TYPE_16UC1;
    cv_type_[RS_STREAM_INFRARED] = CV_16UC1;
    unit_step_size_[RS_STREAM_INFRARED] = sizeof(uint16_t);

    max_z_ = SR300_MAX_Z;

    BaseNodelet::onInit();
  }

  /*
   * Set Dynamic Reconfigure Server and return the dynamic params.
   */
  std::vector<std::string> SR300Nodelet::setDynamicReconfServer()
  {
    dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<realsense_camera::sr300_paramsConfig>(pnh_));

    // Get dynamic options from the dynamic reconfigure server.
    realsense_camera::sr300_paramsConfig params_config;
    dynamic_reconf_server_->getConfigDefault(params_config);
    std::vector<realsense_camera::sr300_paramsConfig::AbstractParamDescriptionConstPtr> param_desc =
        params_config.__getParamDescriptions__();
    std::vector<std::string> dynamic_params;
    for (realsense_camera::sr300_paramsConfig::AbstractParamDescriptionConstPtr param_desc_ptr: param_desc)
    {
      dynamic_params.push_back((* param_desc_ptr).name);
    }

    return dynamic_params;
  }

  /*
   * Start Dynamic Reconfigure Callback.
   */
  void SR300Nodelet::startDynamicReconfCallback()
  {
    dynamic_reconf_server_->setCallback(boost::bind(&SR300Nodelet::configCallback, this, _1, _2));
  }

  /*
   * Get the dynamic param values.
   */
  void SR300Nodelet::configCallback(realsense_camera::sr300_paramsConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Setting dynamic camera options");
    // Set flags
    if (config.enable_depth == false)
    {
      if (enable_[RS_STREAM_COLOR] == false)
      {
        ROS_INFO_STREAM(nodelet_name_ << " - Color stream is also disabled. Cannot disable depth stream");
        config.enable_depth = true;
      }
      else
      {
        enable_[RS_STREAM_DEPTH] = false;
      }
    }
    else
    {
      enable_[RS_STREAM_DEPTH] = true;
    }

    // Set common options
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_BACKLIGHT_COMPENSATION, config.color_backlight_compensation, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_BRIGHTNESS, config.color_brightness, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_CONTRAST, config.color_contrast, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_GAIN, config.color_gain, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_GAMMA, config.color_gamma, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_HUE, config.color_hue, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_SATURATION, config.color_saturation, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_SHARPNESS, config.color_sharpness, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_ENABLE_AUTO_WHITE_BALANCE,
        config.color_enable_auto_white_balance, 0);
    if (config.color_enable_auto_white_balance == 0)
    {
      rs_set_device_option(rs_device_, RS_OPTION_COLOR_WHITE_BALANCE, config.color_white_balance, 0);
    }

    // Set SR300 options that are common with F200
    rs_set_device_option(rs_device_, RS_OPTION_F200_LASER_POWER, config.f200_laser_power, 0);
    rs_set_device_option(rs_device_, RS_OPTION_F200_ACCURACY, config.f200_accuracy, 0);
    rs_set_device_option(rs_device_, RS_OPTION_F200_MOTION_RANGE, config.f200_motion_range, 0);
    rs_set_device_option(rs_device_, RS_OPTION_F200_FILTER_OPTION, config.f200_filter_option, 0);
    rs_set_device_option(rs_device_, RS_OPTION_F200_CONFIDENCE_THRESHOLD, config.f200_confidence_threshold, 0);

    // Set SR300 specific options
    rs_set_device_option(rs_device_, RS_OPTION_SR300_AUTO_RANGE_ENABLE_MOTION_VERSUS_RANGE, config.sr300_auto_range_enable_motion_versus_range, 0);
    if (config.sr300_auto_range_enable_motion_versus_range == 1)
    {
      rs_set_device_option(rs_device_, RS_OPTION_SR300_AUTO_RANGE_MIN_MOTION_VERSUS_RANGE, config.sr300_auto_range_min_motion_versus_range, 0);
      rs_set_device_option(rs_device_, RS_OPTION_SR300_AUTO_RANGE_MAX_MOTION_VERSUS_RANGE, config.sr300_auto_range_max_motion_versus_range, 0);
      rs_set_device_option(rs_device_, RS_OPTION_SR300_AUTO_RANGE_START_MOTION_VERSUS_RANGE, config.sr300_auto_range_start_motion_versus_range, 0);
    }
    rs_set_device_option(rs_device_, RS_OPTION_SR300_AUTO_RANGE_ENABLE_LASER, config.sr300_auto_range_enable_laser, 0);
    if (config.sr300_auto_range_enable_laser == 1)
    {
      rs_set_device_option(rs_device_, RS_OPTION_SR300_AUTO_RANGE_MIN_LASER, config.sr300_auto_range_min_laser, 0);
      rs_set_device_option(rs_device_, RS_OPTION_SR300_AUTO_RANGE_MAX_LASER, config.sr300_auto_range_max_laser, 0);
      rs_set_device_option(rs_device_, RS_OPTION_SR300_AUTO_RANGE_START_LASER, config.sr300_auto_range_start_laser, 0);
    }
    rs_set_device_option(rs_device_, RS_OPTION_SR300_AUTO_RANGE_UPPER_THRESHOLD, config.sr300_auto_range_upper_threshold, 0);
    rs_set_device_option(rs_device_, RS_OPTION_SR300_AUTO_RANGE_LOWER_THRESHOLD, config.sr300_auto_range_lower_threshold, 0);
/*
    rs_set_device_option(rs_device_, RS_OPTION_SR300_WAKEUP_DEV_PHASE1_PERIOD, config.sr300_wakeup_dev_phase1_period, 0);
    rs_set_device_option(rs_device_, RS_OPTION_SR300_WAKEUP_DEV_PHASE1_FPS, config.sr300_wakeup_dev_phase1_fps, 0);
    rs_set_device_option(rs_device_, RS_OPTION_SR300_WAKEUP_DEV_PHASE2_PERIOD, config.sr300_wakeup_dev_phase2_period, 0);
    rs_set_device_option(rs_device_, RS_OPTION_SR300_WAKEUP_DEV_PHASE2_FPS, config.sr300_wakeup_dev_phase2_fps, 0);
    rs_set_device_option(rs_device_, RS_OPTION_SR300_WAKEUP_DEV_RESET, config.sr300_wakeup_dev_reset, 0);
    rs_set_device_option(rs_device_, RS_OPTION_SR300_WAKE_ON_USB_REASON, config.sr300_wake_on_usb_reason, 0);
    rs_set_device_option(rs_device_, RS_OPTION_SR300_WAKE_ON_USB_CONFIDENCE, config.sr300_wake_on_usb_confidence, 0);
*/
  }
}  // end namespace

