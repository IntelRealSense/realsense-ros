/******************************************************************************
 Copyright (c) 2017, Intel Corporation
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

#include <string>
#include <algorithm>
#include <vector>

#include <realsense_camera/zr300_nodelet.h>

PLUGINLIB_EXPORT_CLASS(realsense_camera::ZR300Nodelet, nodelet::Nodelet)

namespace realsense_camera
{
  /*
   * Nodelet Destructor.
   */
  ZR300Nodelet::~ZR300Nodelet()
  {
    if (enable_imu_ == true)
    {
      stopIMU();
      // clean up imu thread
      imu_thread_->join();
    }
  }

  /*
   * Initialize the nodelet.
   */
  void ZR300Nodelet::onInit()
  {
    format_[RS_STREAM_COLOR] = RS_FORMAT_RGB8;
    encoding_[RS_STREAM_COLOR] = sensor_msgs::image_encodings::RGB8;
    cv_type_[RS_STREAM_COLOR] = CV_8UC3;
    unit_step_size_[RS_STREAM_COLOR] = sizeof(unsigned char) * 3;

    format_[RS_STREAM_DEPTH] = RS_FORMAT_Z16;
    encoding_[RS_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;
    cv_type_[RS_STREAM_DEPTH] = CV_16UC1;
    unit_step_size_[RS_STREAM_DEPTH] = sizeof(uint16_t);

    format_[RS_STREAM_INFRARED] = RS_FORMAT_Y8;
    encoding_[RS_STREAM_INFRARED] = sensor_msgs::image_encodings::TYPE_8UC1;
    cv_type_[RS_STREAM_INFRARED] = CV_8UC1;
    unit_step_size_[RS_STREAM_INFRARED] = sizeof(unsigned char);

    format_[RS_STREAM_INFRARED2] = RS_FORMAT_Y8;
    encoding_[RS_STREAM_INFRARED2] = sensor_msgs::image_encodings::TYPE_8UC1;
    cv_type_[RS_STREAM_INFRARED2] = CV_8UC1;
    unit_step_size_[RS_STREAM_INFRARED2] = sizeof(unsigned char);

    format_[RS_STREAM_FISHEYE] = RS_FORMAT_RAW8;
    encoding_[RS_STREAM_FISHEYE] = sensor_msgs::image_encodings::TYPE_8UC1;
    cv_type_[RS_STREAM_FISHEYE] = CV_8UC1;
    unit_step_size_[RS_STREAM_FISHEYE] = sizeof(unsigned char);

    max_z_ = ZR300_MAX_Z;

    BaseNodelet::onInit();

    if (enable_imu_ == true)
    {
      imu_thread_ =
          boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ZR300Nodelet::publishIMU, this)));
    }
  }

  /*
   * Get the nodelet parameters.
   */
  void ZR300Nodelet::getParameters()
  {
    BaseNodelet::getParameters();
    pnh_.param("ir2_frame_id", frame_id_[RS_STREAM_INFRARED2], DEFAULT_IR2_FRAME_ID);
    pnh_.param("ir2_optical_frame_id", optical_frame_id_[RS_STREAM_INFRARED2], DEFAULT_IR2_OPTICAL_FRAME_ID);
    pnh_.param("enable_fisheye", enable_[RS_STREAM_FISHEYE], ENABLE_FISHEYE);
    pnh_.param("enable_imu", enable_imu_, ENABLE_IMU);
    pnh_.param("enable_ir2", enable_[RS_STREAM_INFRARED2], ENABLE_IR2);
    pnh_.param("fisheye_width", width_[RS_STREAM_FISHEYE], FISHEYE_WIDTH);
    pnh_.param("fisheye_height", height_[RS_STREAM_FISHEYE], FISHEYE_HEIGHT);
    pnh_.param("fisheye_fps", fps_[RS_STREAM_FISHEYE], FISHEYE_FPS);
    pnh_.param("fisheye_frame_id", frame_id_[RS_STREAM_FISHEYE], DEFAULT_FISHEYE_FRAME_ID);
    pnh_.param("fisheye_optical_frame_id", optical_frame_id_[RS_STREAM_FISHEYE], DEFAULT_FISHEYE_OPTICAL_FRAME_ID);
    pnh_.param("imu_frame_id", imu_frame_id_, DEFAULT_IMU_FRAME_ID);
    pnh_.param("imu_optical_frame_id", imu_optical_frame_id_, DEFAULT_IMU_OPTICAL_FRAME_ID);

    // set IR2 stream to match depth
    width_[RS_STREAM_INFRARED2] = width_[RS_STREAM_DEPTH];
    height_[RS_STREAM_INFRARED2] = height_[RS_STREAM_DEPTH];
    fps_[RS_STREAM_INFRARED2] = fps_[RS_STREAM_DEPTH];
  }

  /*
   * Advertise topics.
   */
  void ZR300Nodelet::advertiseTopics()
  {
    BaseNodelet::advertiseTopics();
    ros::NodeHandle ir2_nh(nh_, IR2_NAMESPACE);
    image_transport::ImageTransport ir2_image_transport(ir2_nh);
    camera_publisher_[RS_STREAM_INFRARED2] = ir2_image_transport.advertiseCamera(IR2_TOPIC, 1);

    ros::NodeHandle fisheye_nh(nh_, FISHEYE_NAMESPACE);
    image_transport::ImageTransport fisheye_image_transport(fisheye_nh);
    camera_publisher_[RS_STREAM_FISHEYE] = fisheye_image_transport.advertiseCamera(FISHEYE_TOPIC, 1);

    ros::NodeHandle imu_nh(nh_, IMU_NAMESPACE);
    imu_publisher_ = imu_nh.advertise<sensor_msgs::Imu>(IMU_TOPIC, 1000);
  }

  /*
   * Advertise services.
   */
  void ZR300Nodelet::advertiseServices()
  {
    BaseNodelet::advertiseServices();
    get_imu_info_ = pnh_.advertiseService(IMU_INFO_SERVICE, &ZR300Nodelet::getIMUInfo, this);
  }

  /*
   * Get IMU Info.
   */
  bool ZR300Nodelet::getIMUInfo(realsense_camera::GetIMUInfo::Request & req,
      realsense_camera::GetIMUInfo::Response & res)
  {
    ros::Time header_stamp = ros::Time::now();
    std::string header_frame_id;


    rs_motion_intrinsics imu_intrinsics;
    rs_get_motion_intrinsics(rs_device_, &imu_intrinsics, &rs_error_);
    if (rs_error_)
    {
      ROS_ERROR_STREAM(nodelet_name_ << " - Verify camera firmware version!");
    }
    checkError();

    int index = 0;
    res.accel.header.stamp = header_stamp;
    res.accel.header.frame_id = IMU_ACCEL;
    std::transform(res.accel.header.frame_id.begin(), res.accel.header.frame_id.end(),
        res.accel.header.frame_id.begin(), ::tolower);

    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        res.accel.data[index] = imu_intrinsics.acc.data[i][j];
        ++index;
      }
      res.accel.noise_variances[i] = imu_intrinsics.acc.noise_variances[i];
      res.accel.bias_variances[i] = imu_intrinsics.acc.bias_variances[i];
    }

    index = 0;
    res.gyro.header.stamp = header_stamp;
    res.gyro.header.frame_id = IMU_GYRO;
    std::transform(res.gyro.header.frame_id.begin(), res.gyro.header.frame_id.end(),
        res.gyro.header.frame_id.begin(), ::tolower);
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        res.gyro.data[index] = imu_intrinsics.gyro.data[i][j];
        ++index;
      }
      res.gyro.noise_variances[i] = imu_intrinsics.gyro.noise_variances[i];
      res.gyro.bias_variances[i] = imu_intrinsics.gyro.bias_variances[i];
    }

    return true;
  }

  /*
   * Set Dynamic Reconfigure Server and return the dynamic params.
   */
  std::vector<std::string> ZR300Nodelet::setDynamicReconfServer()
  {
    dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<realsense_camera::zr300_paramsConfig>(pnh_));

    // Get dynamic options from the dynamic reconfigure server.
    realsense_camera::zr300_paramsConfig params_config;
    dynamic_reconf_server_->getConfigDefault(params_config);
    std::vector<realsense_camera::zr300_paramsConfig::AbstractParamDescriptionConstPtr> param_desc =
      params_config.__getParamDescriptions__();
    std::vector<std::string> dynamic_params;
    for (realsense_camera::zr300_paramsConfig::AbstractParamDescriptionConstPtr param_desc_ptr : param_desc)
    {
      dynamic_params.push_back((* param_desc_ptr).name);
    }

    return dynamic_params;
  }

  /*
   * Start Dynamic Reconfigure Callback.
   */
  void ZR300Nodelet::startDynamicReconfCallback()
  {
    dynamic_reconf_server_->setCallback(boost::bind(&ZR300Nodelet::configCallback, this, _1, _2));
  }

  /*
   * Change the Depth Control Preset
   */
  void ZR300Nodelet::setDynamicReconfigDepthControlPreset(int preset)
  {
    // There is not a C++ API for dynamic reconfig so we need to use a system call
    // Adding the sleep to ensure the current callback can end before we
    // attempt the next callback from the system call.
    std::vector<std::string> argv;
    argv.push_back("rosrun");
    argv.push_back("dynamic_reconfigure");
    argv.push_back("dynparam");
    argv.push_back("set");
    argv.push_back(nodelet_name_);
    argv.push_back("r200_dc_preset");
    argv.push_back(std::to_string(preset));

    wrappedSystem(argv);
  }

  /*
   * Change the Depth Control Individual values
   * GET ALL of the DC options from librealsense
   * Call dynamic reconfig and set all 10 values as a set
   */
  std::string ZR300Nodelet::setDynamicReconfigDepthControlIndividuals()
  {
    std::string current_param;
    std::string current_dc;
    std::string option_value;

    // There is not a C++ API for dynamic reconfig so we need to use a system call
    // Adding the sleep to ensure the current callback can end before we
    // attempt the next callback from the system call.
    std::vector<std::string> argv;
    argv.push_back("rosrun");
    argv.push_back("dynamic_reconfigure");
    argv.push_back("dynparam");
    argv.push_back("set");
    argv.push_back(nodelet_name_);

    current_param = "{";

    option_value =
      std::to_string(static_cast<uint32_t>(rs_get_device_option(rs_device_,
              RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_DECREMENT, 0)));
    current_param += "'r200_dc_estimate_median_decrement':" + option_value + ", ";
    current_dc += option_value + ":";

    option_value =
      std::to_string(static_cast<uint32_t>(rs_get_device_option(rs_device_,
              RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_INCREMENT, 0)));
    current_param += "'r200_dc_estimate_median_increment':" + option_value + ", ";
    current_dc += option_value + ":";

    option_value =
      std::to_string(static_cast<uint32_t>(rs_get_device_option(rs_device_,
              RS_OPTION_R200_DEPTH_CONTROL_MEDIAN_THRESHOLD, 0)));
    current_param += "'r200_dc_median_threshold':" + option_value + ", ";
    current_dc += option_value + ":";

    option_value =
      std::to_string(static_cast<uint32_t>(rs_get_device_option(rs_device_,
              RS_OPTION_R200_DEPTH_CONTROL_SCORE_MINIMUM_THRESHOLD, 0)));
    current_param += "'r200_dc_score_minimum_threshold':" + option_value + ", ";
    current_dc += option_value + ":";

    option_value =
      std::to_string(static_cast<uint32_t>(rs_get_device_option(rs_device_,
              RS_OPTION_R200_DEPTH_CONTROL_SCORE_MAXIMUM_THRESHOLD, 0)));
    current_param += "'r200_dc_score_maximum_threshold':" + option_value + ", ";
    current_dc += option_value + ":";

    option_value =
      std::to_string(static_cast<uint32_t>(rs_get_device_option(rs_device_,
              RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_COUNT_THRESHOLD, 0)));
    current_param += "'r200_dc_texture_count_threshold':" + option_value + ", ";
    current_dc += option_value + ":";

    option_value =
      std::to_string(static_cast<uint32_t>(rs_get_device_option(rs_device_,
              RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_DIFFERENCE_THRESHOLD, 0)));
    current_param += "'r200_dc_texture_difference_threshold':" + option_value + ", ";
    current_dc += option_value + ":";

    option_value =
      std::to_string(static_cast<uint32_t>(rs_get_device_option(rs_device_,
              RS_OPTION_R200_DEPTH_CONTROL_SECOND_PEAK_THRESHOLD, 0)));
    current_param += "'r200_dc_second_peak_threshold':" + option_value + ", ";
    current_dc += option_value + ":";

    option_value =
      std::to_string(static_cast<uint32_t>(rs_get_device_option(rs_device_,
              RS_OPTION_R200_DEPTH_CONTROL_NEIGHBOR_THRESHOLD, 0)));
    current_param += "'r200_dc_neighbor_threshold':" + option_value + ", ";
    current_dc += option_value + ":";

    option_value =
      std::to_string(static_cast<uint32_t>(rs_get_device_option(rs_device_,
              RS_OPTION_R200_DEPTH_CONTROL_LR_THRESHOLD, 0)));
    current_param += "'r200_dc_lr_threshold':" + option_value + "}";
    current_dc += option_value;

    ROS_DEBUG_STREAM(nodelet_name_ << " - Setting DC: " << current_param);

    argv.push_back(current_param);

    wrappedSystem(argv);

    return current_dc;
  }

  /*
   * Get the dynamic param values.
   */
  void ZR300Nodelet::configCallback(realsense_camera::zr300_paramsConfig &config, uint32_t level)
  {
    // Save the dc_preset value as there is no getter API for this value
    static int dc_preset = -2;
    int previous_dc_preset = dc_preset;
    // Save the last depth control preset values as a string
    static std::string last_dc;

    // level is the ORing of all levels which have a changed value
    std::bitset<32> bit_level{level};

    if (bit_level.test(6))  // 2^6 = 64 : Depth Control Preset
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Setting dynamic camera options" <<
          " (r200_dc_preset=" << config.r200_dc_preset << ")");
    }
    else
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Setting dynamic camera options");
    }

    // set the depth enable
    BaseNodelet::setDepthEnable(config.enable_depth);

    // Set common options
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_BACKLIGHT_COMPENSATION, config.color_backlight_compensation, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_BRIGHTNESS, config.color_brightness, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_CONTRAST, config.color_contrast, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_GAIN, config.color_gain, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_GAMMA, config.color_gamma, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_HUE, config.color_hue, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_SATURATION, config.color_saturation, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_SHARPNESS, config.color_sharpness, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_ENABLE_AUTO_EXPOSURE, config.color_enable_auto_exposure, 0);
    if (config.color_enable_auto_exposure == 0)
    {
      rs_set_device_option(rs_device_, RS_OPTION_COLOR_EXPOSURE, config.color_exposure, 0);
    }
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_ENABLE_AUTO_WHITE_BALANCE,
        config.color_enable_auto_white_balance, 0);
    if (config.color_enable_auto_white_balance == 0)
    {
      rs_set_device_option(rs_device_, RS_OPTION_COLOR_WHITE_BALANCE, config.color_white_balance, 0);
    }

    // Set R200 specific options
    rs_set_device_option(rs_device_, RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED, config.r200_lr_auto_exposure_enabled, 0);
    if (config.r200_lr_auto_exposure_enabled == 0)
    {
      rs_set_device_option(rs_device_, RS_OPTION_R200_LR_GAIN, config.r200_lr_gain, 0);
      rs_set_device_option(rs_device_, RS_OPTION_R200_LR_EXPOSURE, config.r200_lr_exposure, 0);
    }

    rs_set_device_option(rs_device_, RS_OPTION_R200_EMITTER_ENABLED, config.r200_emitter_enabled, 0);
    rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CLAMP_MIN, config.r200_depth_clamp_min, 0);
    rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CLAMP_MAX, config.r200_depth_clamp_max, 0);

    // Depth Control Group Settings
    // NOTE: do NOT use the config.groups values as they are zero the first time called
    if (bit_level.test(5))  // 2^5 = 32 : Individual Depth Control settings
    {
      std::string current_dc;

      ROS_DEBUG_STREAM(nodelet_name_ << " - Setting Individual Depth Control");

      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_DECREMENT,
          config.r200_dc_estimate_median_decrement, 0);
      current_dc += std::to_string(static_cast<uint32_t>(config.r200_dc_estimate_median_decrement)) + ":";
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_INCREMENT,
          config.r200_dc_estimate_median_increment, 0);
      current_dc += std::to_string(static_cast<uint32_t>(config.r200_dc_estimate_median_increment)) + ":";
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_MEDIAN_THRESHOLD,
          config.r200_dc_median_threshold, 0);
      current_dc += std::to_string(static_cast<uint32_t>(config.r200_dc_median_threshold)) + ":";
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_SCORE_MINIMUM_THRESHOLD,
          config.r200_dc_score_minimum_threshold, 0);
      current_dc += std::to_string(static_cast<uint32_t>(config.r200_dc_score_minimum_threshold)) + ":";
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_SCORE_MAXIMUM_THRESHOLD,
          config.r200_dc_score_maximum_threshold, 0);
      current_dc += std::to_string(static_cast<uint32_t>(config.r200_dc_score_maximum_threshold)) + ":";
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_COUNT_THRESHOLD,
          config.r200_dc_texture_count_threshold, 0);
      current_dc += std::to_string(static_cast<uint32_t>(config.r200_dc_texture_count_threshold)) + ":";
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_DIFFERENCE_THRESHOLD,
          config.r200_dc_texture_difference_threshold, 0);
      current_dc += std::to_string(static_cast<uint32_t>(config.r200_dc_texture_difference_threshold)) + ":";
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_SECOND_PEAK_THRESHOLD,
          config.r200_dc_second_peak_threshold, 0);
      current_dc += std::to_string(static_cast<uint32_t>(config.r200_dc_second_peak_threshold)) + ":";
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_NEIGHBOR_THRESHOLD,
          config.r200_dc_neighbor_threshold, 0);
      current_dc += std::to_string(static_cast<uint32_t>(config.r200_dc_neighbor_threshold)) + ":";
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_LR_THRESHOLD,
          config.r200_dc_lr_threshold, 0);
      current_dc += std::to_string(static_cast<uint32_t>(config.r200_dc_lr_threshold));

      // Preset also changed in the same update callback
      // This is either First callback special case, or both set via
      // dynamic configure command line.
      if (bit_level.test(6))  // 2^6 = 64 : Depth Control Preset
      {
        dc_preset = config.r200_dc_preset;

        if (previous_dc_preset != -2)  // not the first pass special case (-2)
        {
          // Changing individual Depth Control params means preset is Unused/Invalid
          // if the individual values are not the same as the preset values
          if (dc_preset != -1 && current_dc != last_dc)
          {
            ROS_DEBUG_STREAM(nodelet_name_ << " - Forcing Depth Control Preset to Unused");
            setDynamicReconfigDepthControlPreset(-1);
          }
        }
        else
        {
          // This is the first pass callback, in this instance we allow the
          // dc_preset to trump the individual values as it might have been
          // set from the launch file. To allow override of individual values,
          // set dc_preset to -1 (Unused) in the launch file.
          if (dc_preset != -1)
          {
            ROS_INFO_STREAM(nodelet_name_ << " - Initializing Depth Control Preset to " << dc_preset);
            ROS_DEBUG_STREAM(nodelet_name_ << " - NOTICE: Individual Depth Control values " <<
                "set by params will be ignored; set r200_dc_preset=-1 to override.");
            rs_apply_depth_control_preset(rs_device_, dc_preset);

            // Save the preset value string
            last_dc = setDynamicReconfigDepthControlIndividuals();
          }
        }
      }
      else
      {
        // Changing individual Depth Control params means preset is Unused/Invalid
        // if the individual values are not the same as the preset values
        if (dc_preset != -1 && current_dc != last_dc)
        {
          ROS_DEBUG_STREAM(nodelet_name_ << " - Forcing Depth Control Preset to Unused");
          setDynamicReconfigDepthControlPreset(-1);
        }
      }
    }
    else
    { // Individual Depth Control not set
      if (bit_level.test(6))  // 2^6 = 64 : Depth Control Preset
      {
        dc_preset = config.r200_dc_preset;

        if (dc_preset != -1)
        {
          ROS_DEBUG_STREAM(nodelet_name_ << " - Set Depth Control Preset to " << dc_preset);
          rs_apply_depth_control_preset(rs_device_, dc_preset);

          // Save the preset value string
          last_dc = setDynamicReconfigDepthControlIndividuals();
        }
      }
    }

    rs_set_device_option(rs_device_, RS_OPTION_FISHEYE_EXPOSURE,
        config.fisheye_exposure, 0);
    rs_set_device_option(rs_device_, RS_OPTION_FISHEYE_GAIN, config.fisheye_gain, 0);
    rs_set_device_option(rs_device_, RS_OPTION_FISHEYE_ENABLE_AUTO_EXPOSURE, config.fisheye_enable_auto_exposure, 0);
    rs_set_device_option(rs_device_, RS_OPTION_FISHEYE_AUTO_EXPOSURE_MODE, config.fisheye_auto_exposure_mode, 0);
    rs_set_device_option(rs_device_, RS_OPTION_FISHEYE_AUTO_EXPOSURE_ANTIFLICKER_RATE,
        config.fisheye_auto_exposure_antiflicker_rate, 0);
    rs_set_device_option(rs_device_, RS_OPTION_FISHEYE_AUTO_EXPOSURE_PIXEL_SAMPLE_RATE,
        config.fisheye_auto_exposure_pixel_sample_rate, 0);
    rs_set_device_option(rs_device_, RS_OPTION_FISHEYE_AUTO_EXPOSURE_SKIP_FRAMES,
        config.fisheye_auto_exposure_skip_frames, 0);
    rs_set_device_option(rs_device_, RS_OPTION_FRAMES_QUEUE_SIZE, config.frames_queue_size, 0);
    rs_set_device_option(rs_device_, RS_OPTION_HARDWARE_LOGGER_ENABLED, config.hardware_logger_enabled, 0);
  }

  /*
   * Publish IMU.
   */
  void ZR300Nodelet::publishIMU()
  {
    prev_imu_ts_ = -1;
    while (ros::ok())
    {
      if (start_stop_srv_called_ == true)
      {
        if (start_camera_ == true)
        {
          ROS_INFO_STREAM(nodelet_name_ << " - " << startCamera());
        }
        else
        {
          ROS_INFO_STREAM(nodelet_name_ << " - " << stopCamera());
        }
        start_stop_srv_called_ = false;
      }

      if (enable_[RS_STREAM_DEPTH] != rs_is_stream_enabled(rs_device_, RS_STREAM_DEPTH, 0))
      {
        stopCamera();
        setStreams();
        startCamera();
      }

      if (imu_publisher_.getNumSubscribers() > 0)
      {
        std::unique_lock<std::mutex> lock(imu_mutex_);

        if (prev_imu_ts_ != imu_ts_)
        {
          sensor_msgs::Imu imu_msg = sensor_msgs::Imu();
          imu_msg.header.stamp = ros::Time(camera_start_ts_) + ros::Duration(imu_ts_ * 0.001);
          imu_msg.header.frame_id = imu_optical_frame_id_;

          imu_msg.orientation.x = 0.0;
          imu_msg.orientation.y = 0.0;
          imu_msg.orientation.z = 0.0;
          imu_msg.orientation.w = 0.0;
          imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

          imu_msg.angular_velocity.x = imu_angular_vel_[0];
          imu_msg.angular_velocity.y = imu_angular_vel_[1];
          imu_msg.angular_velocity.z = imu_angular_vel_[2];

          imu_msg.linear_acceleration.x = imu_linear_accel_[0];
          imu_msg.linear_acceleration.y = imu_linear_accel_[1];
          imu_msg.linear_acceleration.z = imu_linear_accel_[2];

          for (int i = 0; i < 9; ++i)
          {
            imu_msg.angular_velocity_covariance[i] = imu_angular_vel_cov_[i];
            imu_msg.linear_acceleration_covariance[i] = imu_linear_accel_cov_[i];
          }

          imu_publisher_.publish(imu_msg);
          prev_imu_ts_ = imu_ts_;
        }
      }
    }
    stopIMU();
  }

  /*
   * Set up IMU -- overrides base class
   */
  void ZR300Nodelet::setStreams()
  {
    // enable camera streams
    BaseNodelet::setStreams();

    if (enable_imu_ == true)
    {
      // enable IMU
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling IMU");
      setIMUCallbacks();
      rs_enable_motion_tracking_cpp(rs_device_, new rs::motion_callback(motion_handler_),
          new rs::timestamp_callback(timestamp_handler_), &rs_error_);
      checkError();
      rs_source_ = RS_SOURCE_ALL;  // overrides default to enable motion tracking
    }
  }

  /*
   * Set IMU callbacks.
   */
  void ZR300Nodelet::setIMUCallbacks()
  {
    motion_handler_ = [&](rs::motion_data entry)  // NOLINT(build/c++11)
    {
      std::unique_lock<std::mutex> lock(imu_mutex_);

      if (entry.timestamp_data.source_id == RS_EVENT_IMU_GYRO)
      {
        for (int i = 0; i < 3; ++i)
        {
          imu_angular_vel_[i] = entry.axes[i];
          imu_linear_accel_[i] = 0.0;
        }
        imu_angular_vel_cov_[0] = 0.0;
        imu_linear_accel_cov_[0] = -1.0;
      }
      else if (entry.timestamp_data.source_id == RS_EVENT_IMU_ACCEL)
      {
        for (int i = 0; i < 3; ++i)
        {
          imu_angular_vel_[i] = 0.0;
          imu_linear_accel_[i] = entry.axes[i];
        }
        imu_angular_vel_cov_[0] = -1.0;
        imu_linear_accel_cov_[0] = 0.0;
      }
      imu_ts_ = static_cast<double>(entry.timestamp_data.timestamp);

      ROS_DEBUG_STREAM(" - Motion,\t host time " << imu_ts_
          << "\ttimestamp: " << std::setprecision(8) << (double)entry.timestamp_data.timestamp*IMU_UNITS_TO_MSEC
          << "\tsource: " << (rs::event)entry.timestamp_data.source_id
          << "\tframe_num: " << entry.timestamp_data.frame_number
          << "\tx: " << std::setprecision(5) <<  entry.axes[0]
          << "\ty: " << entry.axes[1]
          << "\tz: " << entry.axes[2]);
    };

    // Get timestamp that syncs all sensors.
    timestamp_handler_ = [](rs::timestamp_data entry)
    {
        auto now = std::chrono::system_clock::now().time_since_epoch();
        auto sys_time = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();

        ROS_DEBUG_STREAM(" - TimeEvent, host time " << sys_time
            << "\ttimestamp: " << std::setprecision(8) << (double)entry.timestamp*IMU_UNITS_TO_MSEC
            << "\tsource: " << (rs::event)entry.source_id
            << "\tframe_num: " << entry.frame_number);
    };
  }

  /*
  * Set up the callbacks for the camera streams
  */
  void ZR300Nodelet::setFrameCallbacks()
  {
    // call base nodelet method
    BaseNodelet::setFrameCallbacks();

    fisheye_frame_handler_ = [&](rs::frame frame)  // NOLINT(build/c++11)
    {
      publishTopic(RS_STREAM_FISHEYE, frame);
    };

    ir2_frame_handler_ = [&](rs::frame frame)  // NOLINT(build/c++11)
    {
      publishTopic(RS_STREAM_INFRARED2, frame);
    };

    rs_set_frame_callback_cpp(rs_device_, RS_STREAM_FISHEYE,
        new rs::frame_callback(fisheye_frame_handler_), &rs_error_);
    checkError();

    rs_set_frame_callback_cpp(rs_device_, RS_STREAM_INFRARED2, new rs::frame_callback(ir2_frame_handler_), &rs_error_);
    checkError();
  }

  /*
   * Get the camera extrinsics
   */
  void ZR300Nodelet::getCameraExtrinsics()
  {
    BaseNodelet::getCameraExtrinsics();

    // Get offset between base frame and infrared2 frame
    rs_get_device_extrinsics(rs_device_, RS_STREAM_INFRARED2, RS_STREAM_COLOR, &color2ir2_extrinsic_, &rs_error_);
    if (rs_error_)
    {
      ROS_ERROR_STREAM(nodelet_name_ << " - Verify camera is calibrated!");
    }
    checkError();

    // Get offset between base frame and fisheye frame
    rs_get_device_extrinsics(rs_device_, RS_STREAM_FISHEYE, RS_STREAM_COLOR, &color2fisheye_extrinsic_, &rs_error_);
    if (rs_error_)
    {
      ROS_ERROR_STREAM(nodelet_name_ << " - Verify camera is calibrated!");
    }
    checkError();

    // Get offset between base frame and imu frame
    rs_get_motion_extrinsics_from(rs_device_, RS_STREAM_COLOR, &color2imu_extrinsic_, &rs_error_);
    if (rs_error_)
    {
/*  Temporarily hardcoding the values until fully supported by librealsense API.  */
      // ROS_ERROR_STREAM(nodelet_name_ << " - Verify camera is calibrated!");
      ROS_WARN_STREAM(nodelet_name_ << " - Using Hardcoded extrinsic for IMU.");
      rs_free_error(rs_error_);
      rs_error_ = NULL;

      color2imu_extrinsic_.translation[0] = -0.07;
      color2imu_extrinsic_.translation[1] = 0.0;
      color2imu_extrinsic_.translation[2] = 0.0;
    }
    // checkError();
  }

  /*
   * Publish Static transforms.
   */
  void ZR300Nodelet::publishStaticTransforms()
  {
    BaseNodelet::publishStaticTransforms();

    tf::Quaternion q_i2io;
    tf::Quaternion q_f2fo;
    tf::Quaternion q_imu2imuo;
    geometry_msgs::TransformStamped b2i_msg;
    geometry_msgs::TransformStamped i2io_msg;
    geometry_msgs::TransformStamped b2f_msg;
    geometry_msgs::TransformStamped f2fo_msg;
    geometry_msgs::TransformStamped b2imu_msg;
    geometry_msgs::TransformStamped imu2imuo_msg;

    // Transform base frame to infrared2 frame
    b2i_msg.header.stamp = transform_ts_;
    b2i_msg.header.frame_id = base_frame_id_;
    b2i_msg.child_frame_id = frame_id_[RS_STREAM_INFRARED2];
    b2i_msg.transform.translation.x =  color2ir2_extrinsic_.translation[2];
    b2i_msg.transform.translation.y = -color2ir2_extrinsic_.translation[0];
    b2i_msg.transform.translation.z = -color2ir2_extrinsic_.translation[1];
    b2i_msg.transform.rotation.x = 0;
    b2i_msg.transform.rotation.y = 0;
    b2i_msg.transform.rotation.z = 0;
    b2i_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2i_msg);

    // Transform infrared2 frame to infrared2 optical frame
    q_i2io.setRPY(-M_PI/2, 0.0, -M_PI/2);
    i2io_msg.header.stamp = transform_ts_;
    i2io_msg.header.frame_id = frame_id_[RS_STREAM_INFRARED2];
    i2io_msg.child_frame_id = optical_frame_id_[RS_STREAM_INFRARED2];
    i2io_msg.transform.translation.x = 0;
    i2io_msg.transform.translation.y = 0;
    i2io_msg.transform.translation.z = 0;
    i2io_msg.transform.rotation.x = q_i2io.getX();
    i2io_msg.transform.rotation.y = q_i2io.getY();
    i2io_msg.transform.rotation.z = q_i2io.getZ();
    i2io_msg.transform.rotation.w = q_i2io.getW();
    static_tf_broadcaster_.sendTransform(i2io_msg);

    // Transform base frame to fisheye frame
    b2f_msg.header.stamp = transform_ts_;
    b2f_msg.header.frame_id = base_frame_id_;
    b2f_msg.child_frame_id = frame_id_[RS_STREAM_FISHEYE];
    b2f_msg.transform.translation.x =  color2fisheye_extrinsic_.translation[2];
    b2f_msg.transform.translation.y = -color2fisheye_extrinsic_.translation[0];
    b2f_msg.transform.translation.z = -color2fisheye_extrinsic_.translation[1];
    b2f_msg.transform.rotation.x = 0;
    b2f_msg.transform.rotation.y = 0;
    b2f_msg.transform.rotation.z = 0;
    b2f_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2f_msg);

    // Transform fisheye frame to fisheye optical frame
    q_f2fo.setRPY(-M_PI/2, 0.0, -M_PI/2);
    f2fo_msg.header.stamp = transform_ts_;
    f2fo_msg.header.frame_id = frame_id_[RS_STREAM_FISHEYE];
    f2fo_msg.child_frame_id = optical_frame_id_[RS_STREAM_FISHEYE];
    f2fo_msg.transform.translation.x = 0;
    f2fo_msg.transform.translation.y = 0;
    f2fo_msg.transform.translation.z = 0;
    f2fo_msg.transform.rotation.x = q_f2fo.getX();
    f2fo_msg.transform.rotation.y = q_f2fo.getY();
    f2fo_msg.transform.rotation.z = q_f2fo.getZ();
    f2fo_msg.transform.rotation.w = q_f2fo.getW();
    static_tf_broadcaster_.sendTransform(f2fo_msg);

    // Transform base frame to imu frame
    b2imu_msg.header.stamp = transform_ts_;
    b2imu_msg.header.frame_id = base_frame_id_;
    b2imu_msg.child_frame_id = imu_frame_id_;
    b2imu_msg.transform.translation.x =  color2imu_extrinsic_.translation[2];
    b2imu_msg.transform.translation.y = -color2imu_extrinsic_.translation[0];
    b2imu_msg.transform.translation.z = -color2imu_extrinsic_.translation[1];
    b2imu_msg.transform.rotation.x = 0;
    b2imu_msg.transform.rotation.y = 0;
    b2imu_msg.transform.rotation.z = 0;
    b2imu_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2imu_msg);

    // Transform imu frame to imu optical frame
    q_imu2imuo.setRPY(-M_PI/2, 0.0, -M_PI/2);
    imu2imuo_msg.header.stamp = transform_ts_;
    imu2imuo_msg.header.frame_id = imu_frame_id_;
    imu2imuo_msg.child_frame_id = imu_optical_frame_id_;
    imu2imuo_msg.transform.translation.x = 0;
    imu2imuo_msg.transform.translation.y = 0;
    imu2imuo_msg.transform.translation.z = 0;
    imu2imuo_msg.transform.rotation.x = q_imu2imuo.getX();
    imu2imuo_msg.transform.rotation.y = q_imu2imuo.getY();
    imu2imuo_msg.transform.rotation.z = q_imu2imuo.getZ();
    imu2imuo_msg.transform.rotation.w = q_imu2imuo.getW();
    static_tf_broadcaster_.sendTransform(imu2imuo_msg);
  }

  /*
   * Publish Dynamic transforms.
   */
  void ZR300Nodelet::publishDynamicTransforms()
  {
    tf::Transform tr;
    tf::Quaternion q;

    BaseNodelet::publishDynamicTransforms();

    // Transform base frame to infrared2 frame
    tr.setOrigin(tf::Vector3(
           color2ir2_extrinsic_.translation[2],
          -color2ir2_extrinsic_.translation[0],
          -color2ir2_extrinsic_.translation[1]));
    tr.setRotation(tf::Quaternion(0, 0, 0, 1));
    dynamic_tf_broadcaster_.sendTransform(tf::StampedTransform(tr, transform_ts_,
          base_frame_id_, frame_id_[RS_STREAM_INFRARED2]));

    // Transform infrared2 frame to infrared2 optical frame
    tr.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(-M_PI/2, 0.0, -M_PI/2);
    tr.setRotation(q);
    dynamic_tf_broadcaster_.sendTransform(tf::StampedTransform(tr, transform_ts_,
          frame_id_[RS_STREAM_INFRARED2], optical_frame_id_[RS_STREAM_INFRARED2]));

    // Transform base frame to fisheye frame
    tr.setOrigin(tf::Vector3(
           color2fisheye_extrinsic_.translation[2],
          -color2fisheye_extrinsic_.translation[0],
          -color2fisheye_extrinsic_.translation[1]));
    tr.setRotation(tf::Quaternion(0, 0, 0, 1));
    dynamic_tf_broadcaster_.sendTransform(tf::StampedTransform(tr, transform_ts_,
          base_frame_id_, frame_id_[RS_STREAM_FISHEYE]));

    // Transform fisheye frame to fisheye optical frame
    tr.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(-M_PI/2, 0.0, -M_PI/2);
    tr.setRotation(q);
    dynamic_tf_broadcaster_.sendTransform(tf::StampedTransform(tr, transform_ts_,
          frame_id_[RS_STREAM_FISHEYE], optical_frame_id_[RS_STREAM_FISHEYE]));

    // Transform base frame to imu frame
    tr.setOrigin(tf::Vector3(
           color2imu_extrinsic_.translation[2],
          -color2imu_extrinsic_.translation[0],
          -color2imu_extrinsic_.translation[1]));
    tr.setRotation(tf::Quaternion(0, 0, 0, 1));
    dynamic_tf_broadcaster_.sendTransform(tf::StampedTransform(tr, transform_ts_,
          base_frame_id_, imu_frame_id_));

    // Transform imu frame to imu optical frame
    tr.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(-M_PI/2, 0.0, -M_PI/2);
    tr.setRotation(q);
    dynamic_tf_broadcaster_.sendTransform(tf::StampedTransform(tr, transform_ts_,
          imu_frame_id_, imu_optical_frame_id_));
  }

  /*
   * Stop the IMU and motion tracking
   */
  void ZR300Nodelet::stopIMU()
  {
    rs_stop_source(rs_device_, (rs_source)rs::source::motion_data, &rs_error_);
    checkError();
    rs_disable_motion_tracking(rs_device_, &rs_error_);
    checkError();
  }
}  // namespace realsense_camera
