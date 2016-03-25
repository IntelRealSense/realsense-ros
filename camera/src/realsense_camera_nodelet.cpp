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

#include "realsense_camera_nodelet.h"

using namespace cv;
using namespace std;

// Nodelet dependencies.
#include <pluginlib/class_list_macros.h>
#include <tf/transform_broadcaster.h>

PLUGINLIB_EXPORT_CLASS (realsense_camera::RealsenseNodelet, nodelet::Nodelet)
namespace realsense_camera
{

  /*
   * Public Methods.
   */
  RealsenseNodelet::~RealsenseNodelet()
  {
    device_thread_->join();

    if (enable_tf_ == true)
    {
      transform_thread_->join();
    }

    // Stop device.
    if (is_device_started_ == true)
    {
      rs_stop_device(rs_device_, 0);
      rs_delete_context(rs_context_, &rs_error_);
      checkError();
    }

    ROS_INFO_STREAM ("RealSense Camera - Stopping camera nodelet.");
    ros::shutdown();
  }

  /*
   * Initialize the realsense code.
   */
  void RealsenseNodelet::onInit()
  {
    ROS_INFO_STREAM ("RealSense Camera - Starting camera nodelet.");

    // Set default configurations.
    is_device_started_ = false;

    frame_id_[RS_STREAM_INFRARED] = IR1_DEF_FRAME;
    frame_id_[RS_STREAM_INFRARED2] = IR2_DEF_FRAME;

    for (int i = 0; i < STREAM_COUNT; ++i)
    {
      camera_info_[i] = NULL;
    }

    ros::NodeHandle & nh = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    // Set up the topics.
    image_transport::ImageTransport it (nh);

    setStreamOptions();

    // Advertise the various topics and services.
    camera_publisher_[RS_STREAM_COLOR] = it.advertiseCamera(COLOR_TOPIC, 1);
    camera_publisher_[RS_STREAM_DEPTH] = it.advertiseCamera(DEPTH_TOPIC, 1);
    camera_publisher_[RS_STREAM_INFRARED] = it.advertiseCamera(IR1_TOPIC, 1);
    if (camera_.find (R200) != std::string::npos)
    {
      camera_publisher_[RS_STREAM_INFRARED2] = it.advertiseCamera(IR2_TOPIC, 1);
    }

    pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(PC_TOPIC, 1);


    get_options_service_ = nh.advertiseService(SETTINGS_SERVICE, &RealsenseNodelet::getCameraOptionValues, this);

    dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<realsense_camera::camera_paramsConfig>(getPrivateNodeHandle()));

    bool connected = false;

    connected = connectToCamera();

    if (connected == true)
    {
      // Start working thread.
      device_thread_ =
          boost::shared_ptr <boost::thread>(new boost::thread (boost::bind(&RealsenseNodelet::devicePoll, this)));

      if (enable_tf_ == true)
      {
        transform_thread_ =
        boost::shared_ptr<boost::thread>(new boost::thread (boost::bind(&RealsenseNodelet::publishTransforms, this)));
      }

    }
    else
    {
      ros::shutdown();
    }

    dynamic_reconf_server_->setCallback(boost::bind(&RealsenseNodelet::configCallback, this, _1, _2));
}

  /*
   *Private Methods.
   */
  void RealsenseNodelet::enableColorStream()
  {
    // Enable streams.
    if (mode_.compare ("manual") == 0)
    {
      ROS_INFO_STREAM ("RealSense Camera - Enabling Color Stream: manual mode");
      rs_enable_stream(rs_device_, RS_STREAM_COLOR, color_width_, color_height_, COLOR_FORMAT, color_fps_, &rs_error_);
      checkError();
    }
    else
    {
      ROS_INFO_STREAM ("RealSense Camera - Enabling Color Stream: preset mode");
      rs_enable_stream_preset(rs_device_, RS_STREAM_COLOR, RS_PRESET_BEST_QUALITY, &rs_error_);
      checkError();
    }

    uint32_t stream_index = (uint32_t) RS_STREAM_COLOR;
    if (camera_info_[stream_index] == NULL)
    {
      prepareStreamCalibData (RS_STREAM_COLOR);
    }
  }

  void RealsenseNodelet::enableDepthStream()
  {
    // Enable streams.
    if (mode_.compare ("manual") == 0)
    {
      ROS_INFO_STREAM ("RealSense Camera - Enabling Depth Stream: manual mode");
      rs_enable_stream(rs_device_, RS_STREAM_DEPTH, depth_width_, depth_height_, DEPTH_FORMAT, depth_fps_, &rs_error_);
      checkError();
    }
    else
    {
      ROS_INFO_STREAM ("RealSense Camera - Enabling Depth Stream: preset mode");
      rs_enable_stream_preset(rs_device_, RS_STREAM_DEPTH, RS_PRESET_BEST_QUALITY, &rs_error_);
      checkError();
    }

    uint32_t stream_index = (uint32_t) RS_STREAM_DEPTH;
    if (camera_info_[stream_index] == NULL)
    {
      prepareStreamCalibData (RS_STREAM_DEPTH);
    }
  }

  void RealsenseNodelet::enableInfraredStream()
  {
    // Enable streams.
    if (mode_.compare ("manual") == 0)
    {
      ROS_INFO_STREAM ("RealSense Camera - Enabling Infrared Stream: manual mode");
      rs_enable_stream(rs_device_, RS_STREAM_INFRARED, depth_width_, depth_height_, IR1_FORMAT, depth_fps_, &rs_error_);
      checkError();
    }
    else
    {
      ROS_INFO_STREAM ("RealSense Camera - Enabling Infrared Stream: preset mode");
      rs_enable_stream_preset(rs_device_, RS_STREAM_INFRARED, RS_PRESET_BEST_QUALITY, &rs_error_);
      checkError();
    }

    uint32_t stream_index = (uint32_t) RS_STREAM_INFRARED;
    if (camera_info_[stream_index] == NULL)
    {
      prepareStreamCalibData (RS_STREAM_INFRARED);
    }
  }

  void RealsenseNodelet::enableInfrared2Stream()
  {
    // Enable streams.
    if (mode_.compare ("manual") == 0)
    {
      ROS_INFO_STREAM ("RealSense Camera - Enabling Infrared2 Stream: manual mode");
      rs_enable_stream(rs_device_, RS_STREAM_INFRARED2, depth_width_, depth_height_, IR2_FORMAT, depth_fps_, 0);
    }
    else
    {
      ROS_INFO_STREAM ("RealSense Camera - Enabling Infrared2 Stream: preset mode");
      rs_enable_stream_preset(rs_device_, RS_STREAM_INFRARED2, RS_PRESET_BEST_QUALITY, 0);
    }

    uint32_t stream_index = (uint32_t) RS_STREAM_INFRARED2;
    if (camera_info_[stream_index] == NULL)
    {
      prepareStreamCalibData (RS_STREAM_INFRARED2);
    }
  }



  void RealsenseNodelet::configCallback(realsense_camera::camera_paramsConfig &config, uint32_t level)
  {
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_BACKLIGHT_COMPENSATION, config.color_backlight_compensation, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_BRIGHTNESS, config.color_brightness, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_CONTRAST, config.color_contrast, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_GAIN, config.color_gain, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_GAMMA, config.color_gamma, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_HUE, config.color_hue, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_SATURATION, config.color_saturation, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_SHARPNESS, config.color_sharpness, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_ENABLE_AUTO_WHITE_BALANCE, config.color_enable_auto_white_balance, 0);

    if (config.color_enable_auto_white_balance == 0)
    {
      rs_set_device_option(rs_device_, RS_OPTION_COLOR_WHITE_BALANCE, config.color_white_balance, 0);
    }

    //R200 camera specific options
    rs_set_device_option(rs_device_, RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED, config.r200_lr_auto_exposure_enabled, 0);

    if (config.r200_lr_auto_exposure_enabled == 0)
    {
      rs_set_device_option(rs_device_, RS_OPTION_R200_LR_EXPOSURE, config.r200_lr_exposure, 0);
    }

    rs_set_device_option(rs_device_, RS_OPTION_R200_LR_GAIN, config.r200_lr_gain, 0);
    rs_set_device_option(rs_device_, RS_OPTION_R200_EMITTER_ENABLED, config.r200_emitter_enabled, 0);

    if (config.r200_lr_auto_exposure_enabled == 1)
    {
      if (config.r200_auto_exposure_top_edge >= depth_height_)
      {
        config.r200_auto_exposure_top_edge = depth_height_ - 1;
      }
      if (config.r200_auto_exposure_bottom_edge >= depth_height_)
      {
        config.r200_auto_exposure_bottom_edge = depth_height_ - 1;
      }
      if (config.r200_auto_exposure_left_edge >= depth_width_)
      {
        config.r200_auto_exposure_left_edge = depth_width_ - 1;
      }
      if (config.r200_auto_exposure_right_edge >= depth_width_)
      {
        config.r200_auto_exposure_right_edge = depth_width_ - 1;
      }
      edge_values_[0] = config.r200_auto_exposure_left_edge;
      edge_values_[1] = config.r200_auto_exposure_top_edge;
      edge_values_[2] = config.r200_auto_exposure_right_edge;
      edge_values_[3] = config.r200_auto_exposure_bottom_edge;

      rs_set_device_options(rs_device_, edge_options_, 4, edge_values_, 0);
    }

    if (config.enable_depth == false)
    {
      if (enable_color_ == false)
      {
        ROS_INFO_STREAM ("RealSense Camera - Color stream is also disabled. Cannot disable depth stream");
        config.enable_depth = true;
      }
      else
      {
        enable_depth_ = false;
      }
    }
    else
    {
      enable_depth_ = true;
    }
  }

  void RealsenseNodelet::checkError()
  {
    if (rs_error_)
    {
      ROS_ERROR_STREAM ("RealSense Camera - Error calling " << rs_get_failed_function(rs_error_) << " ( "
          << rs_get_failed_args(rs_error_) << " ): \n" << rs_get_error_message(rs_error_) << " \n");
      rs_free_error(rs_error_);

      ros::shutdown();

      exit (EXIT_FAILURE);
    }
  }

  /*
   * Query the data for every frame and publish it to the different topics.
   */
  void RealsenseNodelet::devicePoll()
  {
    while (ros::ok())
    {
      publishStreams();
    }
  }

  bool RealsenseNodelet::connectToCamera()
  {
    if (enable_depth_ == false && enable_color_ == false)
    {
      ROS_ERROR_STREAM("RealSense Camera - None of the streams are enabled. Exiting.");
      return false;
    }

    rs_context_ = rs_create_context(RS_API_VERSION, &rs_error_);
    checkError();

    num_of_cameras_ = rs_get_device_count(rs_context_, &rs_error_);
    checkError();

    // Exit with error if no cameras are connected.
    if (num_of_cameras_ < 1)
    {
      ROS_ERROR_STREAM("RealSense Camera - No cameras detected. Exiting!");
      return false;
    }

    rs_device_ = getCameraBySerialNumber(); // Get rs_device_ using input serial number.
    std::string detected_camera_msg = "RealSense Camera - Detected following cameras:";
    for (rs_device *rs_detected_device: rs_detected_devices_)
    {
      detected_camera_msg = detected_camera_msg +
          "\n\t\t\t\t- Serial No: " + rs_get_device_serial(rs_detected_device, &rs_error_) +
          "; Firmware: " + rs_get_device_firmware_version(rs_detected_device, &rs_error_) +
          "; Name: " + rs_get_device_name(rs_detected_device, &rs_error_);
      checkError();
    }
    ROS_INFO_STREAM(detected_camera_msg);

    // Exit with error if no serial number is specified and multiple cameras are detected.
    if ((serial_no_.empty() == true) && (num_of_cameras_ > 1))
    {
      ROS_ERROR_STREAM("RealSense Camera - Multiple cameras detected but no input serial_no specified. Exiting!");
      return false;
    }

    // Exit with error if no camera is detected that matches the input serial number.
    if ((serial_no_.empty() != true) && (rs_device_ == NULL))
    {
      ROS_ERROR_STREAM("RealSense Camera - No camera detected with input serial_no = " << serial_no_ << ". Exiting!");
      return false;
    }

    // At this point, rs_device_ will be null if no input serial number was specified and only one camera is connected.
    // This is a valid use case and the code will proceed.
    if (rs_device_ == NULL)
    {
      rs_device_ = rs_get_device(rs_context_, 0, &rs_error_);
      checkError();
    }

    ROS_INFO_STREAM("RealSense Camera - Connecting to camera with Serial No: " <<
        rs_get_device_serial(rs_device_, &rs_error_));
    checkError();

    // Enable streams.
    if (enable_color_ == true)
    {
      enableColorStream();
    }

    if (enable_depth_ == true)
    {
      enableDepthStream();
      enableInfraredStream();
      enableInfrared2Stream();
    }

    getCameraOptions();
    setStaticCameraOptions();

    // Start device.
    rs_start_device(rs_device_, &rs_error_);
    checkError();

    is_device_started_ = true;

    allocateResources();

    return true;
  }

  /*
   * Returns the device details of the camera that matches the input serial_no.
   * Returns NULL if no camera is found that matches the input serial_no.
   * Also populates the list of detected cameras.
   */
  rs_device * RealsenseNodelet::getCameraBySerialNumber()
  {
    rs_device *rs_device_detect;
    rs_device *rs_device_connect = NULL;
    for (int i = 0; i < num_of_cameras_; ++i)
    {
      rs_device_detect = rs_get_device(rs_context_, i, &rs_error_);
      checkError();
      std::string device_serial_no = rs_get_device_serial(rs_device_detect, &rs_error_);
      checkError();
      if (device_serial_no.compare(serial_no_) == 0)
      {
        rs_device_connect = rs_device_detect;
      }
      rs_detected_devices_.push_back(rs_device_detect);
    }
    return rs_device_connect;
  }

  /*
   * Gets the options supported by the camera along with their min, max and step values.
   */
  void RealsenseNodelet::getCameraOptions()
  {
    for (int i = 0; i < RS_OPTION_COUNT; ++i)
    {
      option_str o = { (rs_option) i };

      if (rs_device_supports_option(rs_device_, o.opt, &rs_error_))
      {
        rs_get_device_option_range(rs_device_, o.opt, &o.min, &o.max, &o.step, 0);
        // Skip the camera options where min and max values are the same.
        if (o.min != o.max)
        {
          o.value = rs_get_device_option(rs_device_, o.opt, 0);
          options.push_back(o);
        }
      }
    }
  }

  /*
   * Define buffer for images and prepare camera info for each enabled stream.
   */
  void RealsenseNodelet::allocateResources()
  {
    // Prepare camera for enabled streams (color/depth/infrared/infrared2).
    fillStreamEncoding();

    image_[(uint32_t) RS_STREAM_COLOR] = cv::Mat(color_height_, color_width_, CV_8UC3, cv::Scalar (0, 0, 0));
    image_[(uint32_t) RS_STREAM_DEPTH] = cv::Mat(depth_height_, depth_width_, CV_16UC1, cv::Scalar (0));
    image_[(uint32_t) RS_STREAM_INFRARED] = cv::Mat(depth_height_, depth_width_, CV_8UC1, cv::Scalar (0));
    if (camera_.find(R200) != std::string::npos)
    {
        image_[(uint32_t) RS_STREAM_INFRARED2] = cv::Mat(depth_height_, depth_width_, CV_8UC1, cv::Scalar (0));
    }
  }

  /*
   * Prepare camera_info for each enabled stream.
   */
  void RealsenseNodelet::prepareStreamCalibData(rs_stream rs_strm)
  {
    uint32_t stream_index = (uint32_t) rs_strm;
    rs_intrinsics intrinsic;
    rs_get_stream_intrinsics(rs_device_, rs_strm, &intrinsic, &rs_error_);
    checkError();

    camera_info_[stream_index] = new sensor_msgs::CameraInfo();
    camera_info_ptr_[stream_index] = sensor_msgs::CameraInfoPtr (camera_info_[stream_index]);

    camera_info_[stream_index]->header.frame_id = frame_id_[stream_index];
    camera_info_[stream_index]->width = intrinsic.width;
    camera_info_[stream_index]->height = intrinsic.height;

    camera_info_[stream_index]->K.at(0) = intrinsic.fx;
    camera_info_[stream_index]->K.at(2) = intrinsic.ppx;
    camera_info_[stream_index]->K.at(4) = intrinsic.fy;
    camera_info_[stream_index]->K.at(5) = intrinsic.ppy;
    camera_info_[stream_index]->K.at(8) = 1;

    camera_info_[stream_index]->P.at(0) = camera_info_[stream_index]->K.at(0);
    camera_info_[stream_index]->P.at(1) = 0;
    camera_info_[stream_index]->P.at(2) = camera_info_[stream_index]->K.at(2);
    camera_info_[stream_index]->P.at(3) = 0;

    camera_info_[stream_index]->P.at(4) = 0;
    camera_info_[stream_index]->P.at(5) = camera_info_[stream_index]->K.at(4);
    camera_info_[stream_index]->P.at(6) = camera_info_[stream_index]->K.at(5);
    camera_info_[stream_index]->P.at(7) = 0;

    camera_info_[stream_index]->P.at(8) = 0;
    camera_info_[stream_index]->P.at(9) = 0;
    camera_info_[stream_index]->P.at(10) = 1;
    camera_info_[stream_index]->P.at(11) = 0;

    if (stream_index == RS_STREAM_DEPTH)
    {
      // set depth to color translation values in Projection matrix (P)
      rs_extrinsics z_extrinsic;
      rs_get_device_extrinsics(rs_device_, RS_STREAM_DEPTH, RS_STREAM_COLOR, &z_extrinsic, &rs_error_);
      checkError();
      camera_info_[stream_index]->P.at(3) = z_extrinsic.translation[0]/1000;     // Tx
      camera_info_[stream_index]->P.at(7) = z_extrinsic.translation[1]/1000;     // Ty
      camera_info_[stream_index]->P.at(11) = z_extrinsic.translation[2]/1000;    // Tz
    }

    camera_info_[stream_index]->distortion_model = "plumb_bob";

    // set R (rotation matrix) values to identity matrix
    camera_info_[stream_index]->R.at(0) = (double) 1;
    camera_info_[stream_index]->R.at(1) = (double) 0;
    camera_info_[stream_index]->R.at(2) = (double) 0;
    camera_info_[stream_index]->R.at(3) = (double) 0;
    camera_info_[stream_index]->R.at(4) = (double) 1;
    camera_info_[stream_index]->R.at(5) = (double) 0;
    camera_info_[stream_index]->R.at(6) = (double) 0;
    camera_info_[stream_index]->R.at(7) = (double) 0;
    camera_info_[stream_index]->R.at(8) = (double) 1;

    for (int i = 0; i < 5; i++)
    {
      camera_info_[stream_index]->D.push_back(0);
    }
  }

  /*
   * Get the latest values of the camera options.
   */
  bool RealsenseNodelet::getCameraOptionValues(realsense_camera::cameraConfiguration::Request & req,
      realsense_camera::cameraConfiguration::Response & res)
  {
    std::string get_options_result_str;
    std::string opt_name, opt_value;

    for (option_str o: options)
    {
      opt_name = rs_option_to_string(o.opt);
      std::transform(opt_name.begin(), opt_name.end(), opt_name.begin(), ::tolower);
      o.value = rs_get_device_option(rs_device_, o.opt, 0);
      opt_value = boost::lexical_cast<std::string>(o.value);
      get_options_result_str += opt_name + ":" + opt_value + ";";
    }

    res.configuration_str = get_options_result_str;
    return true;
  }

  /*
   * Set the stream options based on input params.
   */
  void RealsenseNodelet::setStreamOptions()
  {
    pnh_.getParam("serial_no", serial_no_);
    pnh_.param("camera", camera_, (std::string) R200);
    pnh_.param("mode", mode_, DEFAULT_MODE);
    pnh_.param("enable_depth", enable_depth_, ENABLE_DEPTH);
    pnh_.param("enable_color", enable_color_, ENABLE_COLOR);
    pnh_.param("enable_pointcloud", enable_pointcloud_, ENABLE_PC);
    pnh_.param("enable_tf", enable_tf_, ENABLE_TF);
    pnh_.param("depth_width", depth_width_, DEPTH_WIDTH);
    pnh_.param("depth_height", depth_height_, DEPTH_HEIGHT);
    pnh_.param("color_width", color_width_, COLOR_WIDTH);
    pnh_.param("color_height", color_height_, COLOR_HEIGHT);
    pnh_.param("depth_fps", depth_fps_, DEPTH_FPS);
    pnh_.param("color_fps", color_fps_, COLOR_FPS);
    pnh_.param("depth_frame_id", frame_id_[RS_STREAM_DEPTH], (std::string) DEPTH_OPTICAL_DEF_FRAME);
    pnh_.param("rgb_frame_id", frame_id_[RS_STREAM_COLOR], (std::string) COLOR_OPTICAL_DEF_FRAME);
  }

  /*
   * Set the static camera options.
   */
  void RealsenseNodelet::setStaticCameraOptions()
  {
    // Get dynamic options from the dynamic reconfigure server.
    camera_paramsConfig params_config;
    dynamic_reconf_server_->getConfigDefault(params_config);
    std::vector<camera_paramsConfig::AbstractParamDescriptionConstPtr> param_desc = params_config.__getParamDescriptions__();

    // Iterate through the supported camera options
    for (option_str o: options)
    {
      std::string opt_name = rs_option_to_string(o.opt);
      bool found = false;

      std::vector<camera_paramsConfig::AbstractParamDescriptionConstPtr>::iterator it;
      for (camera_paramsConfig::AbstractParamDescriptionConstPtr param_desc_ptr: param_desc)
      {
        std::transform(opt_name.begin(), opt_name.end(), opt_name.begin(), ::tolower);
        if (opt_name.compare((* param_desc_ptr).name) == 0)
        {
          found = true;
          break;
        }
      }
      // Skip the dynamic options and set only the static camera options.
      if (found == false)
      {
        std::string key;
        double val;

        if (pnh_.searchParam(opt_name, key))
        {
          double opt_val;
          pnh_.getParam(key, val);

          // Validate and set the input values within the min-max range
          if (val < o.min)
          {
            opt_val = o.min;
          }
          else if (val > o.max)
          {
            opt_val = o.max;
          }
          else
          {
            opt_val = val;
          }
          ROS_INFO_STREAM ("RealSense Camera - Static Options: " << opt_name << " = " << opt_val);
          rs_set_device_option(rs_device_, o.opt, opt_val, &rs_error_);
          checkError();
        }
      }
    }
  }

  /*
   * Copy frame data from realsense to member cv images.
   */
  void RealsenseNodelet::prepareStreamData(rs_stream rs_strm)
  {
    switch (rs_strm)
    {
      case RS_STREAM_COLOR:
      image_[(uint32_t) RS_STREAM_COLOR].data = (unsigned char *) (rs_get_frame_data(rs_device_, RS_STREAM_COLOR, 0));
      break;
      case RS_STREAM_DEPTH:
      image_depth16_ = reinterpret_cast <const uint16_t * >(rs_get_frame_data(rs_device_, RS_STREAM_DEPTH, 0));
      image_[(uint32_t) RS_STREAM_DEPTH].data = (unsigned char *) image_depth16_;
      break;
      case RS_STREAM_INFRARED:
      image_[(uint32_t) RS_STREAM_INFRARED].data =
      (unsigned char *) (rs_get_frame_data(rs_device_, RS_STREAM_INFRARED, 0));
      break;
      case RS_STREAM_INFRARED2:
      image_[(uint32_t) RS_STREAM_INFRARED2].data =
      (unsigned char *) (rs_get_frame_data(rs_device_, RS_STREAM_INFRARED2, 0));
      break;
      default:
      // no other streams supported
      break;
    }
  }

  /*
   * Populate the encodings for each stream.
   */
  void RealsenseNodelet::fillStreamEncoding()
  {
    stream_encoding_[(uint32_t) RS_STREAM_COLOR] = "rgb8";
    stream_step_[(uint32_t) RS_STREAM_COLOR] = color_width_ * sizeof (unsigned char) * 3;
    stream_encoding_[(uint32_t) RS_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;
    stream_step_[(uint32_t) RS_STREAM_DEPTH] = depth_width_ * sizeof (uint16_t);
    stream_encoding_[(uint32_t) RS_STREAM_INFRARED] = sensor_msgs::image_encodings::TYPE_8UC1;
    stream_step_[(uint32_t) RS_STREAM_INFRARED] = depth_width_ * sizeof (unsigned char);
    stream_encoding_[(uint32_t) RS_STREAM_INFRARED2] = sensor_msgs::image_encodings::TYPE_8UC1;
    stream_step_[(uint32_t) RS_STREAM_INFRARED2] = depth_width_ * sizeof (unsigned char);
  }

  /*
   * Publish streams.
   */
  void RealsenseNodelet::publishStreams()
  {
    if (enable_depth_ == false && rs_is_stream_enabled(rs_device_, RS_STREAM_DEPTH, 0) == 1)
    {
      if (rs_is_device_streaming(rs_device_, 0) == 1)
      {
        rs_stop_device(rs_device_, &rs_error_);
        checkError();
        ROS_INFO_STREAM ("RealSense Camera - Device Stopped");
      }


      //disable depth, infrared1 and infrared2 streams
      rs_disable_stream(rs_device_, RS_STREAM_DEPTH, &rs_error_);
      checkError();
      ROS_INFO_STREAM ("RealSense Camera - Depth Stream Disabled");

      rs_disable_stream(rs_device_, RS_STREAM_INFRARED, &rs_error_);
      checkError();
      ROS_INFO_STREAM ("RealSense Camera - Infrared1 stream Disabled");

      rs_disable_stream(rs_device_, RS_STREAM_INFRARED2, &rs_error_);
      checkError();
      ROS_INFO_STREAM ("RealSense Camera - Infrared2 stream Disabled");

      if (rs_is_device_streaming(rs_device_, 0) == 0)
      {
        rs_start_device(rs_device_, &rs_error_);
        checkError();
        ROS_INFO_STREAM ("RealSense Camera - Device Started");
      }
    }

    if (enable_depth_ == true && rs_is_stream_enabled(rs_device_, RS_STREAM_DEPTH, 0) == 0)
    {
      if (rs_is_device_streaming(rs_device_, 0) == 1)
      {
        rs_stop_device(rs_device_, &rs_error_);
        checkError();
        ROS_INFO_STREAM ("RealSense Camera - Device Stopped");
      }

      enableDepthStream();
      enableInfraredStream();
      enableInfrared2Stream();

      if (rs_is_device_streaming(rs_device_, 0) == 0)
      {
        rs_start_device(rs_device_, &rs_error_);
        checkError();
        ROS_INFO_STREAM ("RealSense Camera - Device Started");
      }
    }

    if (rs_is_device_streaming(rs_device_, 0) == 1)
    {
      rs_wait_for_frames(rs_device_, &rs_error_);
      checkError();
      time_stamp_ = ros::Time::now();

      for (int stream_index = 0; stream_index < STREAM_COUNT; ++stream_index)
      {
        // Publish image stream only if there is at least one subscriber.
        if (camera_publisher_[stream_index].getNumSubscribers() > 0 &&
            rs_is_stream_enabled(rs_device_, (rs_stream) stream_index, 0) == 1)
        {
          prepareStreamData ((rs_stream) stream_index);

          sensor_msgs::ImagePtr msg = cv_bridge::CvImage (std_msgs::Header(),
              stream_encoding_[stream_index],
              image_[stream_index]).toImageMsg();

          msg->header.frame_id = frame_id_[stream_index];
          msg->header.stamp = time_stamp_;	// Publish timestamp to synchronize frames.
          msg->width = image_[stream_index].cols;
          msg->height = image_[stream_index].rows;
          msg->is_bigendian = false;
          msg->step = stream_step_[stream_index];

          camera_info_ptr_[stream_index]->header.stamp = msg->header.stamp;
          camera_publisher_[stream_index].publish (msg, camera_info_ptr_[stream_index]);
        }
      }

      if (pointcloud_publisher_.getNumSubscribers() > 0 &&
          rs_is_stream_enabled(rs_device_, RS_STREAM_DEPTH, 0) == 1 && enable_pointcloud_ == true)
      {
        if (camera_publisher_[(uint32_t) RS_STREAM_DEPTH].getNumSubscribers() <= 0)
        {
          prepareStreamData (RS_STREAM_DEPTH);
        }
        if (camera_publisher_[(uint32_t) RS_STREAM_COLOR].getNumSubscribers() <= 0)
        {
          prepareStreamData (RS_STREAM_COLOR);
        }
        publishPointCloud (image_[(uint32_t) RS_STREAM_COLOR]);
      }
    }
  }

  /*
   * Publish pointcloud.
   */
  void RealsenseNodelet::publishPointCloud (cv::Mat & image_color)
  {
    // Publish pointcloud only if there is at least one subscriber.
    if (pointcloud_publisher_.getNumSubscribers() > 0)
    {
      rs_intrinsics color_intrinsic;
      rs_extrinsics z_extrinsic;

      rs_intrinsics z_intrinsic;
      rs_get_stream_intrinsics(rs_device_, RS_STREAM_DEPTH, &z_intrinsic, &rs_error_);
      checkError();

      if (enable_color_ == true)
      {
        rs_get_stream_intrinsics(rs_device_, RS_STREAM_COLOR, &color_intrinsic, &rs_error_);
        checkError();
        rs_get_device_extrinsics(rs_device_, RS_STREAM_DEPTH, RS_STREAM_COLOR, &z_extrinsic, &rs_error_);
        checkError();
      }

      // Convert pointcloud from the camera to pointcloud object for ROS.
      sensor_msgs::PointCloud2 msg_pointcloud;
      msg_pointcloud.width = depth_width_;
      msg_pointcloud.height = depth_height_;
      msg_pointcloud.header.stamp = time_stamp_;
      msg_pointcloud.is_dense = true;

      sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);

      modifier.setPointCloud2Fields(4, "x", 1,
          sensor_msgs::PointField::FLOAT32, "y", 1,
          sensor_msgs::PointField::FLOAT32, "z", 1,
          sensor_msgs::PointField::FLOAT32, "rgb", 1,
          sensor_msgs::PointField::FLOAT32);

      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

      sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
      sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
      sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");

      sensor_msgs::PointCloud2Iterator<uint8_t>iter_r(msg_pointcloud, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t>iter_g(msg_pointcloud, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t>iter_b(msg_pointcloud, "b");

      float depth_point[3], color_point[3], color_pixel[2], scaled_depth;
      unsigned char *color_data = image_color.data;
      const float depth_scale = rs_get_device_depth_scale(rs_device_, &rs_error_);
      checkError();// Default value is 0.001

      // Fill the PointCloud2 fields.
      for (int y = 0; y < z_intrinsic.height; y++)
      {
        for (int x = 0; x < z_intrinsic.width; x++)
        {
          scaled_depth = ((float) *image_depth16_) * depth_scale;
          float depth_pixel[2] =
          { (float) x, (float) y};
          rs_deproject_pixel_to_point(depth_point, &z_intrinsic, depth_pixel, scaled_depth);

          if (depth_point[2] <= 0 || depth_point[2] > MAX_Z)
          {
            depth_point[0] = 0;
            depth_point[1] = 0;
            depth_point[2] = 0;
          }

          *iter_x = depth_point[0];
          *iter_y = depth_point[1];
          *iter_z = depth_point[2];

          // Default to white color.
          *iter_r = (uint8_t) 255;
          *iter_g = (uint8_t) 255;
          *iter_b = (uint8_t) 255;

          if (enable_color_ == true)
          {
            rs_transform_point_to_point(color_point, &z_extrinsic, depth_point);
            rs_project_point_to_pixel(color_pixel, &color_intrinsic, color_point);

            if (color_pixel[1] < 0 || color_pixel[1] > image_color.rows
                || color_pixel[0] < 0 || color_pixel[0] > image_color.cols)
            {
              // For out of bounds color data, default to a shade of blue in order to visually distinguish holes.
              // This color value is same as the librealsense out of bounds color value.
              *iter_r = 96;
              *iter_g = 157;
              *iter_b = 198;
            }
            else
            {
              int i = (int) color_pixel[0];
              int j = (int) color_pixel[1];

              *iter_r = (uint8_t) color_data[i * 3 + j * image_color.cols * 3];
              *iter_g = (uint8_t) color_data[i * 3 + j * image_color.cols * 3 + 1];
              *iter_b = (uint8_t) color_data[i * 3 + j * image_color.cols * 3 + 2];
            }
          }

          image_depth16_++;
          ++iter_x; ++iter_y; ++iter_z; ++iter_r; ++iter_g; ++iter_b;
        }
      }

      msg_pointcloud.header.frame_id = frame_id_[RS_STREAM_DEPTH];
      pointcloud_publisher_.publish(msg_pointcloud);
    }
  }
  /*
   * Publish camera transforms
   */
  void RealsenseNodelet::publishTransforms()
  {
    // publish transforms for the cameras
    ROS_INFO_STREAM("RealSense Camera - Publishing camera transforms");
    tf::Transform tr;
    tf::Quaternion q;
    tf::TransformBroadcaster tf_broadcaster;
    rs_extrinsics z_extrinsic;

    // extrinsics are offsets between the cameras
    rs_get_device_extrinsics(rs_device_, RS_STREAM_DEPTH, RS_STREAM_COLOR, &z_extrinsic, &rs_error_);
    checkError();

    ros::Duration sleeper(0.1); // 100ms

    while (ros::ok())
    {
      // time stamp is future dated to be valid for given duration
      ros::Time time_stamp = ros::Time::now() + sleeper;

      // transform base frame to depth frame
      tr.setOrigin(tf::Vector3(z_extrinsic.translation[0], z_extrinsic.translation[1], z_extrinsic.translation[2]));
      tr.setRotation(tf::Quaternion(0, 0, 0, 1));
      tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, BASE_DEF_FRAME, DEPTH_DEF_FRAME));

      // transform depth frame to depth optical frame
      tr.setOrigin(tf::Vector3(0,0,0));
      q.setEuler( M_PI/2, 0.0, -M_PI/2 );
      tr.setRotation( q );
      tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, DEPTH_DEF_FRAME, frame_id_[RS_STREAM_DEPTH]));

      // transform base frame to color frame (these are the same)
      tr.setOrigin(tf::Vector3(0,0,0));
      tr.setRotation(tf::Quaternion(0, 0, 0, 1));
      tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, BASE_DEF_FRAME, COLOR_DEF_FRAME));

      // transform color frame to color optical frame
      tr.setOrigin(tf::Vector3(0,0,0));
      q.setEuler( M_PI/2, 0.0, -M_PI/2 );
      tr.setRotation( q );
      tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, COLOR_DEF_FRAME, frame_id_[RS_STREAM_COLOR]));

      sleeper.sleep(); // need sleep or transform won't publish correctly
    }
  }
}  // end namespace
