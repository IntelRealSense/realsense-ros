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
  RealsenseNodelet::~RealsenseNodelet ()
  {
    device_thread_->join ();

    if (enable_tf_ == true)
    {
      transform_thread_->join();
    }

    // Stop device.
    if (is_device_started_ == true)
    {
      rs_stop_device (rs_device_, 0);
      rs_delete_context (rs_context_, &rs_error_);
      check_error ();
    }

    ROS_INFO_STREAM ("RealSense Camera - Stopping camera nodelet.");
    ros::shutdown ();
  }

  /*
   * Initialize the realsense code.
   */
  void RealsenseNodelet::onInit ()
  {
    ROS_INFO_STREAM ("RealSense Camera - Starting camera nodelet.");

    ros::NodeHandle pnh_ = getPrivateNodeHandle();
    pnh_.getParam("device_id", serial_number_);

    // Set default configurations.
    depth_height_ = DEPTH_HEIGHT;
    depth_width_ = DEPTH_WIDTH;
    color_height_ = COLOR_HEIGHT;
    color_width_ = COLOR_WIDTH;
    depth_fps_ = DEPTH_FPS;
    color_fps_ = COLOR_FPS;
    enable_depth_ = ENABLE_DEPTH;
    enable_color_ = ENABLE_COLOR;
    enable_pointcloud_ = ENABLE_PC;
    enable_tf_ = ENABLE_TF;
    is_device_started_ = false;

    camera_configuration_ = getMyArgv ();

    frame_id_[RS_STREAM_DEPTH] = DEPTH_OPTICAL_DEF_FRAME;
    frame_id_[RS_STREAM_COLOR] = COLOR_OPTICAL_DEF_FRAME;
    frame_id_[RS_STREAM_INFRARED] = IR1_DEF_FRAME;
    frame_id_[RS_STREAM_INFRARED2] = IR2_DEF_FRAME;

    ros::NodeHandle & nh = getNodeHandle ();

    // Set up the topics.
    image_transport::ImageTransport it (nh);

    // Parse parameters.
    getConfigValues (camera_configuration_);

    // Advertise the various topics and services.
    // Advertise depth and infrared streams only if depth parameter is set.
    if (enable_depth_ == true)
    {
      camera_publisher_[RS_STREAM_DEPTH] = it.advertiseCamera (DEPTH_TOPIC, 1);
      camera_publisher_[RS_STREAM_INFRARED] = it.advertiseCamera (IR1_TOPIC, 1);
      if (camera_.find (R200) != std::string::npos)
      {
        camera_publisher_[RS_STREAM_INFRARED2] = it.advertiseCamera (IR2_TOPIC, 1);
      }

      // Advertise pointcloud only if pointcloud parameter is set.
      if (enable_pointcloud_ == true)
      {
        pointcloud_publisher_ = nh.advertise < sensor_msgs::PointCloud2 > (PC_TOPIC, 1);
      }
    }

    // Advertise color stream only if color parameter is set.
    if (enable_color_ == true)
    {
      camera_publisher_[RS_STREAM_COLOR] = it.advertiseCamera (COLOR_TOPIC, 1);
    }

    get_options_service_ = nh.advertiseService (SETTINGS_SERVICE, &RealsenseNodelet::getCameraSettings, this);

    dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<realsense_camera::camera_paramsConfig>(getPrivateNodeHandle()));

    
    bool connected = false;

    connected = connectToCamera ();

    if (connected == true)
    {
      // Start working thread.
      device_thread_ =
          boost::shared_ptr < boost::thread > (new boost::thread (boost::bind (&RealsenseNodelet::devicePoll, this)));

      if (enable_tf_ == true)
      {
        transform_thread_ =
        boost::shared_ptr < boost::thread > (new boost::thread (boost::bind (&RealsenseNodelet::publishTransforms, this)));
      }

    }
    else
    {
      ros::shutdown ();
    }

    dynamic_reconf_server_->setCallback(boost::bind(&RealsenseNodelet::configCallback, this, _1, _2));
}

  /*
   *Private Methods.
   */
  void RealsenseNodelet::configCallback(realsense_camera::camera_paramsConfig &config, uint32_t level)
  {
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_BACKLIGHT_COMPENSATION, config.COLOR_BACKLIGHT_COMPENSATION, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_BRIGHTNESS, config.COLOR_BRIGHTNESS, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_CONTRAST, config.COLOR_CONTRAST, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_GAIN, config.COLOR_GAIN, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_GAMMA, config.COLOR_GAMMA, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_HUE, config.COLOR_HUE, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_SATURATION, config.COLOR_SATURATION, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_SHARPNESS, config.COLOR_SHARPNESS, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_ENABLE_AUTO_WHITE_BALANCE, config.COLOR_ENABLE_AUTO_WHITE_BALANCE, 0);

    if(config.COLOR_ENABLE_AUTO_WHITE_BALANCE == 0) {
      rs_set_device_option(rs_device_, RS_OPTION_COLOR_WHITE_BALANCE, config.COLOR_WHITE_BALANCE, 0);
    }

    //R200 camera specific options
    rs_set_device_option(rs_device_, RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED, config.R200_LR_AUTO_EXPOSURE_ENABLED, 0);

    if(config.R200_LR_AUTO_EXPOSURE_ENABLED == 0) {
      rs_set_device_option(rs_device_, RS_OPTION_R200_LR_EXPOSURE, config.R200_LR_EXPOSURE, 0);
    }

    rs_set_device_option(rs_device_, RS_OPTION_R200_LR_GAIN, config.R200_LR_GAIN, 0);
    rs_set_device_option(rs_device_, RS_OPTION_R200_EMITTER_ENABLED, config.R200_EMITTER_ENABLED, 0);
    rs_set_device_option(rs_device_, RS_OPTION_R200_DISPARITY_MULTIPLIER, config.R200_DISPARITY_MULTIPLIER, 0);

    if(config.R200_LR_AUTO_EXPOSURE_ENABLED == 1)
    {
      if(config.R200_AUTO_EXPOSURE_TOP_EDGE >= depth_height_) {
      	config.R200_AUTO_EXPOSURE_TOP_EDGE = depth_height_ - 1;
      }
      if(config.R200_AUTO_EXPOSURE_BOTTOM_EDGE >= depth_height_) {
      	config.R200_AUTO_EXPOSURE_BOTTOM_EDGE = depth_height_ - 1;
      }
      if(config.R200_AUTO_EXPOSURE_LEFT_EDGE >= depth_width_) {
      	config.R200_AUTO_EXPOSURE_LEFT_EDGE = depth_width_ - 1;
      }
      if(config.R200_AUTO_EXPOSURE_RIGHT_EDGE >= depth_width_) {
      	config.R200_AUTO_EXPOSURE_RIGHT_EDGE = depth_width_ - 1;
      }
      edge_values_[0] = config.R200_AUTO_EXPOSURE_LEFT_EDGE;
      edge_values_[1] = config.R200_AUTO_EXPOSURE_TOP_EDGE;
      edge_values_[2] = config.R200_AUTO_EXPOSURE_RIGHT_EDGE;
      edge_values_[3] = config.R200_AUTO_EXPOSURE_BOTTOM_EDGE;

      rs_set_device_options(rs_device_, edge_options_, 4, edge_values_, 0);
   }
  }

  void RealsenseNodelet::check_error ()
  {
    if (rs_error_)
    {
      ROS_ERROR_STREAM ("RealSense Camera - Error calling " << rs_get_failed_function (rs_error_) << " ( "
          << rs_get_failed_args (rs_error_) << " ): \n" << rs_get_error_message (rs_error_) << " \n");
      rs_free_error (rs_error_);

      ros::shutdown ();

      exit (EXIT_FAILURE);
    }
  }

  /*
   * Query the data for every frame and publish it to the different topics.
   */
  void RealsenseNodelet::devicePoll ()
  {
    while (ros::ok ())
    {
      publishStreams ();
    }
  }

  bool RealsenseNodelet::connectToCamera ()
  {
    if (enable_depth_ == false && enable_color_ == false)
    {
      ROS_INFO_STREAM ("RealSense Camera - None of the streams are enabled. Exiting.");
      return false;
    }

    rs_context_ = rs_create_context (RS_API_VERSION, &rs_error_);
    check_error ();

    int num_of_cameras = rs_get_device_count (rs_context_, NULL);
    ROS_INFO_STREAM(num_of_cameras << " connected");
    if (num_of_cameras < 1)
    {
      ROS_ERROR_STREAM ("RealSense Camera - No cameras are connected.");
      return false;
    }

    if(serial_number_.empty())
    {
    	rs_device_ = rs_get_device (rs_context_, 0, &rs_error_);
    	check_error ();
    }
    else // open a particular camera
    {
       ROS_INFO_STREAM("Trying to open " << serial_number_ << " for " << frame_id_[RS_STREAM_DEPTH] << " checking device " << i);
      bool found_device = false;
      // Open the specified device
      for(int i=0; i < num_of_cameras; i++)    
      {
        ROS_INFO_STREAM("Getting device number " << i);
	sleep(rand()/RAND_MAX*3.0);  // Not sure if this is necessary to avoid a race condition?
        rs_device_ = rs_get_device (rs_context_, i, &rs_error_);
	check_error ();
	if(rs_device_ == nullptr)
	  continue;
        std::string serial_no = rs_get_device_serial(rs_device_, &rs_error_);
	check_error ();
        ROS_INFO_STREAM("Fetched device with serial " << serial_no);
        if( serial_no == serial_number_)
        {
          ROS_INFO_STREAM("Found device " << serial_number_);
          found_device = true;
          break;  
        }
      }
      if(!found_device)
      {
	ROS_ERROR("Could not find device with serial number %s", serial_number_.c_str());
        return false;
      }
    }

    ROS_INFO_STREAM ("RealSense Camera - Number of cameras connected: " << num_of_cameras);
    ROS_INFO_STREAM ("RealSense Camera - Firmware version: " <<
        rs_get_device_firmware_version (rs_device_, &rs_error_));
    check_error ();
    ROS_INFO_STREAM ("RealSense Camera - Name: " << rs_get_device_name (rs_device_, &rs_error_));
    check_error ();
    ROS_INFO_STREAM ("RealSense Camera - Serial no: " << rs_get_device_serial (rs_device_, &rs_error_));
    check_error ();

    // Enable streams.
    if (enable_depth_ == true)
    {
      if (mode_.compare ("manual") == 0)
      {
        ROS_INFO_STREAM ("RealSense Camera - Enabling Depth Stream: manual mode");
        rs_enable_stream (rs_device_, RS_STREAM_DEPTH, depth_width_, depth_height_, DEPTH_FORMAT, depth_fps_, &rs_error_);
        rs_enable_stream (rs_device_, RS_STREAM_INFRARED, depth_width_, depth_height_, IR1_FORMAT, depth_fps_, &rs_error_);
        rs_enable_stream (rs_device_, RS_STREAM_INFRARED2, depth_width_, depth_height_, IR2_FORMAT, depth_fps_, 0);
      }
      else
      {
        ROS_INFO_STREAM ("RealSense Camera - Enabling Depth Stream: preset mode");
        rs_enable_stream_preset (rs_device_, RS_STREAM_DEPTH, RS_PRESET_BEST_QUALITY, &rs_error_); check_error ();
        rs_enable_stream_preset (rs_device_, RS_STREAM_INFRARED, RS_PRESET_BEST_QUALITY, &rs_error_); check_error ();
        rs_enable_stream_preset (rs_device_, RS_STREAM_INFRARED2, RS_PRESET_BEST_QUALITY, 0);
      }
    }

    if (enable_color_ == true)
    {
      if (mode_.compare ("manual") == 0)
      {
        ROS_INFO_STREAM ("RealSense Camera - Enabling Color Stream: manual mode");
        rs_enable_stream (rs_device_, RS_STREAM_COLOR, color_width_, color_height_, COLOR_FORMAT, color_fps_, &rs_error_);
      }
      else
      {
        ROS_INFO_STREAM ("RealSense Camera - Enabling Color Stream: preset mode");
        rs_enable_stream_preset (rs_device_, RS_STREAM_COLOR, RS_PRESET_BEST_QUALITY, &rs_error_); check_error ();
      }
    }

    for (int i = 0; i < RS_OPTION_COUNT; ++i)
    {
      option_str o =
      { (rs_option) i};

      if (!rs_device_supports_option (rs_device_, o.opt, &rs_error_))
      {
        continue;
      }

      rs_get_device_option_range (rs_device_, o.opt, &o.min, &o.max, &o.step, 0);
      if (o.min == o.max)
      {
        continue;
      }

      o.value = rs_get_device_option (rs_device_, o.opt, 0);
      options.push_back (o);
    }

    setConfigValues (camera_configuration_, options);

    // Start device.
    rs_start_device (rs_device_, &rs_error_);
    check_error ();
    
    sleep(1.0);
   
    int is_device_streaming = rs_is_device_streaming(rs_device_, &rs_error_);
    if(is_device_streaming) 
    	is_device_started_ = true;
    else
    {
      
      is_device_started_ = false;
      return false;
    }


    allocateResources ();

    return true;
  }

  /*
   * Define buffer for images and prepare camera info for each enabled stream.
   */
  void RealsenseNodelet::allocateResources ()
  {
    // Prepare camera for enabled streams (color/depth/infrared/infrared2).
    fillStreamEncoding ();

    if (rs_is_stream_enabled (rs_device_, RS_STREAM_COLOR, 0))
    {
      image_[(uint32_t) RS_STREAM_COLOR] = cv::Mat (color_height_, color_width_, CV_8UC3, cv::Scalar (0, 0, 0));
      prepareStreamCalibData (RS_STREAM_COLOR);
    }

    if (rs_is_stream_enabled (rs_device_, RS_STREAM_DEPTH, 0))
    {
      // Define message.
      image_[(uint32_t) RS_STREAM_DEPTH] = cv::Mat (depth_height_, depth_width_, CV_16UC1, cv::Scalar (0));
      prepareStreamCalibData (RS_STREAM_DEPTH);

      image_[(uint32_t) RS_STREAM_INFRARED] = cv::Mat (depth_height_, depth_width_, CV_8UC1, cv::Scalar (0));
      prepareStreamCalibData (RS_STREAM_INFRARED);

      if (rs_is_stream_enabled (rs_device_, RS_STREAM_INFRARED2, 0))
      {
        image_[(uint32_t) RS_STREAM_INFRARED2] = cv::Mat (depth_height_, depth_width_, CV_8UC1, cv::Scalar (0));
        prepareStreamCalibData (RS_STREAM_INFRARED2);
      }
    }
  }

  /*
   * Prepare camera_info for each enabled stream.
   */
  void RealsenseNodelet::prepareStreamCalibData (rs_stream rs_strm)
  {
    uint32_t stream_index = (uint32_t) rs_strm;
    rs_intrinsics intrinsic;
    rs_get_stream_intrinsics (rs_device_, rs_strm, &intrinsic, &rs_error_);
    check_error ();

    camera_info_[stream_index] = new sensor_msgs::CameraInfo ();
    camera_info_ptr_[stream_index] = sensor_msgs::CameraInfoPtr (camera_info_[stream_index]);

    camera_info_[stream_index]->header.frame_id = frame_id_[stream_index];
    camera_info_[stream_index]->width = intrinsic.width;
    camera_info_[stream_index]->height = intrinsic.height;

    camera_info_[stream_index]->K.at (0) = intrinsic.fx;
    camera_info_[stream_index]->K.at (2) = intrinsic.ppx;
    camera_info_[stream_index]->K.at (4) = intrinsic.fy;
    camera_info_[stream_index]->K.at (5) = intrinsic.ppy;
    camera_info_[stream_index]->K.at (8) = 1;

    camera_info_[stream_index]->P[0] = camera_info_[stream_index]->K.at (0);
    camera_info_[stream_index]->P[1] = 0;
    camera_info_[stream_index]->P[2] = camera_info_[stream_index]->K.at (2);
    camera_info_[stream_index]->P[3] = 0;

    camera_info_[stream_index]->P[4] = 0;
    camera_info_[stream_index]->P[5] = camera_info_[stream_index]->K.at (4);
    camera_info_[stream_index]->P[6] = camera_info_[stream_index]->K.at (5);
    camera_info_[stream_index]->P[7] = 0;

    camera_info_[stream_index]->P[8] = 0;
    camera_info_[stream_index]->P[9] = 0;
    camera_info_[stream_index]->P[10] = 1;
    camera_info_[stream_index]->P[11] = 0;

    camera_info_[stream_index]->distortion_model = "plumb_bob";

    for (int i = 0; i < 5; i++)
    camera_info_[stream_index]->D.push_back (0);
  }

  /*
   * Service function to get values for supported camera settings.
   */
  bool RealsenseNodelet::getCameraSettings (realsense_camera::cameraConfiguration::Request & req,
      realsense_camera::cameraConfiguration::Response & res)
  {
    std::string get_options_result_str;
    std::string opt_name, opt_value;

    for (int i = 0; i < RS_OPTION_COUNT; ++i)
    {
      option_str o =
      { (rs_option) i};
      if (!rs_device_supports_option (rs_device_, o.opt, &rs_error_))
      {
        continue;
      }

      rs_get_device_option_range (rs_device_, o.opt, &o.min, &o.max,
          &o.step, 0);
      if (o.min == o.max)
      {
        continue;
      }

      o.value = rs_get_device_option (rs_device_, o.opt, 0);
      opt_name = rs_option_to_string (o.opt);
      opt_value = boost::lexical_cast < std::string > (o.value);
      get_options_result_str += opt_name + ":" + opt_value + ";";
    }

    res.configuration_str = get_options_result_str;
    return true;
  }

  /*
   * Parse commandline arguments and set parameters.
   */
  void RealsenseNodelet::getConfigValues (std::vector < std::string > args)
  {
    while (args.size () > 1)
    {
      config_[args[0]] = args[1];
      args.erase (args.begin ());
      args.erase (args.begin ());
    }

    if (config_.find ("mode") != config_.end ())
    {
      mode_ = config_.at ("mode");
    }

    if (config_.find ("enable_color") != config_.end ())
    {
      if (strcmp((config_.at ("enable_color").c_str ()),"true") == 0)
      {
        enable_color_ = true;
      }
      else
      {
        enable_color_ = false;
      }
    }

    if (config_.find ("enable_depth") != config_.end ())
    {
      if (strcmp((config_.at ("enable_depth").c_str ()),"true") == 0)
      {
        enable_depth_ = true;
      }
      else
      {
        enable_depth_ = false;
      }
    }

    if (config_.find ("enable_pointcloud") != config_.end ())
    {
      if (strcmp((config_.at ("enable_pointcloud").c_str ()),"true") == 0)
      {
        enable_pointcloud_ = true;
      }
      else
      {
        enable_pointcloud_ = false;
      }
    }

    if (config_.find ("enable_tf") != config_.end ())
    {
      if (strcmp((config_.at ("enable_tf").c_str ()),"true") == 0)
      {
        enable_tf_ = true;
      }
      else
      {
        enable_tf_ = false;
      }
    }

    if (config_.find ("camera") != config_.end ())
    {
      camera_ = config_.at ("camera").c_str ();
    }

    if (config_.find ("depth_frame_id") != config_.end ())
    {
      frame_id_[RS_STREAM_DEPTH] = config_.at ("depth_frame_id").c_str ();
    }

    if (config_.find ("rgb_frame_id") != config_.end ())
    {
      frame_id_[RS_STREAM_COLOR] = config_.at ("rgb_frame_id").c_str ();
    }

    if (mode_.compare ("manual") == 0)
    {
      if (config_.find ("color_fps") != config_.end ())
      {
        color_fps_ = atoi (config_.at ("color_fps").c_str ());
      }

      if (config_.find ("depth_fps") != config_.end ())
      {
        depth_fps_ = atoi (config_.at ("depth_fps").c_str ());
      }

      if (config_.find ("color_height") != config_.end ())
      {
        color_height_ = atoi (config_.at ("color_height").c_str ());
      }

      if (config_.find ("color_width") != config_.end ())
      {
        color_width_ = atoi (config_.at ("color_width").c_str ());
      }

      if (config_.find ("depth_height") != config_.end ())
      {
        depth_height_ = atoi (config_.at ("depth_height").c_str ());
      }

      if (config_.find ("depth_width") != config_.end ())
      {
        depth_width_ = atoi (config_.at ("depth_width").c_str ());
      }
    }
  }

  /*
   * Change camera settings.
   */
  void RealsenseNodelet::setConfigValues (std::vector < std::string > args,
      std::vector < option_str > cam_options)
  {
    while (args.size () > 1)
    {
      config_[args[0]] = args[1];
      args.erase (args.begin ());
      args.erase (args.begin ());
    }

    while (!cam_options.empty ())
    {
      option_str o = cam_options.back ();
      std::string opt_name = rs_option_to_string (o.opt);
      if (config_.find (opt_name) != config_.end () && rs_device_supports_option (rs_device_, o.opt, 0))
      {
        ROS_INFO_STREAM ("RealSense Camera - Setting" << opt_name.c_str () << " to " << config_.at (opt_name).c_str ());
        rs_set_device_option (rs_device_, o.opt, atoi (config_.at (opt_name).c_str ()), &rs_error_);
        check_error ();
      }
      cam_options.pop_back ();
    }
  }

  /*
   * Copy frame data from realsense to member cv images.
   */
  void RealsenseNodelet::prepareStreamData (rs_stream rs_strm)
  {
    switch (rs_strm)
    {
      case RS_STREAM_COLOR:
      image_[(uint32_t) RS_STREAM_COLOR].data = (unsigned char *) (rs_get_frame_data (rs_device_, RS_STREAM_COLOR, 0));
      break;
      case RS_STREAM_DEPTH:
      image_depth16_ = reinterpret_cast <const uint16_t * >(rs_get_frame_data (rs_device_, RS_STREAM_DEPTH, 0));
      image_[(uint32_t) RS_STREAM_DEPTH].data = (unsigned char *) image_depth16_;
      break;
      case RS_STREAM_INFRARED:
      image_[(uint32_t) RS_STREAM_INFRARED].data =
      (unsigned char *) (rs_get_frame_data (rs_device_, RS_STREAM_INFRARED, 0));
      break;
      case RS_STREAM_INFRARED2:
      image_[(uint32_t) RS_STREAM_INFRARED2].data =
      (unsigned char *) (rs_get_frame_data (rs_device_, RS_STREAM_INFRARED2, 0));
      break;
      default:
      // no other streams supported
      break;
    }
  }

  /*
   * Populate the encodings for each stream.
   */
  void RealsenseNodelet::fillStreamEncoding ()
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
  void RealsenseNodelet::publishStreams ()
  {
    rs_wait_for_frames (rs_device_, &rs_error_);
    check_error ();
    time_stamp_ = ros::Time::now ();

    for (int stream_index = 0; stream_index < STREAM_COUNT; ++stream_index)
    {
      // Publish image stream only if there is at least one subscriber.
      if (camera_publisher_[stream_index].getNumSubscribers () > 0)
      {
        prepareStreamData ((rs_stream) stream_index);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage (std_msgs::Header (),
            stream_encoding_[stream_index],
            image_[stream_index]).toImageMsg ();

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

    if (pointcloud_publisher_.getNumSubscribers () > 0 && rs_is_stream_enabled (rs_device_, RS_STREAM_DEPTH, 0)
        && enable_pointcloud_ == true)
    {
      if (camera_publisher_[(uint32_t) RS_STREAM_DEPTH].getNumSubscribers () <= 0)
      {
        prepareStreamData (RS_STREAM_DEPTH);
      }
      if (camera_publisher_[(uint32_t) RS_STREAM_COLOR].getNumSubscribers () <= 0)
      {
        prepareStreamData (RS_STREAM_COLOR);
      }
      publishPointCloud (image_[(uint32_t) RS_STREAM_COLOR]);
    }
  }

  /*
   * Publish pointcloud.
   */
  void RealsenseNodelet::publishPointCloud (cv::Mat & image_color)
  {
    // Publish pointcloud only if there is at least one subscriber.
    if (pointcloud_publisher_.getNumSubscribers () > 0)
    {
      rs_intrinsics color_intrinsic;
      rs_extrinsics z_extrinsic;

      rs_intrinsics z_intrinsic;
      rs_get_stream_intrinsics (rs_device_, RS_STREAM_DEPTH, &z_intrinsic, &rs_error_);
      check_error ();

      if (enable_color_ == true)
      {
        rs_get_stream_intrinsics (rs_device_, RS_STREAM_COLOR, &color_intrinsic, &rs_error_); check_error ();
        rs_get_device_extrinsics (rs_device_, RS_STREAM_DEPTH, RS_STREAM_COLOR, &z_extrinsic, &rs_error_); check_error ();
      }

      // Convert pointcloud from the camera to pointcloud object for ROS.
      sensor_msgs::PointCloud2 msg_pointcloud;
      msg_pointcloud.width = depth_width_;
      msg_pointcloud.height = depth_height_;
      msg_pointcloud.header.stamp = time_stamp_;
      msg_pointcloud.is_dense = true;

      sensor_msgs::PointCloud2Modifier modifier (msg_pointcloud);

      modifier.setPointCloud2Fields (4, "x", 1,
          sensor_msgs::PointField::FLOAT32, "y", 1,
          sensor_msgs::PointField::FLOAT32, "z", 1,
          sensor_msgs::PointField::FLOAT32, "rgb", 1,
          sensor_msgs::PointField::FLOAT32);

      modifier.setPointCloud2FieldsByString (2, "xyz", "rgb");

      sensor_msgs::PointCloud2Iterator < float >iter_x (msg_pointcloud, "x");
      sensor_msgs::PointCloud2Iterator < float >iter_y (msg_pointcloud, "y");
      sensor_msgs::PointCloud2Iterator < float >iter_z (msg_pointcloud, "z");

      sensor_msgs::PointCloud2Iterator < uint8_t > iter_r (msg_pointcloud, "r");
      sensor_msgs::PointCloud2Iterator < uint8_t > iter_g (msg_pointcloud, "g");
      sensor_msgs::PointCloud2Iterator < uint8_t > iter_b (msg_pointcloud, "b");

      float depth_point[3], color_point[3], color_pixel[2], scaled_depth;
      unsigned char *color_data = image_color.data;
      const float depth_scale = rs_get_device_depth_scale (rs_device_, &rs_error_);
      check_error ();// Default value is 0.001

      // Fill the PointCloud2 fields.
      for (int y = 0; y < z_intrinsic.height; y++)
      {
        for (int x = 0; x < z_intrinsic.width; x++)
        {
          scaled_depth = ((float) *image_depth16_) * depth_scale;
          float depth_pixel[2] =
          { (float) x, (float) y};
          rs_deproject_pixel_to_point (depth_point, &z_intrinsic, depth_pixel, scaled_depth);

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
            rs_transform_point_to_point (color_point, &z_extrinsic, depth_point);
            rs_project_point_to_pixel (color_pixel, &color_intrinsic, color_point);

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
      pointcloud_publisher_.publish (msg_pointcloud);
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
    rs_get_device_extrinsics (rs_device_, RS_STREAM_DEPTH, RS_STREAM_COLOR, &z_extrinsic, &rs_error_); check_error ();

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
