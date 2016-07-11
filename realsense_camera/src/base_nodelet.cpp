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

#include "base_nodelet.h"

using namespace cv;
using namespace std;

PLUGINLIB_EXPORT_CLASS (realsense_camera::BaseNodelet, nodelet::Nodelet)
namespace realsense_camera
{

  /*
   * Public Methods.
   */
  BaseNodelet::~BaseNodelet()
  {
    stream_thread_->join();

    if (enable_tf_ == true)
    {
      transform_thread_->join();
    }

    // Stop device.
    if (is_device_started_ == true)
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Stopping camera");
      rs_stop_device(rs_device_, 0);
      rs_delete_context(rs_context_, &rs_error_);
      checkError();
    }

    ROS_INFO_STREAM(nodelet_name_ << " - Stopping...");
    ros::shutdown();
  }

  /*
   * Initialize the realsense camera
   */
  void BaseNodelet::onInit()
  {
    // Set default configurations.
    is_device_started_ = false;

    for (int i = 0; i < num_streams_; ++i)
    {
      camera_info_ptr_[i] = NULL;
      stream_ts_[i] = -1;
    }

    setStreamOptions();

    if (enable_depth_ == false && enable_color_ == false)
    {
      ROS_ERROR_STREAM(nodelet_name_ << " - None of the streams are enabled. Exiting!");
      ros::shutdown();
    }

    // Advertise the various topics and services.
    image_transport::ImageTransport it (nh_);
    camera_publisher_[RS_STREAM_COLOR] = it.advertiseCamera(COLOR_TOPIC, 1);
    camera_publisher_[RS_STREAM_DEPTH] = it.advertiseCamera(DEPTH_TOPIC, 1);
    camera_publisher_[RS_STREAM_INFRARED] = it.advertiseCamera(IR_TOPIC, 1);

    pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(PC_TOPIC, 1);

    get_options_service_ = nh_.advertiseService(SETTINGS_SERVICE, &BaseNodelet::getCameraOptionValues, this);

    // Poll for camera and connect if found
    while (!connectToCamera())
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Sleeping 5 seconds then retrying to connect");
      ros::Duration(5).sleep();
    }

    // Start working thread.
    stream_thread_ =
        boost::shared_ptr <boost::thread>(new boost::thread (boost::bind(&BaseNodelet::publishStreams, this)));

    if (enable_tf_ == true)
    {
      transform_thread_ =
      boost::shared_ptr<boost::thread>(new boost::thread (boost::bind(&BaseNodelet::publishTransforms, this)));
    }
  }

  /*
   *Protected Methods.
   */
  void BaseNodelet::enableStream(rs_stream stream_index, int width, int height, rs_format format, int fps)
  {
    std::string stream_mode;
    pnh_.param("mode", stream_mode, DEFAULT_MODE);

    // Enable streams.
    if (stream_mode.compare("manual") == 0)
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling " << STREAM_DESC[stream_index] << " stream: manual mode");
      rs_enable_stream(rs_device_, stream_index, width, height, format, fps, &rs_error_);
      checkError();
    }
    else
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling " << STREAM_DESC[stream_index] << " stream: preset mode");
      rs_enable_stream_preset(rs_device_, stream_index, RS_PRESET_BEST_QUALITY, &rs_error_);
      checkError();
    }

    if (camera_info_ptr_[stream_index] == NULL)
    {
      prepareStreamCalibData(stream_index);
    }
  }

  void BaseNodelet::disableStream(rs_stream stream_index)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Disabling " << STREAM_DESC[stream_index] << " stream");
    rs_disable_stream(rs_device_, stream_index, &rs_error_);
    checkError();
  }

  void BaseNodelet::checkError()
  {
    if (rs_error_)
    {
      ROS_ERROR_STREAM(nodelet_name_ << " - Error calling " << rs_get_failed_function(rs_error_) << " ( "
          << rs_get_failed_args(rs_error_) << " ): \n" << rs_get_error_message(rs_error_) << " \n");
      rs_free_error(rs_error_);

      ros::shutdown();

      exit (EXIT_FAILURE);
    }
  }

  bool BaseNodelet::connectToCamera()
  {
    std::string serial_no;
    std::string usb_port_id;

    rs_context_ = rs_create_context(RS_API_VERSION, &rs_error_);
    checkError();

    int num_of_cameras = rs_get_device_count(rs_context_, &rs_error_);
    checkError();

    // Exit with error if no cameras are connected.
    if (num_of_cameras < 1)
    {
      ROS_ERROR_STREAM(nodelet_name_ << " - No cameras detected!");
      rs_delete_context(rs_context_, &rs_error_);
      checkError();
      return false;
    }

    // Print list of all cameras found
    listCameras(num_of_cameras);

    pnh_.getParam("serial_no", serial_no);
    pnh_.getParam("usb_port_id", usb_port_id);

    // Exit with error if no serial number or usb_port_id is specified and multiple cameras are detected.
    if (serial_no.empty() && usb_port_id.empty() && num_of_cameras > 1)
    {
      ROS_ERROR_STREAM(nodelet_name_ << " - Multiple cameras detected but no input serial_no or usb_port_id specified");
      rs_delete_context(rs_context_, &rs_error_);
      checkError();
      return false;
    }

    // init rs_device_ before starting loop
    rs_device_ = nullptr;

    // find camera
    for (int i = 0; i < num_of_cameras; i++)
    {
      rs_device* rs_detected_device = rs_get_device(rs_context_, i, &rs_error_);

      // check serial_no and usb_port_id
      if ((serial_no.empty() || serial_no == rs_get_device_serial(rs_detected_device, &rs_error_)) &&
          (usb_port_id.empty() || usb_port_id == rs_get_device_usb_port_id(rs_detected_device, &rs_error_)))
      {
        // device found
        rs_device_= rs_detected_device;
      }
      // continue loop
    }

    if (rs_device_ == nullptr)
    {
      // camera not found
      string error_msg = " - Couldn't find camera to connect with ";
      error_msg += "serial_no = " + serial_no + ", ";
      error_msg += "usb_port_id = " + usb_port_id;

      ROS_ERROR_STREAM(nodelet_name_ << error_msg);

      rs_delete_context(rs_context_, &rs_error_);
      checkError();
      return false;
    }

    // print device info
    ROS_INFO_STREAM(nodelet_name_ << " - Connecting to camera with Serial No: " <<
        rs_get_device_serial(rs_device_, &rs_error_) <<
        " USB Port ID: " << rs_get_device_usb_port_id(rs_device_, &rs_error_));
    checkError();

    // Enable streams.
    if (enable_color_ == true)
    {
      enableStream(RS_STREAM_COLOR, width_[RS_STREAM_COLOR], height_[RS_STREAM_COLOR], COLOR_FORMAT, fps_[RS_STREAM_COLOR]);
    }
    if (enable_depth_ == true)
    {
      enableStream(RS_STREAM_DEPTH, width_[RS_STREAM_DEPTH], height_[RS_STREAM_DEPTH], DEPTH_FORMAT, fps_[RS_STREAM_DEPTH]);
      enableStream(RS_STREAM_INFRARED, width_[RS_STREAM_DEPTH], height_[RS_STREAM_DEPTH], IR_FORMAT, fps_[RS_STREAM_DEPTH]);
    }
    getCameraOptions();
    setStaticCameraOptions();

    // Start device.
    ROS_INFO_STREAM(nodelet_name_ << " - Starting camera");
    rs_start_device(rs_device_, &rs_error_);
    checkError();

    is_device_started_ = true;

    allocateResources();

    return true;
  }

  void BaseNodelet::listCameras(int num_of_cameras)
  {
    // print list of detected cameras
    std::string detected_camera_msg = " - Detected the following cameras:";
    for (int i = 0; i < num_of_cameras; i++)
    {
      // get device
      rs_device* rs_detected_device = rs_get_device(rs_context_, i, &rs_error_);

      // print device details
      detected_camera_msg = detected_camera_msg +
            "\n\t\t\t\t- Serial No: " + rs_get_device_serial(rs_detected_device, &rs_error_) +
            "; Firmware: " + rs_get_device_firmware_version(rs_detected_device, &rs_error_) +
            "; Name: " + rs_get_device_name(rs_detected_device, &rs_error_) +
            "; USB Port ID: " + rs_get_device_usb_port_id(rs_detected_device, &rs_error_);
      checkError();
    }
    ROS_INFO_STREAM(nodelet_name_ + detected_camera_msg);
  }

  /*
   * Gets the options supported by the camera along with their min, max and step values.
   */
  void BaseNodelet::getCameraOptions()
  {
    for (int i = 0; i < RS_OPTION_COUNT; ++i)
    {
      CameraOptions o = { (rs_option) i };

      if (rs_device_supports_option(rs_device_, o.opt, &rs_error_))
      {
        rs_get_device_option_range(rs_device_, o.opt, &o.min, &o.max, &o.step, 0);

        // Skip the camera options where min and max values are the same.
        if (o.min != o.max)
        {
          o.value = rs_get_device_option(rs_device_, o.opt, 0);
          camera_options_.push_back(o);
        }
      }
    }
  }

  /*
   * Define buffer for images and prepare camera info for each enabled stream.
   */
  void BaseNodelet::allocateResources()
  {
    // Prepare camera for enabled streams (color/depth/infrared)
    fillStreamEncoding();

    image_[RS_STREAM_COLOR] = cv::Mat(height_[RS_STREAM_COLOR], width_[RS_STREAM_COLOR], CV_8UC3, cv::Scalar (0, 0, 0));
    image_[RS_STREAM_DEPTH] = cv::Mat(height_[RS_STREAM_DEPTH], width_[RS_STREAM_DEPTH], CV_16UC1, cv::Scalar (0));
    image_[RS_STREAM_INFRARED] = cv::Mat(height_[RS_STREAM_DEPTH], width_[RS_STREAM_DEPTH], CV_8UC1, cv::Scalar (0));
  }

  /*
   * Prepare camera_info for each enabled stream.
   */
  void BaseNodelet::prepareStreamCalibData(rs_stream stream_index)
  {
    rs_intrinsics intrinsic;
    rs_get_stream_intrinsics(rs_device_, stream_index, &intrinsic, &rs_error_);
    checkError();

    sensor_msgs::CameraInfo * camera_info = new sensor_msgs::CameraInfo();
    camera_info_ptr_[stream_index] = sensor_msgs::CameraInfoPtr (camera_info);

    camera_info->header.frame_id = frame_id_[stream_index];
    camera_info->width = intrinsic.width;
    camera_info->height = intrinsic.height;

    camera_info->K.at(0) = intrinsic.fx;
    camera_info->K.at(2) = intrinsic.ppx;
    camera_info->K.at(4) = intrinsic.fy;
    camera_info->K.at(5) = intrinsic.ppy;
    camera_info->K.at(8) = 1;

    camera_info->P.at(0) = camera_info->K.at(0);
    camera_info->P.at(1) = 0;
    camera_info->P.at(2) = camera_info->K.at(2);
    camera_info->P.at(3) = 0;

    camera_info->P.at(4) = 0;
    camera_info->P.at(5) = camera_info->K.at(4);
    camera_info->P.at(6) = camera_info->K.at(5);
    camera_info->P.at(7) = 0;

    camera_info->P.at(8) = 0;
    camera_info->P.at(9) = 0;
    camera_info->P.at(10) = 1;
    camera_info->P.at(11) = 0;

    if (stream_index == RS_STREAM_DEPTH)
    {
      // set depth to color translation values in Projection matrix (P)
      rs_extrinsics z_extrinsic;
      rs_get_device_extrinsics(rs_device_, RS_STREAM_DEPTH, RS_STREAM_COLOR, &z_extrinsic, &rs_error_);
      checkError();
      camera_info->P.at(3) = z_extrinsic.translation[0];     // Tx
      camera_info->P.at(7) = z_extrinsic.translation[1];     // Ty
      camera_info->P.at(11) = z_extrinsic.translation[2];    // Tz
    }

    camera_info->distortion_model = "plumb_bob";

    // set R (rotation matrix) values to identity matrix
    camera_info->R.at(0) = (double) 1;
    camera_info->R.at(1) = (double) 0;
    camera_info->R.at(2) = (double) 0;
    camera_info->R.at(3) = (double) 0;
    camera_info->R.at(4) = (double) 1;
    camera_info->R.at(5) = (double) 0;
    camera_info->R.at(6) = (double) 0;
    camera_info->R.at(7) = (double) 0;
    camera_info->R.at(8) = (double) 1;

    for (int i = 0; i < 5; i++)
    {
      camera_info->D.push_back(intrinsic.coeffs[i]);
    }
  }

  /*
   * Get the latest values of the camera options.
   */
  bool BaseNodelet::getCameraOptionValues(realsense_camera::cameraConfiguration::Request & req,
      realsense_camera::cameraConfiguration::Response & res)
  {
    std::string get_options_result_str;
    std::string opt_name, opt_value;

    for (CameraOptions o: camera_options_)
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
  void BaseNodelet::setStreamOptions()
  {
    pnh_.param("enable_depth", enable_depth_, ENABLE_DEPTH);
    pnh_.param("enable_color", enable_color_, ENABLE_COLOR);
    pnh_.param("enable_pointcloud", enable_pointcloud_, ENABLE_PC);
    pnh_.param("enable_tf", enable_tf_, ENABLE_TF);
    pnh_.param("depth_width", width_[RS_STREAM_DEPTH], DEPTH_WIDTH);
    pnh_.param("depth_height", height_[RS_STREAM_DEPTH], DEPTH_HEIGHT);
    pnh_.param("color_width", width_[RS_STREAM_COLOR], COLOR_WIDTH);
    pnh_.param("color_height", height_[RS_STREAM_COLOR], COLOR_HEIGHT);
    pnh_.param("depth_fps", fps_[RS_STREAM_DEPTH], DEPTH_FPS);
    pnh_.param("color_fps", fps_[RS_STREAM_COLOR], COLOR_FPS);
    pnh_.param("depth_optical_frame_id", frame_id_[RS_STREAM_DEPTH], DEFAULT_DEPTH_OPTICAL_FRAME_ID);
    pnh_.param("color_optical_frame_id", frame_id_[RS_STREAM_COLOR], DEFAULT_COLOR_OPTICAL_FRAME_ID);
    pnh_.param("ir_frame_id", frame_id_[RS_STREAM_INFRARED], DEFAULT_IR_FRAME_ID);
  }


  /*
   * Copy frame data from realsense to member cv images.
   */
  void BaseNodelet::prepareStreamData(rs_stream rs_strm)
  {
    if (rs_strm == RS_STREAM_DEPTH)
    {
      // fill depth buffer
      image_depth16_ = reinterpret_cast <const uint16_t * >(rs_get_frame_data(rs_device_, RS_STREAM_DEPTH, 0));
    }
    // fill image buffer for stream
    image_[(uint32_t) rs_strm].data = (unsigned char *) (rs_get_frame_data(rs_device_, rs_strm, 0));
  }

  /*
   * Populate the encodings for each stream.
   */
  void BaseNodelet::fillStreamEncoding()
  {
    stream_encoding_[RS_STREAM_COLOR] = "rgb8";
    stream_step_[RS_STREAM_COLOR] = width_[RS_STREAM_COLOR] * sizeof (unsigned char) * 3;
    stream_encoding_[RS_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;
    stream_step_[RS_STREAM_DEPTH] = width_[RS_STREAM_DEPTH] * sizeof (uint16_t);
    stream_encoding_[RS_STREAM_INFRARED] = sensor_msgs::image_encodings::TYPE_8UC1;
    stream_step_[RS_STREAM_INFRARED] = width_[RS_STREAM_DEPTH] * sizeof (unsigned char);
  }

  /*
   * Publish streams.
   */
  void BaseNodelet::publishStreams()
  {
    while (ros::ok())
    {
      if (enable_depth_ == false && rs_is_stream_enabled(rs_device_, RS_STREAM_DEPTH, 0) == 1)
      {
        if (rs_is_device_streaming(rs_device_, 0) == 1)
        {
          ROS_INFO_STREAM(nodelet_name_ << " - Stopping camera");
          rs_stop_device(rs_device_, &rs_error_);
          checkError();
        }

        // disable Depth and IR stream
        disableStream(RS_STREAM_DEPTH);
        disableStream(RS_STREAM_INFRARED);

        if (rs_is_device_streaming(rs_device_, 0) == 0)
        {
          ROS_INFO_STREAM(nodelet_name_ << " - Starting camera");
          rs_start_device(rs_device_, &rs_error_);
          checkError();
        }
      }

      if (enable_depth_ == true && rs_is_stream_enabled(rs_device_, RS_STREAM_DEPTH, 0) == 0)
      {
        if (rs_is_device_streaming(rs_device_, 0) == 1)
        {
          ROS_INFO_STREAM(nodelet_name_ << " - Stopping camera");
          rs_stop_device(rs_device_, &rs_error_);
          checkError();
        }

        enableStream(RS_STREAM_DEPTH, width_[RS_STREAM_DEPTH], height_[RS_STREAM_DEPTH], DEPTH_FORMAT, fps_[RS_STREAM_DEPTH]);
        enableStream(RS_STREAM_INFRARED, width_[RS_STREAM_DEPTH], height_[RS_STREAM_DEPTH], IR_FORMAT, fps_[RS_STREAM_DEPTH]);

        if (rs_is_device_streaming(rs_device_, 0) == 0)
        {
          ROS_INFO_STREAM(nodelet_name_ << " - Starting camera");
          rs_start_device(rs_device_, &rs_error_);
          checkError();
        }
      }

      if (rs_is_device_streaming(rs_device_, 0) == 1)
      {
        rs_wait_for_frames(rs_device_, &rs_error_);
        checkError();

        ros::Time time_stamp = ros::Time::now();
        bool duplicate_depth_color = false;

        for (int stream_index = 0; stream_index < num_streams_; ++stream_index)
        {
          // Publish image stream only if there is at least one subscriber.
          if (camera_publisher_[stream_index].getNumSubscribers() > 0 &&
              rs_is_stream_enabled(rs_device_, (rs_stream) stream_index, 0) == 1)
          {
            int current_ts = rs_get_frame_timestamp(rs_device_, (rs_stream) stream_index, 0);
            if (stream_ts_[stream_index] != current_ts) // Publish frames only if its not duplicate
            {
              prepareStreamData((rs_stream) stream_index);

              sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                  stream_encoding_[stream_index],
                  image_[stream_index]).toImageMsg();

              msg->header.frame_id = frame_id_[stream_index];
              msg->header.stamp = time_stamp; // Publish timestamp to synchronize frames.
              msg->width = image_[stream_index].cols;
              msg->height = image_[stream_index].rows;
              msg->is_bigendian = false;
              msg->step = stream_step_[stream_index];

              camera_info_ptr_[stream_index]->header.stamp = msg->header.stamp;
              camera_publisher_[stream_index].publish (msg, camera_info_ptr_[stream_index]);
            }
            else
            {
              if ((stream_index == RS_STREAM_DEPTH) || (stream_index == RS_STREAM_COLOR))
              {
                duplicate_depth_color = true; // Set this flag to true if Depth and/or Color frame is duplicate
              }
            }
            stream_ts_[stream_index] = current_ts;
          }
        }

        if (pointcloud_publisher_.getNumSubscribers() > 0 &&
            rs_is_stream_enabled(rs_device_, RS_STREAM_DEPTH, 0) == 1 && enable_pointcloud_ == true &&
            (duplicate_depth_color == false)) // Skip publishing PointCloud if Depth and/or Color frame was duplicate
        {
          if (camera_publisher_[RS_STREAM_DEPTH].getNumSubscribers() <= 0)
          {
            prepareStreamData(RS_STREAM_DEPTH);
          }
          if (camera_publisher_[RS_STREAM_COLOR].getNumSubscribers() <= 0)
          {
            prepareStreamData(RS_STREAM_COLOR);
          }
          publishPointCloud(image_[RS_STREAM_COLOR], time_stamp);
        }
      }
    }
  }

  /*
   * Publish pointcloud.
   */
  void BaseNodelet::publishPointCloud (cv::Mat & image_color, ros::Time time_stamp)
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
      msg_pointcloud.width = width_[RS_STREAM_DEPTH];
      msg_pointcloud.height = height_[RS_STREAM_DEPTH];
      msg_pointcloud.header.stamp = time_stamp;
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

          if (depth_point[2] <= 0 || depth_point[2] > max_z_)
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
  void BaseNodelet::publishTransforms()
  {
    // publish transforms for the cameras
    ROS_INFO_STREAM(nodelet_name_ << " - Publishing camera transforms");
    tf::Transform tr;
    tf::Quaternion q;
    tf::TransformBroadcaster tf_broadcaster;
    rs_extrinsics z_extrinsic;

    // extrinsics are offsets between the cameras
    rs_get_device_extrinsics(rs_device_, RS_STREAM_DEPTH, RS_STREAM_COLOR, &z_extrinsic, &rs_error_);
    checkError();

    std::string base_frame_id;
    std::string color_frame_id;
    std::string depth_frame_id;
    pnh_.param("base_frame_id", base_frame_id, DEFAULT_BASE_FRAME_ID);
    pnh_.param("color_frame_id", color_frame_id, DEFAULT_COLOR_FRAME_ID);
    pnh_.param("depth_frame_id", depth_frame_id, DEFAULT_DEPTH_FRAME_ID);

    ros::Duration sleeper(0.1); // 100ms

    while (ros::ok())
    {
      // time stamp is future dated to be valid for given duration
      ros::Time time_stamp = ros::Time::now() + sleeper;

      // transform base frame to depth frame
      tr.setOrigin(tf::Vector3(z_extrinsic.translation[2], -z_extrinsic.translation[0], -z_extrinsic.translation[1]));
      tr.setRotation(tf::Quaternion(0, 0, 0, 1));
      tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, base_frame_id, depth_frame_id));

      // transform depth frame to depth optical frame
      tr.setOrigin(tf::Vector3(0,0,0));
      q.setEuler( M_PI/2, 0.0, -M_PI/2 );
      tr.setRotation( q );
      tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, depth_frame_id, frame_id_[RS_STREAM_DEPTH]));

      // transform base frame to color frame (these are the same)
      tr.setOrigin(tf::Vector3(0,0,0));
      tr.setRotation(tf::Quaternion(0, 0, 0, 1));
      tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, base_frame_id, color_frame_id));

      // transform color frame to color optical frame
      tr.setOrigin(tf::Vector3(0,0,0));
      q.setEuler( M_PI/2, 0.0, -M_PI/2 );
      tr.setRotation( q );
      tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, color_frame_id, frame_id_[RS_STREAM_COLOR]));

      sleeper.sleep(); // need sleep or transform won't publish correctly
    }
  }
}  // end namespace
