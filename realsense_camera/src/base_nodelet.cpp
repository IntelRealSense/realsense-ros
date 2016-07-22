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
   * Nodelet Destructor.
   */
  BaseNodelet::~BaseNodelet()
  {
    topic_thread_->join();

    if (enable_tf_ == true)
    {
      transform_thread_->join();
    }

    stopCamera();

    rs_delete_context(rs_context_, &rs_error_);
    checkError();

    ROS_INFO_STREAM(nodelet_name_ << " - Stopping...");
    ros::shutdown();
  }

  /*
   * Initialize the nodelet.
   */
  void BaseNodelet::onInit()
  {
    getParameters();

    if (enable_[RS_STREAM_DEPTH] == false && enable_[RS_STREAM_COLOR] == false)
    {
      ROS_ERROR_STREAM(nodelet_name_ << " - None of the streams are enabled. Exiting!");
      ros::shutdown();
    }

    while (!connectToCamera()) // Poll for camera and connect if found
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Sleeping 5 seconds then retrying to connect");
      ros::Duration(5).sleep();
    }

    advertiseTopics();
    advertiseServices();
    std::vector<std::string> dynamic_params = setDynamicReconfServer();
    getCameraOptions();
    setStaticCameraOptions(dynamic_params);
    setStreams();
    startCamera();

    // Start topic thread.
    topic_thread_ =
        boost::shared_ptr <boost::thread>(new boost::thread (boost::bind(&BaseNodelet::prepareTopics, this)));

    // Start tranform thread.
    if (enable_tf_ == true)
    {
      transform_thread_ =
      boost::shared_ptr<boost::thread>(new boost::thread (boost::bind(&BaseNodelet::prepareTransforms, this)));
    }

    // Start dynamic reconfigure callback
    startDynamicReconfCallback();
  }

  /*
   * Get the nodelet parameters.
   */
  void BaseNodelet::getParameters()
  {
    nodelet_name_ = getName();
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();
    pnh_.getParam("serial_no", serial_no_);
    pnh_.getParam("usb_port_id", usb_port_id_);
    pnh_.getParam("camera_type", camera_type_);
    pnh_.param("mode", mode_, DEFAULT_MODE);
    pnh_.param("enable_depth", enable_[RS_STREAM_DEPTH], ENABLE_DEPTH);
    pnh_.param("enable_color", enable_[RS_STREAM_COLOR], ENABLE_COLOR);
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
    pnh_.param("base_frame_id", base_frame_id_, DEFAULT_BASE_FRAME_ID);
    pnh_.param("color_frame_id", color_frame_id_, DEFAULT_COLOR_FRAME_ID);
    pnh_.param("depth_frame_id", depth_frame_id_, DEFAULT_DEPTH_FRAME_ID);
    pnh_.param("ir_frame_id", frame_id_[RS_STREAM_INFRARED], DEFAULT_IR_FRAME_ID);
  }

  /*
   * Connect to camera.
   */
  bool BaseNodelet::connectToCamera()
  {
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
    std::vector<int> camera_type_index = listCameras(num_of_cameras);

    // Exit with error if no cameras of correct type are connected.
    if (camera_type_index.size() < 1)
    {
      ROS_ERROR_STREAM(nodelet_name_ << " - No '" << camera_type_ << "' cameras detected!");
      rs_delete_context(rs_context_, &rs_error_);
      checkError();
      return false;
    }

    // Exit with error if no serial_no or usb_port_id is specified and multiple cameras are detected.
    if (serial_no_.empty() && usb_port_id_.empty() && camera_type_index.size() > 1)
    {
      ROS_ERROR_STREAM(nodelet_name_ <<
          " - Multiple cameras of same type detected but no input serial_no or usb_port_id specified");
      rs_delete_context(rs_context_, &rs_error_);
      checkError();
      return false;
    }

    // init rs_device_ before starting loop
    rs_device_ = nullptr;

    // find camera
    for (int i: camera_type_index)
    {
      rs_device* rs_detected_device = rs_get_device(rs_context_, i, &rs_error_);
      checkError();

      // check serial_no and usb_port_id
      if ((serial_no_.empty() || serial_no_ == rs_get_device_serial(rs_detected_device, &rs_error_)) &&
          (usb_port_id_.empty() || usb_port_id_ == rs_get_device_usb_port_id(rs_detected_device, &rs_error_)))
      {
        // device found
        rs_device_= rs_detected_device;
        break;
      }
      // continue loop
    }

    if (rs_device_ == nullptr)
    {
      // camera not found
      string error_msg = " - Couldn't find camera to connect with ";
      error_msg += "serial_no = " + serial_no_ + ", ";
      error_msg += "usb_port_id = " + usb_port_id_ ;

      ROS_ERROR_STREAM(nodelet_name_ << error_msg);

      rs_delete_context(rs_context_, &rs_error_);
      checkError();
      return false;
    }

    // print device info
    ROS_INFO_STREAM(nodelet_name_ << " - Connecting to camera with Serial No: " <<
        rs_get_device_serial(rs_device_, &rs_error_) <<
        ", USB Port ID: " << rs_get_device_usb_port_id(rs_device_, &rs_error_));
    checkError();
    return true;
  }

  /*
   * List details of the detected cameras.
   */
  std::vector<int> BaseNodelet::listCameras(int num_of_cameras)
  {

    // print list of detected cameras
    std::vector<int> camera_type_index;
    std::string detected_camera_msg = " - Detected the following cameras:";
    for (int i = 0; i < num_of_cameras; i++)
    {
      // get device
      rs_device* rs_detected_device = rs_get_device(rs_context_, i, &rs_error_);

      // get camera name
      std::string camera_name = rs_get_device_name(rs_detected_device, &rs_error_);
      checkError();

      if (camera_name.find(camera_type_) != std::string::npos)
      {
        camera_type_index.push_back(i);
      }
      // print device details
      detected_camera_msg = detected_camera_msg +
            "\n\t\t\t\t- Serial No: " + rs_get_device_serial(rs_detected_device, &rs_error_) +
            ", USB Port ID: " + rs_get_device_usb_port_id(rs_detected_device, &rs_error_) +
            ", Name: " + camera_name +
            ", Firmware: " + rs_get_device_firmware_version(rs_detected_device, &rs_error_);
      checkError();
    }
    ROS_INFO_STREAM(nodelet_name_ + detected_camera_msg);
    return camera_type_index;
  }

  /*
   * Advertise topics.
   */
  void BaseNodelet::advertiseTopics()
  {
    image_transport::ImageTransport image_transport(nh_);
    camera_publisher_[RS_STREAM_COLOR] = image_transport.advertiseCamera(COLOR_TOPIC, 1);
    camera_publisher_[RS_STREAM_DEPTH] = image_transport.advertiseCamera(DEPTH_TOPIC, 1);
    camera_publisher_[RS_STREAM_INFRARED] = image_transport.advertiseCamera(IR_TOPIC, 1);
    pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(PC_TOPIC, 1);
  }

  /*
   * Advertise services.
   */
  void BaseNodelet::advertiseServices()
  {
    get_options_service_ = nh_.advertiseService(SETTINGS_SERVICE, &BaseNodelet::getCameraOptionValues, this);
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
   * Get the options supported by the camera along with their min, max and step values.
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
   * Set the static camera options.
   */
  void BaseNodelet::setStaticCameraOptions(std::vector<std::string> dynamic_params)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Setting static camera options");

    // Iterate through the supported camera options
    for (CameraOptions o: camera_options_)
    {
      std::string opt_name = rs_option_to_string(o.opt);
      bool found = false;

      for (std::string param_name: dynamic_params)
      {
        std::transform(opt_name.begin(), opt_name.end(), opt_name.begin(), ::tolower);
        if (opt_name.compare(param_name) == 0)
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
          ROS_DEBUG_STREAM(nodelet_name_ << " - " << opt_name << " = " << opt_val);
          rs_set_device_option(rs_device_, o.opt, opt_val, &rs_error_);
          checkError();
        }
      }
    }
  }

  /*
   * Set the streams according to their corresponding flag values.
   */
  void BaseNodelet::setStreams()
  {
    // Enable streams.
    if (enable_[RS_STREAM_COLOR] == true)
    {
      enableStream(RS_STREAM_COLOR, width_[RS_STREAM_COLOR], height_[RS_STREAM_COLOR], COLOR_FORMAT,
          fps_[RS_STREAM_COLOR]);
      if (camera_info_ptr_[RS_STREAM_COLOR] == NULL)
      {
        ROS_DEBUG_STREAM(nodelet_name_ << " - Allocating resources for " << STREAM_DESC[RS_STREAM_COLOR]);
        getStreamCalibData(RS_STREAM_COLOR);
        step_[RS_STREAM_COLOR] = camera_info_ptr_[RS_STREAM_COLOR]->width * sizeof(unsigned char) * 3;
        image_[RS_STREAM_COLOR] = cv::Mat(camera_info_ptr_[RS_STREAM_COLOR]->height,
            camera_info_ptr_[RS_STREAM_COLOR]->width, CV_8UC3, cv::Scalar(0, 0, 0));
      }
      ts_[RS_STREAM_COLOR] = -1;
      encoding_[RS_STREAM_COLOR] = sensor_msgs::image_encodings::RGB8;
    }
    else if (enable_[RS_STREAM_COLOR] == false)
    {
      disableStream(RS_STREAM_COLOR);
    }

    if (enable_[RS_STREAM_DEPTH] == true)
    {
      enableStream(RS_STREAM_DEPTH, width_[RS_STREAM_DEPTH], height_[RS_STREAM_DEPTH], DEPTH_FORMAT,
          fps_[RS_STREAM_DEPTH]);
      if (camera_info_ptr_[RS_STREAM_DEPTH] == NULL)
      {
        ROS_DEBUG_STREAM(nodelet_name_ << " - Allocating resources for " << STREAM_DESC[RS_STREAM_DEPTH]);
        getStreamCalibData(RS_STREAM_DEPTH);
        step_[RS_STREAM_DEPTH] = camera_info_ptr_[RS_STREAM_DEPTH]->width * sizeof(uint16_t);
        image_[RS_STREAM_DEPTH] = cv::Mat(camera_info_ptr_[RS_STREAM_DEPTH]->height,
            camera_info_ptr_[RS_STREAM_DEPTH]->width, CV_16UC1, cv::Scalar(0, 0, 0));
      }
      ts_[RS_STREAM_DEPTH] = -1;
      encoding_[RS_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;

      enableStream(RS_STREAM_INFRARED, width_[RS_STREAM_DEPTH], height_[RS_STREAM_DEPTH], IR_FORMAT,
          fps_[RS_STREAM_DEPTH]);
      if (camera_info_ptr_[RS_STREAM_INFRARED] == NULL)
      {
        ROS_DEBUG_STREAM(nodelet_name_ << " - Allocating resources for " << STREAM_DESC[RS_STREAM_INFRARED]);
        getStreamCalibData(RS_STREAM_INFRARED);
        step_[RS_STREAM_INFRARED] = camera_info_ptr_[RS_STREAM_INFRARED]->width * sizeof(unsigned char);
        image_[RS_STREAM_INFRARED] = cv::Mat(camera_info_ptr_[RS_STREAM_INFRARED]->height,
            camera_info_ptr_[RS_STREAM_INFRARED]->width, CV_8UC1, cv::Scalar(0, 0, 0));
      }

      ts_[RS_STREAM_INFRARED] = -1;
      encoding_[RS_STREAM_INFRARED] = sensor_msgs::image_encodings::TYPE_8UC1;
    }
    else if (enable_[RS_STREAM_DEPTH] == false)
    {
      disableStream(RS_STREAM_DEPTH);
      disableStream(RS_STREAM_INFRARED);
    }
  }

  /*
   * Enable individual streams.
   */
  void BaseNodelet::enableStream(rs_stream stream_index, int width, int height, rs_format format, int fps)
  {
    if (rs_is_stream_enabled(rs_device_, stream_index, 0) == 0)
    {
      if (mode_.compare("manual") == 0)
      {
        ROS_INFO_STREAM(nodelet_name_ << " - Enabling " << STREAM_DESC[stream_index] << " in manual mode");
        rs_enable_stream(rs_device_, stream_index, width, height, format, fps, &rs_error_);
        checkError();
      }
      else
      {
        ROS_INFO_STREAM(nodelet_name_ << " - Enabling " << STREAM_DESC[stream_index] << " in preset mode");
        rs_enable_stream_preset(rs_device_, stream_index, RS_PRESET_BEST_QUALITY, &rs_error_);
        checkError();
      }
    }
  }

  /*
   * Prepare camera_info for each enabled stream.
   */
  void BaseNodelet::getStreamCalibData(rs_stream stream_index)
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
   * Disable individual streams.
   */
  void BaseNodelet::disableStream(rs_stream stream_index)
  {
    if (rs_is_stream_enabled(rs_device_, stream_index, 0) == 1)
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling " << STREAM_DESC[stream_index] << " stream");
      rs_disable_stream(rs_device_, stream_index, &rs_error_);
      checkError();
    }
  }

  /*
   * Start camera.
   */
  void BaseNodelet::startCamera()
  {
    if (rs_is_device_streaming(rs_device_, 0) == 0)
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Starting camera");
      rs_start_device(rs_device_, &rs_error_);
      checkError();
    }
  }

  /*
   * Stop camera.
   */
  void BaseNodelet::stopCamera()
  {
    if (rs_is_device_streaming(rs_device_, 0) == 1)
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Stopping camera");
      rs_stop_device(rs_device_, 0);
      checkError();
    }
  }

  /*
   * Copy frame data from realsense to member cv images.
   */
  void BaseNodelet::getStreamData(rs_stream stream_index)
  {
    if (stream_index == RS_STREAM_DEPTH)
    {
      // fill depth buffer
      image_depth16_ = reinterpret_cast <const uint16_t * >(rs_get_frame_data(rs_device_, RS_STREAM_DEPTH, 0));
    }
    // fill image buffer for stream
    image_[(uint32_t) stream_index].data = (unsigned char *) (rs_get_frame_data(rs_device_, stream_index, 0));
  }

  /*
   * Prepare and publish topics.
   */
  void BaseNodelet::prepareTopics()
  {
    while (ros::ok())
    {
      if (enable_[RS_STREAM_DEPTH] != rs_is_stream_enabled(rs_device_, RS_STREAM_DEPTH, 0))
      {
        stopCamera();
        setStreams();
        startCamera();
      }

      if (rs_is_device_streaming(rs_device_, 0) == 1)
      {
        rs_wait_for_frames(rs_device_, &rs_error_);
        checkError();

        topic_ts_ = ros::Time::now();
        duplicate_depth_color_ = false;

        publishTopics();

        if (pointcloud_publisher_.getNumSubscribers() > 0 &&
            rs_is_stream_enabled(rs_device_, RS_STREAM_DEPTH, 0) == 1 && enable_pointcloud_ == true &&
            (duplicate_depth_color_ == false)) // Skip publishing PointCloud if Depth and/or Color frame was duplicate
        {
          if (camera_publisher_[RS_STREAM_DEPTH].getNumSubscribers() <= 0)
          {
            getStreamData(RS_STREAM_DEPTH);
          }
          if (camera_publisher_[RS_STREAM_COLOR].getNumSubscribers() <= 0)
          {
            getStreamData(RS_STREAM_COLOR);
          }
          publishPCTopic();
        }
      }
    }
  }

  /*
   * Call publicTopic() for all streams.
   */
  void BaseNodelet::publishTopics()
  {
    publishTopic(RS_STREAM_DEPTH);
    publishTopic(RS_STREAM_COLOR);
    publishTopic(RS_STREAM_INFRARED);
  }

  /*
   * Publish topic.
   */
  void BaseNodelet::publishTopic(rs_stream stream_index)
  {
    // Publish stream only if there is at least one subscriber.
    if (camera_publisher_[stream_index].getNumSubscribers() > 0 &&
        rs_is_stream_enabled(rs_device_, (rs_stream) stream_index, 0) == 1)
    {
      int current_ts = rs_get_frame_timestamp(rs_device_, (rs_stream) stream_index, 0);
      if (ts_[stream_index] != current_ts) // Publish frames only if its not duplicate
      {
        getStreamData(stream_index);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
            encoding_[stream_index],
            image_[stream_index]).toImageMsg();

        msg->header.frame_id = frame_id_[stream_index];
        msg->header.stamp = topic_ts_; // Publish timestamp to synchronize frames.
        msg->width = image_[stream_index].cols;
        msg->height = image_[stream_index].rows;
        msg->is_bigendian = false;
        msg->step = step_[stream_index];

        camera_info_ptr_[stream_index]->header.stamp = msg->header.stamp;
        camera_publisher_[stream_index].publish (msg, camera_info_ptr_[stream_index]);
      }
      else
      {
        if ((stream_index == RS_STREAM_DEPTH) || (stream_index == RS_STREAM_COLOR))
        {
          duplicate_depth_color_ = true; // Set this flag to true if Depth and/or Color frame is duplicate
        }
      }
      ts_[stream_index] = current_ts;
    }
  }

  /*
   * Publish pointcloud topic.
   */
  void BaseNodelet::publishPCTopic()
  {
    cv::Mat & image_color = image_[RS_STREAM_COLOR];
    // Publish pointcloud only if there is at least one subscriber.
    if (pointcloud_publisher_.getNumSubscribers() > 0)
    {
      rs_intrinsics color_intrinsic;
      rs_extrinsics z_extrinsic;

      rs_intrinsics z_intrinsic;
      rs_get_stream_intrinsics(rs_device_, RS_STREAM_DEPTH, &z_intrinsic, &rs_error_);
      checkError();

      if (enable_[RS_STREAM_COLOR] == true)
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
      msg_pointcloud.header.stamp = topic_ts_;
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

          if (enable_[RS_STREAM_COLOR] == true)
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
   * Prepare and publish transforms.
   */
  void BaseNodelet::prepareTransforms()
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

    ros::Duration sleeper(0.1); // 100ms

    while (ros::ok())
    {
      // time stamp is future dated to be valid for given duration
      ros::Time transform_ts = ros::Time::now() + sleeper;

      // transform base frame to depth frame
      tr.setOrigin(tf::Vector3(z_extrinsic.translation[2], -z_extrinsic.translation[0], -z_extrinsic.translation[1]));
      tr.setRotation(tf::Quaternion(0, 0, 0, 1));
      tf_broadcaster.sendTransform(tf::StampedTransform(tr, transform_ts, base_frame_id_, depth_frame_id_));

      // transform depth frame to depth optical frame
      tr.setOrigin(tf::Vector3(0,0,0));
      q.setEuler( M_PI/2, 0.0, -M_PI/2 );
      tr.setRotation( q );
      tf_broadcaster.sendTransform(tf::StampedTransform(tr, transform_ts, depth_frame_id_, frame_id_[RS_STREAM_DEPTH]));

      // transform base frame to color frame (these are the same)
      tr.setOrigin(tf::Vector3(0,0,0));
      tr.setRotation(tf::Quaternion(0, 0, 0, 1));
      tf_broadcaster.sendTransform(tf::StampedTransform(tr, transform_ts, base_frame_id_, color_frame_id_));

      // transform color frame to color optical frame
      tr.setOrigin(tf::Vector3(0,0,0));
      q.setEuler( M_PI/2, 0.0, -M_PI/2 );
      tr.setRotation( q );
      tf_broadcaster.sendTransform(tf::StampedTransform(tr, transform_ts, color_frame_id_, frame_id_[RS_STREAM_COLOR]));

      sleeper.sleep(); // need sleep or transform won't publish correctly
    }
  }

  /*
   * Display error details and shutdown ROS.
   */
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
}  // end namespace
