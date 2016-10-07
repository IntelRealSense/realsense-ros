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

#include <realsense_camera/base_nodelet.h>

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

    while (false == connectToCamera()) // Poll for camera and connect if found
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

    // Start publishing tranforms
    if (enable_tf_ == true)
    {
      publishStaticTransforms();
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
    pnh_.param("base_frame_id", base_frame_id_, DEFAULT_BASE_FRAME_ID);
    pnh_.param("depth_frame_id", frame_id_[RS_STREAM_DEPTH], DEFAULT_DEPTH_FRAME_ID);
    pnh_.param("color_frame_id", frame_id_[RS_STREAM_COLOR], DEFAULT_COLOR_FRAME_ID);
    pnh_.param("ir_frame_id", frame_id_[RS_STREAM_INFRARED], DEFAULT_IR_FRAME_ID);
    pnh_.param("depth_optical_frame_id", optical_frame_id_[RS_STREAM_DEPTH], DEFAULT_DEPTH_OPTICAL_FRAME_ID);
    pnh_.param("color_optical_frame_id", optical_frame_id_[RS_STREAM_COLOR], DEFAULT_COLOR_OPTICAL_FRAME_ID);
    pnh_.param("ir_optical_frame_id", optical_frame_id_[RS_STREAM_INFRARED], DEFAULT_IR_OPTICAL_FRAME_ID);
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
      error_msg += "usb_port_id = " + usb_port_id_;

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
      // print camera details
      detected_camera_msg = detected_camera_msg +
            "\n\t\t\t\t- Serial No: " + rs_get_device_serial(rs_detected_device, &rs_error_) +
            ", USB Port ID: " + rs_get_device_usb_port_id(rs_detected_device, &rs_error_) +
            ", Name: " + camera_name +
            ", Camera FW: " + rs_get_device_firmware_version(rs_detected_device, &rs_error_);
      checkError();

      if (rs_supports(rs_detected_device, RS_CAPABILITIES_ADAPTER_BOARD, &rs_error_))
      {
        const char * adapter_fw = rs_get_device_info(rs_detected_device, RS_CAMERA_INFO_ADAPTER_BOARD_FIRMWARE_VERSION,
            &rs_error_);
        checkError();
        detected_camera_msg = detected_camera_msg + ", Adapter FW: " + adapter_fw;
      }

      if (rs_supports(rs_detected_device, RS_CAPABILITIES_MOTION_EVENTS, &rs_error_))
      {
        const char * motion_module_fw = rs_get_device_info(rs_detected_device, RS_CAMERA_INFO_MOTION_MODULE_FIRMWARE_VERSION,
            &rs_error_);
        checkError();
        detected_camera_msg = detected_camera_msg + ", Motion Module FW: " + motion_module_fw;
      }
    }

    ROS_INFO_STREAM(nodelet_name_ + detected_camera_msg);
    return camera_type_index;
  }

  /*
   * Advertise topics.
   */
  void BaseNodelet::advertiseTopics()
  {
    ros::NodeHandle color_nh(nh_, COLOR_NAMESPACE);
    image_transport::ImageTransport color_image_transport(color_nh);
    camera_publisher_[RS_STREAM_COLOR] = color_image_transport.advertiseCamera(COLOR_TOPIC, 1);

    ros::NodeHandle depth_nh(nh_, DEPTH_NAMESPACE);
    image_transport::ImageTransport depth_image_transport(depth_nh);
    camera_publisher_[RS_STREAM_DEPTH] = depth_image_transport.advertiseCamera(DEPTH_TOPIC, 1);
    pointcloud_publisher_ = depth_nh.advertise<sensor_msgs::PointCloud2>(PC_TOPIC, 1);

    ros::NodeHandle ir_nh(nh_, IR_NAMESPACE);
    image_transport::ImageTransport ir_image_transport(ir_nh);
    camera_publisher_[RS_STREAM_INFRARED] = ir_image_transport.advertiseCamera(IR_TOPIC, 1);
  }

  /*
   * Advertise services.
   */
  void BaseNodelet::advertiseServices()
  {
    get_options_service_ = pnh_.advertiseService(SETTINGS_SERVICE, &BaseNodelet::getCameraOptionValues, this);
    set_power_service_ = pnh_.advertiseService(CAMERA_SET_POWER_SERVICE, &BaseNodelet::setPowerCameraService, this);
    force_power_service_ = pnh_.advertiseService(CAMERA_FORCE_POWER_SERVICE, &BaseNodelet::forcePowerCameraService, this);
    is_powered_service_ = pnh_.advertiseService(CAMERA_IS_POWERED_SERVICE, &BaseNodelet::isPoweredCameraService, this);
  }

  /*
   * Get the latest values of the camera options.
   */
  bool BaseNodelet::getCameraOptionValues(realsense_camera::CameraConfiguration::Request & req,
      realsense_camera::CameraConfiguration::Response & res)
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
   * Check for Nodelet subscribers
   */
  bool BaseNodelet::checkForSubscriber()
  {
    for (int index=0; index < STREAM_COUNT; index++)
    {
      if (camera_publisher_[index].getNumSubscribers() > 0)
      {
        return true;
      }
    }
    if (pointcloud_publisher_.getNumSubscribers() > 0)
    {
        return true;
    }
    return false;
  }

  /*
   * Service to check if camera is powered on or not
   */
  bool BaseNodelet::isPoweredCameraService(realsense_camera::IsPowered::Request & req,
      realsense_camera::IsPowered::Response & res)
  {
      if (rs_is_device_streaming(rs_device_, 0) == 1)
      {
        res.is_powered = true;
      }
      else
      {
        res.is_powered = false;
      }
      return true;
  }

  /*
   * Set Power Camera service
   */
  bool BaseNodelet::setPowerCameraService(realsense_camera::SetPower::Request & req,
      realsense_camera::SetPower::Response & res)
  {
    res.success = true;

    if (req.power_on == true)
    {
      start_camera_ = true;
      start_stop_srv_called_ = true;
    }
    else
    {
      if (rs_is_device_streaming(rs_device_, 0) == 0)
      {
        ROS_INFO_STREAM(nodelet_name_ << " - Camera is already Stopped");
      }
      else
      {
        if (checkForSubscriber() == false)
        {
          start_camera_ = false;
          start_stop_srv_called_ = true;
        }
        else
        {
          ROS_INFO_STREAM(nodelet_name_ << " - Cannot stop the camera. Nodelet has subscriber.");
          res.success = false;
        }
      }
    }
    return res.success;
  }


  /*
   * Force Power Camera service
   */
  bool BaseNodelet::forcePowerCameraService(realsense_camera::ForcePower::Request & req,
      realsense_camera::ForcePower::Response & res)
  {
    start_camera_ = req.power_on;
    start_stop_srv_called_ = true;
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
      enableStream(RS_STREAM_COLOR, width_[RS_STREAM_COLOR], height_[RS_STREAM_COLOR], format_[RS_STREAM_COLOR],
          fps_[RS_STREAM_COLOR]);
      if (camera_info_ptr_[RS_STREAM_COLOR] == NULL)
      {
        ROS_DEBUG_STREAM(nodelet_name_ << " - Allocating resources for " << STREAM_DESC[RS_STREAM_COLOR]);
        getStreamCalibData(RS_STREAM_COLOR);
        step_[RS_STREAM_COLOR] = camera_info_ptr_[RS_STREAM_COLOR]->width * unit_step_size_[RS_STREAM_COLOR];
        image_[RS_STREAM_COLOR] = cv::Mat(camera_info_ptr_[RS_STREAM_COLOR]->height,
            camera_info_ptr_[RS_STREAM_COLOR]->width, cv_type_[RS_STREAM_COLOR], cv::Scalar(0, 0, 0));
      }
      ts_[RS_STREAM_COLOR] = -1;
    }
    else if (enable_[RS_STREAM_COLOR] == false)
    {
      disableStream(RS_STREAM_COLOR);
    }

    if (enable_[RS_STREAM_DEPTH] == true)
    {
      enableStream(RS_STREAM_DEPTH, width_[RS_STREAM_DEPTH], height_[RS_STREAM_DEPTH], format_[RS_STREAM_DEPTH],
          fps_[RS_STREAM_DEPTH]);
      if (camera_info_ptr_[RS_STREAM_DEPTH] == NULL)
      {
        ROS_DEBUG_STREAM(nodelet_name_ << " - Allocating resources for " << STREAM_DESC[RS_STREAM_DEPTH]);
        getStreamCalibData(RS_STREAM_DEPTH);
        step_[RS_STREAM_DEPTH] = camera_info_ptr_[RS_STREAM_DEPTH]->width * unit_step_size_[RS_STREAM_DEPTH];
        image_[RS_STREAM_DEPTH] = cv::Mat(camera_info_ptr_[RS_STREAM_DEPTH]->height,
            camera_info_ptr_[RS_STREAM_DEPTH]->width, cv_type_[RS_STREAM_DEPTH], cv::Scalar(0, 0, 0));
      }
      ts_[RS_STREAM_DEPTH] = -1;

      enableStream(RS_STREAM_INFRARED, width_[RS_STREAM_DEPTH], height_[RS_STREAM_DEPTH], format_[RS_STREAM_INFRARED],
          fps_[RS_STREAM_DEPTH]);
      if (camera_info_ptr_[RS_STREAM_INFRARED] == NULL)
      {
        ROS_DEBUG_STREAM(nodelet_name_ << " - Allocating resources for " << STREAM_DESC[RS_STREAM_INFRARED]);
        getStreamCalibData(RS_STREAM_INFRARED);
        step_[RS_STREAM_INFRARED] = camera_info_ptr_[RS_STREAM_INFRARED]->width * unit_step_size_[RS_STREAM_INFRARED];
        image_[RS_STREAM_INFRARED] = cv::Mat(camera_info_ptr_[RS_STREAM_INFRARED]->height,
            camera_info_ptr_[RS_STREAM_INFRARED]->width, cv_type_[RS_STREAM_INFRARED], cv::Scalar(0, 0, 0));
      }
      ts_[RS_STREAM_INFRARED] = -1;
      depth_scale_meters_ = rs_get_device_depth_scale(rs_device_, &rs_error_);
      checkError();
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

    camera_info->header.frame_id = optical_frame_id_[stream_index];
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
    camera_info->R.at(0) = 1.0;
    camera_info->R.at(1) = 0.0;
    camera_info->R.at(2) = 0.0;
    camera_info->R.at(3) = 0.0;
    camera_info->R.at(4) = 1.0;
    camera_info->R.at(5) = 0.0;
    camera_info->R.at(6) = 0.0;
    camera_info->R.at(7) = 0.0;
    camera_info->R.at(8) = 1.0;

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
  std::string BaseNodelet::startCamera()
  {
    if (rs_is_device_streaming(rs_device_, 0) == 0)
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Starting camera");
      rs_start_device(rs_device_, &rs_error_);
      checkError();
      return "Camera Started Succesfully";
    }
    return "Camera is already Started";
  }

  /*
   * Stop camera.
   */
  std::string BaseNodelet::stopCamera()
  {
    if (rs_is_device_streaming(rs_device_, 0) == 1)
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Stopping camera");
      rs_stop_device(rs_device_, 0);
      checkError();
      return "Camera Stopped Succesfully";
    }
    return "Camera is already Stopped";
  }


  /*
   * Copy frame data from realsense to member cv images.
   */
  void BaseNodelet::getStreamData(rs_stream stream_index)
  {
    if (stream_index == RS_STREAM_DEPTH)
    {
      // fill depth buffer
      image_depth16_ = reinterpret_cast<const uint16_t *>(rs_get_frame_data(rs_device_, stream_index, 0));
      if (depth_scale_meters_ == MILLIMETER_METERS)
      {
        image_[stream_index].data = (unsigned char *) (rs_get_frame_data(rs_device_, stream_index, 0));
      }
      else
      {
        cvWrapper_ = cv::Mat(image_[stream_index].size(), cv_type_[stream_index], (void *) image_depth16_,
              step_[stream_index]);
        cvWrapper_.convertTo(image_[stream_index], cv_type_[stream_index],
              static_cast<double>(depth_scale_meters_) / static_cast<double>(MILLIMETER_METERS));
      }
    }
    else
    {
      image_[stream_index].data = (unsigned char *) (rs_get_frame_data(rs_device_, stream_index, 0));
    }
  }

  /*
   * Prepare and publish topics.
   */
  void BaseNodelet::prepareTopics()
  {
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
      double current_ts = rs_get_frame_timestamp(rs_device_, (rs_stream) stream_index, 0);
      if (ts_[stream_index] != current_ts) // Publish frames only if its not duplicate
      {
        getStreamData(stream_index);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
            encoding_[stream_index],
            image_[stream_index]).toImageMsg();

        msg->header.frame_id = optical_frame_id_[stream_index];
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
      checkError();// Default value is 0.001

      // Fill the PointCloud2 fields.
      for (int y = 0; y < z_intrinsic.height; y++)
      {
        for (int x = 0; x < z_intrinsic.width; x++)
        {
          scaled_depth = static_cast<float>(*image_depth16_) * depth_scale_meters_;
          float depth_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
          rs_deproject_pixel_to_point(depth_point, &z_intrinsic, depth_pixel, scaled_depth);

          if (depth_point[2] <= 0.0f || depth_point[2] > max_z_)
          {
            depth_point[0] = 0.0f;
            depth_point[1] = 0.0f;
            depth_point[2] = 0.0f;
          }

          *iter_x = depth_point[0];
          *iter_y = depth_point[1];
          *iter_z = depth_point[2];

          // Default to white color.
          *iter_r = static_cast<uint8_t>(255);
          *iter_g = static_cast<uint8_t>(255);
          *iter_b = static_cast<uint8_t>(255);

          if (enable_[RS_STREAM_COLOR] == true)
          {
            rs_transform_point_to_point(color_point, &z_extrinsic, depth_point);
            rs_project_point_to_pixel(color_pixel, &color_intrinsic, color_point);

            if (color_pixel[1] < 0.0f || color_pixel[1] > image_color.rows
                || color_pixel[0] < 0.0f || color_pixel[0] > image_color.cols)
            {
              // For out of bounds color data, default to a shade of blue in order to visually distinguish holes.
              // This color value is same as the librealsense out of bounds color value.
              *iter_r = static_cast<uint8_t>(96);
              *iter_g = static_cast<uint8_t>(157);
              *iter_b = static_cast<uint8_t>(198);
            }
            else
            {
              int i = static_cast<int>(color_pixel[0]);
              int j = static_cast<int>(color_pixel[1]);

              *iter_r = static_cast<uint8_t>(color_data[i * 3 + j * image_color.cols * 3]);
              *iter_g = static_cast<uint8_t>(color_data[i * 3 + j * image_color.cols * 3 + 1]);
              *iter_b = static_cast<uint8_t>(color_data[i * 3 + j * image_color.cols * 3 + 2]);
            }
          }

          image_depth16_++;
          ++iter_x; ++iter_y; ++iter_z; ++iter_r; ++iter_g; ++iter_b;
        }
      }

      msg_pointcloud.header.frame_id = optical_frame_id_[RS_STREAM_DEPTH];
      pointcloud_publisher_.publish(msg_pointcloud);
    }
  }

  /*
   * Prepare and publish transforms.
   */
  void BaseNodelet::publishStaticTransforms()
  {
    // Publish transforms for the cameras
    ROS_INFO_STREAM(nodelet_name_ << " - Publishing camera transforms");

    tf::Quaternion q_c2co;
    tf::Quaternion q_d2do;
    tf::Quaternion q_i2io;
    rs_extrinsics z_extrinsic;
    geometry_msgs::TransformStamped b2d_msg;
    geometry_msgs::TransformStamped d2do_msg;
    geometry_msgs::TransformStamped b2c_msg;
    geometry_msgs::TransformStamped c2co_msg;
    geometry_msgs::TransformStamped b2i_msg;
    geometry_msgs::TransformStamped i2io_msg;

    // Get the current timestamp for all static transforms
    static_transform_ts_ = ros::Time::now();

    // The color frame is used as the base frame.
    // Hence no additional transformation is done from base frame to color frame.
    b2c_msg.header.stamp = static_transform_ts_;
    b2c_msg.header.frame_id = base_frame_id_;
    b2c_msg.child_frame_id = frame_id_[RS_STREAM_COLOR];
    b2c_msg.transform.translation.x = 0;
    b2c_msg.transform.translation.y = 0;
    b2c_msg.transform.translation.z = 0;
    b2c_msg.transform.rotation.x = 0;
    b2c_msg.transform.rotation.y = 0;
    b2c_msg.transform.rotation.z = 0;
    b2c_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2c_msg);

    // Transform color frame to color optical frame
    q_c2co.setEuler(M_PI/2, 0.0, -M_PI/2);
    c2co_msg.header.stamp = static_transform_ts_;
    c2co_msg.header.frame_id = frame_id_[RS_STREAM_COLOR];
    c2co_msg.child_frame_id = optical_frame_id_[RS_STREAM_COLOR];
    c2co_msg.transform.translation.x = 0;
    c2co_msg.transform.translation.y = 0;
    c2co_msg.transform.translation.z = 0;
    c2co_msg.transform.rotation.x = q_c2co.getX();
    c2co_msg.transform.rotation.y = q_c2co.getY();
    c2co_msg.transform.rotation.z = q_c2co.getZ();
    c2co_msg.transform.rotation.w = q_c2co.getW();
    static_tf_broadcaster_.sendTransform(c2co_msg);

    // Get offset between base frame and depth frame
    rs_get_device_extrinsics(rs_device_, RS_STREAM_DEPTH, RS_STREAM_COLOR, &z_extrinsic, &rs_error_);
    checkError();

    // Transform base frame to depth frame
    b2d_msg.header.stamp = static_transform_ts_;
    b2d_msg.header.frame_id = base_frame_id_;
    b2d_msg.child_frame_id = frame_id_[RS_STREAM_DEPTH];
    b2d_msg.transform.translation.x =  z_extrinsic.translation[2];
    b2d_msg.transform.translation.y = -z_extrinsic.translation[0];
    b2d_msg.transform.translation.z = -z_extrinsic.translation[1];
    b2d_msg.transform.rotation.x = 0;
    b2d_msg.transform.rotation.y = 0;
    b2d_msg.transform.rotation.z = 0;
    b2d_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2d_msg);

    // Transform depth frame to depth optical frame
    q_d2do.setEuler(M_PI/2, 0.0, -M_PI/2);
    d2do_msg.header.stamp = static_transform_ts_;
    d2do_msg.header.frame_id = frame_id_[RS_STREAM_DEPTH];
    d2do_msg.child_frame_id = optical_frame_id_[RS_STREAM_DEPTH];
    d2do_msg.transform.translation.x = 0;
    d2do_msg.transform.translation.y = 0;
    d2do_msg.transform.translation.z = 0;
    d2do_msg.transform.rotation.x = q_d2do.getX();
    d2do_msg.transform.rotation.y = q_d2do.getY();
    d2do_msg.transform.rotation.z = q_d2do.getZ();
    d2do_msg.transform.rotation.w = q_d2do.getW();
    static_tf_broadcaster_.sendTransform(d2do_msg);

    // Get offset between base frame and infrared frame
    rs_get_device_extrinsics(rs_device_, RS_STREAM_INFRARED, RS_STREAM_COLOR, &z_extrinsic, &rs_error_);
    checkError();

    // Transform base frame to infrared frame
    b2i_msg.header.stamp = static_transform_ts_;
    b2i_msg.header.frame_id = base_frame_id_;
    b2i_msg.child_frame_id = frame_id_[RS_STREAM_INFRARED];
    b2i_msg.transform.translation.x =  z_extrinsic.translation[2];
    b2i_msg.transform.translation.y = -z_extrinsic.translation[0];
    b2i_msg.transform.translation.z = -z_extrinsic.translation[1];
    b2i_msg.transform.rotation.x = 0;
    b2i_msg.transform.rotation.y = 0;
    b2i_msg.transform.rotation.z = 0;
    b2i_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2i_msg);

    // Transform infrared frame to infrared optical frame
    q_i2io.setEuler(M_PI/2, 0.0, -M_PI/2);
    i2io_msg.header.stamp = static_transform_ts_;
    i2io_msg.header.frame_id = frame_id_[RS_STREAM_INFRARED];
    i2io_msg.child_frame_id = optical_frame_id_[RS_STREAM_INFRARED];
    i2io_msg.transform.translation.x = 0;
    i2io_msg.transform.translation.y = 0;
    i2io_msg.transform.translation.z = 0;
    i2io_msg.transform.rotation.x = q_i2io.getX();
    i2io_msg.transform.rotation.y = q_i2io.getY();
    i2io_msg.transform.rotation.z = q_i2io.getZ();
    i2io_msg.transform.rotation.w = q_i2io.getW();
    static_tf_broadcaster_.sendTransform(i2io_msg);
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
