/******************************************************************************
 INTEL CORPORATION PROPRIETARY INFORMATION
 This software is supplied under the terms of a license agreement or nondisclosure
 agreement with Intel Corporation and may not be copied or disclosed except in
 accordance with the terms of that agreement
 Copyright(c) 2011-2016 Intel Corporation. All Rights Reserved.
 *******************************************************************************/

#pragma once
#ifndef REALSENSE_NODELET
#define REALSENSE_NODELET

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <iostream>
#include <boost/thread.hpp>
#include <thread>
#include <cstdlib>
#include <cctype>
#include <algorithm>
#include <sstream>
#include <memory>
#include <map>

#include <librealsense/rs.hpp>
#include <realsense_camera/cameraConfiguration.h>

namespace realsense_camera
{
class RealsenseNodelet: public nodelet::Nodelet
{
public:

  // Interfaces.
  virtual void onInit();
  virtual ~ RealsenseNodelet();

  // Default Constants.
  const int MAX_Z = 8;	// in meters
  const int DEFAULT_CAMERA = 0;	// default camera index
  const int DEPTH_HEIGHT = 360;
  const int DEPTH_WIDTH = 480;
  const int COLOR_HEIGHT = 480;
  const int COLOR_WIDTH = 640;
  const int DEPTH_FPS = 60;
  const int COLOR_FPS = 60;
  const bool ENABLE_DEPTH = true;
  const bool ENABLE_COLOR = true;
  const bool ENABLE_PC = true;
  const uint32_t SERIAL_NUMBER = 0xFFFFFFFF;
  const rs_format DEPTH_FORMAT = RS_FORMAT_Z16;
  const rs_format COLOR_FORMAT = RS_FORMAT_RGB8;
  const rs_format IR1_FORMAT = RS_FORMAT_Y8;
  const rs_format IR2_FORMAT = RS_FORMAT_Y8;
  const char *BASE_DEF_FRAME = "realsense_frame";
  const char *DEPTH_DEF_FRAME = "camera_depth_frame";
  const char *COLOR_DEF_FRAME = "camera_color_frame";
  const char *DEPTH_OPTICAL_DEF_FRAME = "camera_depth_optical_frame";
  const char *COLOR_OPTICAL_DEF_FRAME = "camera_color_optical_frame";
  const char *IR1_DEF_FRAME = "camera_infrared_optical_frame";
  const char *IR2_DEF_FRAME = "camera_infrared2_optical_frame";
  const char *DEPTH_TOPIC = "camera/depth/image_raw";
  const char *COLOR_TOPIC = "camera/color/image_raw";
  const char *IR1_TOPIC = "camera/infrared1/image_raw";
  const char *IR2_TOPIC = "camera/infrared2/image_raw";
  const char *PC_TOPIC = "camera/depth/points";
  const char *SETTINGS_SERVICE = "camera/get_settings";
  const char *R200 = "R200";
  const static int STREAM_COUNT = 4;

private:
  // Member Variables.
  boost::shared_ptr<boost::thread> device_thread_;
  boost::shared_ptr<boost::thread> transform_thread_;

  rs_error *rs_error_ = 0;
  rs_context *rs_context_;
  rs_device *rs_device_;

  int color_height_;
  int color_width_;
  int depth_height_;
  int depth_width_;
  int depth_fps_;
  int color_fps_;
  bool is_device_started_;
  bool enable_color_;
  bool enable_depth_;
  bool enable_pointcloud_;
  std::vector<std::string> camera_configuration_;
  std::string camera_ = "R200";
  const uint16_t *image_depth16_;

  cv::Mat image_[STREAM_COUNT];

  sensor_msgs::CameraInfoPtr camera_info_ptr_[STREAM_COUNT];
  sensor_msgs::CameraInfo * camera_info_[STREAM_COUNT];
  image_transport::CameraPublisher camera_publisher_[STREAM_COUNT];

  ros::Time time_stamp_;
  ros::Publisher pointcloud_publisher_;
  ros::ServiceServer get_options_service_;

  std::string frame_id_[STREAM_COUNT];
  std::string stream_encoding_[STREAM_COUNT];
  std::string mode_;
  std::map<std::string, std::string> config_;
  int stream_step_[STREAM_COUNT];

  struct option_str
  {
    rs_option opt;
    double min, max, step, value;
  };
  std::vector<option_str> options;

  // Member Functions.
  void check_error();
  void fetchCalibData();
  void prepareStreamCalibData(rs_stream calib_data);
  void prepareStreamData(rs_stream rs_strm);
  void publishStreams();
  void publishPointCloud(cv::Mat & image_rgb);
  void publishTransforms();
  void devicePoll();
  void allocateResources();
  bool connectToCamera();
  void fillStreamEncoding();
  void getConfigValues(std::vector<std::string> args);
  void setConfigValues(std::vector<std::string> args, std::vector<struct option_str> cam_options);
  bool getCameraSettings(realsense_camera::cameraConfiguration::Request & req, realsense_camera::cameraConfiguration::Response & res);

};
}

#endif
