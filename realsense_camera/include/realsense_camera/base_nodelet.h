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

#pragma once
#ifndef REALSENSE_CAMERA_BASE_NODELET_H
#define REALSENSE_CAMERA_BASE_NODELET_H

#include <iostream>
#include <boost/thread.hpp>
#include <boost/algorithm/string/join.hpp>
#include <thread>  // NOLINT(build/c++11)
#include <cstdlib>
#include <cctype>
#include <algorithm>
#include <sstream>
#include <memory>
#include <map>
#include <queue>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <librealsense/rs.hpp>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <realsense_camera/CameraConfiguration.h>
#include <realsense_camera/IsPowered.h>
#include <realsense_camera/SetPower.h>
#include <realsense_camera/ForcePower.h>
#include <realsense_camera/constants.h>

namespace realsense_camera
{
class BaseNodelet: public nodelet::Nodelet
{
public:
  // Interfaces.
  virtual void onInit();
  virtual ~BaseNodelet();
  virtual void setDepthEnable(bool &enable_depth);
  virtual bool getCameraOptionValues(realsense_camera::CameraConfiguration::Request & req,
      realsense_camera::CameraConfiguration::Response & res);
  virtual bool setPowerCameraService(realsense_camera::SetPower::Request & req,
      realsense_camera::SetPower::Response & res);
  virtual bool forcePowerCameraService(realsense_camera::ForcePower::Request & req,
      realsense_camera::ForcePower::Response & res);
  virtual bool isPoweredCameraService(realsense_camera::IsPowered::Request & req,
      realsense_camera::IsPowered::Response & res);

protected:
  // Member Variables.
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Time camera_start_ts_;
  ros::Publisher pointcloud_publisher_;
  ros::ServiceServer get_options_service_;
  ros::ServiceServer set_power_service_;
  ros::ServiceServer force_power_service_;
  ros::ServiceServer is_powered_service_;
  rs_error *rs_error_ = NULL;
  rs_context *rs_context_ = NULL;
  rs_device *rs_device_;
  std::string nodelet_name_;
  std::string serial_no_;
  std::string usb_port_id_;
  std::string camera_type_;
  std::string mode_;
  bool enable_[STREAM_COUNT] = {false};
  int width_[STREAM_COUNT];
  int height_[STREAM_COUNT];
  int fps_[STREAM_COUNT];
  rs_format format_[STREAM_COUNT];
  std::string encoding_[STREAM_COUNT];
  int cv_type_[STREAM_COUNT];
  int unit_step_size_[STREAM_COUNT];
  int step_[STREAM_COUNT];
  double ts_[STREAM_COUNT];
  std::string frame_id_[STREAM_COUNT];
  std::string optical_frame_id_[STREAM_COUNT];
  cv::Mat image_[STREAM_COUNT] = {};
  image_transport::CameraPublisher camera_publisher_[STREAM_COUNT] = {};
  sensor_msgs::CameraInfoPtr camera_info_ptr_[STREAM_COUNT] = {};
  std::string base_frame_id_;
  float max_z_ = -1.0f;
  bool enable_pointcloud_;
  bool enable_tf_;
  bool enable_tf_dynamic_;
  double tf_publication_rate_;
  const uint16_t *image_depth16_;
  cv::Mat cvWrapper_;
  std::mutex frame_mutex_[STREAM_COUNT];

  boost::shared_ptr<boost::thread> transform_thread_;
  ros::Time transform_ts_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  tf::TransformBroadcaster dynamic_tf_broadcaster_;
  rs_extrinsics color2depth_extrinsic_;  // color frame is base frame
  rs_extrinsics color2ir_extrinsic_;     // color frame is base frame
  rs_source rs_source_ = RS_SOURCE_VIDEO;
  bool start_camera_ = true;
  bool start_stop_srv_called_ = false;

  struct CameraOptions
  {
    rs_option opt;
    double min, max, step, value;
  };
  std::vector<CameraOptions> camera_options_;

  std::queue<pid_t> system_proc_groups_;

  // Member Functions.
  virtual void getParameters();
  virtual bool connectToCamera();
  virtual std::vector<int> listCameras(int num_of_camera);
  virtual void advertiseTopics();
  virtual void advertiseServices();
  virtual std::vector<std::string> setDynamicReconfServer() { return {}; }  // must be defined in derived class
  virtual void startDynamicReconfCallback() { return; }  // must be defined in derived class
  virtual void getCameraOptions();
  virtual void setStaticCameraOptions(std::vector<std::string> dynamic_params);
  virtual void setStreams();
  virtual void enableStream(rs_stream stream_index, int width, int height, rs_format format, int fps);
  virtual void getStreamCalibData(rs_stream stream_index);
  virtual void disableStream(rs_stream stream_index);
  virtual std::string startCamera();
  virtual std::string stopCamera();
  virtual ros::Time getTimestamp(rs_stream stream_index, double frame_ts);
  virtual void publishTopic(rs_stream stream_index, rs::frame &  frame);
  virtual void setImageData(rs_stream stream_index, rs::frame &  frame);
  virtual void publishPCTopic();
  virtual void getCameraExtrinsics();
  virtual void publishStaticTransforms();
  virtual void publishDynamicTransforms();
  virtual void prepareTransforms();
  virtual void checkError();
  virtual bool checkForSubscriber();
  virtual void wrappedSystem(std::vector<std::string> string_argv);
  virtual void setFrameCallbacks();
  virtual std::string checkFirmwareValidation(std::string fw_type, std::string current_fw, std::string camera_name,
        std::string camera_serial_number);
  std::function<void(rs::frame f)> depth_frame_handler_, color_frame_handler_, ir_frame_handler_;
};
}  // namespace realsense_camera
#endif  // REALSENSE_CAMERA_BASE_NODELET_H
