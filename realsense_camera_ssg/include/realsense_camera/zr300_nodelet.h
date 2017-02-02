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

#pragma once
#ifndef REALSENSE_CAMERA_ZR300_NODELET_H
#define REALSENSE_CAMERA_ZR300_NODELET_H

#include <string>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Imu.h>

#include <realsense_camera/zr300_paramsConfig.h>
#include <realsense_camera/IMUInfo.h>
#include <realsense_camera/GetIMUInfo.h>
#include <realsense_camera/base_nodelet.h>

namespace realsense_camera
{
class ZR300Nodelet: public realsense_camera::BaseNodelet
{
public:
  ~ZR300Nodelet();
  void onInit();

protected:
  // Member Variables.
  ros::ServiceServer get_imu_info_;
  boost::shared_ptr<dynamic_reconfigure::Server<realsense_camera::zr300_paramsConfig>> dynamic_reconf_server_;
  bool enable_imu_;
  std::string imu_frame_id_;
  std::string imu_optical_frame_id_;
  float imu_angular_vel_[3];
  float imu_linear_accel_[3];
  float imu_angular_vel_cov_[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float imu_linear_accel_cov_[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double imu_ts_;
  double prev_imu_ts_;
  ros::Publisher imu_publisher_;
  boost::shared_ptr<boost::thread> imu_thread_;
  std::function<void(rs::motion_data)> motion_handler_;
  std::function<void(rs::timestamp_data)> timestamp_handler_;
  std::mutex imu_mutex_;

  rs_extrinsics color2ir2_extrinsic_;      // color frame is base frame
  rs_extrinsics color2fisheye_extrinsic_;  // color frame is base frame
  rs_extrinsics color2imu_extrinsic_;      // color frame is base frame

  // Member Functions.
  void getParameters();
  void advertiseTopics();
  void advertiseServices();
  bool getIMUInfo(realsense_camera::GetIMUInfo::Request & req, realsense_camera::GetIMUInfo::Response & res);
  std::vector<std::string> setDynamicReconfServer();
  void startDynamicReconfCallback();
  void setDynamicReconfigDepthControlPreset(int preset);
  std::string setDynamicReconfigDepthControlIndividuals();
  void configCallback(realsense_camera::zr300_paramsConfig &config, uint32_t level);
  void getCameraExtrinsics();
  void publishStaticTransforms();
  void publishDynamicTransforms();
  void publishIMU();
  void setStreams();
  void setIMUCallbacks();
  void setFrameCallbacks();
  std::function<void(rs::frame f)> fisheye_frame_handler_, ir2_frame_handler_;
  void stopIMU();
};
}  // namespace realsense_camera
#endif  // REALSENSE_CAMERA_ZR300_NODELET_H
