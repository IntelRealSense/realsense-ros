// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once
#include <librealsense/rs.hpp>
#include <librealsense/object_recognition/or_data_interface.h>
#include <librealsense/object_recognition/or_configuration_interface.h>
#include <librealsense/object_recognition/or_video_module_impl.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>


class ros_camera_utils
{
public:
  ros_camera_utils();
  ~ros_camera_utils();

  void get_intrinsics(const sensor_msgs::CameraInfo::ConstPtr & cameraInfo, rs::intrinsics& intrins);
  void get_extrinsics(const sensor_msgs::CameraInfo::ConstPtr & cameraInfo, rs::extrinsics& extrins);
  rs::core::status init_or(const rs::intrinsics& colorInt, const rs::intrinsics& depthInt, const rs::extrinsics& ext, rs::object_recognition::or_video_module_impl& impl,
                           rs::object_recognition::or_data_interface** or_data, rs::object_recognition::or_configuration_interface** or_configuration);

  rs::core::correlated_sample_set* get_sample_set(rs::core::image_info& colorInfo, rs::core::image_info& depthInfo,
      const void* colorBuffer, const void*depthBuffer, int frameNumber, uint64_t timestamp);

private:

  void release_images();

  rs::core::correlated_sample_set* m_sample_set;

};
