// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <rs/core/correlated_sample_set.h>
#include <rs/core/video_module_interface.h>

namespace realsense_ros_person
{
class Ros2RealSenseSdkConverter
{
public:
  /**
   * @brief Create Realsense SDK image from ROS image
   * @param imageRos ros image
   * @return RealSense sdk image, IMPORTANT shalow copy - image memory managed by ImageConstPtr
   */
  rs::core::image_interface* CreateSdkImage(rs::core::stream_type streamType, const sensor_msgs::ImageConstPtr &imageRos);

  /**
   * @brief Create RealSense SDK sample set from ROS images
   * @param colorImageRos color image
   * @param depthImageRos depth image
   * @return RealSense SDK sample set, IMPORTANT images memory managed by ImageConstPtr & ImageConstPtr,
   * sample set should be release with ReleaseSampleSet method
   */
  rs::core::correlated_sample_set CreateSdkSampleSet(const sensor_msgs::ImageConstPtr& colorImageRos,
      const sensor_msgs::ImageConstPtr& depthImageRos);

  /**
   * @brief Release RealSense SDK sample set - release image objects
   * @param sampleSet RealSense SDK sample set
   */
  void ReleaseSampleSet(rs::core::correlated_sample_set& sampleSet);

  /**
   * @brief Create RealSense SDK module config from ROS camera parameters
   * @param colorCameraInfo color camera info
   * @param depthCameraInfo depth camera info
   * @return RealSense SDK module config
   */
  rs::core::video_module_interface::actual_module_config CreateSdkModuleConfig(const sensor_msgs::CameraInfoConstPtr& colorCameraInfo,
      const sensor_msgs::CameraInfoConstPtr& depthCameraInfo);

  /**
   * @brief Convert ROS image format(e.g. rgba8) to RealSense SDK format
   * @param rosImageFormat
   * @return RealSense SDK pixel format
   */
  rs::core::pixel_format RosImageFormat2RealSenseSdkFormat(std::string rosImageFormat);

  /**
   * @brief Create RealSense SDK intrinsics from ROS camera info
   * @param rosCameraInfo
   * @return RealSense SDK intrinsics
   */
  rs::core::intrinsics CreateSdkIntrinsics(const sensor_msgs::CameraInfoConstPtr& rosCameraInfo);
};
}


