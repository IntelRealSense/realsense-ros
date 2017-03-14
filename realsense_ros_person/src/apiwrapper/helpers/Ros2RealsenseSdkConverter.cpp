// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <ros/ros.h>
#include <rs/core/projection_interface.h>
#include <nodelet/nodelet.h>
#include "Ros2RealsenseSdkConverter.h"

namespace realsense_ros_person
{
rs::core::image_interface* Ros2RealSenseSdkConverter::CreateSdkImage(rs::core::stream_type streamType, const sensor_msgs::ImageConstPtr &imageRos)
{
  int width = imageRos->width;
  int height = imageRos->height;
  rs::core::pixel_format  pixelFormat = RosImageFormat2RealSenseSdkFormat(imageRos->encoding);
  rs::core::image_info info =
  {
    width,
    height,
    pixelFormat,
    static_cast<int32_t>(imageRos->step)
  };

  return rs::core::image_interface::create_instance_from_raw_data(
           &info,
           imageRos->data.data(),
           streamType,
           rs::core::image_interface::flag::any,
           0,
           0);
}

rs::core::pixel_format Ros2RealSenseSdkConverter::RosImageFormat2RealSenseSdkFormat(std::string rosImageFormat)
{
  if (rosImageFormat.compare("mono8") == 0)
  {
    return rs::core::pixel_format::raw8;
  }
  if (rosImageFormat.compare("16UC1") == 0)
  {
    return rs::core::pixel_format::z16;
  }
  else if (rosImageFormat.compare("bgr8") == 0)
  {
    return rs::core::pixel_format::bgr8;
  }
  else if (rosImageFormat.compare("bgra8") == 0)
  {
    return rs::core::pixel_format::bgra8;
  }
  else if (rosImageFormat.compare("rgb8") == 0)
  {
    return rs::core::pixel_format::rgb8;
  }
  else if (rosImageFormat.compare("rgba8") == 0)
  {
    return rs::core::pixel_format::rgba8;
  }
  ROS_ERROR_STREAM("Unsupported ROS image format: " << rosImageFormat);
  throw std::runtime_error(std::string("Unsupported ROS image format"));
}

rs::core::correlated_sample_set Ros2RealSenseSdkConverter::CreateSdkSampleSet(
  const sensor_msgs::ImageConstPtr &colorImageRos,
  const sensor_msgs::ImageConstPtr &depthImageRos)
{
  rs::core::correlated_sample_set sampleSet;
  sampleSet[rs::core::stream_type::color] = CreateSdkImage(rs::core::stream_type::color, colorImageRos);
  sampleSet[rs::core::stream_type::depth] = CreateSdkImage(rs::core::stream_type::color, depthImageRos);
  return sampleSet;
}

void Ros2RealSenseSdkConverter::ReleaseSampleSet(rs::core::correlated_sample_set &sampleSet)
{
  //release images
  for (uint32_t i = 0; i < static_cast<uint8_t>(rs::core::stream_type::max); ++i)
  {
    rs::core::image_interface* image = sampleSet.images[i];
    if (image)
    {
      image->release();
    }
  }
}

rs::core::video_module_interface::actual_module_config
Ros2RealSenseSdkConverter::CreateSdkModuleConfig(const sensor_msgs::CameraInfoConstPtr &colorCameraInfo,
    const sensor_msgs::CameraInfoConstPtr &depthCameraInfo)
{

  rs::core::intrinsics  depthIntrinsics, colorIntrinsics;
  rs::core::extrinsics  extrinsics;

  colorIntrinsics = CreateSdkIntrinsics(colorCameraInfo);
  depthIntrinsics = CreateSdkIntrinsics(depthCameraInfo);
  std::copy(depthCameraInfo->R.begin(), depthCameraInfo->R.end(), extrinsics.rotation);
  extrinsics.translation[0] = depthCameraInfo->P[3];
  extrinsics.translation[1] = depthCameraInfo->P[7];
  extrinsics.translation[2] = depthCameraInfo->P[11];


  rs::core::video_module_interface::actual_module_config actualModuleConfig = {};
  std::vector<rs::core::stream_type> possible_streams = {rs::core::stream_type::depth,
                                                         rs::core::stream_type::color //person tracking uses only color & depth
                                                        };
  std::map<rs::core::stream_type, rs::core::intrinsics> intrinsics;
  intrinsics[rs::core::stream_type::depth] = depthIntrinsics;
  intrinsics[rs::core::stream_type::color] = colorIntrinsics;

  //fill stream configurations
  for (auto &stream : possible_streams)
  {
    rs::core::video_module_interface::actual_image_stream_config &actualStreamConfig = actualModuleConfig[stream];
    actualStreamConfig.size.width = intrinsics[stream].width;
    actualStreamConfig.size.height = intrinsics[stream].height;
    actualStreamConfig.frame_rate = 30;
    actualStreamConfig.intrinsics = intrinsics[stream];
    actualStreamConfig.extrinsics = extrinsics;
    actualStreamConfig.is_enabled = true;
  }

  //create projection object
  actualModuleConfig.projection = rs::core::projection_interface::create_instance(&colorIntrinsics, &depthIntrinsics, &extrinsics);
  return actualModuleConfig;
}

rs::core::intrinsics
Ros2RealSenseSdkConverter::CreateSdkIntrinsics(const sensor_msgs::CameraInfoConstPtr &rosCameraInfo)
{
  rs::core::intrinsics intrinsics;
  intrinsics.width = rosCameraInfo->width;
  intrinsics.height = rosCameraInfo->height;
  intrinsics.fx = rosCameraInfo->K[0];
  intrinsics.fy = rosCameraInfo->K[4];
  intrinsics.ppx = rosCameraInfo->K[2];
  intrinsics.ppy = rosCameraInfo->K[5];
  std::copy(rosCameraInfo->D.begin(), rosCameraInfo->D.end(), intrinsics.coeffs);
  std::string distortion_model = rosCameraInfo->distortion_model;
  if (distortion_model.compare("Modified_Brown-Conrady") == 0)
  {
    intrinsics.model = rs::core::distortion_type::modified_brown_conrady;
  }
  else
  {
    intrinsics.model = rs::core::distortion_type::none;
  }
  return intrinsics;
}
}
