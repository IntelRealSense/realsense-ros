// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include "utils.h"
#include <rs_sdk.h>
ros_camera_utils::ros_camera_utils()
{
  m_sample_set = nullptr;
}

ros_camera_utils::~ros_camera_utils()
{
  release_images();
  delete m_sample_set;
}

void ros_camera_utils::release_images()
{
  if (m_sample_set)
  {
    if (m_sample_set->images[(int)rs::stream::color])
    {
      m_sample_set->images[(int)rs::stream::color]->release();
      m_sample_set->images[(int)rs::stream::color] = nullptr;
    }

    if (m_sample_set->images[(int)rs::stream::depth])
    {
      m_sample_set->images[(int)rs::stream::depth]->release();
      m_sample_set->images[(int)rs::stream::depth] = nullptr;
    }
  }

}
void ros_camera_utils::get_intrinsics(const sensor_msgs::CameraInfo::ConstPtr & cameraInfo, rs::intrinsics& intrins)
{

  intrins.width = cameraInfo->width;    /* width of the image in pixels */
  intrins.height = cameraInfo->height;  /* height of the image in pixels */
  intrins.ppx = cameraInfo->K[2];       /* horizontal coordinate of the principal point of the image, as a pixel offset from the left edge */
  intrins.ppy = cameraInfo->K[5];       /* vertical coordinate of the principal point of the image, as a pixel offset from the top edge */
  intrins.fx = cameraInfo->K[0];        /* focal length of the image plane, as a multiple of pixel width */
  intrins.fy = cameraInfo->K[4];        /* focal length of the image plane, as a multiple of pixel height */
  intrins.rs_intrinsics::model = RS_DISTORTION_MODIFIED_BROWN_CONRADY ;     /* distortion model of the image */


  /* distortion coefficients */
  for (unsigned int i = 0; i < sizeof(cameraInfo->D) / sizeof(double); i++)
  {
    intrins.coeffs[i] = cameraInfo->D[i];
  }

}

void ros_camera_utils::get_extrinsics(const sensor_msgs::CameraInfo::ConstPtr & cameraInfo, rs::extrinsics& extrins)
{
  //get extrinsics
  for (unsigned int i = 0; i < sizeof(cameraInfo->R) / sizeof(double); i++)
  {
    extrins.rotation[i] = cameraInfo->R[i];
  }

  // copy projection matrix

  for (unsigned int i = 3, j = 0; i < sizeof(cameraInfo->P) / sizeof(double); i = i + 4, j++)
  {
    extrins.translation[j] = (float)(cameraInfo->P[i]);
  }


}

rs::core::status ros_camera_utils::init_or(const rs::intrinsics& colorInt, const rs::intrinsics& depthInt, const rs::extrinsics& ext, rs::object_recognition::or_video_module_impl& impl,
    rs::object_recognition::or_data_interface** or_data, rs::object_recognition::or_configuration_interface** or_configuration)
{
  rs::core::status st;

  // after getting all parameters from the camera we need to set the actual_module_config
  rs::core::video_module_interface::actual_module_config actualConfig;
  // get the extrisics paramters from the camera
  rs::core::extrinsics core_ext;

  //get color intrinsics
  rs::core::intrinsics core_colorInt;

  //get depth intrinsics
  rs::core::intrinsics core_depthInt;

  rs::core::video_module_interface::supported_module_config cfg;
  bool supported = false;

  for (int i = 0; impl.query_supported_module_config(i, cfg) == rs::core::status_no_error; i++)
  {
    // add check that the cfg is ok
    if (cfg.image_streams_configs[(int)rs::stream::color].size.width == colorInt.width &&
        cfg.image_streams_configs[(int)rs::stream::color].size.height == colorInt.height &&
        cfg.image_streams_configs[(int)rs::stream::depth].size.width == depthInt.width &&
        cfg.image_streams_configs[(int)rs::stream::depth].size.height == depthInt.height)
    {
      supported = true;
      break;
    }
  }
  if (!supported)
    return rs::core::status_param_unsupported;

  //1. copy the extrinsics
  memcpy(&actualConfig.image_streams_configs[(int)rs::stream::color].extrinsics, &ext, sizeof(rs::extrinsics));
  core_ext =  rs::utils::convert_extrinsics(ext);

  //2. copy the color intrinsics
  memcpy(&actualConfig.image_streams_configs[(int)rs::stream::color].intrinsics, &colorInt, sizeof(rs::intrinsics));
  core_colorInt = rs::utils::convert_intrinsics(colorInt);

  //3. copy the depth intrinsics
  memcpy(&actualConfig.image_streams_configs[(int)rs::stream::depth].intrinsics, &depthInt, sizeof(rs::intrinsics));
  core_depthInt = rs::utils::convert_intrinsics(depthInt);

  // handling projection
  rs::core::projection_interface* proj = rs::core::projection_interface::create_instance(&core_colorInt, &core_depthInt, &core_ext);
  actualConfig.projection = proj;

  //setting the selected configuration (after projection)
  st = impl.set_module_config(actualConfig);
  if (st != rs::core::status_no_error)
  {
    return st;
  }

  //create or data object
  *or_data = impl.create_output();
  //create or data object
  *or_configuration = impl.create_active_configuration();

  m_sample_set = new rs::core::correlated_sample_set();

  m_sample_set->images[(int)rs::stream::color] = nullptr;
  m_sample_set->images[(int)rs::stream::depth] = nullptr;

  return st;
}

rs::core::correlated_sample_set* ros_camera_utils::get_sample_set(rs::core::image_info& colorInfo, rs::core::image_info& depthInfo, const void* colorBuffer, const void*depthBuffer, int frameNumber, uint64_t timestamp)
{


  // release images from the prevoius frame
  release_images();
  //create images from buffers

  rs::core::image_interface::image_data_with_data_releaser color_container(colorBuffer);
  rs::core::image_interface::image_data_with_data_releaser depth_container(depthBuffer);
  auto colorImg = rs::core::image_interface::create_instance_from_raw_data(&colorInfo, color_container, rs::core::stream_type::color, rs::core::image_interface::any, frameNumber, timestamp);
  auto depthImg = rs::core::image_interface::create_instance_from_raw_data(&depthInfo, depth_container, rs::core::stream_type::depth, rs::core::image_interface::any, frameNumber, timestamp);

  //create sample from both images

  m_sample_set->images[(int)rs::stream::color] = colorImg;
  m_sample_set->images[(int)rs::stream::depth] = depthImg;


  return m_sample_set;

}
