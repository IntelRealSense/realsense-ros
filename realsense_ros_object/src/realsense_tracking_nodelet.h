// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once
# ifndef RS_TRACKING_NODELET
# define RS_TRACKING_NODELET

///////////////////////////////////////////////
/// Dependencies
///////////////////////////////////////////////

#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <librealsense/rs.hpp>
#include "utils.h"


namespace realsense_ros_object
{
///////////////////////////////////////////////
/// CTracking -
///////////////////////////////////////////////
class CTracking
{
public :
  //===================================
  //  Interface
  //===================================
  void init(ros::NodeHandle& nh);
  void getParams(ros::NodeHandle& nh);
  void getParams(const std::vector<std::string> & argv);

  CTracking();
  virtual ~CTracking();

private:
  //===================================
  //  Member Functions
  //===================================

  int initialize();
  int unInitialize();



  //Static member functions:
  void colorCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & cameraInfo);
  void depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & cameraInfo);
  void objectsToTrackCallback(const realsense_ros_object::TrackedObjectsArray& msg);
  void localizedObjectsToTrackCallback(const realsense_ros_object::ObjectsInBoxes& msg);


  void TrackingCallback(const sensor_msgs::ImageConstPtr& color , const sensor_msgs::ImageConstPtr& depth);

  //===================================
  //  Member Variables
  //===================================

  image_transport::Subscriber m_sub_color;
  image_transport::Subscriber m_sub_depth;
  ros::Subscriber m_sub_objects_with_pos;
  ros::Subscriber m_sub_colorCameraInfo;
  ros::Subscriber m_sub_depthCameraInfo;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>      mDepthSubscriber;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>      mColorSubscriber;
  std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>> mTimeSynchronizer;

  ros::Publisher m_tracked_objects_pub;

  cv::Mat m_imageColor;
  cv::Mat m_imageDepth;
  int m_frame_number;

  bool m_show_rgb;
  bool m_localization_msg;
  bool m_estimateCenter;


  bool m_no_subscribers;
  int m_numberOfObjects;
  int m_maxNumberOfObjects;

  ros::NodeHandle m_nh;


  std::string m_colorInfoTopic;

  int m_colorHeight;
  int m_colorWidth;
  int m_depthHeight;
  int m_depthWidth;

  rs::extrinsics m_ext;
  rs::intrinsics m_colorInt;
  rs::intrinsics m_depthInt;

  ros_camera_utils utils;

  rs::core::image_info m_colorInfo;
  rs::core::image_info m_depthInfo;

  rs::object_recognition::or_video_module_impl m_impl;
  rs::object_recognition::or_data_interface* m_or_data;
  rs::object_recognition::or_configuration_interface* m_or_configuration;



};

#ifdef RUN_AS_NODELET
///////////////////////////////////////////////
/// CTrackingNodelet
///////////////////////////////////////////////
class CTrackingNodelet : public nodelet::Nodelet
{
public :
  //===================================
  //  Interface
  //===================================
  virtual void onInit();

  ~CTrackingNodelet();

private:
  //===================================
  //  Member Functions
  //===================================

  //===================================
  //  Member Variables
  //===================================

  CTracking m_TrackingNodelet;

};
#endif
};


#endif // RS_TRACKING_NODELET

