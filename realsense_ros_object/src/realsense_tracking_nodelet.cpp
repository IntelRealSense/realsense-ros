// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <ros/ros.h>

#include "realsense_ros_object/TrackedObject.h"
#include "realsense_ros_object/TrackedObjectsArray.h"
#include "realsense_ros_object/ObjectInBox.h"
#include "realsense_ros_object/ObjectsInBoxes.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "realsense_tracking_nodelet.h"

#include <pluginlib/class_list_macros.h>

#ifdef RUN_AS_NODELET
PLUGINLIB_EXPORT_CLASS(realsense_ros_object::CTrackingNodelet, nodelet::Nodelet)
#endif


namespace realsense_ros_object
{


//******************************
// Public Methods
//******************************

CTracking::CTracking()
{
  initialize();
}


CTracking::~CTracking()
{
  unInitialize();
  if (m_show_rgb)
  {
    cv::destroyAllWindows();
  }
}
int CTracking::unInitialize()
{

  return 0;
}

int CTracking::initialize()
{
  m_localization_msg = false;
  m_estimateCenter = true;
  m_maxNumberOfObjects = 5;
  return 0;
}

void CTracking::getParams(ros::NodeHandle& nh)
{

  nh.getParam("show_rgb", m_show_rgb);
  nh.getParam("subscribe_to_localization", m_localization_msg);
  nh.getParam("estimate_center", m_estimateCenter);
  nh.getParam("max_number_of_objects", m_maxNumberOfObjects);

}

void CTracking::getParams(const std::vector<std::string> & argv)
{

  for (int i = 0; i < argv.size(); i++)
  {
    if (argv[i] == "show_rgb")
    {
      m_show_rgb = true;
    }
    else if (argv[i] == "-max_number_of_objects") // max objects to track
    {
      m_maxNumberOfObjects = atoi(argv[++i].c_str());
    }
    else if (argv[i] == "-subscribe_to_localization")
    {
      m_localization_msg = true;
    }
    else if (argv[i] == "-estimate_center")
    {
      m_estimateCenter = true;
    }
  }


}

void CTracking::init(ros::NodeHandle& nh)
{

  m_or_data = nullptr;
  m_or_configuration = nullptr;
  m_no_subscribers = true;
  m_nh = nh;
  m_sub_depthCameraInfo = m_nh.subscribe("camera/depth/camera_info", 1, &CTracking::depthCameraInfoCallback, this);
  m_tracked_objects_pub = m_nh.advertise<realsense_ros_object::TrackedObjectsArray>("realsense/tracked_objects", 1);


  if (m_show_rgb)
  {
    cv::namedWindow("tracking");
  }
}

void CTracking::depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & cameraInfo)
{
  m_sub_depthCameraInfo.shutdown();
  // this callback is used in trackMiddle mode.
  // get the image size and set the bbox in the middle of the image
  m_depthHeight = cameraInfo->height;
  m_depthWidth = cameraInfo->width;

  ROS_INFO("%s", "got depth info");
  utils.get_extrinsics(cameraInfo, m_ext);
  utils.get_intrinsics(cameraInfo, m_depthInt);

  m_sub_colorCameraInfo = m_nh.subscribe("camera/color/camera_info", 1, &CTracking::colorCameraInfoCallback, this);

}

void CTracking::colorCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & cameraInfo)
{
  m_sub_colorCameraInfo.shutdown();
  ROS_INFO("got color info");
  utils.get_intrinsics(cameraInfo, m_colorInt);
  // get the image size and set the bbox in the middle of the image
  m_colorHeight = cameraInfo->height;
  m_colorWidth = cameraInfo->width;

  if (utils.init_or(m_colorInt, m_depthInt, m_ext, m_impl, &m_or_data, &m_or_configuration) != rs::core::status_no_error)
  {
    ROS_ERROR("Unable to init Object Recgonition!");
    exit(0);
  }


  m_colorInfo.height = m_colorHeight;
  m_colorInfo.width = m_colorWidth;
  m_colorInfo.format = rs::core::pixel_format::rgb8;
  m_colorInfo.pitch = m_colorInfo.width * 3;


  m_depthInfo.height = m_depthHeight;
  m_depthInfo.width = m_depthWidth;
  m_depthInfo.format = rs::core::pixel_format::z16;
  m_depthInfo.pitch = m_depthInfo.width * 2;


  m_or_configuration->set_recognition_mode(rs::object_recognition::recognition_mode::TRACKING);
  m_or_configuration->enable_object_center_estimation(m_estimateCenter);

  m_or_configuration->apply_changes();
  ROS_INFO("init done");

  // Subscribe to color, depth using synchronization filter
  mColorSubscriber = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
                     (new message_filters::Subscriber<sensor_msgs::Image>(m_nh, "camera/color/image_raw", 1));
  mDepthSubscriber = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
                     (new message_filters::Subscriber<sensor_msgs::Image>(m_nh, "camera/depth/image_raw", 1));
  mTimeSynchronizer = std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>>
                      (new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>(*mColorSubscriber, *mDepthSubscriber, 60));

  if (m_localization_msg)
    m_sub_objects_with_pos = m_nh.subscribe("realsense/localized_objects", 1, &CTracking::localizedObjectsToTrackCallback, this);
  else
    m_sub_objects_with_pos = m_nh.subscribe("realsense/objects_to_track", 1, &CTracking::objectsToTrackCallback, this);


}


void CTracking::objectsToTrackCallback(const realsense_ros_object::TrackedObjectsArray& msg)
{

  //  m_sub_objects_with_pos.shutdown();


  int array_size = std::min(m_maxNumberOfObjects, (int)msg.tracked_objects_vector.size()); //todo: insert m_maxNumberOfObjects as parameter to orconfiguration
  rs::core::rect* trackingRois = new rs::core::rect[array_size];

  for (int i = 0; i < array_size ; i++)
  {
    trackingRois[i].x = msg.tracked_objects_vector[i].bbox.x;
    trackingRois[i].y = msg.tracked_objects_vector[i].bbox.y;
    trackingRois[i].width =  msg.tracked_objects_vector[i].bbox.width;
    trackingRois[i].height = msg.tracked_objects_vector[i].bbox.height;
  }

  rs::core::status st = m_or_configuration->set_tracking_rois(trackingRois, array_size);
  st = m_or_configuration->apply_changes();
  delete[] trackingRois;


  mTimeSynchronizer->registerCallback(boost::bind(&CTracking::TrackingCallback, this, _1, _2));

}


void CTracking::localizedObjectsToTrackCallback(const realsense_ros_object::ObjectsInBoxes& msg)
{

  m_sub_objects_with_pos.shutdown();
  ROS_INFO("new objects to track");

  int array_size = std::min(m_maxNumberOfObjects, (int)msg.objects_vector.size());
  rs::core::rect* trackingRois = new rs::core::rect[array_size];

  for (int i = 0; i < array_size ; i++)
  {
    trackingRois[i].x = msg.objects_vector[i].object_bbox.x;
    trackingRois[i].y = msg.objects_vector[i].object_bbox.y;
    trackingRois[i].width =  msg.objects_vector[i].object_bbox.width;
    trackingRois[i].height = msg.objects_vector[i].object_bbox.height;
  }

  rs::core::status st = m_or_configuration->set_tracking_rois(trackingRois, array_size);
  st = m_or_configuration->apply_changes();
  delete[] trackingRois;


  mTimeSynchronizer->registerCallback(boost::bind(&CTracking::TrackingCallback, this, _1, _2));

}

void CTracking::TrackingCallback(const sensor_msgs::ImageConstPtr& color , const sensor_msgs::ImageConstPtr& depth)
{
  size_t timeRos1 = std::clock();
  if (m_tracked_objects_pub.getNumSubscribers() <= 0)
    return;
  try
  {
    m_imageColor =  cv_bridge::toCvShare(color, "rgb8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'rgb8'.", color->encoding.c_str());
    return;
  }

  try
  {
    m_imageDepth = cv_bridge::toCvShare(depth, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("toCvShare failed");
    return;
  }



  int array_size = 0;
  rs::core::status st;

  rs::core::correlated_sample_set* sample_set = utils.get_sample_set(m_colorInfo, m_depthInfo, m_imageColor.data, m_imageDepth.data, color->header.seq, (uint64_t)color->header.stamp.nsec);
  st = m_impl.process_sample_set(*sample_set);
  if (st != rs::core::status_no_error)
    ROS_ERROR("process_sample_set_sync failed");

  rs::object_recognition::tracking_data* tracking_data = nullptr;

  //retrieve tracking data from the or_data object

  st = m_or_data->query_tracking_result(&tracking_data, array_size);
  if (st != rs::core::status_no_error)
    ROS_ERROR("query_tracking_result failed");

  realsense_ros_object::TrackedObjectsArray outROIs;
  realsense_ros_object::TrackedObject to;
  for (int i = 0; i < array_size; i++)
  {


    to.bbox.width = tracking_data[i].roi.width;
    to.bbox.height = tracking_data[i].roi.height;
    to.bbox.x = tracking_data[i].roi.x;
    to.bbox.y = tracking_data[i].roi.y;
    to.location.coordinates.x = tracking_data[i].object_center.coordinates.x;
    to.location.coordinates.y = tracking_data[i].object_center.coordinates.y;
    to.location.coordinates.z = tracking_data[i].object_center.coordinates.z;
    to.location.horiz_margin = tracking_data[i].object_center.horiz_margin;
    to.location.vert_margin = tracking_data[i].object_center.vert_margin;
    to.id = i;
    outROIs.tracked_objects_vector.push_back(to);

  }
  // copy the header for image and tracking sync;
  outROIs.header = color->header;
  m_tracked_objects_pub.publish(outROIs);
}


#ifdef RUN_AS_NODELET

//******************************
// Public Methods
//******************************

CTrackingNodelet::~CTrackingNodelet()
{
}

void CTrackingNodelet::onInit()
{
  NODELET_INFO("Object Tracking OnInit.");
  ros::NodeHandle& local_nh = getPrivateNodeHandle();
  m_TrackingNodelet.getParams(local_nh);

  ros::NodeHandle& nh = getNodeHandle();
  m_TrackingNodelet.init(nh);

  NODELET_INFO("Objects Tracker nodelet initialized.");
}

#endif

}

