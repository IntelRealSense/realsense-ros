// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <sys/types.h>
#include <sys/syscall.h>

#include <geometry_msgs/Twist.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "realsense_ros_object/ObjectArray.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include "realsense_recognition_nodelet.h"

#include <rs_sdk.h>





#include <pluginlib/class_list_macros.h>


#ifdef RUN_AS_NODELET
PLUGINLIB_EXPORT_CLASS(realsense_ros_object::CRecognitionNodelet, nodelet::Nodelet)
#endif


namespace realsense_ros_object
{


//******************************
// Public Methods
//******************************

CRecognition::~CRecognition()
{
  unInitialize();
  if (m_show_rgb)
  {
    cv::destroyAllWindows();
  }
}
int CRecognition::unInitialize()
{

  return 0;
}

int CRecognition::initialize()
{
  return 0;
}

void CRecognition::init(ros::NodeHandle& nh, const std::vector<std::string> & argv)
{

  m_or_data = nullptr;
  m_or_configuration = nullptr;
  m_no_subscribers = true;
  m_show_rgb = false;
  m_nh = nh;

  for (int i = 0; i < argv.size(); i++)
  {
    if (argv[i] == "show_rgb")
    {
      m_show_rgb = true;
    }
  }

  image_transport::ImageTransport it(nh);

  m_sub_depthCameraInfo = nh.subscribe("camera/depth/camera_info", 1, &CRecognition::depthCameraInfoCallback, this);
  m_recognized_objects_pub = nh.advertise<realsense_ros_object::ObjectArray>("realsense/recognized_objects", 1);



  if (m_show_rgb)
  {
    cv::namedWindow("viewColor");
  }
}

void CRecognition::depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & cameraInfo)
{
  // this callback is used in trackMiddle mode.
  // get the image size and set the bbox in the middle of the image
  m_depthHeight = cameraInfo->height;
  m_depthWidth = cameraInfo->width;


  utils.get_extrinsics(cameraInfo, m_ext);
  utils.get_intrinsics(cameraInfo, m_depthInt);

  m_sub_colorCameraInfo = m_nh.subscribe("camera/color/camera_info", 1, &CRecognition::colorCameraInfoCallback, this);
  m_sub_depthCameraInfo.shutdown();
}

void CRecognition::colorCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & cameraInfo)
{

  utils.get_intrinsics(cameraInfo, m_colorInt);
  // get the image size and set the bbox in the middle of the image
  m_colorHeight = cameraInfo->height;
  m_colorWidth = cameraInfo->width;

  image_transport::ImageTransport it(m_nh);
//    m_sub_color = it.subscribe("camera/color/image_raw", 1, &CRecognition::imageColorCallback, this);
  m_sub_colorCameraInfo.shutdown();

  if (utils.init_or(m_colorInt, m_depthInt, m_ext, m_impl, &m_or_data, &m_or_configuration) != rs::core::status_no_error)
  {
    ROS_ERROR("Unable to init Object Recgonition!");
    exit(0);
  }


  m_colorInfo.height = m_colorHeight;
  m_colorInfo.width = m_colorWidth;
  m_colorInfo.format = rs::core::pixel_format::bgr8;
  m_colorInfo.pitch = m_colorInfo.width * 3;


  m_depthInfo.height = m_depthHeight;
  m_depthInfo.width = m_depthWidth;
  m_depthInfo.format = rs::core::pixel_format::z16;
  m_depthInfo.pitch = m_depthInfo.width * 2;

  //add bounding box to recognition
  m_or_configuration->set_roi(rs::core::rect{(int)(m_colorInfo.width / 4), (int)(m_colorInfo.height / 4), (int)(m_colorInfo.width / 2), (int)(m_colorInfo.height / 2)});
  m_or_configuration->apply_changes();


  // Subscribe to color, depth using synchronization filter
  mDepthSubscriber = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
                     (new message_filters::Subscriber<sensor_msgs::Image>(m_nh, "camera/color/image_raw", 1));
  mColorSubscriber = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
                     (new message_filters::Subscriber<sensor_msgs::Image>(m_nh, "camera/depth/image_raw", 1));

  mTimeSynchronizer = std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>>
                      (new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>(*mDepthSubscriber, *mColorSubscriber, 40));

  mTimeSynchronizer->registerCallback(boost::bind(&CRecognition::RecognitionCallback, this, _1, _2));

}


void CRecognition::RecognitionCallback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth)
{
  try
  {
    m_imageColor =  cv_bridge::toCvShare(color, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color->encoding.c_str());
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
  {
    ROS_ERROR("process_sample_set_sync failed");
    return;
  }
  rs::object_recognition::recognition_data* recognition_data = nullptr;

  //retrieve recognition data from the or_data object
  st = m_or_data->query_single_recognition_result(&recognition_data, array_size);
  if (st != rs::core::status_no_error)
  {
    ROS_ERROR("query_single_recognition_result failed");
    return;
  }

  realsense_ros_object::ObjectArray object_vector;
  realsense_ros_object::Object one_object;
  for (int i = 0; i < array_size; i++)
  {

    one_object.object_name = m_or_configuration->query_object_name_by_id(recognition_data[i].label);
    one_object.probability =  recognition_data[i].probability;
    one_object.label =  recognition_data[i].label;

    object_vector.objects_vector.push_back(one_object);
  }

  m_recognized_objects_pub.publish(object_vector);


}

#ifdef RUN_AS_NODELET

//******************************
// Public Methods
//******************************

CRecognitionNodelet::~CRecognitionNodelet()
{

}

void CRecognitionNodelet::onInit()
{
  ROS_INFO_STREAM("Object Recognition OnInit");
  NODELET_INFO("Object Recognition OnInit.");

  const std::vector<std::string> & argv = getMyArgv();
  ros::NodeHandle& nh = getPrivateNodeHandle();// getNodeHandle();
  m_RecgonitionNodelet.init(nh, argv);

  NODELET_INFO("Objects Tracker nodelet initialized.");
}

#endif

}

