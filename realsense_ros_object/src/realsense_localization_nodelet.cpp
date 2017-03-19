// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include "ros/ros.h"


#include "realsense_ros_object/Rect.h"
#include "realsense_ros_object/ObjectInBox.h"
#include "realsense_ros_object/ObjectsInBoxes.h"
#include "realsense_ros_object/cpu_gpu.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include "realsense_localization_nodelet.h"

#include <rs_sdk.h>

#include <pluginlib/class_list_macros.h>

#ifdef RUN_AS_NODELET
PLUGINLIB_EXPORT_CLASS(realsense_ros_object::CLocalizationNodelet, nodelet::Nodelet)
#endif


namespace realsense_ros_object
{


//******************************
// Public Methods
//******************************

CLocalization::CLocalization()
{
  initialize();
}

CLocalization::~CLocalization()
{
  unInitialize();
  if (m_show_rgb)
  {
    cv::destroyAllWindows();
  }
}
int CLocalization::unInitialize()
{

  return 0;
}

int CLocalization::initialize()
{
  m_show_rgb = false;
  m_confidence = 0.7;
  m_estimateCenter = true;
  m_use_CPU = true;
  return 0;
}

void CLocalization::getParams(ros::NodeHandle& nh)
{
  nh.getParam("show_rgb", m_show_rgb);
  nh.getParam("confidence", m_confidence);
  nh.getParam("estimate_center", m_estimateCenter);
  nh.getParam("use_CPU", m_use_CPU);
}

void CLocalization::getParams(const std::vector<std::string> & argv)
{

  for (int i = 0; i < argv.size(); i++)
  {
    if (argv[i] == "show_rgb")
    {
      m_show_rgb = true;
    }
    else if (argv[i] == "-confidence")
    {
      m_confidence = atof(argv[++i].c_str());
    }
    else if (argv[i] == "-estimate_center")
    {
      m_estimateCenter = true;
    }
  }

}

void CLocalization::init(ros::NodeHandle& nh)
{
  m_or_data = nullptr;
  m_or_configuration = nullptr;

  m_no_subscribers = true;
  m_nh = nh;

  image_transport::ImageTransport it(nh);

  m_sub_depthCameraInfo = nh.subscribe("camera/depth/camera_info", 1, &CLocalization::depthCameraInfoCallback, this);
  m_recognized_objects_pub = nh.advertise<realsense_ros_object::ObjectsInBoxes>("realsense/localized_objects", 1);
  m_cpu_gpu_pub = nh.advertise<realsense_ros_object::cpu_gpu>("realsense/cpu_gpu", 1);
  if (m_show_rgb)
  {
    cv::namedWindow("localization");
  }
}


void CLocalization::depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & cameraInfo)
{
  // this callback is used in trackMiddle mode.
  // get the image size and set the bbox in the middle of the image
  m_depthHeight = cameraInfo->height;
  m_depthWidth = cameraInfo->width;

  ROS_INFO("got depth info");
  utils.get_extrinsics(cameraInfo, m_ext);
  utils.get_intrinsics(cameraInfo, m_depthInt);

  m_sub_colorCameraInfo = m_nh.subscribe("camera/color/camera_info", 1, &CLocalization::colorCameraInfoCallback, this);
  m_sub_depthCameraInfo.shutdown();

}

void CLocalization::colorCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & cameraInfo)
{
  m_sub_colorCameraInfo.shutdown();
  ROS_INFO("got color info");
  utils.get_intrinsics(cameraInfo, m_colorInt);
  // get the image size and set the bbox in the middle of the image
  m_colorHeight = cameraInfo->height;
  m_colorWidth = cameraInfo->width;
  rs::core::status st = utils.init_or(m_colorInt, m_depthInt, m_ext, m_impl, &m_or_data, &m_or_configuration);
  if (st != rs::core::status_no_error)
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

  //change mode to localization
  m_or_configuration->set_recognition_mode(rs::object_recognition::recognition_mode::LOCALIZATION);
  //set the localization mechnizm to use CNN
  m_or_configuration->set_localization_mechanism(rs::object_recognition::localization_mechanism::CNN);
  //ignore all objects under 0.7 probabilty (confidence)
  m_or_configuration->set_recognition_confidence(m_confidence);
  //enable object center feature
  m_or_configuration->enable_object_center_estimation(m_estimateCenter);
  if (false == m_use_CPU)
  {
    //enable GPU computing
    m_or_configuration->set_compute_engine(rs::object_recognition::compute_engine::GPU);
    ROS_INFO("using GPU - opencl driver");
  }
  else
  {
    ROS_INFO("using CPU");
  }
  try
  {
    m_or_configuration->apply_changes();
    ROS_INFO("init done");
  }
  catch (...)
  {
    ROS_INFO("Unable to use GPU, using CPU");
    m_or_configuration->set_compute_engine(rs::object_recognition::compute_engine::CPU);
    m_or_configuration->apply_changes();
    realsense_ros_object::cpu_gpu msg;
    msg.CPU_GPU = "Unable to use GPU, using CPU";
    m_cpu_gpu_pub.publish(msg);
  }

  // Subscribe to color, depth using synchronization filter
  mDepthSubscriber = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
                     (new message_filters::Subscriber<sensor_msgs::Image>(m_nh, "camera/color/image_raw", 1));
  mColorSubscriber = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
                     (new message_filters::Subscriber<sensor_msgs::Image>(m_nh, "camera/depth/image_raw", 1));

  mTimeSynchronizer = std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>>
                      (new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>(*mDepthSubscriber, *mColorSubscriber, 60));

  mTimeSynchronizer->registerCallback(boost::bind(&CLocalization::LocalizationCallback, this, _1, _2));

}


void CLocalization::LocalizationCallback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth)
{
  // if there are no subscribers do nothing

  if (m_recognized_objects_pub.getNumSubscribers() <= 0)
  {
    return;
  }


  ROS_INFO("working on localization");

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
  {
    ROS_ERROR("process_sample_set_sync failed");
    return;
  }
  rs::object_recognition::localization_data* localization_data = nullptr;

  //retrieve localization data from the or_data object
  st = m_or_data->query_localization_result(&localization_data, array_size);
  if (st != rs::core::status_no_error)
  {
    ROS_ERROR("query_localization_result failed");
    return;
  }

  //ROS_INFO(("array" + std::to_string(array_size)).c_str() );
  realsense_ros_object::ObjectsInBoxes objects_with_pos;
  realsense_ros_object::ObjectInBox objectInBox;
  for (int i = 0; i < array_size; i++)
  {
    objectInBox.object.label = localization_data[i].label;
    objectInBox.object.object_name = m_or_configuration->query_object_name_by_id(localization_data[i].label);
    objectInBox.object.probability =  localization_data[i].probability;
    objectInBox.object_bbox.x = localization_data[i].roi.x;
    objectInBox.object_bbox.y = localization_data[i].roi.y;
    objectInBox.object_bbox.width = localization_data[i].roi.width;
    objectInBox.object_bbox.height = localization_data[i].roi.height;
    objectInBox.location.coordinates.x = localization_data[i].object_center.coordinates.x;
    objectInBox.location.coordinates.y = localization_data[i].object_center.coordinates.y;
    objectInBox.location.coordinates.z = localization_data[i].object_center.coordinates.z;
    objectInBox.location.horiz_margin = localization_data[i].object_center.horiz_margin;
    objectInBox.location.vert_margin = localization_data[i].object_center.vert_margin;

    objects_with_pos.objects_vector.push_back(objectInBox);

  }

  // copy the header for image and localization sync;
  objects_with_pos.header = color->header;

  m_recognized_objects_pub.publish(objects_with_pos);
  ROS_INFO("localization done");
  if (m_show_rgb)
  {
    draw_results(localization_data, array_size, m_or_configuration);
    cv::imshow("localization", m_imageColor);
    cv::waitKey(1);
  }

}


void CLocalization::draw_results(rs::object_recognition::localization_data* localization_data, int array_size, rs::object_recognition::or_configuration_interface* or_configuration)
{
  for (int i = 0; i < array_size; i++)
  {
    std::stringstream text;
    std::string object_name = or_configuration->query_object_name_by_id(localization_data[i].label);
    text.precision(2);
    text << std::fixed << object_name << ": " << localization_data[i].probability;
    if (or_configuration->is_object_center_estimation_enabled())
    {
      text.precision(0);
      text << " COM: (" << localization_data[i].object_center.coordinates.x << ", " << localization_data[i].object_center.coordinates.y << ", "  << localization_data[i].object_center.coordinates.z << ")" ;
    }    //todo: print margin vertical and horizental
    cv::Point txtLocation(localization_data[i].roi.x - 5, std::max(localization_data[i].roi.y, 20));
    cv::putText(m_imageColor, text.str(), txtLocation, cv::FONT_HERSHEY_PLAIN, 1.5, cvScalar(0, 0, 0), 2); // black text outlining
    cv::putText(m_imageColor, text.str(), txtLocation, cv::FONT_HERSHEY_PLAIN, 1.5, cvScalar(255, 128, 128), 1);
    cv::Rect roiRect(localization_data[i].roi.x, localization_data[i].roi.y, localization_data[i].roi.width, localization_data[i].roi.height);
    cv::rectangle(m_imageColor, roiRect, cvScalar(255, 0, 0));
  }
}

#ifdef RUN_AS_NODELET

//******************************
// Public Methods
//******************************

CLocalizationNodelet::~CLocalizationNodelet()
{
}

void CLocalizationNodelet::onInit()
{
  ROS_INFO_STREAM("Object Localization OnInit");
  NODELET_INFO("Object Localization OnInit.");

  ros::NodeHandle& local_nh = getPrivateNodeHandle();
  m_RecgonitionNodelet.getParams(local_nh);

  ros::NodeHandle& nh = getNodeHandle();
  m_RecgonitionNodelet.init(nh);


  NODELET_INFO("Objects Localization nodelet initialized.");
}

#endif

}


