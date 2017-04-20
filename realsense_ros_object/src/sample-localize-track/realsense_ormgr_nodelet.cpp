// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include "ros/ros.h"

#include "realsense_ros_object/Rect.h"
#include "realsense_ros_object/TrackedObjectsArray.h"
#include "realsense_ros_object/ObjectInBox.h"
#include "realsense_ros_object/ObjectsInBoxes.h"
#include "realsense_ros_object/UI.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "realsense_ormgr_nodelet.h"
#include <pluginlib/class_list_macros.h>


#if defined(RUN_AS_NODELET)
PLUGINLIB_EXPORT_CLASS(realsense_ros_object::COrmgrNodelet, nodelet::Nodelet)
#endif

namespace realsense_ros_object
{
//******************************
// Public Methods
//******************************

COrmgr::COrmgr()
{
  initialize();
}

COrmgr::~COrmgr()
{
  unInitialize();
}

int COrmgr::unInitialize()
{
  objects_vector_.objects_vector.clear();
  return 0;
}

int COrmgr::initialize()
{
  threshold_ = 1;
  max_number_of_objects_ = 5;
  return 0;
}

void COrmgr::getParams(ros::NodeHandle& nh)
{
  nh.getParam("threshold", threshold_);
  nh.getParam("max_number_of_objects", max_number_of_objects_);
  nh.getParam("localize_every_frame", localize_every_frame_);
}

void COrmgr::getParams(const std::vector<std::string> & argv)
{
  for (int i = 0; i < argv.size(); i++)
  {
    if (argv[i] == "-threshold") // max objects to track
    {
      threshold_ = atoi(argv[++i].c_str());
    }
    else if (argv[i] == "-max_number_of_objects") // max objects to track
    {
      max_number_of_objects_ = atoi(argv[++i].c_str());
    }
  }
}

void COrmgr::init(ros::NodeHandle& nh)
{
  nh_ = nh;
  sub_localized_objects_ = nh_.subscribe("realsense/localized_objects", 1, &COrmgr::localizeidObjectsCallback , this);
  sub_UI_ = nh_.subscribe("realsense/UI", 1, &COrmgr::UICallback , this);

  objects_to_track_pub_ = nh_.advertise<realsense_ros_object::TrackedObjectsArray>("realsense/objects_to_track", 1);
  tracked_localized_pub_ = nh_.advertise<realsense_ros_object::ObjectsInBoxes>(
              "realsense/localized_tracked_objects", 1);
}



void COrmgr::localizeidObjectsCallback(const realsense_ros_object::ObjectsInBoxes& msg)
{
// get objects roi from localization and publish it
  int array_size = std::min(max_number_of_objects_, (int)msg.objects_vector.size());
  if (array_size <= 0)
    return ;
  sub_localized_objects_.shutdown();
  ROS_INFO("new objects to track");
  realsense_ros_object::TrackedObjectsArray outROIs;
  objects_vector_.objects_vector.clear();

  realsense_ros_object::TrackedObject to;
  for (int i = 0; i < array_size; i++)
  {
    to.bbox = msg.objects_vector[i].object_bbox;
    to.location = msg.objects_vector[i].location;
    to.id = i;
    outROIs.tracked_objects_vector.push_back(to);
    objects_vector_.objects_vector.push_back(msg.objects_vector[i]);
  }

  sub_tracked_objects_ = nh_.subscribe("realsense/tracked_objects", 1, &COrmgr::trackedObjectCallback , this);
  objects_to_track_pub_.publish(outROIs);
}

void COrmgr::UICallback(const realsense_ros_object::UI& msg)
{
  if (true == localize_every_frame_)
  {
    return;
  }
  ROS_INFO("re-localization\n ");
  sub_tracked_objects_.shutdown();
  sub_localized_objects_ = nh_.subscribe("realsense/localized_objects", 1, &COrmgr::localizeidObjectsCallback , this);
}

void COrmgr::trackedObjectCallback(const realsense_ros_object::TrackedObjectsArray::ConstPtr & msg)
{
  int nCtr = 0;
  for (int i = 0; i < (int)msg->tracked_objects_vector.size(); i++)
  {
    objects_vector_.objects_vector[i].object_bbox = msg->tracked_objects_vector[i].bbox;
    objects_vector_.objects_vector[i].location = msg->tracked_objects_vector[i].location;

    if (msg->tracked_objects_vector[i].bbox.width == 0 || msg->tracked_objects_vector[i].bbox.height == 0)
      nCtr++;
  }

  if (localize_every_frame_)
  {
    sub_tracked_objects_.shutdown();
    sub_localized_objects_ = nh_.subscribe("realsense/localized_objects", 1, &COrmgr::localizeidObjectsCallback , this);
  }

  objects_vector_.header = msg->header;
  tracked_localized_pub_.publish(objects_vector_);
}


#if defined(RUN_AS_NODELET)

//******************************
// Public Methods
//******************************

COrmgrNodelet::~COrmgrNodelet()
{
}

void COrmgrNodelet::onInit()
{
  ROS_INFO_STREAM("OR manager OnInit");
  NODELET_INFO("OR manager OnInit.");

  ros::NodeHandle& local_nh = getPrivateNodeHandle();
  manager_nodelet_.getParams(local_nh);

  ros::NodeHandle& nh = getNodeHandle();
  manager_nodelet_.init(nh);

  NODELET_INFO("OR manager nodelet initialized.");
}

#endif

}

#if !defined(RUN_AS_NODELET)

int main(int argc, char **argv)
{

  ros::init(argc, argv, "realsense_ormgr");
  ros::NodeHandle nh;
  realsense::COrmgr or_manager;

  std::vector<std::string> argv_strings;
  for (int i = 0; i < argc; i++)
  {
    argvStrings.push_back(argv[i]);
  }

  or_manager.getParams(argv_strings);
  or_manager.init(nh);
  ros::spin();
}
#endif

