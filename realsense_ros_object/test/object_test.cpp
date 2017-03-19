// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "realsense_ros_object/ObjectsInBoxes.h"
#include <image_transport/image_transport.h>

bool objects_in_boxes_recieved = false;
bool camera_info_recieved = false;

int image_width = 0;
int image_height = 0;
realsense_ros_object::ObjectsInBoxes last_objs;


TEST(RealsenseTests, testObj)
{
  EXPECT_TRUE(camera_info_recieved);
  EXPECT_TRUE(objects_in_boxes_recieved);
  if (!objects_in_boxes_recieved) return;
  for(auto objs : last_objs.objects_vector)
  {
    EXPECT_GT(objs.object.object_name.length(), 1);
    EXPECT_GT(objs.object_bbox.width, 0);
    EXPECT_GT(objs.object_bbox.height, 0);
    if(objs.object_bbox.height > objs.object_bbox.width)
    {
      if(objs.location.vert_margin != 0 || objs.location.horiz_margin != 0)
      {
        EXPECT_GT(objs.location.vert_margin, objs.location.horiz_margin);
      }
    }
    else if (objs.object_bbox.height < objs.object_bbox.width)
    {
      if(objs.location.vert_margin != 0 || objs.location.horiz_margin != 0)
      {
        EXPECT_LT(objs.location.vert_margin, objs.location.horiz_margin);
      }
    }
  }
}

void localized_tracked_objects_callback(
            const realsense_ros_object::ObjectsInBoxes::ConstPtr& msg)
{
  objects_in_boxes_recieved = true;
  last_objs = *msg;
}


void color_camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr & camera_info)
{
  camera_info_recieved = true;
  image_width = camera_info->width;
  image_height = camera_info->height;
}

int main(int argc, char **argv) try
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "utest");
  ros::NodeHandle nh;


  ROS_INFO_STREAM("RealSense OBJECT test - Initializing Tests...");



  ros::Subscriber sub_cam_pose = nh.subscribe("camera/color/camera_info", 10, &color_camera_info_callback);
  ros::Subscriber sub_reloc_pose = nh.subscribe("realsense/localized_tracked_objects", 10, &localized_tracked_objects_callback);


  ros::Duration duration;
  duration.sec = 10;
  duration.sleep();
  ros::spinOnce();

  ROS_INFO_STREAM("RealSense OBJECT test - Running Tests...");

  return RUN_ALL_TESTS();
}
catch (...) {} // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
