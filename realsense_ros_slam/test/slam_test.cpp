// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <gtest/gtest.h>
#include <string>  // Added to satisfy roslint
#include <vector>  // Added to satisfy roslint
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <realsense_ros_slam/TrackingAccuracy.h>
#include <nav_msgs/OccupancyGrid.h>

bool camera_pose_received = false;
bool reloc_pose_received = false;
bool pose2d_received = false;
bool accuracy_received = false;
bool map_received = false;
uint32_t highest_accuracy = 0;

geometry_msgs::PoseStamped last_cam_pose;
geometry_msgs::PoseStamped last_reloc_pose;
geometry_msgs::Pose2D last_pose2d;
nav_msgs::OccupancyGrid last_map;

void testPose(geometry_msgs::PoseStamped pose_msg)
{
  EXPECT_GT(pose_msg.header.seq, 0);
  EXPECT_GT(pose_msg.header.frame_id.length(), 0);
  EXPECT_TRUE(pose_msg.header.stamp.isValid());

  bool isPositionAllZeros = true;
  if (pose_msg.pose.position.x || pose_msg.pose.position.y || pose_msg.pose.position.z) isPositionAllZeros = false;
  EXPECT_FALSE(isPositionAllZeros);

  bool isOrientationAllZeros = true;
  if (pose_msg.pose.orientation.x || pose_msg.pose.orientation.y || pose_msg.pose.orientation.z || pose_msg.pose.orientation.w) isOrientationAllZeros = false;
  EXPECT_FALSE(isOrientationAllZeros);
}

TEST(RealsenseTests, testCamPose)
{
  EXPECT_TRUE(camera_pose_received);
  if (!camera_pose_received) return;
  testPose(last_cam_pose);
}

TEST(RealsenseTests, testRelocPose)
{
  EXPECT_TRUE(reloc_pose_received);
  if (!reloc_pose_received) return;
  testPose(last_reloc_pose);
}

TEST(RealsenseTests, testPose2D)
{
  EXPECT_TRUE(pose2d_received);
  if (!pose2d_received) return;
  EXPECT_TRUE(last_pose2d.x || last_pose2d.y);
  // we are not testing last_pose2d.theta because zero could be valid.
}

TEST(RealsenseTests, testAccuracy)
{
  EXPECT_TRUE(accuracy_received);
  if (!accuracy_received) return;
  EXPECT_GE(highest_accuracy, 2);
}

TEST(RealsenseTests, testMap)
{
  EXPECT_TRUE(map_received);
  if (!map_received) return;

  EXPECT_GT(last_map.header.seq, 0);
  EXPECT_TRUE(last_map.header.stamp.isValid());
  //EXPECT_GT(last_map.header.frame_id.length(), 0); // map header doesn't have frame_id

  EXPECT_GT(last_map.info.resolution, 0);
  EXPECT_GT(last_map.info.width, 0);
  EXPECT_GT(last_map.info.height, 0);
  EXPECT_NE(last_map.info.origin.position.x, 0);
  EXPECT_NE(last_map.info.origin.position.y, 0);

  EXPECT_GT(last_map.data.size(), 0);
}

void camera_pose_callback(const geometry_msgs::PoseStampedConstPtr &ptr)
{
  camera_pose_received = true;
  last_cam_pose = *ptr;
}

void reloc_pose_callback(const geometry_msgs::PoseStampedConstPtr &ptr)
{
  reloc_pose_received = true;
  last_reloc_pose = *ptr;
}

void pose2d_callback(const geometry_msgs::Pose2DConstPtr &ptr)
{
  pose2d_received = true;
  last_pose2d = *ptr;
}

void accuracy_callback(const realsense_ros_slam::TrackingAccuracyConstPtr &ptr)
{
  accuracy_received = true;
  if (ptr->tracking_accuracy > highest_accuracy) highest_accuracy = ptr->tracking_accuracy;
}

void map_callback(const nav_msgs::OccupancyGridConstPtr &ptr)
{
  map_received = true;
  last_map = *ptr;
}

int main(int argc, char **argv) try
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "utest");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("RealSense SLAM test - Initializing Tests...");

  ros::Subscriber sub_cam_pose = nh.subscribe("camera_pose", 10, &camera_pose_callback);
  ros::Subscriber sub_reloc_pose = nh.subscribe("reloc_pose", 10, &reloc_pose_callback);
  ros::Subscriber sub_pose2d = nh.subscribe("pose2d", 10, &pose2d_callback);
  ros::Subscriber sub_tracking_acc = nh.subscribe("tracking_accuracy", 10, &accuracy_callback);
  ros::Subscriber sub_map = nh.subscribe("map", 10, &map_callback);

  ros::Duration duration;
  duration.sec = 59;
  duration.sleep();
  ros::spinOnce();

  ROS_INFO_STREAM("RealSense SLAM test - Running Tests...");

  return RUN_ALL_TESTS();
}
catch (...) {} // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
