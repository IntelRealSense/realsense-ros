// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved

#include <gtest/gtest.h>
#include "slam_test.h"  // NOLINT(build/include)
#include <string>  // Added to satisfy roslint
#include <vector>  // Added to satisfy roslint
#include <geometry_msgs/Pose2D.h>
#include <realsense_ros_slam/TrackingAccuracy.h>
#include <nav_msgs/OccupancyGrid.h>

bool camera_pose_received = false;
bool pose2d_received = false;
bool accuracy_received = false;
bool map_received = false;
uint32_t highest_accuracy = 0;

geometry_msgs::PoseStamped last_cam_pose;
geometry_msgs::Pose2D last_pose2d;
nav_msgs::OccupancyGrid last_map;

TEST(RealsenseTests, testAllMessagesReceived)
{
    EXPECT_TRUE(camera_pose_received);
    EXPECT_TRUE(pose2d_received);
    EXPECT_TRUE(accuracy_received);
    EXPECT_TRUE(map_received);
}

TEST(RealsenseTests, testAccuracyMediumOrBetter)
{
    EXPECT_GE(highest_accuracy, 2);
}

TEST(RealsenseTests, testCamPose)
{
    bool isPositionAllZeros = true;
    if (last_cam_pose.pose.position.x || last_cam_pose.pose.position.y || last_cam_pose.pose.position.z) isPositionAllZeros = false;
    EXPECT_FALSE(isPositionAllZeros);
    
    bool isOrientationAllZeros = true;
    if (last_cam_pose.pose.orientation.x || last_cam_pose.pose.orientation.y || last_cam_pose.pose.orientation.z || last_cam_pose.pose.orientation.w) isOrientationAllZeros = false;
    EXPECT_FALSE(isOrientationAllZeros);
}

void camera_pose_callback(const geometry_msgs::PoseStampedConstPtr &ptr)
{
     camera_pose_received = true;
     last_cam_pose = *ptr;
}

void pose2d_callback(const geometry_msgs::Pose2DConstPtr &ptr)
{
     pose2d_received = true;
     last_pose2d = *ptr;
}

void accuracy_callback(const realsense_ros_slam::TrackingAccuracyConstPtr &ptr)
{
     accuracy_received = true;
     highest_accuracy = ptr->tracking_accuracy;
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

    ros::Subscriber sub_cam_pose, sub_pose2d, sub_tracking_acc, sub_map;

    sub_cam_pose = nh.subscribe("camera_pose", 10, &camera_pose_callback);
    sub_pose2d = nh.subscribe("pose2d", 10, &pose2d_callback);
    sub_tracking_acc = nh.subscribe("tracking_accuracy", 10, &accuracy_callback);
    sub_map = nh.subscribe("map", 10, &map_callback);

    ros::Duration duration;
    duration.sec = 10;
    duration.sleep();
    ros::spinOnce();

    ROS_INFO_STREAM("RealSense SLAM test - Running Tests...");

    return RUN_ALL_TESTS();
}
catch(...) {}  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
