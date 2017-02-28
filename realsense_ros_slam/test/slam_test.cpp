// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved

#include <gtest/gtest.h>
#include "slam_test.h"  // NOLINT(build/include)
#include <string>  // Added to satisfy roslint
#include <vector>  // Added to satisfy roslint


TEST(RealsenseTests, testPass)
{
  EXPECT_TRUE(true);
}

int main(int argc, char **argv) try
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "utest");
  ros::NodeHandle n;
  ros::NodeHandle nh(n, "camera");

  ROS_INFO_STREAM("RealSense SLAM test - Initializing Tests...");

//   ros::NodeHandle depth_nh(nh, "depth");
//   image_transport::ImageTransport depth_image_transport(depth_nh);
//   g_camera_subscriber[0] = depth_image_transport.subscribeCamera("image_raw", 1, imageDepthCallback, 0);
// 
//   ros::NodeHandle color_nh(nh, "color");
//   image_transport::ImageTransport color_image_transport(color_nh);
//   g_camera_subscriber[1] = color_image_transport.subscribeCamera("image_raw", 1, imageColorCallback, 0);
// 
//   ZR300 cameras have Fisheye and IMU
//   ros::NodeHandle fisheye_nh(nh, "fisheye");
//   image_transport::ImageTransport fisheye_image_transport(fisheye_nh);
//   if (g_camera_type == "ZR300")
//   {  
//     g_camera_subscriber[4] = fisheye_image_transport.subscribeCamera("image_raw", 1, imageFisheyeCallback, 0);
//     g_sub_accel = n.subscribe<sensor_msgs::Imu>("camera/accel/sample", 1, accelCallback);
//     g_sub_gyro = n.subscribe<sensor_msgs::Imu>("camera/gyro/sample", 1, gyroCallback);
//   }

  ros::Duration duration;
  duration.sec = 5;
  duration.sleep();
  ros::spinOnce();
  
  ROS_INFO_STREAM("RealSense SLAM test - Running Tests...");

  return RUN_ALL_TESTS();
}
catch(...) {}  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
