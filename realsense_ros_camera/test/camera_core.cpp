// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <gtest/gtest.h>
#include "camera_core.h"  // NOLINT(build/include)
#include <string>  // Added to satisfy roslint
#include <vector>  // Added to satisfy roslint

using std::string;
using std::stringstream;
using std::vector;

void getMsgInfo(rs_stream stream, const sensor_msgs::ImageConstPtr &msg)
{
  g_encoding_recv[stream] = msg->encoding;
  g_width_recv[stream] = msg->width;
  g_height_recv[stream] = msg->height;
  g_step_recv[stream] = msg->step;
}

void getCameraInfo(rs_stream stream, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
  g_caminfo_height_recv[stream] = info_msg->height;
  g_caminfo_width_recv[stream] = info_msg->width;

  g_dmodel_recv[stream] = info_msg->distortion_model;

  // copy rotation matrix
  for (unsigned int i = 0; i < sizeof(info_msg->R) / sizeof(double); i++)
  {
    g_caminfo_rotation_recv[stream][i] = info_msg->R[i];
  }

  // copy projection matrix
  for (unsigned int i = 0; i < sizeof(info_msg->P) / sizeof(double); i++)
  {
    g_caminfo_projection_recv[stream][i] = info_msg->P[i];
  }
}

void imageDepthCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
  cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  uint16_t *image_data = reinterpret_cast<uint16_t *>(image.data);

  double depth_total = 0;
  int depth_count = 0;
  for (unsigned int i = 0; i < msg->height * msg->width; ++i)
  {
    if ((0 < *image_data) && (*image_data <= g_max_z))
    {
      depth_total += *image_data;
      depth_count++;
    }
    image_data++;
  }
  if (depth_count != 0)
  {
    g_depth_avg = static_cast<float>(depth_total / depth_count);
  }

  getMsgInfo(RS_STREAM_DEPTH, msg);
  getCameraInfo(RS_STREAM_DEPTH, info_msg);

  for (unsigned int i = 0; i < 5; i++)
  {
    g_depth_caminfo_D_recv[i] = info_msg->D[i];
  }

  g_depth_recv = true;
}

void imageColorCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
  cv::Mat image = cv_bridge::toCvShare(msg, "rgb8")->image;
  uchar *color_data = image.data;

  int64 color_total = 0;
  int color_count = 1;
  for (unsigned int i = 0; i < msg->height * msg->width * 3; i++)
  {
    if (*color_data > 0 && *color_data < 255)
    {
      color_total += *color_data;
      color_count++;
    }
    color_data++;
  }
  if (color_count != 0)
  {
    g_color_avg = static_cast<float>(color_total / color_count);
  }

  getMsgInfo(RS_STREAM_COLOR, msg);
  getCameraInfo(RS_STREAM_COLOR, info_msg);

  for (unsigned int i = 0; i < 5; i++)
  {
    g_color_caminfo_D_recv[i] = info_msg->D[i];
  }

  g_color_recv = true;
}

void imageFisheyeCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
  cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;
  uchar *fisheye_data = image.data;

  double fisheye_total = 0.0;
  int fisheye_count = 1;
  for (unsigned int i = 0; i < msg->height * msg->width; i++)
  {
    if (*fisheye_data > 0 && *fisheye_data < 255)
    {
      fisheye_total += *fisheye_data;
      fisheye_count++;
    }
    fisheye_data++;
  }
  if (fisheye_count != 0)
  {
    g_fisheye_avg = static_cast<float>(fisheye_total / fisheye_count);
  }

  getMsgInfo(RS_STREAM_FISHEYE, msg);
  getCameraInfo(RS_STREAM_FISHEYE, info_msg);

  for (unsigned int i = 0; i < 5; i++)
  {
    g_fisheye_caminfo_D_recv[i] = info_msg->D[i];
  }

  g_fisheye_recv = true;
}

void accelCallback(const sensor_msgs::ImuConstPtr &imu)
{
  g_accel_recv = false;
  if (imu->linear_acceleration_covariance[0] != -1.0)
  {
    if ((imu->linear_acceleration.x != 0.000) ||
        (imu->linear_acceleration.y != 0.000) ||
        (imu->linear_acceleration.z != 0.000))
    {
      g_accel_recv = true;
    }
  }
}

void gyroCallback(const sensor_msgs::ImuConstPtr &imu)
{
  g_gyro_recv = false;
  if (imu->angular_velocity_covariance[0] != -1.0)
  {
    if ((imu->angular_velocity.x != 0.0) ||
        (imu->angular_velocity.y != 0.0) ||
        (imu->angular_velocity.z != 0.0))
    {
      g_gyro_recv = true;
    }
  }
}

TEST(RealsenseTests, testIsColorStreamEnabled)
{
  if (g_enable_color)
  {
    EXPECT_TRUE(g_color_recv);
  }
  else
  {
    EXPECT_FALSE(g_color_recv);
  }
}

TEST(RealsenseTests, testColorStream)
{
  if (g_enable_color)
  {
    EXPECT_GT(g_color_avg, 0);
    if (!g_color_encoding_exp.empty())
    {
      EXPECT_EQ(g_color_encoding_exp, g_encoding_recv[RS_STREAM_COLOR]);
    }
    if (g_color_step_exp > 0)
    {
      EXPECT_EQ(g_color_step_exp, g_step_recv[RS_STREAM_COLOR]);
    }
  }
}

TEST(RealsenseTests, testColorResolution)
{
  if (g_enable_color)
  {
    EXPECT_TRUE(g_color_recv);
    if (g_color_height_exp > 0)
    {
      EXPECT_EQ(g_color_height_exp, g_height_recv[RS_STREAM_COLOR]);
    }
    if (g_color_width_exp > 0)
    {
      EXPECT_EQ(g_color_width_exp, g_width_recv[RS_STREAM_COLOR]);
    }
  }
}

TEST(RealsenseTests, testColorCameraInfo)
{
  if (g_enable_color)
  {
    EXPECT_EQ(g_width_recv[RS_STREAM_COLOR], g_caminfo_width_recv[RS_STREAM_COLOR]);
    EXPECT_EQ(g_height_recv[RS_STREAM_COLOR], g_caminfo_height_recv[RS_STREAM_COLOR]);
    EXPECT_STREQ(g_dmodel_recv[RS_STREAM_COLOR].c_str(), "plumb_bob");

    // verify rotation is equal to identity matrix
    for (unsigned int i = 0; i < sizeof(realsense_ros_camera::ROTATION_IDENTITY) / sizeof(double); i++)
    {
      EXPECT_EQ(realsense_ros_camera::ROTATION_IDENTITY[i], g_caminfo_rotation_recv[RS_STREAM_COLOR][i]);
    }

    // check projection matrix values are set
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_COLOR][0] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][1], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_COLOR][2] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][3], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][4], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_COLOR][5] != 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_COLOR][6] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][7], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][8], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][9], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_COLOR][10] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][11], 0.0);

    // R200 and ZR300 cameras have Color distortion parameters
    if ((g_camera_type == "R200") || (g_camera_type == "ZR300"))
    {
      bool any_are_zero = false;
      // Ignoring the 5th value since it always appears to be 0.0
      for (unsigned int i = 0; i < 4; i++)
      {
        if (g_color_caminfo_D_recv[i] == 0.0)
        {
          any_are_zero = true;
        }
      }
      EXPECT_FALSE(any_are_zero);
    }
  }
}

TEST(RealsenseTests, testIsDepthStreamEnabled)
{
  if (g_enable_depth)
  {
    EXPECT_TRUE(g_depth_recv);
  }
  else
  {
    EXPECT_FALSE(g_depth_recv);
  }
}

TEST(RealsenseTests, testDepthStream)
{
  if (g_enable_depth)
  {
    EXPECT_GT(g_depth_avg, 0);
    if (!g_depth_encoding_exp.empty())
    {
      EXPECT_EQ(g_depth_encoding_exp, g_encoding_recv[RS_STREAM_DEPTH]);
    }
    if (g_depth_step_exp > 0)
    {
      EXPECT_EQ(g_depth_step_exp, g_step_recv[RS_STREAM_DEPTH]);
    }
  }
}

TEST(RealsenseTests, testDepthResolution)
{
  if (g_enable_depth)
  {
    EXPECT_TRUE(g_depth_recv);
    if (g_depth_height_exp > 0)
    {
      EXPECT_EQ(g_depth_height_exp, g_height_recv[RS_STREAM_DEPTH]);
    }
    if (g_depth_width_exp > 0)
    {
      EXPECT_EQ(g_depth_width_exp, g_width_recv[RS_STREAM_DEPTH]);
    }
  }
}

TEST(RealsenseTests, testDepthCameraInfo)
{
  if (g_enable_depth)
  {
    EXPECT_EQ(g_width_recv[RS_STREAM_DEPTH], g_caminfo_width_recv[RS_STREAM_DEPTH]);
    EXPECT_EQ(g_height_recv[RS_STREAM_DEPTH], g_caminfo_height_recv[RS_STREAM_DEPTH]);
    EXPECT_STREQ(g_dmodel_recv[RS_STREAM_DEPTH].c_str(), "none");

    // verify rotation is equal to identity matrix
    // TODO: Investigate why this is commented out on camera_node.cpp, func 'getStreamCalibData'
    // for (unsigned int i = 0; i < sizeof(realsense_ros_camera::ROTATION_IDENTITY)/sizeof(double); i++)
    // {
    //   EXPECT_EQ(realsense_ros_camera::ROTATION_IDENTITY[i], g_caminfo_rotation_recv[RS_STREAM_DEPTH][i]);
    // }

    // check projection matrix values are set
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][0] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_DEPTH][1], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][2] != 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][3] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_DEPTH][4], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][5] != 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][6] != 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][7] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_DEPTH][8], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_DEPTH][9], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][10] != 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][11] != 0.0);
  }
}


TEST(RealsenseTests, testIsFisheyeStreamEnabled)
{
  if (g_camera_type != "ZR300")
    return;

  if (g_enable_fisheye)
  {
    EXPECT_TRUE(g_fisheye_recv);
  }
  else
  {
    EXPECT_FALSE(g_fisheye_recv);
  }
}


TEST(RealsenseTests, testFisheyeStream)
{
  if (g_camera_type != "ZR300")
    return;

  if (g_enable_fisheye)
  {
    EXPECT_GT(g_fisheye_avg, 0);
  }
}

TEST(RealsenseTests, testFisheyeResolution)
{
  if (g_camera_type != "ZR300")
    return;

  if (g_enable_fisheye)
  {
    EXPECT_TRUE(g_fisheye_recv);
    if (g_fisheye_height_exp > 0)
    {
      EXPECT_EQ(g_fisheye_height_exp, g_height_recv[RS_STREAM_FISHEYE]);
    }
    if (g_fisheye_width_exp > 0)
    {
      EXPECT_EQ(g_fisheye_width_exp, g_width_recv[RS_STREAM_FISHEYE]);
    }
  }
}

TEST(RealsenseTests, testFisheyeCameraInfo)
{
  if (g_camera_type != "ZR300")
    return;

  if (g_enable_fisheye)
  {
    EXPECT_EQ(g_width_recv[RS_STREAM_FISHEYE], g_caminfo_width_recv[RS_STREAM_FISHEYE]);
    EXPECT_EQ(g_height_recv[RS_STREAM_FISHEYE], g_caminfo_height_recv[RS_STREAM_FISHEYE]);
    EXPECT_STREQ(g_dmodel_recv[RS_STREAM_FISHEYE].c_str(), "distortion_ftheta");

    // verify rotation is equal to identity matrix
    for (unsigned int i = 0; i < sizeof(realsense_ros_camera::ROTATION_IDENTITY) / sizeof(double); i++)
    {
      EXPECT_EQ(realsense_ros_camera::ROTATION_IDENTITY[i], g_caminfo_rotation_recv[RS_STREAM_FISHEYE][i]);
    }

    // check projection matrix values are set
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_FISHEYE][0] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_FISHEYE][1], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_FISHEYE][2] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_FISHEYE][3], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_FISHEYE][4], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_FISHEYE][5] != 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_FISHEYE][6] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_FISHEYE][7], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_FISHEYE][8], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_FISHEYE][9], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_FISHEYE][10] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_FISHEYE][11], 0.0);

    // ZR300 cameras have Fisheye distortion parameters
    // Only the first coefficient is used/valid
    if (g_camera_type == "ZR300")
    {
      bool any_are_zero = false;
      for (unsigned int i = 0; i < 1; i++)
      {
        if (g_fisheye_caminfo_D_recv[i] == 0.0)
        {
          any_are_zero = true;
        }
      }
      EXPECT_FALSE(any_are_zero);
    }
  }
}

TEST(RealsenseTests, testImu)
{
  if (g_camera_type != "ZR300")
    return;

  if (g_enable_imu)
  {
    EXPECT_TRUE(g_accel_recv);
    EXPECT_TRUE(g_gyro_recv);
  }
  else
  {
    EXPECT_FALSE(g_accel_recv);
    EXPECT_FALSE(g_gyro_recv);
  }
}

void fillConfigMap(int argc, char **argv)
{
  std::vector<std::string> args;

  for (int i = 1; i < argc; ++i)
  {
    args.push_back(argv[i]);
  }

  while (args.size() > 1)
  {
    g_config_args[args[0]] = args[1];

    args.erase(args.begin());
    args.erase(args.begin());
  }

  if (argc > 1)
  {
    if (g_config_args.find("camera_type") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "camera_type", g_config_args.at("camera_type").c_str());
      g_camera_type = g_config_args.at("camera_type").c_str();
    }

    // Set depth arguments.
    if (g_config_args.find("enable_depth") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "enable_depth", g_config_args.at("enable_depth").c_str());
      if (strcmp((g_config_args.at("enable_depth").c_str()), "true") == 0)
      {
        g_enable_depth = true;
      }
      else
      {
        g_enable_depth = false;
      }
    }

    if (g_config_args.find("depth_encoding") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "depth_encoding", g_config_args.at("depth_encoding").c_str());
      g_depth_encoding_exp = g_config_args.at("depth_encoding").c_str();
    }
    if (g_config_args.find("depth_height") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "depth_height", g_config_args.at("depth_height").c_str());
      g_depth_height_exp = atoi(g_config_args.at("depth_height").c_str());
    }
    if (g_config_args.find("depth_width") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "depth_width", g_config_args.at("depth_width").c_str());
      g_depth_width_exp = atoi(g_config_args.at("depth_width").c_str());
    }
    if (g_config_args.find("depth_step") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "depth_step", g_config_args.at("depth_step").c_str());
      g_depth_step_exp = atoi(g_config_args.at("depth_step").c_str());
    }

    // Set color arguments.
    if (g_config_args.find("enable_color") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "enable_color", g_config_args.at("enable_color").c_str());
      if (strcmp((g_config_args.at("enable_color").c_str()), "true") == 0)
      {
        g_enable_color = true;
      }
      else
      {
        g_enable_color = false;
      }
    }
    if (g_config_args.find("color_encoding") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "color_encoding", g_config_args.at("color_encoding").c_str());
      g_color_encoding_exp = g_config_args.at("color_encoding").c_str();
    }
    if (g_config_args.find("color_height") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "color_height", g_config_args.at("color_height").c_str());
      g_color_height_exp = atoi(g_config_args.at("color_height").c_str());
    }
    if (g_config_args.find("color_width") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "color_width", g_config_args.at("color_width").c_str());
      g_color_width_exp = atoi(g_config_args.at("color_width").c_str());
    }
    if (g_config_args.find("color_step") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "color_step", g_config_args.at("color_step").c_str());
      g_color_step_exp = atoi(g_config_args.at("color_step").c_str());
    }

    // Set fisheye arguments.
    if (g_config_args.find("enable_fisheye") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "enable_fisheye",
               g_config_args.at("enable_fisheye").c_str());
      if (strcmp((g_config_args.at("enable_fisheye").c_str()), "true") == 0)
      {
        g_enable_fisheye = true;
        g_enable_imu = true;
      }
      else
      {
        g_enable_fisheye = false;
        g_enable_imu = false;
      }
    }

    if (g_config_args.find("fisheye_height") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "fisheye_height", g_config_args.at("fisheye_height").c_str());
      g_depth_height_exp = atoi(g_config_args.at("fisheye_height").c_str());
    }
    if (g_config_args.find("fisheye_width") != g_config_args.end())
    {
      ROS_INFO("RealSense camera test - Setting %s to %s", "fisheye_width", g_config_args.at("fisheye_width").c_str());
      g_depth_width_exp = atoi(g_config_args.at("fisheye_width").c_str());
    }
  }
}

int main(int argc, char **argv) //try
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "utest");
  ros::NodeHandle n;
  ros::NodeHandle nh(n, "camera");

  image_transport::CameraSubscriber camera_subscriber[STREAM_COUNT];
  ros::Subscriber sub_accel;
  ros::Subscriber sub_gyro;

  fillConfigMap(argc, argv);

  ROS_INFO_STREAM("RealSense camera test - Initializing Tests...");

  ros::NodeHandle depth_nh(nh, "depth");
  image_transport::ImageTransport depth_image_transport(depth_nh);
  camera_subscriber[0] = depth_image_transport.subscribeCamera("image_raw", 1, imageDepthCallback, 0);

  ros::NodeHandle color_nh(nh, "color");
  image_transport::ImageTransport color_image_transport(color_nh);
  camera_subscriber[1] = color_image_transport.subscribeCamera("image_raw", 1, imageColorCallback, 0);

  // ZR300 cameras have Fisheye and IMU
  ros::NodeHandle fisheye_nh(nh, "fisheye");
  image_transport::ImageTransport fisheye_image_transport(fisheye_nh);
  if (g_camera_type == "ZR300")
  {
    camera_subscriber[4] = fisheye_image_transport.subscribeCamera("image_raw", 1, imageFisheyeCallback, 0);
    sub_accel = n.subscribe<sensor_msgs::Imu>("camera/accel/sample", 1, accelCallback);
    sub_gyro = n.subscribe<sensor_msgs::Imu>("camera/gyro/sample", 1, gyroCallback);
  }

  ros::Duration duration;
  duration.sec = 5;
  duration.sleep();
  ros::spinOnce();

  ROS_INFO_STREAM("RealSense camera test - Running Tests...");

  return RUN_ALL_TESTS();
}
//catch(...) {}  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
