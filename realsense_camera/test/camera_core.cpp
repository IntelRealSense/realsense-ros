/******************************************************************************
 Copyright (c) 2016, Intel Corporation
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/
#include "gtest/gtest.h"
#include "camera_core.h"

using namespace std;
using namespace realsense_camera;

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
  for (unsigned int i = 0; i < sizeof(info_msg->R)/sizeof(double); i++)
  {
    g_caminfo_rotation_recv[stream][i] = info_msg->R[i];
  }

  // copy projection matrix
  for (unsigned int i = 0; i < sizeof(info_msg->P)/sizeof(double); i++)
  {
    g_caminfo_projection_recv[stream][i] = info_msg->P[i];
  }
}

void imageInfrared1Callback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
  cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;

  uchar *infrared1_data = image.data;

  long infrared1_total = 0;
  int infrared1_count = 1;
  for (unsigned int i = 0; i < msg->height * msg->width; i++)
  {
    if (*infrared1_data > 0 && *infrared1_data < 255)
    {
      infrared1_total += *infrared1_data;
      infrared1_count++;
    }
    infrared1_data++;
  }
  if (infrared1_count != 0)
  {
    g_infrared1_avg = infrared1_total / infrared1_count;
  }

  getMsgInfo(RS_STREAM_INFRARED, msg);
  getCameraInfo(RS_STREAM_INFRARED, info_msg);

  g_infrared1_recv = true;
}

void imageInfrared2Callback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
  cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;

  uchar *infrared2_data = image.data;

  long infrared2_total = 0;
  int infrared2_count = 1;
  for (unsigned int i = 0; i < msg->height * msg->width; i++)
  {
    if (*infrared2_data > 0 && *infrared2_data < 255)
    {
      infrared2_total += *infrared2_data;
      infrared2_count++;
    }
    infrared2_data++;
  }
  if (infrared2_count != 0)
  {
    g_infrared2_avg = infrared2_total / infrared2_count;
  }

  getMsgInfo(RS_STREAM_INFRARED2, msg);
  getCameraInfo(RS_STREAM_INFRARED2, info_msg);

  g_infrared2_recv = true;
}

void imageDepthCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
  cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  uint16_t *image_data = (uint16_t *) image.data;

  long depth_total = 0;
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
    g_depth_avg = depth_total / depth_count;
  }

  getMsgInfo(RS_STREAM_DEPTH, msg);
  getCameraInfo(RS_STREAM_DEPTH, info_msg);

  g_depth_recv = true;
}


void pcCallback(const sensor_msgs::PointCloud2ConstPtr pc)
{
  pcl::PointCloud < pcl::PointXYZRGB > pointcloud;
  pcl::fromROSMsg(*pc, pointcloud);

  long pc_depth_total = 0;
  int pc_depth_count = 0;
  for (unsigned int i = 0; i < pointcloud.width * pointcloud.height; ++i)
  {
    pcl::PointXYZRGB point = pointcloud.points[i];
    float pc_depth = (float) std::ceil(point.z);
    if ((0 < pc_depth) && (pc_depth <= g_max_z))
    {
      pc_depth_total += pc_depth;
      pc_depth_count++;
    }
  }
  if (pc_depth_count != 0)
  {
    g_pc_depth_avg = pc_depth_total / pc_depth_count;
  }

  g_pc_recv = true;
}


void imageColorCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
  cv::Mat image = cv_bridge::toCvShare(msg, "rgb8")->image;

  uchar *color_data = image.data;
  long color_total = 0;
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
    g_color_avg = color_total / color_count;
  }

  getMsgInfo(RS_STREAM_COLOR, msg);
  getCameraInfo(RS_STREAM_COLOR, info_msg);

  for (unsigned int i = 0; i < 5; i++)
  {
    g_color_caminfo_D_recv[i] = info_msg->D[i];
  }

  g_color_recv = true;
}

TEST(RealsenseTests, testColorStream)
{
  if (g_enable_color)
  {
    EXPECT_TRUE(g_color_avg > 0);
    EXPECT_TRUE(g_color_recv);

    if (!g_color_encoding_exp.empty ())
    {
      EXPECT_EQ(g_color_encoding_exp, g_encoding_recv[RS_STREAM_COLOR]);
    }
    if (g_color_step_exp > 0)
    {
      EXPECT_EQ(g_color_step_exp, g_step_recv[RS_STREAM_COLOR]);
    }
  }
  else
  {
    EXPECT_FALSE(g_color_recv);
  }
}

TEST(RealsenseTests, testColorResolution)
{
  if (g_enable_color)
  {
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
    for (unsigned int i = 0; i < sizeof(ROTATION_IDENTITY)/sizeof(double); i++)
    {
      EXPECT_EQ(ROTATION_IDENTITY[i], g_caminfo_rotation_recv[RS_STREAM_COLOR][i]);
    }

    // check projection matrix values are set
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_COLOR][0] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][1], (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_COLOR][2] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][3], (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][4], (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_COLOR][5] != (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_COLOR][6] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][7], (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][8], (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][9], (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_COLOR][10] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_COLOR][11], (double) 0);

    float color_caminfo_D = 1;
    // ignoring the 5th value since it always appears to be 0.0 on tested R200 cameras
    for (unsigned int i = 0; i < 4; i++)
    {
      color_caminfo_D = color_caminfo_D && g_color_caminfo_D_recv[i];
    }

    EXPECT_TRUE(color_caminfo_D != (float) 0);

  }
}

TEST(RealsenseTests, testIsDepthStreamEnabled)
{
  if (g_enable_depth)
  {
    EXPECT_TRUE(g_depth_recv);
    EXPECT_TRUE(g_infrared1_recv);
    if (g_camera_type == "R200")
    {
      EXPECT_TRUE(g_infrared2_recv);
    }
  }
  else
  {
    EXPECT_FALSE(g_depth_recv);
    EXPECT_FALSE(g_infrared1_recv);
    if (g_camera_type == "R200")
    {
      EXPECT_FALSE(g_infrared2_recv);
    }
  }
}

TEST(RealsenseTests, testDepthStream)
{
  if (g_enable_depth)
  {
    ROS_INFO_STREAM("RealSense Camera - depth_avg: " << g_depth_avg << " mm");
    EXPECT_TRUE(g_depth_avg > 0);
    EXPECT_TRUE(g_depth_recv);
    if (!g_depth_encoding_exp.empty ())
    {
      EXPECT_EQ(g_depth_encoding_exp, g_encoding_recv[RS_STREAM_DEPTH]);
    }
    if (g_depth_step_exp > 0)
    {
      EXPECT_EQ(g_depth_step_exp, g_step_recv[RS_STREAM_DEPTH]);
    }
  }
  else
  {
    EXPECT_FALSE(g_depth_recv);
  }
}

TEST(RealsenseTests, testDepthResolution)
{
  if (g_enable_depth)
  {
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
    EXPECT_STREQ(g_dmodel_recv[RS_STREAM_DEPTH].c_str (), "plumb_bob");

    // verify rotation is equal to identity matrix
    for (unsigned int i = 0; i < sizeof(ROTATION_IDENTITY)/sizeof(double); i++)
    {
      EXPECT_EQ(ROTATION_IDENTITY[i], g_caminfo_rotation_recv[RS_STREAM_DEPTH][i]);
    }

    // check projection matrix values are set
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][0] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_DEPTH][1], (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][2] != (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][3] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_DEPTH][4], (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][5] != (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][6] != (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][7] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_DEPTH][8], (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_DEPTH][9], (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][10] != (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_DEPTH][11] != (double) 0);
  }
}

TEST(RealsenseTests, testInfrared1Stream)
{
  if (g_enable_depth)
  {
    EXPECT_TRUE(g_infrared1_avg > 0);
    EXPECT_TRUE(g_infrared1_recv);
    if (!g_infrared1_encoding_exp.empty ())
    {
      EXPECT_EQ(g_infrared1_encoding_exp, g_encoding_recv[RS_STREAM_INFRARED]);
    }
    if (g_infrared1_step_exp > 0)
    {
      EXPECT_EQ(g_infrared1_step_exp, g_step_recv[RS_STREAM_INFRARED]);
    }
  }
  else
  {
    EXPECT_FALSE(g_infrared1_recv);
  }
}

TEST(RealsenseTests, testInfrared1Resolution)
{
  if (g_enable_depth)
  {
    if (g_depth_width_exp > 0)
    {
      EXPECT_EQ(g_depth_width_exp, g_width_recv[RS_STREAM_INFRARED]);
    }
    if (g_depth_height_exp > 0)
    {
      EXPECT_EQ(g_depth_height_exp, g_height_recv[RS_STREAM_INFRARED]);
    }
  }
}

TEST(RealsenseTests, testInfrared1CameraInfo)
{
  if (g_enable_depth)
  {
    EXPECT_EQ(g_width_recv[RS_STREAM_INFRARED], g_caminfo_width_recv[RS_STREAM_INFRARED]);
    EXPECT_EQ(g_height_recv[RS_STREAM_INFRARED], g_caminfo_height_recv[RS_STREAM_INFRARED]);
    EXPECT_STREQ(g_dmodel_recv[RS_STREAM_INFRARED].c_str (), "plumb_bob");

    // verify rotation is equal to identity matrix
    for (unsigned int i = 0; i < sizeof(ROTATION_IDENTITY)/sizeof(double); i++)
    {
      EXPECT_EQ(ROTATION_IDENTITY[i], g_caminfo_rotation_recv[RS_STREAM_INFRARED][i]);
    }

    // check projection matrix values are set
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_INFRARED][0] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED][1], (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_INFRARED][2] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED][3], (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED][4], (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_INFRARED][5] != (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_INFRARED][6] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED][7], (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED][8], (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED][9], (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_INFRARED][10] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED][11], (double) 0);
  }
}

TEST(RealsenseTests, testInfrared2Stream)
{
  if (g_enable_depth)
  {
    EXPECT_TRUE(g_infrared2_avg > 0);
    EXPECT_TRUE(g_infrared2_recv);
  }
  else
  {
    EXPECT_FALSE(g_infrared2_recv);
  }
}

TEST(RealsenseTests, testInfrared2Resolution)
{
  if (g_enable_depth)
  {
    if (g_depth_width_exp > 0)
    {
      EXPECT_EQ(g_depth_width_exp, g_width_recv[RS_STREAM_INFRARED2]);
    }
    if (g_depth_height_exp > 0)
    {
      EXPECT_EQ(g_depth_height_exp, g_height_recv[RS_STREAM_INFRARED2]);
    }
  }
}

TEST(RealsenseTests, testInfrared2CameraInfo)
{
  if (g_enable_depth)
  {
    EXPECT_EQ(g_width_recv[RS_STREAM_INFRARED2], g_caminfo_width_recv[RS_STREAM_INFRARED2]);
    EXPECT_EQ(g_height_recv[RS_STREAM_INFRARED2], g_caminfo_height_recv[RS_STREAM_INFRARED2]);
    EXPECT_STREQ(g_dmodel_recv[RS_STREAM_INFRARED2].c_str (), "plumb_bob");

    // verify rotation is equal to identity matrix
    for (unsigned int i = 0; i < sizeof(ROTATION_IDENTITY)/sizeof(double); i++)
    {
      EXPECT_EQ(ROTATION_IDENTITY[i], g_caminfo_rotation_recv[RS_STREAM_INFRARED2][i]);
    }

    // check projection matrix values are set
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_INFRARED2][0] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED2][1], (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_INFRARED2][2] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED2][3], (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED2][4], (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_INFRARED2][5] != (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_INFRARED2][6] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED2][7], (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED2][8], (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED2][9], (double) 0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS_STREAM_INFRARED2][10] != (double) 0);
    EXPECT_EQ(g_caminfo_projection_recv[RS_STREAM_INFRARED2][11], (double) 0);
  }
}


 TEST(RealsenseTests, testPointCloud)
{
  if (g_enable_pointcloud)
  {
    ROS_INFO_STREAM("RealSense Camera - pc_depth_avg: " << g_pc_depth_avg);
    EXPECT_TRUE(g_pc_depth_avg > 0);
    EXPECT_TRUE(g_pc_recv);
  }
  else
  {
    EXPECT_FALSE(g_pc_recv);
  }
}

TEST(RealsenseTests, testTransforms)
{
  // make sure all transforms are being broadcast as expected
  tf::TransformListener tf_listener;

  EXPECT_TRUE(tf_listener.waitForTransform(DEFAULT_DEPTH_FRAME_ID, DEFAULT_BASE_FRAME_ID, ros::Time(0),
      ros::Duration(3.0)));
  EXPECT_TRUE(tf_listener.waitForTransform(DEFAULT_DEPTH_OPTICAL_FRAME_ID, DEFAULT_DEPTH_FRAME_ID, ros::Time(0),
      ros::Duration(3.0)));
  EXPECT_TRUE(tf_listener.waitForTransform(DEFAULT_COLOR_FRAME_ID, DEFAULT_BASE_FRAME_ID, ros::Time(0),
      ros::Duration(3.0)));
  EXPECT_TRUE(tf_listener.waitForTransform(DEFAULT_COLOR_OPTICAL_FRAME_ID, DEFAULT_COLOR_FRAME_ID, ros::Time(0),
      ros::Duration(3.0)));
}

TEST(RealsenseTests, testCameraOptions)
{
  g_service_client.call(g_srv);
  stringstream settings_ss (g_srv.response.configuration_str);
  string setting;
  string setting_name;
  string setting_value;

  while (getline (settings_ss, setting, ';'))
  {
    stringstream setting_ss (setting);
    getline (setting_ss, setting_name, ':');
    setting_value = (setting.substr (setting.rfind (":") + 1));
    if (g_config_args.find (setting_name) != g_config_args.end ())
    {
      int option_recv = atoi(setting_value.c_str());
      int option_exp = atoi(g_config_args.at (setting_name).c_str());
      EXPECT_EQ(option_exp, option_recv) << setting_name;
    }
  }
}

TEST(RealsenseTests, testGetSettingsService)
{
  // Verify the service is available
  EXPECT_TRUE(g_service_client.call(g_srv));
}

void fillConfigMap(int argc, char **argv)
{
  std::vector < std::string > args;

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
      ROS_INFO("RealSense Camera - Setting %s to %s", "camera_type", g_config_args.at("camera_type").c_str());
      g_camera_type = g_config_args.at("camera_type").c_str();
    }

    // Set depth arguments.
    if (g_config_args.find("enable_depth") != g_config_args.end())
    {
      ROS_INFO("RealSense Camera - Setting %s to %s", "enable_depth", g_config_args.at("enable_depth").c_str());
      if (strcmp((g_config_args.at("enable_depth").c_str ()),"true") == 0)
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
      ROS_INFO("RealSense Camera - Setting %s to %s", "depth_encoding", g_config_args.at("depth_encoding").c_str());
      g_depth_encoding_exp = g_config_args.at("depth_encoding").c_str();
    }
    if (g_config_args.find("depth_height") != g_config_args.end())
    {
      ROS_INFO("RealSense Camera - Setting %s to %s", "depth_height", g_config_args.at("depth_height").c_str());
      g_depth_height_exp = atoi(g_config_args.at("depth_height").c_str());
    }
    if (g_config_args.find("depth_width") != g_config_args.end())
    {
      ROS_INFO("RealSense Camera - Setting %s to %s", "depth_width", g_config_args.at("depth_width").c_str());
      g_depth_width_exp = atoi(g_config_args.at("depth_width").c_str());
    }
    if (g_config_args.find("depth_step") != g_config_args.end())
    {
      ROS_INFO("RealSense Camera - Setting %s to %s", "depth_step", g_config_args.at("depth_step").c_str());
      g_depth_step_exp = atoi(g_config_args.at("depth_step").c_str());
    }

    // Set color arguments.
    if (g_config_args.find("enable_color") != g_config_args.end())
    {
      ROS_INFO("RealSense Camera - Setting %s to %s", "enable_color", g_config_args.at("enable_color").c_str());
      if (strcmp((g_config_args.at("enable_color").c_str ()),"true") == 0)
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
      ROS_INFO("RealSense Camera - Setting %s to %s", "color_encoding", g_config_args.at("color_encoding").c_str());
      g_color_encoding_exp = g_config_args.at("color_encoding").c_str();
    }
    if (g_config_args.find("color_height") != g_config_args.end())
    {
      ROS_INFO("RealSense Camera - Setting %s to %s", "color_height", g_config_args.at("color_height").c_str());
      g_color_height_exp = atoi(g_config_args.at("color_height").c_str());
    }
    if (g_config_args.find("color_width") != g_config_args.end())
    {
      ROS_INFO("RealSense Camera - Setting %s to %s", "color_width", g_config_args.at("color_width").c_str());
      g_color_width_exp = atoi(g_config_args.at("color_width").c_str());
    }
    if (g_config_args.find("color_step") != g_config_args.end())
    {
      ROS_INFO("RealSense Camera - Setting %s to %s", "color_step", g_config_args.at("color_step").c_str());
      g_color_step_exp = atoi(g_config_args.at("color_step").c_str());
    }

    // Set pointcloud arguments.
    if (g_config_args.find("enable_pointcloud") != g_config_args.end())
    {
      ROS_INFO("RealSense Camera - Setting %s to %s", "enable_pointcloud",
          g_config_args.at("enable_pointcloud").c_str());
      if (strcmp((g_config_args.at("enable_pointcloud").c_str()),"true") == 0)
      {
        g_enable_pointcloud = true;
      }
      else
      {
        g_enable_pointcloud = false;
      }
    }
  }
}

int main(int argc, char **argv) try
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "utest");
  fillConfigMap(argc, argv);

  ROS_INFO_STREAM("RealSense Camera - Starting Tests...");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  g_camera_subscriber[0] = it.subscribeCamera(DEPTH_TOPIC, 1, imageDepthCallback, 0);
  g_camera_subscriber[1] = it.subscribeCamera(COLOR_TOPIC, 1, imageColorCallback, 0);
  g_camera_subscriber[2] = it.subscribeCamera(IR_TOPIC, 1, imageInfrared1Callback, 0);
  if (g_camera_type == "R200")
  {
    g_camera_subscriber[3] = it.subscribeCamera(IR2_TOPIC, 1, imageInfrared2Callback, 0);
  }

  g_sub_pc = nh.subscribe <sensor_msgs::PointCloud2> (PC_TOPIC, 1, pcCallback);
  g_service_client = nh.serviceClient < realsense_camera::cameraConfiguration > (SETTINGS_SERVICE);

  ros::Duration duration;
  duration.sec = 10;
  duration.sleep();
  ros::spinOnce();

  return RUN_ALL_TESTS();
}
catch(...) {} // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
