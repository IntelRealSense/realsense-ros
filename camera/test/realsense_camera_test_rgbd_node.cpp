#include "realsense_camera_test_rgbd_node.h"
using namespace std;

void topic0Callback(const sensor_msgs::ImageConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_0_recv = true;
  }
}

void topic1Callback(const sensor_msgs::ImageConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_1_recv = true;
  }
}

void topic2Callback(const sensor_msgs::ImageConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_2_recv = true;
  }
}

void topic3Callback(const sensor_msgs::ImageConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_3_recv = true;
  }
}

void topic4Callback(const sensor_msgs::ImageConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_4_recv = true;
  }
}

void topic5Callback(const sensor_msgs::ImageConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_5_recv = true;
  }
}

void topic6Callback(const sensor_msgs::ImageConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_6_recv = true;
  }
}

void topic7Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_7_recv = true;
  }
}

void topic8Callback(const sensor_msgs::ImageConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_8_recv = true;
  }
}

void topic9Callback(const sensor_msgs::ImageConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_9_recv = true;
  }
}

void topic10Callback(const sensor_msgs::CameraInfoConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_10_recv = true;
  }
}

void topic11Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_11_recv = true;
  }
}

void topic12Callback(const sensor_msgs::ImageConstPtr msg)
{
  if ((msg->height != 0) && (msg->width != 0))
  {
    topic_12_recv = true;
  }
}

TEST (RealsenseTests, RGB_IMAGE_MONO)
{
  EXPECT_TRUE (topic_0_recv);
}

TEST (RealsenseTests, RGB_IMAGE_COLOR)
{
  EXPECT_TRUE (topic_1_recv);
}

TEST (RealsenseTests, RGB_IMAGE_RECT_MONO)
{
  EXPECT_TRUE (topic_2_recv);
}

TEST (RealsenseTests, RGB_IMAGE_RECT_COLOR)
{
  EXPECT_TRUE (topic_3_recv);
}

TEST (RealsenseTests, DEPTH_IMAGE_RECT_RAW)
{
  EXPECT_TRUE (topic_4_recv);
}

TEST (RealsenseTests, DEPTH_IMAGE_RECT)
{
  EXPECT_TRUE (topic_5_recv);
}

TEST (RealsenseTests, DEPTH_IMAGE)
{
  EXPECT_TRUE (topic_6_recv);
}

TEST (RealsenseTests, DEPTH_POINTS)
{
  EXPECT_TRUE (topic_7_recv);
}

TEST (RealsenseTests, IR_IMAGE_RECT_IR)
{
  EXPECT_TRUE (topic_8_recv);
}

TEST (RealsenseTests, DEPTH_REG_SW_REG_IMAGE_RECT_RAW)
{
  EXPECT_TRUE (topic_9_recv);
}

TEST (RealsenseTests, DEPTH_REG_SW_REG_CAMERA_INFO)
{
  EXPECT_TRUE (topic_10_recv);
}

TEST (RealsenseTests, DEPTH_REG_POINTS)
{
  EXPECT_TRUE (topic_11_recv);
}

TEST (RealsenseTests, DEPTH_REG_SW_REG_IMAGE_RECT)
{
  EXPECT_TRUE (topic_12_recv);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "utest");

  ROS_INFO_STREAM("RealSense Camera - Starting rgbd_launch Tests...");

  ros::NodeHandle nh;

  subscriber[0] = nh.subscribe < sensor_msgs::Image > (RGB_IMAGE_MONO, 1, topic0Callback, 0);
  subscriber[1] = nh.subscribe < sensor_msgs::Image > (RGB_IMAGE_COLOR, 1, topic1Callback, 0);
  subscriber[2] = nh.subscribe < sensor_msgs::Image > (RGB_IMAGE_RECT_MONO, 1, topic2Callback, 0);
  subscriber[3] = nh.subscribe < sensor_msgs::Image > (RGB_IMAGE_RECT_COLOR, 1, topic3Callback, 0);
  subscriber[4] = nh.subscribe < sensor_msgs::Image > (DEPTH_IMAGE_RECT_RAW, 1, topic4Callback, 0);
  subscriber[5] = nh.subscribe < sensor_msgs::Image > (DEPTH_IMAGE_RECT, 1, topic5Callback, 0);
  subscriber[6] = nh.subscribe < sensor_msgs::Image > (DEPTH_IMAGE, 1, topic6Callback, 0);
  subscriber[7] = nh.subscribe < sensor_msgs::PointCloud2 > (DEPTH_POINTS, 1, topic7Callback);
  subscriber[8] = nh.subscribe < sensor_msgs::Image > (IR_IMAGE_RECT_IR, 1, topic8Callback, 0);
  subscriber[9] = nh.subscribe < sensor_msgs::Image > (DEPTH_REG_SW_REG_IMAGE_RECT_RAW, 1, topic9Callback, 0);
  subscriber[10] = nh.subscribe < sensor_msgs::CameraInfo > (DEPTH_REG_SW_REG_CAMERA_INFO, 1, topic10Callback);
  subscriber[11] = nh.subscribe < sensor_msgs::PointCloud2 > (DEPTH_REG_POINTS, 1, topic11Callback, 0);
  subscriber[12] = nh.subscribe < sensor_msgs::Image > (DEPTH_REG_SW_REG_IMAGE_RECT, 1, topic12Callback, 0);

  ros::Duration duration;
  duration.sec = 10;
  duration.sleep();
  ros::spinOnce();

  return RUN_ALL_TESTS();
}
