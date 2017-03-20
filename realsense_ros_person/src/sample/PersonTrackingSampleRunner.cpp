// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "PersonTrackingSample.h"

int main(int argc, char **argv)
{
  ROS_INFO("Start person tracking sample");

  ros::init(argc, argv, "realsense_ros_person_sample");
  ros::NodeHandle nodeHandle;

  PersonTrackingSample test;
  test.ProcessCommandLineArgs();
  test.InitMessaging(nodeHandle);

  ros::spin();

  return 0;
}
