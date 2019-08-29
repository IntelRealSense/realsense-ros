#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

using namespace ddynamic_reconfigure;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_dynamic_reconfigure");

  ros::NodeHandle nh("fake_dyanmic_reconfigure");

  double value = 0;

  DDynamicReconfigure ddr_1(ros::NodeHandle(nh, "ddr1"));
  ddr_1.RegisterVariable(&value, "value");
  ddr_1.PublishServicesTopics();

  DDynamicReconfigure ddr_2(ros::NodeHandle(nh, "ddr2"));
  ddr_2.RegisterVariable(&value, "value");
  ddr_2.PublishServicesTopics();

  while (nh.ok())
  {
    ros::spinOnce();
    ROS_INFO_STREAM(value);
    ros::Duration(0.1).sleep();
  }
}

