#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

using namespace ddynamic_reconfigure;

/**
  Topics:
  * /dynamic_tutorials/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
  * /dynamic_tutorials/parameter_updates [dynamic_reconfigure/Config]

  Services:
  * /dynamic_tutorials/set_parameter:  dynamic_reconfigure/Reconfigure
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_bool_dynamic_reconfigure");

  ros::NodeHandle nh("fake_dyanmic_reconfigure");

  bool bool_test = false;

  DDynamicReconfigure ddr(nh);

  ddr.RegisterVariable(&bool_test, "bool_test");

  ddr.PublishServicesTopics();

  ROS_INFO("Spinning node");

  while (nh.ok())
  {
    std::cerr << "bool " << bool_test << std::endl;
    std::cerr << "*********" << std::endl;
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  return 0;
}
