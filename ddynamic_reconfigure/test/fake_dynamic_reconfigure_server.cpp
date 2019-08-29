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
  ros::init(argc, argv, "fake_dynamic_reconfigure");

  ros::NodeHandle nh("fake_dyanmic_reconfigure");

  double double_test = 0.0;
  double double_range = 2;
  int int_test = 0;
  bool bool_test = false;
  double changing_variable = 0.0;
  std::string str_test = "";
  DDynamicReconfigure ddr(nh);
  DDynamicReconfigure ddr2(ros::NodeHandle(nh, "nh2"));
  DDynamicReconfigure ddr3(ros::NodeHandle(nh, "nh3"));

  ddr.RegisterVariable(&double_test, "double_test");
  ddr.RegisterVariable(&double_range, "double_range_test", 0, 10);
  ddr.RegisterVariable(&int_test, "int_test");
  ddr.RegisterVariable(&bool_test, "bool_test");
  ddr.registerVariable("str_test", &str_test);
  ddr.RegisterVariable(&changing_variable, "changing_variable");
  
  std::map<std::string, int> enum_map = {{"ZERO", 0}, {"ONE", 1}, {"ONE_HUNDRED", 100}};
  ddr.registerEnumVariable<int>("enum_int", enum_map["ONE"], [](int new_value) {
    ROS_INFO_STREAM("Value changed to " << new_value);
  }, "Enum parameter", enum_map, "enum description");
  
  std::map<std::string, std::string> str_enum_map = {{"ZERO", "zero"}, {"ONE", "one"}, {"ONE_HUNDRED", "one hundred"}};
  ddr.registerEnumVariable<std::string>("enum_string", str_enum_map["ONE"], [](std::string new_value) {
    ROS_INFO_STREAM("Value changed to " << new_value);
  }, "Enum parameter", str_enum_map, "enum description");
  
  std::map<std::string, double> double_enum_map = {{"ZERO", 0.0}, {"ONE", 1.1}, {"ONE_HUNDRED", 100.001}};
  ddr.registerEnumVariable<double>("enum_double", double_enum_map["ONE"], [](double new_value) {
    ROS_INFO_STREAM("Value changed to " << new_value);
  }, "Enum parameter", double_enum_map, "enum description");
  
  
  std::map<std::string, bool> bool_enum_map = {{"false", false}, {"true", true}, {"also true", true}};
  ddr.registerEnumVariable<bool>("enum_bool", bool_enum_map["ONE"], [](bool new_value) {
    ROS_INFO_STREAM("Value changed to " << new_value);
  }, "Enum parameter", bool_enum_map, "enum description");
  
  ddr2.RegisterVariable(&double_test, "double_test");
  ddr2.RegisterVariable(&int_test, "int_test");
  ddr2.RegisterVariable(&bool_test, "bool_test");

  ddr.PublishServicesTopics();
  ddr2.PublishServicesTopics();
  ddr3.PublishServicesTopics();


  ROS_INFO("Spinning node");

  while (nh.ok())
  {
    changing_variable += 0.5;
    std::cerr << "changing_variable " << changing_variable << std::endl;
    std::cerr << "double " << double_test << std::endl;
    std::cerr << "double range" << double_range << std::endl;
    std::cerr << "int " << int_test << std::endl;
    std::cerr << "bool " << bool_test << std::endl;
    std::cerr << "str " << str_test << std::endl;
    std::cerr << "*********" << std::endl;
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
  return 0;
}
