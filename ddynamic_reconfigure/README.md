#ddynamic_reconfigure

The ddynamic_reconfigure package is a C++ extension of [dynamic_reconfigure](https://github.com/ros/dynamic_reconfigure) that allows modifying parameters of a ROS node using the dynamic_reconfigure framework without having to write cfg files.

## Usage

Modifying in place a variable:
```cpp
#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

int main(int argc, char **argv) {
    // ROS init stage
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh;
    ddynamic_reconfigure::DDynamicReconfigure ddr;
    int int_param = 0;
    ddr.registerVariable<int>("int_param", &int_param, "param description");
    ddr.publishServicesTopics();
    // Now parameter can be modified from the dynamic_reconfigure GUI or other tools and the variable int_param is updated automatically
    
    int_param = 10; //This will also update the dynamic_reconfigure tools with the new value 10
    ros::spin();
    return 0;
 }
```

Modifying a variable via a callback:
```cpp
#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

int global_int;

void paramCb(int new_value)
{
   global_int = new_value;
   ROS_INFO("Param modified");
}

int main(int argc, char **argv) {
    // ROS init stage
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh;
    ddynamic_reconfigure::DDynamicReconfigure ddr;
    
    ddr.registerVariable<int>("int_param", 10 /* initial value */, boost::bind(paramCb, _1), "param description");
    ddr.publishServicesTopics();
    // Now parameter can be modified from the dynamic_reconfigure GUI or other tools and the callback is called on each update
    ros::spin();
    return 0;
 }
```

Registering an enum:

```cpp

#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

int main(int argc, char **argv) {
    // ROS init stage
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh;
    ddynamic_reconfigure::DDynamicReconfigure ddr;
    
    std::map<std::string, std::string> enum_map = {{"Key 1", "Value 1"}, {"Key 2", "Value 2"}};
    std::string enum_value = enum_map["Key 1"];
    ddr.registerEnumVariable<std::string>("string_enum", &enum_value,"param description", enum_map);
    ddr.publishServicesTopics();
    ros::spin();
    return 0;
 }
```

## Issues
### Undefined reference to registerVariable or registerEnumVariable

These methods are templated, but the implementation is hidden, and there are explicit template instantiations for `int`, `bool`, `double` and `std::string`. If you are getting an undefined reference to one of these methods, make sure that you are passing parameters of this type.





