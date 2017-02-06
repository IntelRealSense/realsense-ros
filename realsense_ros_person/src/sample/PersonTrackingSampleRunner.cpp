//#include <ros/ros.h>
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
