#include "ros/ros.h"
#include "PersonTrackingTest.h"

int main(int argc, char **argv)
{
    ROS_INFO("Start person tracking test");

    ros::init(argc, argv, "perosn_tracking_test");
    ros::NodeHandle nodeHandle;

    PersonTrackingTest test;
    test.ProcessCommandLineArgs();
    test.InitMessaging(nodeHandle);

    ros::spin();

    return 0;
}
