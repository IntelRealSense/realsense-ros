#include "ros/ros.h"
#include <ros/console.h>
#include "PersonTrackingNodelet.h"

int main(int argc, char **argv)
{
	ROS_INFO("Start person tracking node runner");

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }
	
    ros::init(argc, argv, "person_tracking");
	ros::NodeHandle nodeHandle;
	
	person_tracking::PersonTrackingNodelet personTracking;
    personTracking.onInit(nodeHandle);
	
	ros::spin();
	
	return 0;
}
