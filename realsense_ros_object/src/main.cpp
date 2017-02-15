// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.

#include "ros/ros.h"

#include "realsense_ros_object/TrackedObject.h"
#include "realsense_ros_object/TrackedObjectsArray.h"
#include "realsense_ros_object/ObjectInBox.h"
#include "realsense_ros_object/ObjectsInBoxes.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "realsense_localization_nodelet.h"
#include "realsense_recognition_nodelet.h"
#include "realsense_tracking_nodelet.h"

#include <pluginlib/class_list_macros.h>


#ifdef RUN_AS_NODELET         

PLUGINLIB_EXPORT_CLASS(realsense_ros_object::CLocalizationNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(realsense_ros_object::CTrackingNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(realsense_ros_object::CRecognitionNodelet, nodelet::Nodelet)

#else

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "realsense_ros_object");
	ros::NodeHandle nh;
	
	realsense::CLocalization objectsTracker;
	realsense::CRecognition objectsRecognizer;
	realsense::CTracking objectsTracker;
	
	std::vector<std::string> argvStrings;
	for(int i=0; i < argc; i++)
	{
		argvStrings.push_back(argv[i]);
	}
	
	objectsTracker.getParams(argvStrings);
	objectsTracker.init(nh);

	objectsRecognizer.getParams(argvStrings);
	objectsRecognizer.init(nh);

	objectsTracker.getParams(argvStrings);
	objectsTracker.init(nh);

	ros::spin();
}

#endif

