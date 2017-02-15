// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.

#include "ros/ros.h"

#include "realsense_ros_object/UI.h"
#include "realsense_ros_object/ObjectsInBoxes.h" 
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>



#include "realsense_orview_nodelet.h"

#include <pluginlib/class_list_macros.h>



#ifdef RUN_AS_NODELET         
PLUGINLIB_EXPORT_CLASS(realsense_ros_object::COrViewNodelet, nodelet::Nodelet)
#endif




namespace realsense_ros_object
{
 	

	//******************************
	// Public Methods
	//******************************
  
    	COrView::COrView()
	{	
		initialize();
		gui.init_GUI("Localization + Tracking");
	}
  

	COrView::~COrView()
	{
		unInitialize();
	}
	int COrView::unInitialize()
	{
		cv::destroyAllWindows();
	
		return 0;
	}

	int COrView::initialize()
	{	
	
		return 0;
	}
	
	void COrView::getParams(ros::NodeHandle& nh)
	{
	
	}
	
	void COrView::getParams(const std::vector<std::string> & argv)
	{
	  
	  	for(int i=0; i < argv.size(); i++)
		{
		}
 
	}

	void COrView::init(ros::NodeHandle& nh)
	{
		m_nh = nh;
		
		// Subscribe to color, tracking using synchronization filter
		mColorSubscriber.reset(new message_filters::Subscriber<sensor_msgs::Image>(m_nh, "camera/color/image_raw", 1));
		
		mTrackingSubscriber.reset(new message_filters::Subscriber<realsense_ros_object::ObjectsInBoxes>(m_nh, "realsense/localized_tracked_objects", 1));
	
		m_UI_pub = m_nh.advertise<realsense_ros_object::UI>("realsense/UI", 1);	
 
		mTimeSynchronizer.reset(new message_filters::TimeSynchronizer<sensor_msgs::Image, realsense_ros_object::ObjectsInBoxes>(*mColorSubscriber, *mTrackingSubscriber,60));
 
		mTimeSynchronizer->registerCallback(boost::bind(&COrView::localizedTrackedObjectsCallback, this, _1, _2));

	 }


	
	void COrView::localizedTrackedObjectsCallback(const sensor_msgs::ImageConstPtr& color,const realsense_ros_object::ObjectsInBoxes::ConstPtr& msg)
	{

		int key = cv::waitKey(1);
		if ((char)key == ' ')
		{
			realsense_ros_object::UI msg;
			msg.key = key;
			m_UI_pub.publish(msg);
			//ROS_INFO(std::to_string(key).c_str());	
		} 
			
		gui.draw_results(color, *msg); 
		gui.show_results(); 
	}
	
#ifdef RUN_AS_NODELET

	//******************************
	// Public Methods
	//******************************
	
	COrViewNodelet::~COrViewNodelet()
	{
	}

	void COrViewNodelet::onInit()
	{
		
		ROS_INFO_STREAM("OR viewer OnInit");
		NODELET_INFO("OR viewer OnInit.");
		
		ros::NodeHandle& local_nh = getPrivateNodeHandle();		
		m_ViewNodelet.getParams(local_nh);
		
		ros::NodeHandle& nh = getNodeHandle();
		m_ViewNodelet.init(nh);

		NODELET_INFO("OR viewer nodelet initialized.");
	 }

#endif
  
}



#ifndef RUN_AS_NODELET


int main(int argc, char **argv)
{
  
  	ros::init(argc, argv, "realsense_orview");
	ros::NodeHandle nh;
	
	realsense::COrView ormanager;
	

	
	std::vector<std::string> argvStrings;
	for(int i=0; i < argc; i++)
	{
		argvStrings.push_back(argv[i]);
	}
	
	ormanager.getParams(argvStrings);
	ormanager.init(nh);

	ros::spin();
}
#endif

