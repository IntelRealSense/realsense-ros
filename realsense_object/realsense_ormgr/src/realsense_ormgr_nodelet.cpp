/******************************************************************************
	INTEL CORPORATION PROPRIETARY INFORMATION
	This software is supplied under the terms of a license agreement or nondisclosure
	agreement with Intel Corporation and may not be copied or disclosed except in
	accordance with the terms of that agreement
	Copyright(c) 2011-2015 Intel Corporation. All Rights Reserved.
*******************************************************************************/

#include "ros/ros.h"


#include "realsense_msgs/Rect.h"
#include "realsense_msgs/TrackedObjectsArray.h"
#include "realsense_msgs/ObjectInBox.h"
#include "realsense_msgs/ObjectsInBoxes.h"
#include "realsense_msgs/UI.h"
 
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include "../include/realsense_ormgr_nodelet.h"
#include <pluginlib/class_list_macros.h>


#ifdef RUN_AS_NODELET         
PLUGINLIB_EXPORT_CLASS(realsense::COrmgrNodelet, nodelet::Nodelet)
#endif


namespace realsense
{
 	

	//******************************
	// Public Methods
	//******************************
  
    	COrmgr::COrmgr()
	{
		initialize();
	}
  

	COrmgr::~COrmgr()
	{
		unInitialize();
	}
	int COrmgr::unInitialize()
	{
		m_objects_vector.objects_vector.clear();
		return 0;
	}

	int COrmgr::initialize()
	{	
		m_threshold = 1;
		m_debug = false;
		m_maxNumberOfObjects = 5;
		return 0;
	}
	
	void COrmgr::getParams(ros::NodeHandle& nh)
	{
		nh.getParam("debug", m_debug);		
		nh.getParam("threshold", m_threshold);	
		nh.getParam("max_number_of_objects", m_maxNumberOfObjects);
		nh.getParam("localize_every_frame", m_localize_every_frame);
	}
	
	void COrmgr::getParams(const std::vector<std::string> & argv)
	{
	  
	  	for(int i=0; i < argv.size(); i++)
		{
			if(argv[i] == "debug")
			{
				m_debug = true;
			}
			else if(argv[i] == "-threshold") // max objects to track
			{
				m_threshold = atoi(argv[++i].c_str());
			}
			else if(argv[i] == "-max_number_of_objects") // max objects to track
			{
				m_maxNumberOfObjects = atoi(argv[++i].c_str());
			}

		}
 
	}

	void COrmgr::init(ros::NodeHandle& nh)
	{
		
		m_nh = nh;
		m_sub_localized_objects = m_nh.subscribe("realsense/localized_objects", 1, &COrmgr::localizeidObjectsCallback , this);
		m_sub_UI = m_nh.subscribe("realsense/UI", 1, &COrmgr::UICallback , this);
			
		m_objects_to_track_pub = m_nh.advertise<realsense_msgs::TrackedObjectsArray>("realsense/objects_to_track", 1);
		m_tracked_localized_pub = m_nh.advertise<realsense_msgs::ObjectsInBoxes>("realsense/localized_tracked_objects", 1);
	 }


	
	void COrmgr::localizeidObjectsCallback(const realsense_msgs::ObjectsInBoxes& msg)
	{
	  // get objects roi from localization and publish it
	  
		int array_size = std::min(m_maxNumberOfObjects,(int)msg.objects_vector.size());	
		if (array_size <=0)
			return ;
		m_sub_localized_objects.shutdown();	
		ROS_INFO("new objects to track");		
		realsense_msgs::TrackedObjectsArray outROIs;
		m_objects_vector.objects_vector.clear();
		 
		realsense_msgs::TrackedObject to;	 
		 for(int i=0; i < array_size; i++)
		 {
			to.bbox = msg.objects_vector[i].object_bbox;
			to.location = msg.objects_vector[i].location;
			to.id = i;	
			outROIs.tracked_objects_vector.push_back(to);
			m_objects_vector.objects_vector.push_back( msg.objects_vector[i]);
		 }
		 
		 m_sub_tracked_objects = m_nh.subscribe("realsense/tracked_objects", 1, &COrmgr::trackedObjectCallback , this);
		 m_objects_to_track_pub.publish(outROIs);				  
	}
	
	void COrmgr::UICallback(const realsense_msgs::UI& msg)
	{
	  if (true == m_localize_every_frame)
	  {
		return; 
	  }			
	  ROS_INFO("re-localization\n ");
	  m_sub_tracked_objects.shutdown();		  		     	   
          m_sub_localized_objects = m_nh.subscribe("realsense/localized_objects", 1, &COrmgr::localizeidObjectsCallback , this);						
	  ROS_INFO(std::to_string(msg.key).c_str());			  
	}
	
	void COrmgr::trackedObjectCallback(const realsense_msgs::TrackedObjectsArray::ConstPtr & msg)
	{	
		int nCtr = 0;
		for(int i=0; i < (int)msg->tracked_objects_vector.size(); i++)
		{

			m_objects_vector.objects_vector[i].object_bbox = msg->tracked_objects_vector[i].bbox;
			m_objects_vector.objects_vector[i].location = msg->tracked_objects_vector[i].location;

		    if (msg->tracked_objects_vector[i].bbox.width == 0 || msg->tracked_objects_vector[i].bbox.height == 0)
				nCtr++;	     
		}
				
		if (/*(msg->tracked_objects_vector.size() == 0 || nCtr > m_threshold || nCtr == msg->tracked_objects_vector.size()) &&*/ m_localize_every_frame )
		{
		  
		      ROS_INFO("lost too many objects - asking for new\n ");
		      m_sub_tracked_objects.shutdown();		  		     	   
		      m_sub_localized_objects = m_nh.subscribe("realsense/localized_objects", 1, &COrmgr::localizeidObjectsCallback , this);		
		  
		}
		
		if (m_debug)
		{
		    m_objects_vector.header = msg->header;
		    m_tracked_localized_pub.publish(m_objects_vector);
		}
	}



#ifdef RUN_AS_NODELET

	//******************************
	// Public Methods
	//******************************
	
	COrmgrNodelet::~COrmgrNodelet()
	{
	}

	void COrmgrNodelet::onInit()
	{
		ROS_INFO_STREAM("OR manager OnInit");
		NODELET_INFO("OR manager OnInit.");
		
		ros::NodeHandle& local_nh = getPrivateNodeHandle();		
		m_ManagerNodelet.getParams(local_nh);
		
		ros::NodeHandle& nh = getNodeHandle();
		m_ManagerNodelet.init(nh);

		NODELET_INFO("OR manager nodelet initialized.");
	 }

#endif
  
}



#ifndef RUN_AS_NODELET


int main(int argc, char **argv)
{
  
  	ros::init(argc, argv, "realsense_ormgr");
	ros::NodeHandle nh;
	
	realsense::COrmgr ormanager;
	

	
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

