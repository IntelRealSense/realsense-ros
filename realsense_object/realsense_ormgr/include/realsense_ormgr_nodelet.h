/******************************************************************************
ClassifyImage	INTEL CORPORATION PROPRIETARY INFORMATION
	This software is supplied under the terms of a license agreement or nondisclosure
	agreement with Intel Corporation and may not be copied or disclosed except in
	accordance with the terms of that agreement
	Copyright(c) 2011-2015 Intel Corporation. All Rights Reserved.
*******************************************************************************/

#pragma once
# ifndef RS_ORMGR_NODELET
# define RS_ORMGR_NODELET

///////////////////////////////////////////////
/// Dependencies
///////////////////////////////////////////////

#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>





namespace realsense
{
	///////////////////////////////////////////////
	///	CTracking - 
	///////////////////////////////////////////////
	class COrmgr 
	{
	public :
		//===================================
		//	Interface
		//===================================
	  	void getParams(ros::NodeHandle& nh);
		void getParams(const std::vector<std::string> & argv);
		void init(ros::NodeHandle& nh);
  
		COrmgr();
		virtual ~COrmgr();

	private:
		//===================================
		//	Member Functions
		//===================================

		int initialize();
		int unInitialize();
		//Static member functions:
		void localizeidObjectsCallback(const realsense_or-msgs::ObjectsInBoxes& msg);		
		void trackedObjectCallback(const realsense_or_msgs::TrackedObjectsArray::ConstPtr & msg);
		void UICallback(const realsense_or_msgs::UI & msg);
		
		//===================================
		//	Member Variables
		//===================================

		ros::Subscriber m_sub_localized_objects;	
		ros::Subscriber m_sub_tracked_objects;	
		ros::Subscriber m_sub_UI;
	
		ros::Publisher m_objects_to_track_pub;
		ros::Publisher m_tracked_localized_pub;
		
		ros::NodeHandle m_nh;
		
		realsense_or_msgs::ObjectsInBoxes m_objects_vector;
		
		bool m_debug;
		int m_maxNumberOfObjects;	
		int m_threshold;
		bool m_localize_every_frame;
	};
	
#ifdef RUN_AS_NODELET
	///////////////////////////////////////////////
	///	COrmgrNodelet 
	///////////////////////////////////////////////
	class COrmgrNodelet : public nodelet::Nodelet
	{
	public :
		//===================================
		//	Interface
		//===================================
		virtual void onInit();

		~COrmgrNodelet();

	private:
		//===================================
		//	Member Functions
		//===================================

		//===================================
		//	Member Variables
		//===================================

		COrmgr m_ManagerNodelet;

	};
#endif
};


#endif // RS_TRACKING_NODELET

