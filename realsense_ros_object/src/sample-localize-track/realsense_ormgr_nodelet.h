// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once
# ifndef RS_ORMGR_NODELET
# define RS_ORMGR_NODELET

///////////////////////////////////////////////
/// Dependencies
///////////////////////////////////////////////

#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace realsense_ros_object
{
///////////////////////////////////////////////
/// CTracking -
///////////////////////////////////////////////
class COrmgr
{
public :
  //===================================
  //  Interface
  //===================================
  void getParams(ros::NodeHandle& nh);
  void getParams(const std::vector<std::string> & argv);
  void init(ros::NodeHandle& nh);

  COrmgr();
  virtual ~COrmgr();

private:
  //===================================
  //  Member Functions
  //===================================

  int initialize();
  int unInitialize();

  //Static member functions:
  void localizeidObjectsCallback(const realsense_ros_object::ObjectsInBoxes& msg);
  void trackedObjectCallback(const realsense_ros_object::TrackedObjectsArray::ConstPtr & msg);
  void UICallback(const realsense_ros_object::UI & msg);

  //===================================
  //  Member Variables
  //===================================

  ros::Subscriber sub_localized_objects_;
  ros::Subscriber sub_tracked_objects_;
  ros::Subscriber sub_UI_;

  ros::Publisher objects_to_track_pub_;
  ros::Publisher tracked_localized_pub_;

  ros::NodeHandle nh_;

  realsense_ros_object::ObjectsInBoxes objects_vector_;

  int max_number_of_objects_;
  int threshold_;
  bool localize_every_frame_;
};

#ifdef RUN_AS_NODELET
///////////////////////////////////////////////
/// COrmgrNodelet
///////////////////////////////////////////////
class COrmgrNodelet : public nodelet::Nodelet
{
public :
  //===================================
  //  Interface
  //===================================
  virtual void onInit();

  ~COrmgrNodelet();

private:
  //===================================
  //  Member Functions
  //===================================

  //===================================
  //  Member Variables
  //===================================

  COrmgr manager_nodelet_;
};
#endif
};


#endif // RS_TRACKING_NODELET

