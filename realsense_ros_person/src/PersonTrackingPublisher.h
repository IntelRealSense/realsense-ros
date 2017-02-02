#pragma once
#include "ros/ros.h"
#include "person_tracking_video_module_factory.h"
#include <sensor_msgs/Image.h>
#include "realsense_ros_person/User.h"
#include "realsense_ros_person/Frame.h"
#include "PersonTrackingConfig.h"
class PersonTrackingPublisher
{
public:
  PersonTrackingPublisher();
  void onInit(ros::NodeHandle & nodeHandle, rs::person_tracking::person_tracking_video_module_interface * personTracking);
  typedef Intel::RealSense::PersonTracking::PersonTrackingData PXCPersonTrackingData;
  typedef Intel::RealSense::Image PXCImage;
  void publishOutput(PXCPersonTrackingData& trackingData);
private:
  void addBlobToOutput(PXCPersonTrackingData::PersonTracking * personData, realsense_ros_person::User & user);
  void addSkeletonToOutput(PXCPersonTrackingData::Person * personData, realsense_ros_person::User & user);
  void addGesturesToOutout(PXCPersonTrackingData::Person * personData, realsense_ros_person::User & user);
  std::string RecognitionStatusToString(PXCPersonTrackingData::PersonRecognition::RecognitionStatus status);
  ros::Publisher mPublisher;
  rs::person_tracking::person_tracking_video_module_interface * mPersonTracking;
};
