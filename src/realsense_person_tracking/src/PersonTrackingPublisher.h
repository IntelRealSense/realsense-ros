#pragma once
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include "realsense_msgs/User.h"

namespace PT = Intel::RealSense::PersonTracking;

class PersonTrackingPublisher
{
public:

    PersonTrackingPublisher();

    void onInit(ros::NodeHandle& nodeHandle, rs::person_tracking::person_tracking_video_module_interface* personTracking);

    void setDebug(bool debug);

    void enableBlob(bool blob, bool segmentation);

    void publishOutput(const sensor_msgs::ImageConstPtr& colorImageMsg, PT::PersonTrackingData& trackingData);

private:

    void addBlobToOutput(PT::PersonTrackingData::PersonTracking* personTrackingData, realsense_msgs::User& user);

    void addSkeletonToOutput(PT::PersonTrackingData::Person* personData, realsense_msgs::User& user);

    void addGesturesToOutout(PT::PersonTrackingData::Person* personData, realsense_msgs::User& user);

    ros::Publisher mPublisher;
    ros::Publisher mDebugPublisher;

    rs::person_tracking::person_tracking_video_module_interface* mPersonTrackingVideoModule;

    bool mBlobEnabled;
    bool mSegmentationEnabled;

    bool mDebug;
};
