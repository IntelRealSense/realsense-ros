#pragma once
#include "ros/ros.h"
#include "PtOpencvAdapter.h"
#include <sensor_msgs/Image.h>
#include "realsense_msgs/User.h"

namespace PT = Intel::RealSense::PersonTracking;

class PersonTrackingPublisher
{
public:

    PersonTrackingPublisher();

    void onInit(ros::NodeHandle& nodeHandle, PtOpencvAdapter* personTracking);

    void setDebug(bool debug);

    void enableBlob(bool blob, bool segmentation);

    void publishOutput(const sensor_msgs::ImageConstPtr& colorImageMsg, PT::PersonTrackingData& trackingData);

private:

    void addBlobToOutput(PT::PersonTrackingData::PersonTracking* personTrackingData, realsense_msgs::User& user);

    void addSkeletonToOutput(PT::PersonTrackingData::Person* personData, realsense_msgs::User& user);

    void addGesturesToOutout(PT::PersonTrackingData::Person* personData, realsense_msgs::User& user);

    ros::Publisher mPublisher;
    ros::Publisher mDebugPublisher;

    PtOpencvAdapter* mPersonTracking;

    bool mBlobEnabled;
    bool mSegmentationEnabled;

    bool mDebug;
};
