#pragma once
#include "ros/ros.h"
#include "realsense_person_tracking/PersonTrackingOut.h"
#include "TrackingRenderer/TrackingRenderer.h"
#include <mutex>

const char WINDOW_NAME[] = "PersonTracking";

class PersonTrackingTest
{
public:
    PersonTrackingTest();

    void ProcessCommandLineArgs();

    void InitMessaging(ros::NodeHandle& nodeHandle);

private:
    void EnableTrackingFeatures(ros::NodeHandle& nodeHandle);

    void PersonTrackingCallback(const realsense_person_tracking::PersonTrackingOut& msg);

    void DrawDetectionResults(cv::Mat& colorImage, const realsense_person_tracking::PersonTrackingOut& msg);

    void DrawPersonResults(cv::Mat& colorImage, const realsense_person_tracking::PersonTrackingOut& msg, realsense_msgs::User& user);

    void DrawPersonSegmentation(cv::Mat& colorImage, realsense_msgs::Frame& frame);

    void DrawPersonSkeleton(cv::Mat& colorImage, realsense_msgs::User& user);

    void DrawPersonGestures(cv::Mat& colorImage, realsense_msgs::User& user);

    void PersonSelectedHandler(PersonData& data, TrackingRenderer::SelectType type);

    int  mSegmentation;
    bool mEnableBlob;
    bool mEnableSkeleton;
    bool mEnableRecognition;
    bool mEnableGestures;
    int  mTrackingMode = 0;

    ros::Subscriber mTrackingOutputSubscriber;
    ros::ServiceClient mRecognitionRequestClient;
    ros::ServiceClient mTrackingRequestClient;

    std::mutex mMutex;

    Viewer m_viewer;
    TrackingRenderer m_trackingRenderer;
};
