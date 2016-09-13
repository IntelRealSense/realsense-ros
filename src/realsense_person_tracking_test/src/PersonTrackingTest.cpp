#include "PersonTrackingTest.h"

#include "SafeMutex.h"

#include "realsense_srvs/TrackingConfig.h"
#include "realsense_srvs/RecognitionRequest.h"
#include "realsense_srvs/TrackingRequest.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>

PersonTrackingTest::PersonTrackingTest() : mSegmentation(0), m_viewer(false), m_trackingRenderer(m_viewer)
{
    m_trackingRenderer.SetPersonSelectionHandler([this] (PersonData& data, TrackingRenderer::SelectType type) { PersonSelectedHandler(data, type); });
}

void PersonTrackingTest::ProcessCommandLineArgs()
{
    ros::NodeHandle nodeHandle("~");

    nodeHandle.getParam("segmentation", mSegmentation);
    ROS_INFO_STREAM("mSegmentation = " << mSegmentation);

    nodeHandle.getParam("blob", mEnableBlob);
    ROS_INFO_STREAM("mEnableBlob = " << mEnableBlob);

    nodeHandle.getParam("skeleton", mEnableSkeleton);
    ROS_INFO_STREAM("mEnableSkeleton = " << mEnableSkeleton);

    nodeHandle.getParam("recognition", mEnableRecognition);
    ROS_INFO_STREAM("mEnableRecognition = " << mEnableRecognition);

    nodeHandle.getParam("gestures", mEnableGestures);
    ROS_INFO_STREAM("mEnableGestures = " << mEnableGestures);

    nodeHandle.getParam("trackingMode", mTrackingMode);
    ROS_INFO_STREAM("mTrackingMode = " << mTrackingMode);

    m_trackingRenderer.SetSegmentationType(SegmentationRenderer::Type(mSegmentation));
}

void PersonTrackingTest::InitMessaging(ros::NodeHandle& nodeHandle)
{
    EnableTrackingFeatures(nodeHandle);

    mTrackingOutputSubscriber = nodeHandle.subscribe("person_tracking/person_tracking_output_debug", 1, &PersonTrackingTest::PersonTrackingCallback, this);
    mRecognitionRequestClient = nodeHandle.serviceClient<realsense_srvs::RecognitionRequest>("person_tracking/recognition_request");
    mTrackingRequestClient = nodeHandle.serviceClient<realsense_srvs::TrackingRequest>("person_tracking/tracking_request");
}

void PersonTrackingTest::EnableTrackingFeatures(ros::NodeHandle& nodeHandle)
{
    ros::ServiceClient configClient = nodeHandle.serviceClient<realsense_srvs::TrackingConfig>("person_tracking/tracking_config");

    realsense_srvs::TrackingConfig config;
    config.request.enableSegmentation = mSegmentation;
    config.request.enableBlob = mEnableBlob;
    config.request.enableRecognition = mEnableRecognition;
    config.request.enableSkeleton = mEnableSkeleton;
    config.request.enableGestures = mEnableGestures;
    config.request.trackingMode = mTrackingMode;

    configClient.call(config);
}

void PersonTrackingTest::PersonTrackingCallback(const realsense_person_tracking::PersonTrackingOut& msg)
{
    //ROS_INFO_STREAM("Received person tracking output message. Number of people: " << msg.frameData.numberOfUsers);

    cv_bridge::CvImageConstPtr ptr;
    cv::Mat colorImage = cv_bridge::toCvShare(msg.colorImage, ptr, sensor_msgs::image_encodings::BGR8)->image;

    DrawDetectionResults(colorImage, msg);

    m_viewer.ShowImage(colorImage);
}

void PersonTrackingTest::DrawDetectionResults(cv::Mat& colorImage, const realsense_person_tracking::PersonTrackingOut& msg)
{
    SAFE_MUTEX(mMutex);

    m_trackingRenderer.Reset();

    realsense_msgs::Frame frame = msg.frameData;

    for (realsense_msgs::User& user : frame.usersData)
    {
        DrawPersonResults(colorImage, msg, user);
    }
    DrawPersonSegmentation(colorImage, frame);
}

void PersonTrackingTest::DrawPersonResults(cv::Mat& colorImage, const realsense_person_tracking::PersonTrackingOut& msg, realsense_msgs::User& user)
{
    // person rectangle
    cv::Point pt1(user.userRectCorners[0].x, user.userRectCorners[0].y);
    cv::Point pt2(user.userRectCorners[1].x, user.userRectCorners[1].y);
    cv::Rect userRectangle(pt1, pt2);

    int personId = user.userInfo.Id;

    // center of mass point
    cv::Point centerMass(user.centerOfMassImage.x, user.centerOfMassImage.y);
    cv::Point3f centerMassWorld(user.centerOfMassWorld.x, user.centerOfMassWorld.y, user.centerOfMassWorld.z);

    m_trackingRenderer.DrawPerson(colorImage, personId, userRectangle, centerMass, centerMassWorld);

    DrawPersonSkeleton(colorImage, user);
    DrawPersonGestures(colorImage, user);
}

void PersonTrackingTest::DrawPersonSkeleton(cv::Mat& colorImage, realsense_msgs::User& user)
{
    if (!mEnableSkeleton) return;
    std::vector<cv::Point> points;
    for (realsense_msgs::Landmark landmark : user.landmarks)
    {
        cv::Point location(landmark.location.x, landmark.location.y);
        points.push_back(location);
    }
    m_trackingRenderer.DrawSkeleton(colorImage, points);
}

void PersonTrackingTest::DrawPersonSegmentation(cv::Mat& colorImage, realsense_msgs::Frame& frame)
{
    std::vector<cv::Mat> segmentedImages;
    for (const realsense_msgs::User& user : frame.usersData)
    {
        if (!user.userSegmentationImage.data.empty())
        {
            cv_bridge::CvImageConstPtr ptr;
            cv::Mat segmentedImage = cv_bridge::toCvShare(user.userSegmentationImage, ptr, "8UC1")->image;
            segmentedImages.push_back(segmentedImage);
        }
    }
    m_trackingRenderer.DrawSegmentation(colorImage, segmentedImages);
}

void PersonTrackingTest::DrawPersonGestures(cv::Mat& colorImage, realsense_msgs::User& user)
{
    if (!mEnableGestures) return;

    if (user.gestures.pointing.confidence)
    {
        realsense_msgs::Pointing pointing = user.gestures.pointing;

        //ROS_INFO("Found gesture at [%.0f,%.0f]", pointing.originColor.x, pointing.originColor.y);
        //ROS_INFO("Found gesture orientation: [%f,%f]", pointing.orientationColor.x, pointing.orientationColor.y);

        cv::Point origin(pointing.originColor.x, pointing.originColor.y);
        cv::Point2f direction(pointing.orientationColor.x, pointing.orientationColor.y);

        m_trackingRenderer.DrawPointing(colorImage, origin, direction);
    }
}

void PersonTrackingTest::PersonSelectedHandler(PersonData& data, TrackingRenderer::SelectType type)
{
    SAFE_MUTEX(mMutex);

    //ROS_INFO_STREAM("Matched person to point: " << personId);

    if  (type == TrackingRenderer::SelectType::RECOGNITION)
    {
        realsense_srvs::RecognitionRequest request;
        request.request.personId = data.Id;
        request.request.recognitionId = data.rid;
        mRecognitionRequestClient.call(request);
        ROS_INFO("Recognition result: %s", request.response.description.c_str());
        data.rid = request.response.recognitionId;
    }
    else if  (type == TrackingRenderer::SelectType::TRACKING)
    {
        realsense_srvs::TrackingRequest request;
        request.request.personId = data.Id;
        mTrackingRequestClient.call(request);
    }
}


