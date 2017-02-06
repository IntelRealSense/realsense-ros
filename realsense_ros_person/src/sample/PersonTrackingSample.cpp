#include "PersonTrackingSample.h"

#include "SafeMutex.h"

#include "realsense_ros_person/TrackingConfig.h"
#include "realsense_ros_person/Recognition.h"
#include "realsense_ros_person/TrackingRequest.h"
#include "realsense_ros_person/RecognitionRegister.h"
#include <realsense_ros_person/RecognitionRegisterResponse.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>

std::string RegistrationResultToString(int status);
std::string RecognitionResultToString(int status);



PersonTrackingSample::PersonTrackingSample() : m_viewer(false), m_trackingRenderer(m_viewer)
{
    m_trackingRenderer.SetPersonSelectionHandler([this] (PersonData& data, TrackingRenderer::SelectType type) { PersonSelectedHandler(data, type); });
    m_trackingRenderer.SetGlobalHandler([this] (TrackingRenderer::SelectType type) { GlobalHandler(type); });
}

void PersonTrackingSample::ProcessCommandLineArgs()
{
    ros::NodeHandle nodeHandle("~");
    nodeHandle.getParam("skeleton", mEnableSkeleton);
    ROS_INFO_STREAM("mEnableSkeleton = " << mEnableSkeleton);

    nodeHandle.getParam("recognition", mEnableRecognition);
    ROS_INFO_STREAM("mEnableRecognition = " << mEnableRecognition);

    nodeHandle.getParam("gestures", mEnableGestures);
    ROS_INFO_STREAM("mEnableGestures = " << mEnableGestures);

    nodeHandle.getParam("landmarks", mEnableLandmarks);
    ROS_INFO_STREAM("mEnableLandmarks = " << mEnableLandmarks);

    nodeHandle.getParam("headBoundingBox", mEnableHeadBoundingBox);
    ROS_INFO_STREAM("headBoundingBox = " << mEnableHeadBoundingBox);
}

void PersonTrackingSample::InitMessaging(ros::NodeHandle& nodeHandle)
{
    EnableTrackingFeatures(nodeHandle);
    mTrackingOutputSubscriber = nodeHandle.subscribe("person_tracking/person_tracking_output_test", 1, &PersonTrackingSample::PersonTrackingCallback, this);
    mRecognitionRequestClient = nodeHandle.serviceClient<realsense_ros_person::Recognition>("person_tracking/recognition_request");
    mRegisterRequestClient = nodeHandle.serviceClient<realsense_ros_person::RecognitionRegister>("person_tracking/register_request");
    mTrackingRequestClient = nodeHandle.serviceClient<realsense_ros_person::TrackingRequest>("person_tracking/tracking_request");
}

void PersonTrackingSample::EnableTrackingFeatures(ros::NodeHandle& nodeHandle)
{
    mConfigClient = nodeHandle.serviceClient<realsense_ros_person::TrackingConfig>("person_tracking/tracking_config");

    realsense_ros_person::TrackingConfig config;
    config.request.enableRecognition = mEnableRecognition;
    config.request.enableSkeleton = mEnableSkeleton;
    config.request.enableGestures = mEnableGestures;
    config.request.enableLandmarks = mEnableLandmarks;
    config.request.enableHeadBoundingBox = mEnableHeadBoundingBox;

    mConfigClient.call(config);
}

void PersonTrackingSample::PersonTrackingCallback(const realsense_ros_person::FrameTest& msg)
{
//    ROS_INFO_STREAM("Received person tracking output message. Number of people: " << msg.frameData.numberOfUsers);

    cv_bridge::CvImageConstPtr ptr;
    cv::Mat colorImage = cv_bridge::toCvShare(msg.colorImage, ptr, sensor_msgs::image_encodings::BGR8)->image;

    DrawDetectionResults(colorImage, msg);

    m_viewer.ShowImage(colorImage);
}

void PersonTrackingSample::DrawDetectionResults(cv::Mat& colorImage, const realsense_ros_person::FrameTest& msg)
{
    SAFE_MUTEX(mMutex);

    m_trackingRenderer.Reset();

    realsense_ros_person::Frame frame = msg.frameData;

    for (realsense_ros_person::User& user : frame.usersData)
    {
        DrawPersonResults(colorImage, msg, user);
    }
}

void PersonTrackingSample::DrawPersonResults(cv::Mat& colorImage, const realsense_ros_person::FrameTest& msg, realsense_ros_person::User& user)
{
    // person rectangle
    cv::Point pt1(user.userRect.rect.x, user.userRect.rect.y);
    cv::Point pt2(user.userRect.rect.x + user.userRect.rect.width, user.userRect.rect.y  + user.userRect.rect.height);
    cv::Rect userRectangle(pt1, pt2);

    int personId = user.userInfo.Id;

    // center of mass point
    cv::Point centerMass(user.centerOfMassImage.x, user.centerOfMassImage.y);
    cv::Point3f centerMassWorld(user.centerOfMassWorld.x, user.centerOfMassWorld.y, user.centerOfMassWorld.z);

    m_trackingRenderer.DrawPerson(colorImage, personId, userRectangle, centerMass, centerMassWorld);

    DrawPersonSkeleton(colorImage, user);
    DrawPersonGestures(colorImage, user);
    DrawPersonLandmarks(colorImage, user);
    DrawFace(colorImage, user);
}

void PersonTrackingSample::DrawPersonSkeleton(cv::Mat& colorImage, realsense_ros_person::User& user)
{
    if (!mEnableSkeleton) return;
    std::vector<cv::Point> points;
    for (realsense_ros_person::SkeletonJoint joint : user.skeletonJoints)
    {
        if (joint.confidence > JOINT_CONFIDENCE_THR)
        {
            std::string x = std::to_string(joint.location.x);
            std::string y = std::to_string(joint.location.y);

            cv::Point location(joint.location.x, joint.location.y);
            points.push_back(location);
        }
    }
    m_trackingRenderer.DrawSkeleton(colorImage, points);
}

void PersonTrackingSample::DrawPersonLandmarks(cv::Mat& colorImage, realsense_ros_person::User& user)
{
    if (!mEnableLandmarks) return;
    if (user.landmarksInfo.confidence <  LANDMARKS_CONFIDENCE_THR) return;

    std::vector<cv::Point> points;
    for (realsense_ros_person::Landmark landmark : user.landmarksInfo.landmarks)
    {
        cv::Point location(landmark.location.x, landmark.location.y);
        points.push_back(location);
    }
    m_trackingRenderer.DrawLandmarks(colorImage, points);
}

void PersonTrackingSample::DrawFace(cv::Mat& colorImage, realsense_ros_person::User& user)
{
    if (!mEnableHeadBoundingBox) return;
    if (user.headBoundingBox.confidence > HEAD_BOUNDING_BOX_THR)
    {
        cv::Point pt1(user.headBoundingBox.rect.x, user.headBoundingBox.rect.y);
        cv::Point pt2(user.headBoundingBox.rect.x + user.headBoundingBox.rect.width, user.headBoundingBox.rect.y + user.headBoundingBox.rect.height);
        cv::Rect headBoundingBoxRect(pt1, pt2);
        m_trackingRenderer.DrawFace(colorImage, headBoundingBoxRect);
    }
}


void PersonTrackingSample::DrawPersonGestures(cv::Mat& colorImage, realsense_ros_person::User& user)
{
    if (!mEnableGestures) return;


    if (user.gestures.pointing.confidence > 0)
    {
        realsense_ros_person::Pointing pointing = user.gestures.pointing;
        cv::Point origin(pointing.originColor.x, pointing.originColor.y);
        cv::Point2f direction(pointing.orientationColor.x, pointing.orientationColor.y);

        m_trackingRenderer.DrawPointing(colorImage, origin, direction);
    }
}

void PersonTrackingSample::PersonSelectedHandler(PersonData& data, TrackingRenderer::SelectType type)
{
    SAFE_MUTEX(mMutex);

    if  (type == TrackingRenderer::SelectType::RECOGNITION)
    {
        realsense_ros_person::Recognition request;
        request.request.personId = data.Id;
        mRecognitionRequestClient.call(request);
        data.rid = request.response.recognitionId;
        ROS_INFO_STREAM("Recognition Status = " + RecognitionResultToString(request.response.status));
    }
    else if  (type == TrackingRenderer::SelectType::REGISTRATION)
    {
        realsense_ros_person::RecognitionRegister request;
        request.request.personId = data.Id;
        mRegisterRequestClient.call(request);
        data.rid = request.response.recognitionId;
        ROS_INFO_STREAM("Registration Status = " + RegistrationResultToString(request.response.status));

    }
    else if  (type == TrackingRenderer::SelectType::TRACKING)
    {
        realsense_ros_person::TrackingRequest request;
        request.request.personId = data.Id;
        mTrackingRequestClient.call(request);
        std::string res = request.response.status ? " SUCCEEDED" : " FAILED";
        ROS_INFO_STREAM("Tracking of user ID " + std::to_string(data.Id) + res);
    }
}

//Function For Handling glbal events (that are not specific for a user)
void PersonTrackingSample::GlobalHandler(TrackingRenderer::SelectType type)
{
    SAFE_MUTEX(mMutex);
}



std::string RegistrationResultToString(int status)
{
    switch(status)
    {
        case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_SUCCESSFULL:
            return "REGISTRATION_SUCCESSFULL";
        case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED:
            return "REGISTRATION_FAILED";
        case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_ALREADY_REGISTERED:
            return "REGISTRATION_FAILED_ALREADY_REGISTERED";
        case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_FACE_NOT_DETECTED:
            return "REGISTRATION_FAILED_FACE_NOT_DETECTED";
        case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_FACE_NOT_CLEAR:
            return "REGISTRATION_FAILED_FACE_NOT_CLEAR";
        case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_PERSON_TO_FAR:
            return "REGISTRATION_FAILED_PERSON_TO_FAR";
        case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_PERSON_TO_CLOSE:
            return "REGISTRATION_FAILED_PERSON_TO_CLOSE";
        default:
            return "REGISTRATION_UNKNOWN_ERROR";
    }
}


std::string RecognitionResultToString(int status)
{
    switch(status)
    {
        case realsense_ros_person::RecognitionResponse::RECOGNITION_PASSED_PERSON_RECOGNIZED:
            return "RECOGNITION_PASSED_PERSON_RECOGNIZED";
        case realsense_ros_person::RecognitionResponse::RECOGNITION_PASSED_PERSON_NOT_RECOGNIZED:
            return "RECOGNITION_PASSED_PERSON_NOT_RECOGNIZED";
        case realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED:
            return "RECOGNITION_FAILED";
        case realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_FACE_NOT_DETECTED:
            return "RECOGNITION_FAILED_FACE_NOT_DETECTED";
        case realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_FACE_NOT_CLEAR:
            return "RECOGNITION_FAILED_FACE_NOT_CLEAR";
        case realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_PERSON_TOO_FAR:
            return "RECOGNITION_FAILED_PERSON_TOO_FAR";
        case realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_PERSON_TOO_CLOSE:
            return "RECOGNITION_FAILED_PERSON_TOO_CLOSE";
        case realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_FACE_AMBIGUITY:
            return "RECOGNITION_FAILED_FACE_AMBIGUITY";
        default:
            return "RECOGNITION_UNKNOWN_ERROR";
    }
}
