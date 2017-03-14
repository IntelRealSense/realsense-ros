// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "PersonTrackingSample.h"

#include "realsense_ros_person/TrackingConfig.h"
#include "realsense_ros_person/Recognition.h"
#include "realsense_ros_person/StartTracking.h"
#include "realsense_ros_person/StopTracking.h"
#include "realsense_ros_person/RecognitionRegister.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>
#include <realsense_ros_person/PersonModuleState.h>

std::string PersonTrackingSample::PERSON_MODULE_STATE_TOPIC = "/person_tracking/module_state";
std::string RegistrationResultToString(int status);
std::string RecognitionResultToString(int status);
std::string WaveGestureToString(int32_t waveGestureRos);


PersonTrackingSample::PersonTrackingSample() : m_viewer(false), m_trackingRenderer(m_viewer)
{
  m_trackingRenderer.SetPersonSelectionHandler([this](PersonData & data, TrackingRenderer::SelectType type)
  {
    PersonSelectedHandler(data, type);
  });
  m_trackingRenderer.SetGlobalHandler([this](TrackingRenderer::SelectType type)
  {
    GlobalHandler(type);
  });
}

void PersonTrackingSample::ProcessCommandLineArgs()
{
  ros::NodeHandle nodeHandle("~");
  nodeHandle.param<bool>("skeletonEnabled", mEnableSkeleton, false);
  ROS_INFO_STREAM("mEnableSkeleton = " << mEnableSkeleton);

  nodeHandle.param<bool>("recognitionEnabled", mEnableRecognition, false);
  ROS_INFO_STREAM("mEnableRecognition = " << mEnableRecognition);

  nodeHandle.param<bool>("pointingGestureEnabled", mEnablePointingGesture, false);
  ROS_INFO_STREAM("mEnablePointingGesture = " << mEnablePointingGesture);

  nodeHandle.param<bool>("waveGestureEnabled", mEnableWaveGesture, false);
  ROS_INFO_STREAM("mEnableWaveGesture = " << mEnableWaveGesture);

  nodeHandle.param<bool>("landmarksEnabled", mEnableLandmarks, false);
  ROS_INFO_STREAM("mEnableLandmarks = " << mEnableLandmarks);

  nodeHandle.param<bool>("headBoundingBoxEnabled", mEnableHeadBoundingBox, false);
  ROS_INFO_STREAM("headBoundingBox = " << mEnableHeadBoundingBox);

  nodeHandle.param<bool>("headPoseEnabled", mEnableHeadPose, false);
  ROS_INFO_STREAM("headPose = " << mEnableHeadPose);
}

void PersonTrackingSample::InitMessaging(ros::NodeHandle& nodeHandle)
{
  EnableTrackingFeatures(nodeHandle);
  mTrackingOutputSubscriber = nodeHandle.subscribe("person_tracking_output_test", 1, &PersonTrackingSample::PersonTrackingCallback, this);
  mRecognitionRequestClient = nodeHandle.serviceClient<realsense_ros_person::Recognition>("person_tracking/recognition_request");
  mRegisterRequestClient = nodeHandle.serviceClient<realsense_ros_person::RecognitionRegister>("person_tracking/register_request");
  mStartTrackingRequestClient = nodeHandle.serviceClient<realsense_ros_person::StartTracking>("person_tracking/start_tracking_request");
  mStopTrackingRequestClient = nodeHandle.serviceClient<realsense_ros_person::StopTracking>("person_tracking/stop_tracking_request");
}

void PersonTrackingSample::EnableTrackingFeatures(ros::NodeHandle& nodeHandle)
{
  mConfigClient = nodeHandle.serviceClient<realsense_ros_person::TrackingConfig>("person_tracking/tracking_config");

  realsense_ros_person::TrackingConfig config;
  config.request.enableRecognition = mEnableRecognition;
  config.request.enableSkeleton = mEnableSkeleton;
  config.request.enablePointingGesture = mEnablePointingGesture;
  config.request.enableWaveGesture = mEnableWaveGesture;
  config.request.enableLandmarks = mEnableLandmarks;
  config.request.enableHeadBoundingBox = mEnableHeadBoundingBox;
  config.request.enableHeadPose = mEnableHeadPose;

  while (!mConfigClient.call(config))
  {
    ROS_INFO_STREAM("failed to send config to person tracking node, sleep 1 second and retry");
    usleep(1e6);
  }
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
  std::lock_guard<std::mutex> guard(mMutex);

  m_trackingRenderer.Reset();

  realsense_ros_person::Frame frame = msg.frameData;

  for (realsense_ros_person::User& user : frame.usersData)
  {
    DrawPersonResults(colorImage, user);
  }
}


void PersonTrackingSample::DrawPersonResults(cv::Mat& colorImage, realsense_ros_person::User& user)
{
  // person rectangle
  cv::Point pt1(user.userRect.rectCorners[0].x, user.userRect.rectCorners[0].y);
  cv::Point pt2(user.userRect.rectCorners[1].x, user.userRect.rectCorners[1].y);
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
  DrawPersonSummaryReport(colorImage, user);
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
    cv::Point pt1(user.headBoundingBox.rectCorners[0].x, user.headBoundingBox.rectCorners[0].y);
    cv::Point pt2(user.headBoundingBox.rectCorners[1].x, user.headBoundingBox.rectCorners[1].y);
    cv::Rect headBoundingBoxRect(pt1, pt2);
    m_trackingRenderer.DrawFace(colorImage, headBoundingBoxRect);
  }
}


void PersonTrackingSample::DrawPersonGestures(cv::Mat& colorImage, realsense_ros_person::User& user)
{
  if (!mEnablePointingGesture) return;

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
  std::lock_guard<std::mutex> guard(mMutex);

  if (type == TrackingRenderer::SelectType::RECOGNITION)
  {
    realsense_ros_person::Recognition request;
    request.request.personId = data.Id;
    mRecognitionRequestClient.call(request);
    data.rid = request.response.recognitionId;
    ROS_INFO_STREAM("Recognition Status = " + RecognitionResultToString(request.response.status));
  }
  else if (type == TrackingRenderer::SelectType::REGISTRATION)
  {
    realsense_ros_person::RecognitionRegister request;
    request.request.personId = data.Id;
    mRegisterRequestClient.call(request);
    data.rid = request.response.recognitionId;
    ROS_INFO_STREAM("Registration Status = " + RegistrationResultToString(request.response.status));

  }
  else if (type == TrackingRenderer::SelectType::TRACKING)
  {
    auto personState = ros::topic::waitForMessage<realsense_ros_person::PersonModuleState>(PERSON_MODULE_STATE_TOPIC, ros::Duration(5));
    if (personState == nullptr)
    {
      ROS_ERROR_STREAM("Failed to get person tracking state");
      return;
    }
    if (personState->trackingState == realsense_ros_person::PersonModuleState::TRACKING_STATE_DETECTING)
    {
      realsense_ros_person::StartTracking request;
      request.request.personId = data.Id;
      mStartTrackingRequestClient.call(request);
      std::string res = request.response.status ? " SUCCEEDED" : " FAILED";
      ROS_INFO_STREAM("Start tracking of user ID " + std::to_string(data.Id) + res);
    }
    else
    {
      realsense_ros_person::StopTracking request;
      request.request.personId = data.Id;
      mStopTrackingRequestClient.call(request);
      std::string res = request.response.status ? " SUCCEEDED" : " FAILED";
      ROS_INFO_STREAM("Stop tracking of user ID " + std::to_string(data.Id) + res);
    }
  }
}

//Function For Handling glbal events (that are not specific for a user)
void PersonTrackingSample::GlobalHandler(TrackingRenderer::SelectType type)
{
//    std::lock_guard<std::mutex> guard(mMutex);
}


void PersonTrackingSample::DrawPersonSummaryReport(cv::Mat image, realsense_ros_person::User &user)
{
  std::stringstream summaryText;// summary text at at top left corner of image (center of mass, orientation etc.)

  //add center of mass (world coordinates)
  summaryText << user.userInfo.Id << ": " <<
              std::fixed << std::setprecision(3) <<
              "(" << user.centerOfMassWorld.x << "," << user.centerOfMassWorld.y << "," << user.centerOfMassWorld.z << ")";

  if (user.headPose.confidence > 0)
  {
    //add head pose
    summaryText << std::setprecision(1) << std::fixed << " head orientation " << "(" <<
                user.headPose.angles.pitch << ", " <<
                user.headPose.angles.roll << ", " <<
                user.headPose.angles.yaw << ")";
  }

  //add wave gesture
  int32_t waveGesture = user.gestures.wave.type;
  if (waveGesture != (int32_t)realsense_ros_person::Wave::WAVE_NOT_DETECTED)
  {
    summaryText << " wave gesture: " << WaveGestureToString(waveGesture).c_str() << "\n";
  }

  m_trackingRenderer.DrawLineAtSummaryReport(image, summaryText.str());
}

std::string RegistrationResultToString(int status)
{
  switch (status)
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
  switch (status)
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

std::string WaveGestureToString(int32_t waveGestureRos)
{
  switch (waveGestureRos)
  {
  case realsense_ros_person::Wave::WAVE_NOT_DETECTED:
    return "Wave not detected";
  case realsense_ros_person::Wave::WAVE_LEFT_LA:
    return "Wave left left area";
  case realsense_ros_person::Wave::WAVE_RIGHT_LA:
    return "Wave right left area";
  case realsense_ros_person::Wave::WAVE_LEFT_RA:
    return "Wave left right area";
  case  realsense_ros_person::Wave::WAVE_RIGHT_RA:
    return "Wave right right area";
  default:
    throw std::runtime_error("unsupported wave gesture value");
  }
}
