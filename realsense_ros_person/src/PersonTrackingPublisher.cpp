#include "PersonTrackingPublisher.h"

PersonTrackingPublisher::PersonTrackingPublisher(){}

void PersonTrackingPublisher::onInit(ros::NodeHandle & nodeHandle, rs::person_tracking::person_tracking_video_module_interface * personTracking)
{
  mPersonTracking = personTracking;
  std::string toStream = "person_tracking/person_tracking_output";
  ROS_INFO_STREAM("Publishing to " << toStream);
  mPublisher = nodeHandle.advertise<realsense_ros_person::Frame>(toStream, 1);
}
void PersonTrackingPublisher::publishOutput(PXCPersonTrackingData & trackingData)
{
  realsense_ros_person::Frame frame;
  frame.numberOfUsers = trackingData.QueryNumberOfPeople();
  for (int index = 0; index < frame.numberOfUsers; ++index)
  {
    PXCPersonTrackingData::Person* personData = trackingData.QueryPersonData(PXCPersonTrackingData::AccessOrderType::ACCESS_ORDER_BY_INDEX, index);
    if (personData)
    {
      realsense_ros_person::User user;
      PXCPersonTrackingData::PersonTracking* personTrackingData = personData->QueryTracking();
      PXCPersonTrackingData::BoundingBox2D box = personTrackingData->Query2DBoundingBox();
      PXCPersonTrackingData::BoundingBox2D facebox = personTrackingData->QueryHeadBoundingBox();
      PXCPersonTrackingData::PointCombined centerMass = personTrackingData->QueryCenterMass();
      user.userInfo.Id = personTrackingData->QueryId();
      if(mPersonTracking->QueryConfiguration()->QueryRecognition()->IsEnabled())
      {
        if (personData->QueryRecognition()!=nullptr)
        {
          PXCPersonTrackingData::PersonRecognition* recognition = personData->QueryRecognition();
          PXCPersonTrackingData::PersonRecognition::RecognizerData recognizerData;
          PXCPersonTrackingData::PersonRecognition::RecognitionStatus status = recognition->RecognizeUser(&recognizerData);
          if (status == PXCPersonTrackingData::PersonRecognition::RecognitionPassedPersonRecognized)
          {
            user.userInfo.recognitionId = recognizerData.recognitionId;
            ROS_DEBUG_STREAM("Recognized person: " << recognizerData.recognitionId);
          }
          else
          {
            ROS_DEBUG_STREAM("Failed to recognize person: " << RecognitionStatusToString(status).c_str());
            user.userInfo.recognitionId = -1;
          }
        }
        else
        {
          ROS_DEBUG("Failed to find recognition");
          user.userInfo.recognitionId = -1;
        }
      }
      geometry_msgs::Point32 point1;
      point1.x = box.rect.x;
      point1.y = box.rect.y;
      geometry_msgs::Point32 point2;
      point2.x = box.rect.x + box.rect.w;
      point2.y = box.rect.y + box.rect.h;
      user.userRectCorners[0] = point1;
      user.userRectCorners[1] = point2;
      point1.x=facebox.rect.x;
      point1.y=facebox.rect.y;
      point2.x=facebox.rect.x+box.rect.w;
      point2.y=facebox.rect.y+box.rect.h;
      user.userFaceRectCorners[0]=point1;
      user.userFaceRectCorners[1]=point2;
      user.centerOfMassImage.x = centerMass.image.point.x;
      user.centerOfMassImage.y = centerMass.image.point.y;
      user.centerOfMassImage.z = centerMass.image.point.z;
      user.centerOfMassWorld.x = centerMass.world.point.x;
      user.centerOfMassWorld.y = centerMass.world.point.y;
      user.centerOfMassWorld.z = centerMass.world.point.z;
      addBlobToOutput(personTrackingData,user);
      addSkeletonToOutput(personData, user);
      addGesturesToOutout(personData, user);
      frame.usersData.push_back(user);
    }
  }
  mPublisher.publish(frame);
}
void PersonTrackingPublisher::addBlobToOutput(PXCPersonTrackingData::PersonTracking * personTrackingData, realsense_ros_person::User & user)
{
  if (!mPersonTracking->QueryConfiguration()->QueryTracking()->IsSegmentationEnabled()) return;
  if (mPersonTracking->QueryConfiguration()->QueryTracking()->IsBlobEnabled())
  {
    PXCImage* blobMask = personTrackingData->QueryBlobMask();
    if (blobMask != nullptr)
    {
      PXCImage::ImageInfo blobImageInfo = blobMask->QueryInfo();
      PXCImage::ImageData blobImageData;
      blobMask->ExportData(&blobImageData, 0);
      user.userBlobImage.width = blobImageInfo.width;
      user.userBlobImage.height = blobImageInfo.height;
      user.userBlobImage.data.insert(user.userBlobImage.data.begin(), blobImageData.planes[0], blobImageData.planes[0] + blobImageInfo.width * blobImageInfo.height);
      user.userBlobImage.encoding = "8UC1";
      user.userBlobImage.step = blobImageInfo.width;
      blobMask->Release();
    }
  }
  if (mPersonTracking->QueryConfiguration()->QueryTracking()->IsSegmentationEnabled())
  {
    PXCImage* segmentationMask = personTrackingData->QuerySegmentationImage();
    if (segmentationMask != nullptr)
    {
      PXCImage::ImageInfo segImageInfo = segmentationMask->QueryInfo();
      PXCImage::ImageData segImageData;
      segmentationMask->ExportData(&segImageData, 0);
      user.userSegmentationImage.width = segImageInfo.width;
      user.userSegmentationImage.height = segImageInfo.height;
      user.userSegmentationImage.data.insert(user.userSegmentationImage.data.begin(), segImageData.planes[0], segImageData.planes[0] + segImageInfo.width * segImageInfo.height);
      user.userSegmentationImage.encoding = "8UC1";
      user.userSegmentationImage.step = segImageInfo.width;
      segmentationMask->Release();
    }
  }
}
void PersonTrackingPublisher::addSkeletonToOutput(PXCPersonTrackingData::Person * personData, realsense_ros_person::User & user)
{
  if (!mPersonTracking->QueryConfiguration()->QuerySkeletonJoints()->IsEnabled()) return;
  PXCPersonTrackingData::PersonJoints* personJoints = personData->QuerySkeletonJoints();
  int numberOfJoints = personJoints->QueryNumJoints();
  if (numberOfJoints == 0) return;
  std::vector<PXCPersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(numberOfJoints);
  personJoints->QueryJoints(skeletonPoints.data());
  for (PXCPersonTrackingData::PersonJoints::SkeletonPoint& skeletonPoint : skeletonPoints)
  {
    realsense_ros_person::Landmark landmark;
    landmark.type = skeletonPoint.jointType;
    landmark.location.x = skeletonPoint.image.x;
    landmark.location.y = skeletonPoint.image.y;
    user.landmarks.push_back(landmark);
  }
}
void PersonTrackingPublisher::addGesturesToOutout(PXCPersonTrackingData::Person * personData, realsense_ros_person::User & user)
{
  if (!mPersonTracking->QueryConfiguration()->QueryGestures()->IsEnabled()) return;
  PXCPersonTrackingData::PersonGestures * personGestures = personData->QueryGestures();
  if (personGestures == nullptr)
  {
    ROS_WARN("personGestures == nullptr");
    return;
  }
  if (personGestures->IsPointing())
  {
    PXCPersonTrackingData::PersonGestures::PointingInfo poitingInfo = personGestures->QueryPointingInfo();
    user.gestures.pointing.originColor.x = poitingInfo.colorPointingData.origin.x;
    user.gestures.pointing.originColor.y = poitingInfo.colorPointingData.origin.y;
    user.gestures.pointing.originWorld.x = poitingInfo.worldPointingData.origin.x;
    user.gestures.pointing.originWorld.y = poitingInfo.worldPointingData.origin.y;
    user.gestures.pointing.originWorld.z = poitingInfo.worldPointingData.origin.z;
    user.gestures.pointing.orientationColor.x = poitingInfo.colorPointingData.direction.x;
    user.gestures.pointing.orientationColor.y = poitingInfo.colorPointingData.direction.y;
    user.gestures.pointing.orientationWorld.x = poitingInfo.worldPointingData.direction.x;
    user.gestures.pointing.orientationWorld.y = poitingInfo.worldPointingData.direction.y;
    user.gestures.pointing.orientationWorld.z = poitingInfo.worldPointingData.direction.z;
    user.gestures.pointing.confidence = poitingInfo.confidence;
  }
}
std::string PersonTrackingPublisher::RecognitionStatusToString(PXCPersonTrackingData::PersonRecognition::RecognitionStatus status)
{
  switch (status)
  {
    case PXCPersonTrackingData::PersonRecognition::RecognitionPassedPersonRecognized:
         return "RecognitionPassedPersonRecognized";
    case PXCPersonTrackingData::PersonRecognition::RecognitionPassedPersonNotRecognized:
         return "RecognitionPassedPersonNotRecognized";
    case PXCPersonTrackingData::PersonRecognition::RecognitionFailed:
         return "RecognitionFailed";
    case PXCPersonTrackingData::PersonRecognition::RecognitionFailedFaceNotDetected:
         return "RecognitionFailedFaceNotDetected";
    case PXCPersonTrackingData::PersonRecognition::RecognitionFailedFaceNotClear:
         return  "RecognitionFailedFaceNotClear";
    case PXCPersonTrackingData::PersonRecognition::RecognitionFailedPersonTooFar:
         return  "RecognitionFailedPersonTooFar";
    case PXCPersonTrackingData::PersonRecognition::RecognitionFailedPersonTooClose:
         return  "RecognitionFailedPersonTooClose";
    default:
         return  "unknown recognition status please check the code";
  }
}

