#include "PersonTrackingPublisher.h"
#include "realsense_person_tracking/PersonTrackingOut.h"

using namespace PT;
using namespace Intel::RealSense;

PersonTrackingPublisher::PersonTrackingPublisher()
    : mPersonTracking(nullptr), mSegmentationEnabled(false), mBlobEnabled(false), mDebug(false) {}

void PersonTrackingPublisher::onInit(ros::NodeHandle& nodeHandle, PtOpencvAdapter* personTracking)
{
    mPersonTracking = personTracking;

    std::string toStream = "person_tracking/person_tracking_output";
    ROS_INFO_STREAM("Publishing to " << toStream);
    mPublisher = nodeHandle.advertise<realsense_msgs::Frame>(toStream, 1);

    if (mDebug)
    {
        std::string toDebugStream = "person_tracking/person_tracking_output_debug";
        ROS_INFO_STREAM("Publishing to " << toDebugStream);
        mDebugPublisher = nodeHandle.advertise<realsense_person_tracking::PersonTrackingOut>(toDebugStream, 1);
    }
}

void PersonTrackingPublisher::setDebug(bool debug)
{
    mDebug = debug;
}

void PersonTrackingPublisher::enableBlob(bool blob, bool segmentation)
{
    mBlobEnabled = blob;
    mSegmentationEnabled = segmentation;
}

void PersonTrackingPublisher::publishOutput(const sensor_msgs::ImageConstPtr& colorImageMsg, PersonTrackingData& trackingData)
{
    realsense_msgs::Frame frame;

    frame.numberOfUsers = trackingData.QueryNumberOfPeople();

    for (int index = 0; index < frame.numberOfUsers; ++index)
    {
        PersonTrackingData::Person* personData = trackingData.QueryPersonData(PersonTrackingData::ACCESS_ORDER_NEAR_TO_FAR, index);
        if (personData)
        {
            realsense_msgs::User user;

            PersonTrackingData::PersonTracking* personTrackingData = personData->QueryTracking();
            PersonTrackingData::BoundingBox2D box = personTrackingData->Query2DBoundingBox();
            PersonTrackingData::PersonTracking::PointCombined centerMass = personTrackingData->QueryCenterMass();

            user.userInfo.Id = personTrackingData->QueryId();
//            user.userInfo.recognitionId = (personData->QueryRecognition() != nullptr) ? personData->QueryRecognition()->QueryRecognitionID() : -1;

            geometry_msgs::Point32 point1;
            point1.x = box.rect.x;
            point1.y = box.rect.y;

            geometry_msgs::Point32 point2;
            point2.x = box.rect.x + box.rect.w;
            point2.y = box.rect.y + box.rect.h;

            user.userRectCorners[0] = point1;
            user.userRectCorners[1] = point2;

            user.centerOfMassImage.x = centerMass.image.point.x;
            user.centerOfMassImage.y = centerMass.image.point.y;
            user.centerOfMassImage.z = centerMass.image.point.z;

            user.centerOfMassWorld.x = centerMass.world.point.x;
            user.centerOfMassWorld.y = centerMass.world.point.y;
            user.centerOfMassWorld.z = centerMass.world.point.z;

            addBlobToOutput(personTrackingData, user);
            addSkeletonToOutput(personData, user);
            addGesturesToOutout(personData, user);

            frame.usersData.push_back(user);
        }
    }

    mPublisher.publish(frame);

    if (mDebug)
    {
        realsense_person_tracking::PersonTrackingOut output;
        output.frameData = frame;
        output.colorImage.height = colorImageMsg->height;
        output.colorImage.width = colorImageMsg->width;
        output.colorImage.encoding = colorImageMsg->encoding;
        output.colorImage.is_bigendian = colorImageMsg->is_bigendian;
        output.colorImage.step = colorImageMsg->step;
        output.colorImage.data = colorImageMsg->data;
        mDebugPublisher.publish(output);
    }
}

void PersonTrackingPublisher::addBlobToOutput(PersonTrackingData::PersonTracking* personTrackingData, realsense_msgs::User& user)
{
    if (!mPersonTracking->QueryConfiguration()->QueryTracking()->IsSegmentationEnabled()) return;

    if (mBlobEnabled)
    {
        Image* blobMask = personTrackingData->QueryBlobMask();
        if (blobMask != nullptr)
        {
            Image::ImageInfo blobImageInfo = blobMask->QueryInfo();

            Image::ImageData blobImageData;
            blobMask->ExportData(&blobImageData, 0);

            user.userBlobImage.width = (uint32_t)blobImageInfo.width;
            user.userBlobImage.height = (uint32_t)blobImageInfo.height;
            user.userBlobImage.data.insert(user.userBlobImage.data.begin(), blobImageData.planes[0], blobImageData.planes[0] + blobImageInfo.width * blobImageInfo.height);
            user.userBlobImage.encoding = "8UC1";
            user.userBlobImage.step = (uint32_t)blobImageInfo.width;

            blobMask->Release();
        }
    }

    if (mSegmentationEnabled)
    {
        Image* segmentationMask = personTrackingData->QuerySegmentationImage();
        if (segmentationMask != nullptr)
        {
            Image::ImageInfo segImageInfo = segmentationMask->QueryInfo();

            Image::ImageData segImageData;
            segmentationMask->ExportData(&segImageData, 0);

            user.userSegmentationImage.width = (uint32_t)segImageInfo.width;
            user.userSegmentationImage.height = (uint32_t)segImageInfo.height;
            user.userSegmentationImage.data.insert(user.userSegmentationImage.data.begin(), segImageData.planes[0], segImageData.planes[0] + segImageInfo.width * segImageInfo.height);
            user.userSegmentationImage.encoding = "8UC1";
            user.userSegmentationImage.step = (uint32_t)segImageInfo.width;

            segmentationMask->Release();
        }
    }
}

void PersonTrackingPublisher::addSkeletonToOutput(PersonTrackingData::Person* personData, realsense_msgs::User& user)
{
    if (!mPersonTracking->QueryConfiguration()->QuerySkeletonJoints()->IsEnabled()) return;

    PersonTrackingData::PersonJoints* personJoints = personData->QuerySkeletonJoints();
    int numberOfJoints = personJoints->QueryNumJoints();

    if (numberOfJoints == 0) return;

    std::vector<PersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(numberOfJoints);
    personJoints->QueryJoints(skeletonPoints.data());

    /*skeletonPoints.erase(std::remove_if(
            skeletonPoints.begin(), skeletonPoints.end(),
            [] (PersonTrackingData::PersonJoints::SkeletonPoint& skeletonPoint) { return skeletonPoint.confidenceImage < 100; }),
        skeletonPoints.end());*/

    //ROS_DEBUG_STREAM("Number of confidence joints: " << skeletonPoints.size());

    for (PersonTrackingData::PersonJoints::SkeletonPoint& skeletonPoint : skeletonPoints)
    {
        //ROS_DEBUG("Joint of type: %d -> [%.0f,%.0f]", skeletonPoint.jointType, skeletonPoint.image.x, skeletonPoint.image.y);
        realsense_msgs::Landmark landmark;
        landmark.type = skeletonPoint.jointType;
        landmark.location.x = skeletonPoint.image.x;
        landmark.location.y = skeletonPoint.image.y;
        landmark.realWorldCoordinates.x = skeletonPoint.world.x;
        landmark.realWorldCoordinates.y = skeletonPoint.world.y;
        landmark.realWorldCoordinates.z = skeletonPoint.world.z;
        user.landmarks.push_back(landmark);
    }
}

void PersonTrackingPublisher::addGesturesToOutout(PersonTrackingData::Person* personData, realsense_msgs::User& user)
{
    if (!mPersonTracking->QueryConfiguration()->QueryGestures()->IsEnabled()) return;

    PersonTrackingData::PersonGestures* personGestures = personData->QueryGestures();
    if (personGestures == nullptr)
    {
        ROS_WARN("personGestures == nullptr");
        return;
    }

    if (personGestures->IsPointing())
    {
        PersonTrackingData::PersonGestures::PointingInfo poitingInfo = personGestures->QueryPointingInfo();
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
        //ROS_INFO("Person is pointing: [%.0f,%.0f] -> [%.0f,%.0f]", poitingInfo.colorPointingData.origin.x, poitingInfo.colorPointingData.origin.y, poitingInfo.colorPointingData.direction.x, poitingInfo.colorPointingData.direction.y);
        //ROS_INFO("Pointing world    : [%.0f,%.0f] -> [%.0f,%.0f]", poitingInfo.worldPointingData.origin.x, poitingInfo.worldPointingData.origin.y, poitingInfo.worldPointingData.direction.x, poitingInfo.worldPointingData.direction.y);
    }
}
