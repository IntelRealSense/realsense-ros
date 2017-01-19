#include "PersonTracker.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <list>
#include <iomanip>

using namespace RS::PersonTracking;

PersonTracker::PersonTracker(TrackingRenderer& trackingRenderer, RS::PersonTracking::PersonTrackingConfiguration* ptConfiguration):
    m_trackingRenderer(trackingRenderer)
    ,m_personTrackingConfig(ptConfiguration){}

float PersonTracker::DrawFrame(cv::Mat image, PersonTrackingData* trackingData)
{
    float tmp = 0.0f;
    std::lock_guard<std::mutex> lock(mMutex);
    m_trackingRenderer.Reset();
    m_personTrackingData = trackingData;
    for (int index = 0; index < trackingData->QueryNumberOfPeople(); ++index)
    {
        PersonTrackingData::Person* personData = trackingData->QueryPersonData(PersonTrackingData::ACCESS_ORDER_BY_INDEX, index);
        if (personData)
        {
            tmp = DrawPerson(image, personData);
            DrawSkeleton(image, personData);
            DrawGestures(image, personData);
            DrawLandmarks(image, personData);
            DrawFace(image, personData);
            DrawSummaryReport(image, personData);
        }
    }
    DrawSegmentation(image, trackingData);
    return tmp;
}

void PersonTracker::DrawSkeleton(cv::Mat image, PersonTrackingData::Person* personData)
{
    if (!m_personTrackingConfig->QuerySkeletonJoints()->IsEnabled()) return;

    PersonTrackingData::PersonJoints* personJoints = personData->QuerySkeletonJoints();
    int numberOfJoints = personJoints->QueryNumJoints();

    if (numberOfJoints == 0) return;

    std::vector<PersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(numberOfJoints);
    personJoints->QueryJoints(skeletonPoints.data());

    skeletonPoints.erase(std::remove_if(
            skeletonPoints.begin(), skeletonPoints.end(),
            [] (PersonTrackingData::PersonJoints::SkeletonPoint& skeletonPoint) { return skeletonPoint.confidenceImage > SKELETON_POINT_CONFIDENCE_THR; }),
        skeletonPoints.end());

    std::vector<cv::Point> points;
    for (PersonTrackingData::PersonJoints::SkeletonPoint& skeletonPoint : skeletonPoints)
    {
        points.push_back(cv::Point(skeletonPoint.image.x, skeletonPoint.image.y));
    }

    m_trackingRenderer.DrawSkeleton(image, points);
}

void PersonTracker::DrawGestures(cv::Mat image, PersonTrackingData::Person* personData)
{
    if (!m_personTrackingConfig->QueryGestures()->IsEnabled()) return;

    PersonTrackingData::PersonGestures* personGestures = personData->QueryGestures();
    if (personGestures == nullptr)
    {
        return;
    }

    if (personGestures->IsPointing() && personGestures->QueryPointingInfo().confidence > 0)
    {
        PersonTrackingData::PersonGestures::PointingInfo poitingInfo = personGestures->QueryPointingInfo();
        cv::Point origin(poitingInfo.colorPointingData.origin.x, poitingInfo.colorPointingData.origin.y);
        cv::Point2f direction(poitingInfo.colorPointingData.direction.x, poitingInfo.colorPointingData.direction.y);
        m_trackingRenderer.DrawPointing(image, origin, direction);
    }
}

std::mutex& PersonTracker::GetPersonTrackingMutex()
{
    return mMutex;
}

void PersonTracker::DrawSegmentation(cv::Mat image, RS::PersonTracking::PersonTrackingData* trackingData)
{

    if (!m_personTrackingConfig->QueryTracking()->IsSegmentationEnabled()) return;

    std::vector<RS::Image*> pxcImages;
    std::vector<cv::Mat> segmentedImages;
    for (int index = 0; index < trackingData->QueryNumberOfPeople(); ++index)
    {
        RS::PersonTracking::PersonTrackingData::Person* personData = trackingData->QueryPersonData(RS::PersonTracking::PersonTrackingData::ACCESS_ORDER_BY_INDEX, index);
        if (personData)
        {
            RS::PersonTracking::PersonTrackingData::PersonTracking* personTrackingData = personData->QueryTracking();
            RS::Image* segmentationMask = personTrackingData->QuerySegmentationImage();

            if (segmentationMask != nullptr)
            {
                int a = segmentationMask->QueryInfo().width;
                RS::Image::ImageInfo segImageInfo = segmentationMask->QueryInfo();


                RS::Image::ImageData segImageData;
                segmentationMask->ExportData(&segImageData, 0);

                cv::Mat segmentedImage(segImageInfo.height, segImageInfo.width, CV_8UC1, (void*)segImageData.planes[0]);

                pxcImages.push_back(segmentationMask);
                segmentedImages.push_back(segmentedImage);
            }
        }
    }

    m_trackingRenderer.DrawSegmentation(image, segmentedImages);

    for (RS::Image* image : pxcImages) image->Release();
}

void PersonTracker::DrawLandmarks(cv::Mat image, RS::PersonTracking::PersonTrackingData::Person *personData)
{
    typedef RS::PersonTracking::PersonTrackingData PersonTrackingData;
    if (!m_personTrackingConfig->QueryFace()->IsFaceLandmarksEnabled()) return;

    auto landmarksInfo = personData->QueryFace()->QueryLandmarks();
    int numberOfLandmarks = personData->QueryFace()->QueryNumLandmarks();
    if (numberOfLandmarks == 0 || landmarksInfo->confidence < LANDMARKS_CONFIDENCE_THR) return;

    std::vector<cv::Point> landmarkPoints(landmarksInfo->numLandmarks);
    std::transform(landmarksInfo->landmarks, landmarksInfo->landmarks + landmarksInfo->numLandmarks, landmarkPoints.begin(),
        [](PersonTrackingData::PersonFace::LandmarksLocation location) {return cv::Point(location.image.x, location.image.y);});

    m_trackingRenderer.DrawLandmarks(image, landmarkPoints);
}


float PersonTracker::DrawPerson(cv::Mat image, RS::PersonTracking::PersonTrackingData::Person* personData)
{
    PersonTrackingData::PersonTracking* personTrackingData = personData->QueryTracking();
    PersonTrackingData::BoundingBox2D box = personTrackingData->Query2DBoundingBox();
    PersonTrackingData::PointCombined centerMass = personTrackingData->QueryCenterMass();

    int id = personTrackingData->QueryId();

    cv::Point pt1(box.rect.x, box.rect.y);
    cv::Point pt2(box.rect.x + box.rect.w, box.rect.y + box.rect.h);
    cv::Rect userRectangle(pt1, pt2);

    cv::Point com(centerMass.image.point.x, centerMass.image.point.y);
    cv::Point3f comWorld(centerMass.world.point.x, centerMass.world.point.y, centerMass.world.point.z);
    int rid=-1;

    if(m_personTrackingConfig->QueryRecognition()->IsEnabled()){
        if(personData->QueryRecognition()!=nullptr)
        {
            PersonTrackingData::PersonRecognition* recognition = personData->QueryRecognition();
            PersonTrackingData::PersonRecognition::RecognizerData recognizerData;
            PersonTrackingData::PersonRecognition::RecognitionStatus status = recognition->RecognizeUser(&recognizerData);
            if (status == PersonTrackingData::PersonRecognition::RecognitionPassedPersonRecognized)
            {
                rid = recognizerData.recognitionId;
            }
         }

    }
    m_trackingRenderer.DrawPerson(image, id, userRectangle, com, comWorld,rid);
    return comWorld.z;
}

void PersonTracker::DrawFace(cv::Mat image, RS::PersonTracking::PersonTrackingData::Person* personData)
{
    auto headBoundingBox = personData->QueryTracking()->QueryHeadBoundingBox();
    if (headBoundingBox.confidence > HEAD_BOUNDING_BOX_THR)
    {
        cv::Point pt1(headBoundingBox.rect.x, headBoundingBox.rect.y);
        cv::Point pt2(headBoundingBox.rect.x + headBoundingBox.rect.w, headBoundingBox.rect.y + headBoundingBox.rect.h);
        cv::Rect headBoundingBoxRect(pt1, pt2);
        m_trackingRenderer.DrawFace(image, headBoundingBoxRect);
    }
}

void PersonTracker::DrawSummaryReport(cv::Mat image, RS::PersonTracking::PersonTrackingData::Person* personData)
{
    auto personTrackingData = personData->QueryTracking();
    RS::PersonTracking::PointCombined centerMass = personTrackingData->QueryCenterMass();
    std::stringstream summaryText;// summary text at at top left corner of image (center of mass, orientation etc.)

    //add center of mass (world coordinates)
    summaryText << personData->QueryTracking()->QueryId() << ": " <<
    std::fixed << std::setprecision(3) <<
    "(" << centerMass.world.point.x << "," << centerMass.world.point.y << "," << centerMass.world.point.z << ")";//TODO check confidence????
#ifdef PT_MW_DEV

    //add orientation
    auto personOrientation = personData->QueryTracking()->QueryPersonOrientation();
    if (personOrientation.confidence > ORIENTATION_CONFIDENCE_THR) //if orientation feature is disabled, confidence should be zero
    {
        summaryText << " orientation: " << OrientationToString(personOrientation.orientation) << " confidence: " << personOrientation.confidence << "\n";
    }
#endif
    //add head angles
    RS::PersonTracking::PersonTrackingData::PoseEulerAngles headAngles;
    if (personData->QueryFace()->QueryHeadPose(headAngles))
    {
        summaryText << " head orientation " << "(" << headAngles.pitch << ", " << headAngles.roll << ", " << headAngles.yaw << ")";
    }
    m_trackingRenderer.DrawLineAtSummaryReport(image, summaryText.str());
}


std::string PersonTracker::OrientationToString(RS::PersonTracking::PersonTrackingData::PersonTracking::PersonOrientation orientation)
{
    typedef  RS::PersonTracking::PersonTrackingData::PersonTracking PersonTracking;
    switch (orientation)
    {
        case PersonTracking::ORIENTATION_FRONTAL:
            return "frontal";
        case PersonTracking::ORIENTATION_45_DEGREE_RIGHT:
            return "45 degree right";
        case PersonTracking::ORIENTATION_45_DEGREE_LEFT:
            return "45 degree left";
        case PersonTracking::ORIENTATION_PROFILE_RIGHT:
            return "right";
        case PersonTracking::ORIENTATION_PROFILE_LEFT:
            return "left";
        case PersonTracking::ORIENTATION_REAR:
            return "right";
        default:
            throw std::runtime_error("unsupported orientation value");
    }
}

std::string PersonTracker::RecognitionStatusToString(RecognitionStatus status)
{
    switch (status)
    {

        case PersonTrackingData::PersonRecognition::RecognitionPassedPersonRecognized:
            return "RecognitionPassedPersonRecognized";
        case PersonTrackingData::PersonRecognition::RecognitionPassedPersonNotRecognized:
            return "RecognitionPassedPersonNotRecognized";
        case PersonTrackingData::PersonRecognition::RecognitionFailed:
            return "RecognitionFailed";
        case PersonTrackingData::PersonRecognition::RecognitionFailedFaceNotDetected:
            return "RecognitionFailedFaceNotDetected";
        case PersonTrackingData::PersonRecognition::RecognitionFailedFaceNotClear:
            return  "RecognitionFailedFaceNotClear";
        case PersonTrackingData::PersonRecognition::RecognitionFailedPersonTooFar:
            return  "RecognitionFailedPersonTooFar";
        case PersonTrackingData::PersonRecognition::RecognitionFailedPersonTooClose:
            "RecognitionFailedPersonTooClose";
        default:
            return  "unknown recognition status please check the code";
    }
}
