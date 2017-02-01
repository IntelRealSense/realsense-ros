#pragma once
#include "TrackingRenderer.h"
#include <mutex>
#include <map>

namespace RS = Intel::RealSense;
class PersonTracker
{

public:
    PersonTracker(TrackingRenderer& trackingRenderer, RS::PersonTracking::PersonTrackingConfiguration* ptConfiguration);

    float DrawFrame(cv::Mat image, RS::PersonTracking::PersonTrackingData* trackingData);
    void SetSaveDbCallback(std::function<void(RS::PersonTracking::PersonTrackingConfiguration*)> saveDbCallback);
    std::mutex& GetPersonTrackingMutex();
    void DrawSegmentation(cv::Mat image, RS::PersonTracking::PersonTrackingData* trackingData);
private:
    void TrackPerson(RS::PersonTracking::PersonTrackingData::Person* personData);

    float DrawPerson(cv::Mat image, RS::PersonTracking::PersonTrackingData::Person* personData);
    void DrawFace(cv::Mat image, RS::PersonTracking::PersonTrackingData::Person* personData);
    void DrawSkeleton(cv::Mat image, RS::PersonTracking::PersonTrackingData::Person* personData);
    void DrawGestures(cv::Mat image, RS::PersonTracking::PersonTrackingData::Person* personData);
    void DrawLandmarks(cv::Mat image, RS::PersonTracking::PersonTrackingData::Person* personData);
    //draw text report for person
    void DrawSummaryReport(cv::Mat image, RS::PersonTracking::PersonTrackingData::Person* personData);

    std::string OrientationToString(RS::PersonTracking::PersonTrackingData::PersonTracking::PersonOrientation orientation);
    std::string RecognitionStatusToString(RS::PersonTracking::RecognitionStatus status);

    TrackingRenderer& m_trackingRenderer;
    RS::PersonTracking::PersonTrackingConfiguration* m_personTrackingConfig;
    RS::PersonTracking::PersonTrackingData* m_personTrackingData;
    std::function<void(RS::PersonTracking::PersonTrackingConfiguration*)> m_saveDbCallback;
    std::mutex mMutex;

    static constexpr  float LANDMARKS_CONFIDENCE_THR = 50.0f;
    static constexpr  float ORIENTATION_CONFIDENCE_THR = 1.0f;
    static constexpr  float HEAD_BOUNDING_BOX_THR = 50.0f;
    static constexpr  float SKELETON_POINT_CONFIDENCE_THR = 99.0f;
};
