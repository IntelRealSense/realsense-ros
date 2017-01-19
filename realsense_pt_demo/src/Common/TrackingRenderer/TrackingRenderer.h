#pragma once
#include "SegmentationRenderer.h"
#include <opencv2/core/core.hpp>
#include <vector>
#include <functional>
#include <memory>

#include "RealSense/PersonTracking/PersonTrackingModule.h"
#include "RealSense/PersonTracking/PersonTrackingData.h"

class TrackingRenderer
{
public:
    TrackingRenderer();

    void Reset();
    void DrawPerson(cv::Mat image, int id, cv::Rect boundingBox, cv::Point com, cv::Point3f comWorld,int rid);
    void DrawFace(cv::Mat image, cv::Rect boundingBox);
    void DrawSkeleton(cv::Mat image, std::vector<cv::Point>& points);
    void DrawLandmarks(cv::Mat image, std::vector<cv::Point>& points);
    void DrawPointing(cv::Mat image, cv::Point origin, cv::Point2f direction);
    void DrawSegmentation(cv::Mat image, std::vector<cv::Mat>& segmentedImages);
    void DrawLineAtSummaryReport(cv::Mat image, std::string line);

    void SetSegmentationType(SegmentationRenderer::Type type);

private:
    void DrawPoints(cv::Mat image, std::vector<cv::Point>& points, cv::Scalar color);

    int mSummaryTextBottom = 0;

    std::unique_ptr<SegmentationRenderer> m_segmentationRenderer;
};
