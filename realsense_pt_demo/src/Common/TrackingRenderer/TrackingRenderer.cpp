#include "TrackingRenderer.h"
#include "Colors.h"
#include "OpencvUtils.h"
#include "BlackSegmentationRenderer.h"
#include "WinStyleSegmentationRenderer.h"
#include <iomanip>
#include <iostream>

TrackingRenderer::TrackingRenderer() :  m_segmentationRenderer(new BlackSegmentationRenderer()){}
void TrackingRenderer::Reset()
{
    mSummaryTextBottom = 0;
}

void TrackingRenderer::DrawPerson(cv::Mat image, int id, cv::Rect boundingBox, cv::Point com, cv::Point3f comWorld,int rid)
{
    cv::Point pt1(boundingBox.x, boundingBox.y);
    cv::Point pt2(boundingBox.x + boundingBox.width, boundingBox.y + boundingBox.height);

    cv::rectangle(image, boundingBox, YELLOW, 2);

    // person ID at top left corner of person rectangle
    int personId = id;
    std::string idText = "ID: " + std::to_string(personId);
    Rectangle personIdTextRect = setLabel(image, idText, pt1 + cv::Point(5, 5), 1, 0.4);

    // recognition ID under person ID
    {
        std::string recognitionIdText = "RID: " + ((rid > 0) ? std::to_string(rid) : std::string("?"));
        setLabel(image, recognitionIdText, pt1 + cv::Point(5, 5 + personIdTextRect.height), 1, 0.4);
    }

    // center of mass point
    cv::Point centerMass = com;
    cv::circle(image, centerMass, 2, RED, -1, 8, 0);


    //print the distance_z to screen and file
    setLabel(image, std::to_string(comWorld.z), pt1 + cv::Point(5, 5 + personIdTextRect.height*3), 2, 0.4);

}

void TrackingRenderer::DrawFace(cv::Mat image, cv::Rect boundingBox)
{
    cv::Point pt1(boundingBox.x, boundingBox.y);
    cv::Point pt2(boundingBox.x + boundingBox.width, boundingBox.y + boundingBox.height);
    cv::rectangle(image, boundingBox, RED, 2);
}

void TrackingRenderer::DrawLineAtSummaryReport(cv::Mat image, std::string line)
{
    Rectangle summaryTextRect = setLabel(image, line, cv::Point(0, mSummaryTextBottom), 2,  0.5);
    mSummaryTextBottom = summaryTextRect.bottomRight.y;
}
void TrackingRenderer::DrawPoints(cv::Mat image, std::vector<cv::Point>& points, cv::Scalar color)
{
    for (auto point : points)
    {
        cv::circle(image, point, 3, GREEN, -1, 8, 0);
    }
}
void TrackingRenderer::DrawSkeleton(cv::Mat image, std::vector<cv::Point>& points)
{
    DrawPoints(image, points, GREEN);
}

void TrackingRenderer::DrawLandmarks(cv::Mat image, std::vector<cv::Point>& points)
{
    DrawPoints(image, points, GREEN);
}

void TrackingRenderer::DrawPointing(cv::Mat image, cv::Point origin, cv::Point2f direction)
{
    cv::circle(image, origin, 3, BLUE, -1, 8, 0);
    cv::Point vector(origin.x + 400 * direction.x, origin.y + 400 * direction.y);
    cv::arrowedLine(image, origin, vector, GREEN, 2, 8, 0, 0.15);
}

void TrackingRenderer::DrawSegmentation(cv::Mat image, std::vector<cv::Mat>& segmentedImages)
{
    if (m_segmentationRenderer) m_segmentationRenderer->RenderSegmentation(image, segmentedImages);
}

void TrackingRenderer::SetSegmentationType(SegmentationRenderer::Type type)
{
    if (type == SegmentationRenderer::Type::NONE)
    {
        m_segmentationRenderer = nullptr;
    }
    if (type == SegmentationRenderer::Type::BLACK)
    {
        m_segmentationRenderer = std::unique_ptr<SegmentationRenderer>(new BlackSegmentationRenderer());
    }
    if (type == SegmentationRenderer::Type::WIN_STYLE)
    {
        m_segmentationRenderer = std::unique_ptr<SegmentationRenderer>(new WinStyleSegmentationRenderer());
    }
}




