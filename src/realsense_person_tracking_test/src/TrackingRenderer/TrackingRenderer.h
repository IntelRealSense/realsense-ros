#pragma once
#include "Viewer.h"
#include "PersonDataStorage.h"
#include "SegmentationRenderer.h"
#include <opencv2/core/core.hpp>
#include <vector>
#include <functional>
#include <memory>

class TrackingRenderer
{
public:
    enum SelectType
    {
        RECOGNITION,
        TRACKING
    };

    TrackingRenderer(Viewer& viewer);

    void SetPersonSelectionHandler(std::function<void (PersonData&, SelectType)> handler) { m_personSelectedHandler = handler; }

    void Reset();
    void DrawPerson(cv::Mat image, int id, cv::Rect boundingBox, cv::Point com, cv::Point3f comWorld);
    void DrawSkeleton(cv::Mat image, std::vector<cv::Point>& points);
    void DrawPointing(cv::Mat image, cv::Point origin, cv::Point2f direction);
    void DrawSegmentation(cv::Mat image, std::vector<cv::Mat>& segmentedImages);

    void SetSegmentationType(SegmentationRenderer::Type type);

private:
    void HandleMouseEvent(int event, int x, int y, int flags);

    Viewer& m_viewer;

    int mLastCenterOfMassTextBottom = 0;

    std::function<void (PersonData&, SelectType)> m_personSelectedHandler;

    PersonDataStorage m_personDataStorage;

    std::unique_ptr<SegmentationRenderer> m_segmentationRenderer;
};
