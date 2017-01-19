#pragma once
#include <opencv2/core/core.hpp>
#include <vector>

class SegmentationRenderer
{
public:
    enum Type
    {
        NONE = 0,
        BLACK,
        WIN_STYLE
    };

    virtual void RenderSegmentation(cv::Mat& colorImage, std::vector<cv::Mat>& segmentedImages) const = 0;
};
