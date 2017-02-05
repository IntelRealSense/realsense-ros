#pragma once
#include "SegmentationRenderer.h"

class BlackSegmentationRenderer : public SegmentationRenderer
{
public:
    void RenderSegmentation(cv::Mat& colorImage, std::vector<cv::Mat>& segmentedImages) const override
    {
        for (cv::Mat segmentedImage : segmentedImages)
        {
            for (int row = 0; row < segmentedImage.rows; ++row)
            {
                for (int column = 0; column < segmentedImage.cols; ++column)
                {
                    int position = row * segmentedImage.cols + column;
                    if (segmentedImage.data[position] > 0)
                    {
                        colorImage.at<cv::Vec3b>(row,column)[0] = 0;
                        colorImage.at<cv::Vec3b>(row,column)[1] = 0;
                        colorImage.at<cv::Vec3b>(row,column)[2] = 0;
                    }
                }
            }
        }
    }
};
