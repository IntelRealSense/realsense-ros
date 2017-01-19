#pragma once
#include "SegmentationRenderer.h"

class WinStyleSegmentationRenderer : public SegmentationRenderer
{
public:
    void RenderSegmentation(cv::Mat& colorImage, std::vector<cv::Mat>& segmentedImages) const override
    {
        if (segmentedImages.empty()) return;

        cv::Mat segmentedImage = segmentedImages[0];

        for (int i = 1; i < segmentedImages.size(); ++i)
        {
            cv::Mat segmentedImage2 = segmentedImages[i];

            for (int i = 0; i < colorImage.rows * colorImage.cols; ++i)
            {
                segmentedImage.data[i] = segmentedImage.data[i] | segmentedImage2.data[i];
            }
        }

        BlendColorPop(colorImage, segmentedImage);
    }

    void BlendColorPop(cv::Mat& colorImage, cv::Mat& segmentedImage) const
    {
        const char GREY = 0x7f;
        for (int row = 0; row < colorImage.rows; ++row)
        {
            for (int column = 0; column < colorImage.cols; ++column)
            {
                int position = row * colorImage.cols + column;
                int alpha = segmentedImage.data[position];

                colorImage.at<cv::Vec3b>(row,column)[0] = ( alpha*colorImage.at<cv::Vec3b>(row,column)[0] ) + ( 1-alpha )*( (colorImage.at<cv::Vec3b>(row,column)[0] >> 4) + GREY );
                colorImage.at<cv::Vec3b>(row,column)[1] = ( alpha*colorImage.at<cv::Vec3b>(row,column)[1] ) + ( 1-alpha )*( (colorImage.at<cv::Vec3b>(row,column)[1] >> 4) + GREY );
                colorImage.at<cv::Vec3b>(row,column)[2] = ( alpha*colorImage.at<cv::Vec3b>(row,column)[2] ) + ( 1-alpha )*( (colorImage.at<cv::Vec3b>(row,column)[2] >> 4) + GREY );
            }
        }
    }
};
