// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct Rectangle
{
  Rectangle(cv::Point topLeft, cv::Point bottomRight) : topLeft(topLeft), bottomRight(bottomRight)
  {
    width = bottomRight.x - topLeft.x;
    height = bottomRight.y - topLeft.y;
  }
  cv::Point topLeft;
  cv::Point bottomRight;
  int width;
  int height;
};

/**
    Creates a black text on white background

    @param image - add the text on this image.
  @param label - the text to add.
  @param origin - bottom left corner of the text.
  @param thickness - thickness of the text font
    @return the rectangle of the text
*/
Rectangle setLabel(cv::Mat& image, const std::string label, const cv::Point& origin, int thickness = 2, double scale = 0.5)
{
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  int baseline = 0;

  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);

  cv::Point topLeft = origin + cv::Point(-3, -3);
  cv::Point bottomRight = origin + cv::Point(text.width + 3, text.height + 3);

  cv::rectangle(image, topLeft, bottomRight, WHITE, CV_FILLED);
  cv::putText(image, label, origin + cv::Point(0, text.height), fontface, scale, BLACK, thickness, 8);

  return Rectangle(topLeft, bottomRight);
}

namespace cv
{
void arrowedLine(cv::Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness = 1, int line_type = 8, int shift = 0, double tipLength = 0.1)
{
  const double tipSize = norm(pt1 - pt2) * tipLength; // Factor to normalize the size of the tip depending on the length of the arrow
  line(img, pt1, pt2, color, thickness, line_type, shift);
  const double angle = atan2((double) pt1.y - pt2.y, (double) pt1.x - pt2.x);
  Point p(cvRound(pt2.x + tipSize * cos(angle + CV_PI / 4)),
          cvRound(pt2.y + tipSize * sin(angle + CV_PI / 4)));
  line(img, p, pt2, color, thickness, line_type, shift);
  p.x = cvRound(pt2.x + tipSize * cos(angle - CV_PI / 4));
  p.y = cvRound(pt2.y + tipSize * sin(angle - CV_PI / 4));
  line(img, p, pt2, color, thickness, line_type, shift);
}
}
