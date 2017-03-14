// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "Viewer.h"
#include <opencv2/highgui/highgui.hpp>

const char COLOR_WINDOW_NAME[] = "Person Tracking Sample";
const char DEPTH_WINDOW_NAME[] = "Person Tracking Sample Depth";

void mouseCallBack(int event, int x, int y, int flags, void* userdata)
{
  Viewer* viewer = (Viewer*)userdata;
  viewer->MouseEventCallback(event, x, y, flags);
}

Viewer::Viewer(bool showDepth) : m_showDepth(showDepth)
{
  cv::namedWindow(COLOR_WINDOW_NAME, CV_WINDOW_AUTOSIZE);
  cv::setMouseCallback(COLOR_WINDOW_NAME, mouseCallBack, this);
  cv::startWindowThread();

  if (showDepth)
  {
    cv::namedWindow(DEPTH_WINDOW_NAME, CV_WINDOW_AUTOSIZE);
    cv::startWindowThread();
  }
}

void Viewer::ShowImage(cv::Mat image)
{
  cv::imshow(COLOR_WINDOW_NAME, image);
  cv::waitKey(25);
}

void Viewer::ShowDepth(cv::Mat depth)
{
  cv::imshow(DEPTH_WINDOW_NAME, depth);
}

void Viewer::SetMouseEventHandler(std::function<void(int, int, int, int)> mouseEventHandler) \
{
  m_mouseEventHandler = mouseEventHandler;
}

void Viewer::MouseEventCallback(int event, int x, int y, int flags)
{
  if (m_mouseEventHandler) m_mouseEventHandler(event, x, y, flags);
}
