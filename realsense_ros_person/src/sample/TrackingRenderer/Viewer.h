// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once
#include <opencv2/core/core.hpp>
#include <functional>

class Viewer
{
public:
  Viewer(bool showDepth);

  void ShowImage(cv::Mat image);
  void ShowDepth(cv::Mat depth);

  void SetMouseEventHandler(std::function<void(int, int, int, int)> mouseEventHandler);

  void MouseEventCallback(int event, int x, int y, int flags);

private:
  bool m_showDepth;

  std::function<void(int, int, int, int)> m_mouseEventHandler;
};
