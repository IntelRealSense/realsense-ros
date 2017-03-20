// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once
#include "Viewer.h"
#include "PersonDataStorage.h"
#include <opencv2/core/core.hpp>
#include <vector>
#include <functional>
#include <memory>

class TrackingRenderer
{
public:
  enum SelectType
  {
    ACTIVATE_RECOGNITION,
    RECOGNITION,
    REGISTRATION,
    TRACKING
  };

  TrackingRenderer(Viewer& viewer);

  void SetPersonSelectionHandler(std::function<void (PersonData&, SelectType)> handler)
  {
    m_personSelectedHandler = handler;
  }
  void SetGlobalHandler(std::function<void (SelectType)> handler)
  {
    m_globaldHandler = handler;
  }

  void Reset();
  void DrawPoints(cv::Mat image, std::vector<cv::Point>& points, cv::Scalar color);
  void DrawPerson(cv::Mat image, int id, cv::Rect boundingBox, cv::Point com, cv::Point3f comWorld);
  void DrawFace(cv::Mat image, cv::Rect boundingBox);
  void DrawSkeleton(cv::Mat image, std::vector<cv::Point>& points);
  void DrawLandmarks(cv::Mat image, std::vector<cv::Point>& points);
  void DrawPointing(cv::Mat image, cv::Point origin, cv::Point2f direction);
  void DrawLineAtSummaryReport(cv::Mat image, std::string line);

private:
  void HandleMouseEvent(int event, int x, int y, int flags);

  int mSummaryTextBottom = 0;
  Viewer& m_viewer;

  std::function<void (PersonData&, SelectType)> m_personSelectedHandler;
  std::function<void (SelectType)> m_globaldHandler;

  PersonDataStorage m_personDataStorage;
};
