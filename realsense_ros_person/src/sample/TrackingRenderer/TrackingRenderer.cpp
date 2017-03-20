// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <iomanip>
#include <opencv2/opencv.hpp>

#include "Colors.h"
#include "OpencvUtils.h"
#include "TrackingRenderer.h"

TrackingRenderer::TrackingRenderer(Viewer& viewer) : m_viewer(viewer)
{
  viewer.SetMouseEventHandler([this](int event, int x, int y, int flags)
  {
    HandleMouseEvent(event, x, y, flags);
  });
}

void TrackingRenderer::Reset()
{
  mSummaryTextBottom = 0;
}

void TrackingRenderer::DrawPerson(cv::Mat image, int id, cv::Rect boundingBox, cv::Point com, cv::Point3f comWorld)
{
  cv::Point pt1(boundingBox.x, boundingBox.y);
//    cv::Point pt2(boundingBox.x + boundingBox.width, boundingBox.y + boundingBox.height);

  cv::rectangle(image, boundingBox, YELLOW, 2);

  // person ID at top left corner of person rectangle
  int personId = id;
  std::string idText = "ID: " + std::to_string(personId);
  Rectangle personIdTextRect = setLabel(image, idText, pt1 + cv::Point(5, 5), 1, 0.4);

  // recognition ID under person ID
  auto personData = m_personDataStorage.get(id);
  int rid = (personData != nullptr && personData->rid > 0) ? personData->rid : -1;
  {
    std::string recognitionIdText = "RID: " + ((rid > 0) ? std::to_string(rid) : std::string("?"));
    setLabel(image, recognitionIdText, pt1 + cv::Point(5, 5 + personIdTextRect.height), 1, 0.4);
  }

  // center of mass point
  cv::Point centerMass = com;
  cv::circle(image, centerMass, 2, RED, -1, 8, 0);

  // Add person data to storage
  PersonData data;
  data.Id = personId;
  data.rid = rid;
  data.rectangle = boundingBox;
  m_personDataStorage.Add(data);
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

void TrackingRenderer::DrawFace(cv::Mat image, cv::Rect boundingBox)
{
  cv::Point pt1(boundingBox.x, boundingBox.y);
  cv::Point pt2(boundingBox.x + boundingBox.width, boundingBox.y + boundingBox.height);
  cv::rectangle(image, boundingBox, RED, 2);
}


void TrackingRenderer::HandleMouseEvent(int event, int x, int y, int flags)
{
  if (!m_personSelectedHandler) return;
  PersonData *data = m_personDataStorage.MatchPersonToPoint(cv::Point(x, y));
  if (data == nullptr) return;

  if (event == cv::EVENT_LBUTTONDOWN)
  {
    if ((flags & cv::EVENT_FLAG_CTRLKEY))
    {
      m_personSelectedHandler(*data, SelectType::RECOGNITION);
    }
    else
    {
      m_personSelectedHandler(*data, SelectType::REGISTRATION);
    }
  }
  else if (event == cv::EVENT_MBUTTONDOWN)
  {
    m_personSelectedHandler(*data, SelectType::TRACKING);
  }
}

void TrackingRenderer::DrawLineAtSummaryReport(cv::Mat image, std::string line)
{
  Rectangle summaryTextRect = setLabel(image, line, cv::Point(0, mSummaryTextBottom), 2,  0.5);
  mSummaryTextBottom = summaryTextRect.bottomRight.y + 0.02 * image.rows;
}