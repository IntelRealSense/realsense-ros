// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once

#include <opencv2/opencv.hpp>
#define INNAV_RENDER_DATA_INSIDE

enum emNavStatus
{
  navInvalid,
  navContinue,
  navStay,
  navNavigationFinished,
  navNavigationFailed,
  navExploreFinished,
  navExploreFailed
};

typedef enum INNAV_TRACKING_ACCURACY : int
{
  innavLOW,
  innavMED,
  innavHIGH,
  innavFAILED
} INNAV_TRACKING_ACCURACY;

enum emNavMapPixel
{
  navUnobserved = 0,
  navUnObservedOpenThreshold = 41,
  navOpen = 80,
  navOpenObstacleThreshold = 121,
  navObstacle = 160,
  navObstacleUncertainOrSmall = 180
};

enum emMapUpdateStrategy
{
  innavNoUpdate,
  innavReplace,
  innavReplaceOnlyLargeObject,
  innavAddOnlyLargeObject,
  innavAdd,
  innavClear
};

enum emCollisionAvoidanceCommand
{
  avoidCollisionNoNeed = 0,
  avoidCollisionStop = 1,
  avoidCollisionTurn = 2
};

enum emRobotControlCommand
{
  robotcontrolStop = 0,
  robotcontrolMoveAndTurn = 1,
  robotcontrolTurnBody = 2,
  robotcontrolTurnHead = 3,
  robotcontrolIdle = 4
};

typedef struct stRobotControlCommand
{
  emRobotControlCommand type;
  float distance;
  float dtheta;
  //
  float dpan;
  float dtilt;

  float velocity_sugesstion;

  void Init()
  {
    type = robotcontrolIdle;
    distance = 0.0f;
    dtheta = 0.0f;
    dpan = 0.0f;
    dtilt = 0.0f;

    velocity_sugesstion = 1.0f;
  }
} stRobotControlCommand;

typedef struct stTrajTopology
{
  int hnext;
  int vnext;
  int node;
  void Default()
  {
    node = -1;
    hnext = -1;
    vnext = -1;
  }
} stTrajTopology;

typedef struct stRobotPG
{
  INNAV_TRACKING_ACCURACY trackingAccuracy;

  float x;
  float y;
  float theta;
  float pan;
  float tilt;
  emRobotControlCommand commandtype;
  //
  void initdefault()
  {
    trackingAccuracy = innavMED;
    x = 0.0f;
    y = 0.0f;
    theta = 0.0f;
    pan = 0.0f;
    tilt = 0.0f;
    commandtype = robotcontrolIdle;
  }
} stRobotPG;

typedef struct stTrajLeaf
{
  stRobotPG leafPG;
  stRobotPG rootPG;
  int rootID;
  bool bLive;
} stTrajLeaf;

float dis2D(CvPoint p1, CvPoint p2);
float dis2D(CvPoint2D32f p1, CvPoint2D32f p2);
CvPoint linelineIntersection(CvPoint P1, CvPoint P2, CvPoint P3, CvPoint P4);
float linelineAngle(CvPoint line1[2], CvPoint line2[2]);
float pointlineDis(CvPoint2D32f p1, CvPoint2D32f p2, CvPoint2D32f q);
//int min(int i, int j);
//int max(int i, int j);
void GetPixelofLine(CvPoint p1, CvPoint p2, std::vector<CvPoint> &pts);
void GetPixelofLine(CvPoint p1, CvPoint p2, int w, int h, int thickness, std::vector<CvPoint> &pts);
void GetPixelofLine(CvPoint p1, CvPoint p2, int thickness, std::vector<CvPoint> &pts);
void RoundTheta(float &theta);
stRobotPG AddRobotPG(stRobotPG PG1, stRobotPG PG2);
stRobotPG SubRobotPG(stRobotPG PG1, stRobotPG PG2);
