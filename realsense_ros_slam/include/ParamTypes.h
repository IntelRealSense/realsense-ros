// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once
#include "stdio.h"
typedef struct stCameraWorldParam
{
  float ffar;
  float fnear;
  float fFOV;
  float fCameraHeight2Width;

  void Default()
  {
    ffar = 2.5f;
    fnear = 0.3f;
    fFOV = 58.9f;
    fCameraHeight2Width = 9.0f / 16.0f;

  }
} stCameraParam;

struct stCameraImageParam
{
  int width;
  int height;
  int fps;
  void Init(int w, int h, int ifps)
  {
    width = w;
    height = h;
    fps = ifps;
  }
};

struct stCameraIntrinsics
{
  unsigned int iImageWidth;
  unsigned int iImageHeight;
  float fFocalLengthHorizontal;
  float fFocalLengthVertical;
  float fPrincipalPointCoordU;
  float fPrincipalPointCoordV;

  void Default()
  {
    iImageWidth = 320u;
    iImageHeight = 240u;
    fFocalLengthHorizontal = 316.f;
    fFocalLengthVertical = 316.f;
    fPrincipalPointCoordU = 160.f;
    fPrincipalPointCoordV = 120.f;
  }
};

typedef struct stMapInfo
{
  int w, h;
  float fLengthPerPixel;
  float x0, y0;
  int ix0, iy0;

  void Default()
  {
    w = 960;
    h = 960;
    fLengthPerPixel = 0.02f;
    x0 = 0.0f;
    y0 = 0.0f;
    ix0 = 960 / 2;
    iy0 = 960 / 2;
  }
  void Set(int _w, int _h, float _fLengthPerPixel, float _x0, float _y0, int _ix0, int _iy0)
  {
    w = _w;
    h = _h;
    fLengthPerPixel = _fLengthPerPixel;
    x0 = _x0;
    y0 = _y0;
    ix0 = _ix0;
    iy0 = _iy0;
  }
  void WorldToMap(float fx, float fy, int &ix, int &iy)
  {
    ix = (fx - x0) / fLengthPerPixel + ix0;
    iy = (fy - y0) / fLengthPerPixel + iy0;
  }
  void MapToWorld(int ix, int iy, float &fx, float &fy)
  {
    fx = (ix - ix0) * fLengthPerPixel + x0;
    fy = (iy - iy0) * fLengthPerPixel + y0;
  }
} stMapInfo;

typedef struct stRobotParam
{
  float fRobotHorizontalScale;
  float fMinHeightLowerThanCamera;
  float fMaxHeightHigherThanCamera;
  float fTolerateRadiusForNoise;

  void Default()
  {
    fRobotHorizontalScale = 0.6f;
    fMinHeightLowerThanCamera = 0.4f;
    fMaxHeightHigherThanCamera = 0.4f;
    fTolerateRadiusForNoise = 0.1f;
  }
} stRobotParam;

#define THETA_SAMPLE_NUM 31
#define THETA_SAMPLE_DTHETA 2.0f

typedef struct stCollisionAvoidanceParam
{
  float fDis2ObstacleMustStop;

  void Default()
  {
    fDis2ObstacleMustStop = 0.5f;
  }
} stCollisionAvoidanceParam;

// FOR JNI
typedef struct stCollisionAvoidanceCommand
{
  int command;
  float theta;
  void Default()
  {
    command = 0;
    theta = 0.0f;
  }
} stCollisionAvoidanceCommand;