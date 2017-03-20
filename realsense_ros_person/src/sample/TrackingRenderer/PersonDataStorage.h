// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#pragma once
#include <opencv2/core/core.hpp>
#include <map>

class PersonData
{
public:
  int     Id;
  int         rid = 0;
  cv::Rect  rectangle;
  bool    valid;
};

class PersonDataStorage
{
public:
  void Add(PersonData data)
  {
    mPersonsData[data.Id] = data;
  }

  const PersonData* get(int id)
  {
    auto iter = mPersonsData.find(id);
    return (iter != mPersonsData.end()) ? &iter->second : nullptr;
  }

  //void clear() { mPersonsData.clear(); }

  PersonData* MatchPersonToPoint(cv::Point point)
  {
    for (auto iter = mPersonsData.rbegin(); iter != mPersonsData.rend(); ++iter)
    {
      if (iter->second.rectangle.contains(point))
      {
        return &iter->second;
      }
    }
    return nullptr;
  }

private:
  std::map<int, PersonData> mPersonsData;
};
