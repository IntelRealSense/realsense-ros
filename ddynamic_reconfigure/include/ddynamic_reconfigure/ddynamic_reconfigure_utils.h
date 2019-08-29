/**
 * Copyright 2019 PAL Robotics S.L.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef DDYNAMIC_RECONFIGURE_UTILS_H
#define DDYNAMIC_RECONFIGURE_UTILS_H
#include <string>
#include <limits>
#include <vector>

template <typename T>
inline T getMin()
{
  return std::numeric_limits<T>::min();
}

template <>
inline bool getMin()
{
  return false;
}

template <>
inline std::string getMin<std::string>()
{
  return "";
}

template <typename T>
inline T getMax()
{
  return std::numeric_limits<T>::min();
}

template <>
inline bool getMax()
{
  return true;
}

template <>
inline std::string getMax()
{
  return "";
}


template <class T, class V>
bool assignValue(std::vector<T> &v, std::string name, V value)
{
  for (unsigned int i = 0; i < v.size(); ++i)
  {
    if (v[i]->name_ == name)
    {
      v[i]->updateValue(value);
      return true;
    }
  }
  return false;
}

template <typename T>
void attemptGetParam(ros::NodeHandle &nh, const std::string &name, T &param, T default_value)
{
  if (nh.hasParam(name))
  {
    nh.param<T>(name, param, default_value);
  }
}
template <typename T>
std::pair<T, T> getMinMax(const std::map<std::string, T> &enum_map)
{
  T min, max;
  if (enum_map.empty())
  {
    throw std::runtime_error("Trying to register an empty enum");
  }
  
  min = enum_map.begin()->second;
  max = enum_map.begin()->second;
  
  for (const auto &it : enum_map)
  {
    min = std::min(min, it.second);
    max = std::max(min, it.second);
  }
  
  return std::make_pair(min, max);
}



#endif // DDYNAMIC_RECONFIGURE_UTILS_H
