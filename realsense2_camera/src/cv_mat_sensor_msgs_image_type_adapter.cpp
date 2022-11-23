// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <variant>  // NOLINT[build/include_order]

#include "opencv2/core/mat.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "cv_mat_sensor_msgs_image_type_adapter.hpp"

namespace cv_type_adapt
{

namespace
{
int
encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else if (encoding == "yuv422") {
    return CV_8UC2;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

template<typename T>
struct NotNull
{
  NotNull(const T * pointer_in, const char * msg)
  : pointer(pointer_in)
  {
    if (pointer == nullptr) {
      throw std::invalid_argument(msg);
    }
  }

  const T * pointer;
};

}  // namespace

ROSCvMatContainer::ROSCvMatContainer(
  std::unique_ptr<sensor_msgs::msg::Image> unique_sensor_msgs_image)
: header_(NotNull(
      unique_sensor_msgs_image.get(),
      "unique_sensor_msgs_image cannot be nullptr"
).pointer->header),
  frame_(
    unique_sensor_msgs_image->height,
    unique_sensor_msgs_image->width,
    encoding2mat_type(unique_sensor_msgs_image->encoding),
    unique_sensor_msgs_image->data.data(),
    unique_sensor_msgs_image->step),
  storage_(std::move(unique_sensor_msgs_image))
{}

ROSCvMatContainer::ROSCvMatContainer(
  std::shared_ptr<sensor_msgs::msg::Image> shared_sensor_msgs_image)
: header_(shared_sensor_msgs_image->header),
  frame_(
    shared_sensor_msgs_image->height,
    shared_sensor_msgs_image->width,
    encoding2mat_type(shared_sensor_msgs_image->encoding),
    shared_sensor_msgs_image->data.data(),
    shared_sensor_msgs_image->step),
  storage_(shared_sensor_msgs_image)
{}

ROSCvMatContainer::ROSCvMatContainer(
  const cv::Mat & mat_frame,
  const std_msgs::msg::Header & header,
  bool is_bigendian)
: header_(header),
  frame_(mat_frame),
  storage_(nullptr),
  is_bigendian_(is_bigendian)
{}

ROSCvMatContainer::ROSCvMatContainer(
  cv::Mat && mat_frame,
  const std_msgs::msg::Header & header,
  bool is_bigendian)
: header_(header),
  frame_(std::forward<cv::Mat>(mat_frame)),
  storage_(nullptr),
  is_bigendian_(is_bigendian)
{}

ROSCvMatContainer::ROSCvMatContainer(
  const sensor_msgs::msg::Image & sensor_msgs_image)
: ROSCvMatContainer(std::make_unique<sensor_msgs::msg::Image>(sensor_msgs_image))
{}

bool
ROSCvMatContainer::is_owning() const
{
  return std::holds_alternative<std::nullptr_t>(storage_);
}

const cv::Mat &
ROSCvMatContainer::cv_mat() const
{
  return frame_;
}

cv::Mat
ROSCvMatContainer::cv_mat()
{
  return frame_;
}

const std_msgs::msg::Header &
ROSCvMatContainer::header() const
{
  return header_;
}

std_msgs::msg::Header &
ROSCvMatContainer::header()
{
  return header_;
}

std::shared_ptr<const sensor_msgs::msg::Image>
ROSCvMatContainer::get_sensor_msgs_msg_image_pointer() const
{
  if (!std::holds_alternative<std::shared_ptr<sensor_msgs::msg::Image>>(storage_)) {
    return nullptr;
  }
  return std::get<std::shared_ptr<sensor_msgs::msg::Image>>(storage_);
}

std::unique_ptr<sensor_msgs::msg::Image>
ROSCvMatContainer::get_sensor_msgs_msg_image_pointer_copy() const
{
  auto unique_image = std::make_unique<sensor_msgs::msg::Image>();
  this->get_sensor_msgs_msg_image_copy(*unique_image);
  return unique_image;
}

void
ROSCvMatContainer::get_sensor_msgs_msg_image_copy(
  sensor_msgs::msg::Image & sensor_msgs_image) const
{
  sensor_msgs_image.height = frame_.rows;
  sensor_msgs_image.width = frame_.cols;
  switch (frame_.type()) {
    case CV_8UC1:
      sensor_msgs_image.encoding = "mono8";
      break;
    case CV_8UC3:
      sensor_msgs_image.encoding = "bgr8";
      break;
    case CV_16SC1:
      sensor_msgs_image.encoding = "mono16";
      break;
    case CV_8UC4:
      sensor_msgs_image.encoding = "rgba8";
      break;
    default:
      throw std::runtime_error("unsupported encoding type");
  }
  sensor_msgs_image.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);
  size_t size = frame_.step * frame_.rows;
  sensor_msgs_image.data.resize(size);
  memcpy(&sensor_msgs_image.data[0], frame_.data, size);
  sensor_msgs_image.header = header_;
}

bool
ROSCvMatContainer::is_bigendian() const
{
  return is_bigendian_;
}

}  // namespace cv_type_adapt
