// Copyright (c) 2020 OUXT Polaris
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

#ifndef PERCEPTION_CAMERA_DRIVER__CONVERSION_HPP_
#define PERCEPTION_CAMERA_DRIVER__CONVERSION_HPP_

#include <perception_camera_app.pb.h>

#include <chrono>
#include <opencv2/opencv.hpp>
#include <zmqpp/zmqpp.hpp>

namespace perception_camera_direver
{
cv::Mat convert(const perception_camera_app::Image & image);
perception_camera_app::Image convert(const cv::Mat & image);
perception_camera_app::ImageStamped convert(
  const cv::Mat & image, const std::chrono::system_clock::time_point & time);
Time convert(const std::chrono::system_clock::time_point & time);
Time now();

template <typename Proto>
void toZMQ(const Proto & proto, zmqpp::message & msg)
{
  std::string serialized_str = "";
  proto.SerializeToString(&serialized_str);
  msg << serialized_str;
}

template <typename Proto>
void toProto(const zmqpp::message & msg, Proto & proto)
{
  std::string serialized_str = msg.get(0);
  proto.ParseFromString(serialized_str);
}
}  // namespace perception_camera_direver

#endif  // PERCEPTION_CAMERA_DRIVER__CONVERSION_HPP_
