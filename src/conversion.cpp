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

#include <perception_camera_driver/conversion.hpp>
#include <rclcpp/rclcpp.hpp>

namespace perception_camera_driver
{
cv::Mat convert(const perception_camera_app::Image & image)
{
  if (image.format() == perception_camera_app::ImageFormat::CV8U) {
    std::vector<uint8_t> bytes(image.data().begin(), image.data().end());
    return cv::Mat(image.height(), image.width(), CV_8U, &bytes[0]);
  }
  if (image.format() == perception_camera_app::ImageFormat::CV8UC1) {
    std::vector<uint8_t> bytes(image.data().begin(), image.data().end());
    return cv::Mat(image.height(), image.width(), CV_8UC1, &bytes[0]);
  }
  if (image.format() == perception_camera_app::ImageFormat::CV8UC2) {
    std::vector<uint8_t> bytes(image.data().begin(), image.data().end());
    return cv::Mat(image.height(), image.width(), CV_8UC2, &bytes[0]);
  }
  if (image.format() == perception_camera_app::ImageFormat::CV8UC3) {
    std::vector<uint8_t> bytes(image.data().begin(), image.data().end());
    const auto mat = cv::Mat(image.height(), image.width(), CV_8UC3, &bytes[0]);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("logger"), mat.at<cv::Vec3b>(719,1279)[0] << "," << mat.at<cv::Vec3b>(719,1279)[1] << "," << mat.at<cv::Vec3b>(719,1279)[2]);
    return mat;
  }
  if (image.format() == perception_camera_app::ImageFormat::CV8UC4) {
    std::vector<uint8_t> bytes(image.data().begin(), image.data().end());
    return cv::Mat(image.height(), image.width(), CV_8UC4, &bytes[0]);
  }
  throw std::runtime_error("unsupported image format!!");
}

perception_camera_app::Image convert(const cv::Mat & image)
{
  if (image.type() == CV_8U) {
    perception_camera_app::Image proto;
    proto.set_height(image.size().height);
    proto.set_width(image.size().width);
    proto.set_format(perception_camera_app::ImageFormat::CV8U);
    int size = image.size().height * image.size().width;
    proto.set_data(reinterpret_cast<uint8_t const *>(image.data), size);
    return proto;
  }
  if (image.type() == CV_8UC1) {
    perception_camera_app::Image proto;
    proto.set_height(image.size().height);
    proto.set_width(image.size().width);
    proto.set_format(perception_camera_app::ImageFormat::CV8UC1);
    int size = image.size().height * image.size().width;
    proto.set_data(reinterpret_cast<uint8_t const *>(image.data), size);
    return proto;
  }
  if (image.type() == CV_8UC2) {
    perception_camera_app::Image proto;
    proto.set_height(image.size().height);
    proto.set_width(image.size().width);
    proto.set_format(perception_camera_app::ImageFormat::CV8UC2);
    int size = image.size().height * image.size().width * 2;
    proto.set_data(reinterpret_cast<uint8_t const *>(image.data), size);
    return proto;
  }
  if (image.type() == CV_8UC3) {
    perception_camera_app::Image proto;
    proto.set_height(image.size().height);
    proto.set_width(image.size().width);
    proto.set_format(perception_camera_app::ImageFormat::CV8UC3);
    int size = image.size().height * image.size().width * 3;
    proto.set_data(reinterpret_cast<uint8_t const *>(image.data), size);
    return proto;
  }
  if (image.type() == CV_8UC4) {
    perception_camera_app::Image proto;
    proto.set_height(image.size().height);
    proto.set_width(image.size().width);
    proto.set_format(perception_camera_app::ImageFormat::CV8UC4);
    int size = image.size().height * image.size().width * 4;
    proto.set_data(reinterpret_cast<uint8_t const *>(image.data), size);
    return proto;
  }
  throw std::runtime_error("unsupported image format!!");
}

perception_camera_app::Time convert(const std::chrono::system_clock::time_point & time)
{
  perception_camera_app::Time proto;
  std::int64_t seconds =
    std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
  proto.set_sec(seconds);
  std::int64_t nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
  proto.set_nanosec(nanoseconds - seconds * 1000000000);
  return proto;
}

perception_camera_app::Time now() { return convert(std::chrono::system_clock::now()); }

rclcpp::Time convert(const perception_camera_app::Time & time)
{
  return rclcpp::Time(time.sec(), time.nanosec());
}

perception_camera_app::ImageStamped convert(
  const cv::Mat & image, const std::chrono::system_clock::time_point & time)
{
  perception_camera_app::ImageStamped proto;
  *proto.mutable_image() = convert(image);
  *proto.mutable_stamp() = convert(time);
  return proto;
}
}  // namespace perception_camera_driver