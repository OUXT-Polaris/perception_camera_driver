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

#ifndef PERCEPTION_CAMERA_DRIVER__IMAGE_SUBSCRIBER_COMPONENT_HPP_
#define PERCEPTION_CAMERA_DRIVER__IMAGE_SUBSCRIBER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_EXPORT __attribute__((dllexport))
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_EXPORT __declspec(dllexport)
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_BUILDING_DLL
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_PUBLIC \
  PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_EXPORT
#else
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_PUBLIC \
  PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_IMPORT
#endif
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_PUBLIC_TYPE \
  PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_PUBLIC
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_LOCAL
#else
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_PUBLIC
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_LOCAL
#endif
#define PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>
#include <memory>
#include <perception_camera_driver/subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace perception_camera_driver
{
class ImageSubscriberComponent : public rclcpp::Node
{
public:
  PERCEPTION_CAMERA_DRIVER_IMAGE_SUBSCRIBER_COMPONENT_PUBLIC
  explicit ImageSubscriberComponent(const rclcpp::NodeOptions & options);
  std::string getIpAddress() const { return ip_address_; }
  int getPort() const { return port_; }
  std::string getEndpoint() const { return endpoint_; }

private:
  void messageCallback(const zmqpp::message & message);
  void imageCallback(const cv::Mat & image, const rclcpp::Time & stamp);
  image_transport::Publisher image_pub_;
  std::unique_ptr<perception_camera_direver::Subscriber> subscriber_;
  std::string ip_address_;
  int port_;
  std::string endpoint_;
  std::string frame_id_;
};
}  // namespace perception_camera_driver

#endif  // PERCEPTION_CAMERA_DRIVER__IMAGE_SUBSCRIBER_COMPONENT_HPP_