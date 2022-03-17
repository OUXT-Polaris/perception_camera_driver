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
#include <perception_camera_driver/image_subscriber_component.hpp>

namespace perception_camera_driver
{
ImageSubscriberComponent::ImageSubscriberComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_subscriber", options),
  subscriber_(
    "image_raw",
    perception_camera_driver::resolve(perception_camera_driver::Transport::kTcp, "localhost", 8000))
{
  image_pub_ = image_transport::create_publisher(this, "image_raw");
}

void ImageSubscriberComponent::imageCallback(const cv::Mat & image)
{
  /*
  if (image_pub_.getNumSubscribers() < 1) {
    return;
  }
  */
  std_msgs::msg::Header header;
  sensor_msgs::msg::Image::SharedPtr rect_image =
    cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  image_pub_.publish(rect_image);
}

void ImageSubscriberComponent::messageCallback(const zmqpp::message & message)
{
  perception_camera_app::ImageStamped proto;
  toProto(message, proto);
  imageCallback(convert(proto.image()));
}

// void ImageSubscriberComponent::startPoll() {}
}  // namespace perception_camera_driver