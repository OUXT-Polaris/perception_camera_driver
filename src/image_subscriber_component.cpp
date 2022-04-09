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
#include <rclcpp_components/register_node_macro.hpp>

namespace perception_camera_driver
{
ImageSubscriberComponent::ImageSubscriberComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_subscriber", options)
{
  declare_parameter<std::string>("ip_address", "localhost");
  get_parameter("ip_address", ip_address_);
  declare_parameter<int>("port", 8000);
  get_parameter("port", port_);
  declare_parameter<std::string>("frame_id", "base_link");
  get_parameter<std::string>("frame_id", frame_id_);
  std::string image_topic_name;
  declare_parameter<std::string>("image_topic_name", "image_topic_name");
  get_parameter<std::string>("image_topic_name", image_topic_name);
  endpoint_ = perception_camera_driver::resolve(
    perception_camera_driver::Transport::kTcp, ip_address_, port_);
  image_pub_ = image_transport::create_publisher(this, image_topic_name);
  subscriber_ = std::unique_ptr<perception_camera_direver::Subscriber>(
    new perception_camera_direver::Subscriber(
      std::bind(&ImageSubscriberComponent::messageCallback, this, std::placeholders::_1),
      get_logger(), endpoint_));
}

void ImageSubscriberComponent::imageCallback(const cv::Mat & image, const rclcpp::Time & stamp)
{
  std_msgs::msg::Header header;
  header.frame_id = frame_id_;
  header.stamp = stamp;
  sensor_msgs::msg::Image ros_image;
  ros_image.header = header;
  ros_image.height = image.rows;
  ros_image.width = image.cols;
  ros_image.encoding = "bgr8";
  ros_image.is_bigendian = false;
  ros_image.step = image.cols * image.elemSize();
  std::vector<uint8_t> image_vec;
  image_vec.assign((uint8_t *)image.datastart, (uint8_t *)image.dataend);
  ros_image.data = image_vec;
  image_pub_.publish(ros_image);
}

void ImageSubscriberComponent::messageCallback(const zmqpp::message & message)
{
  perception_camera_app::ImageStamped proto;
  toProto(message, proto);
  imageCallback(convert(proto.image()), convert(proto.stamp()));
}

}  // namespace perception_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(perception_camera_driver::ImageSubscriberComponent)