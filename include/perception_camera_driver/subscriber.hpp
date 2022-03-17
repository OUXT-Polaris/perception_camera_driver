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

#ifndef PERCEPTION_CAMERA_DRIVER__SUBSCRIBER_HPP_
#define PERCEPTION_CAMERA_DRIVER__SUBSCRIBER_HPP_

#include <perception_camera_app.pb.h>

#include <memory>
#include <perception_camera_driver/endpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <zmqpp/zmqpp.hpp>

namespace perception_camera_direver
{
class Subscriber
{
public:
  explicit Subscriber(
    const std::string & topic, const std::string & endpoint = perception_camera_driver::resolve(
                                 perception_camera_driver::Transport::kTcp, "localhost", 8000));
  void startPoll();
  const std::string endpoint;

private:
  void poll();
  const zmqpp::context context_;
  zmqpp::socket socket_;
  std::thread thread_;
  zmqpp::poller poller_;
};
}  // namespace perception_camera_direver

#endif  // PERCEPTION_CAMERA_DRIVER__SUBSCRIBER_HPP_