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

#include <perception_camera_driver/subscriber.hpp>

namespace perception_camera_direver
{
Subscriber::Subscriber(
  std::function<void(const zmqpp::message & message)> callback, const rclcpp::Logger & logger,
  const std::string & endpoint)
: callback_(callback),
  logger_(logger),
  endpoint(endpoint),
  context_(zmqpp::context()),
  socket_(context_, zmqpp::socket_type::subscribe)
{
  RCLCPP_INFO_STREAM(logger_, "start connecting to endpoint : " << endpoint);
  socket_.connect(endpoint);
  socket_.subscribe("");
  poller_.add(socket_);
  RCLCPP_INFO_STREAM(logger_, "start polling : " << endpoint);
  thread_ = std::thread(&Subscriber::startPoll, this);
}

Subscriber::~Subscriber() { thread_.join(); }

void Subscriber::startPoll()
{
  while (rclcpp::ok()) {
    poll();
  }
  RCLCPP_INFO_STREAM(logger_, "start disconnecting : " << endpoint);
  socket_.disconnect(endpoint);
}

void Subscriber::poll()
{
  constexpr long timeout_ms = 1L;
  poller_.poll(timeout_ms);
  if (poller_.has_input(socket_)) {
    std::lock_guard<std::mutex> lock(mutex_);
    zmqpp::message message;
    socket_.receive(message);
    callback_(message);
  }
}
}  // namespace perception_camera_direver