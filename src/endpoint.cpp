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

#include <perception_camera_driver/endpoint.hpp>
#include <stdexcept>

namespace perception_camera_driver
{
std::string resolve(Transport transport, const std::string & address, std::uint32_t port)
{
  std::string endpoint;
  switch (transport) {
    case Transport::kInproc:
      throw std::runtime_error("inproc can be used in only localhost.");
      break;
    case Transport::kTcp:
      endpoint = "tcp://";
      break;
    case Transport::kUdp:
      endpoint = "udp://";
      break;
  }
  endpoint = endpoint + address + ":" + std::to_string(port);
  return endpoint;
}

std::string resolve(const std::string & socket_name) { return "inproc://" + socket_name; }
}  // namespace perception_camera_driver