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

#ifndef PERCEPTION_CAMERA_DRIVER__ENDPOINT_HPP_
#define PERCEPTION_CAMERA_DRIVER__ENDPOINT_HPP_

#include <string>

namespace perception_camera_driver
{
enum class Transport { kInproc = 0, kTcp = 1, kUdp = 2 };

std::string resolve(Transport transport, const std::string & address, std::uint32_t port);
std::string resolve(const std::string & socket_name);
}  // namespace perception_camera_driver

#endif  // PERCEPTION_CAMERA_DRIVER__ENDPOINT_HPP_
