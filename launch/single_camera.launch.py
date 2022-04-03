# Copyright (c) 2020 OUXT Polaris
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():
    ip_address = LaunchConfiguration("ip_address", default="localhost")
    port = LaunchConfiguration("port", default=8000)
    frame_id = LaunchConfiguration("frame_id", default="frame_id")
    description = LaunchDescription(
        [
            DeclareLaunchArgument(
                "ip_address",
                default_value=ip_address,
                description="Ip address of perception camera.",
            ),
            DeclareLaunchArgument(
                "port", default_value=port, description="Port of perception camera."
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value=frame_id,
                description="Frame ID of perception camera.",
            ),
            Node(
                package="perception_camera_driver",
                executable="perception_camera_driver_node",
                parameters=[
                    {"ip_address": ip_address, "port": port, "frame_id": frame_id}
                ],
            ),
        ]
    )
    return description
