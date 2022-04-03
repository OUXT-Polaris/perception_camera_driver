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

from ipaddress import ip_address
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions.launch_configuration import LaunchConfiguration


import os
import yaml


def generate_launch_description():
    ip_address = LaunchConfiguration("ip_address", default="localhost")
    port = LaunchConfiguration("port", default=8000)
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
            Node(
                package="perception_camera_driver",
                executable="perception_camera_driver_node",
                parameters=[{"ip_address": ip_address, "port": port}],
            ),
        ]
    )
    return description
