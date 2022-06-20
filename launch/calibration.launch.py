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

from ast import arguments
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node


import os


def generate_launch_description():
    perception_camera_driver_launch_file_dir = os.path.join(
        get_package_share_directory("perception_camera_driver"), "launch"
    )
    ip_address = LaunchConfiguration("ip_address", default="localhost")
    port = LaunchConfiguration("port", default=8000)
    frame_id = LaunchConfiguration("frame_id", default="frame_id")
    calibration_arguments = LaunchConfiguration(
        "calibration_arguments",
        default="--size=9x6 --square=0.024 --approximate=0.1 --no-service-check",
    )
    return LaunchDescription(
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
            DeclareLaunchArgument(
                "calibration_arguments",
                default_value=calibration_arguments,
                description="Arguments of calibrator.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        perception_camera_driver_launch_file_dir,
                        "/single_camera.launch.py",
                    ]
                ),
                launch_arguments={
                    "ip_address": ip_address,
                    "port": port,
                    "frame_id": frame_id,
                    "image_topic_name": "image",
                }.items(),
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "camera_calibration",
                    "cameracalibrator",
                    calibration_arguments,
                ],
                output="screen",
                shell=True,
            ),
        ]
    )
