#!/usr/bin/env python

# Copyright 1996-2019 Cyberbotics Ltd.
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

"""Launch Webots and the controller."""

import os

from pathlib import Path

import launch
import launch_ros.actions

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Webots
    package_dir = get_package_share_directory('webots_simple_arm')
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments={
            'executable': 'webots_robotic_arm_node',
            'world': os.path.join(package_dir, 'worlds', 'universal_robot_rviz.wbt'),
        }.items()
    )
    config_name = "ur_configs"
    config_share = get_package_share_directory(config_name)

    # Copy .rviz config file and update path ro URDF file.
    rviz_file = os.path.join(config_share,'rviz', 'webots_ur10.rviz')
    # Rviz node
    rviz = launch_ros.actions.Node(package='rviz2',
                                   executable='rviz2',
                                   arguments=['-d', rviz_file],
                                   output='screen')
    return launch.LaunchDescription([
        #rviz,
        webots
    ])
