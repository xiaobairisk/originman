# Copyright (c) 2024，D-Robotics.
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

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    web_smart_topic_arg = DeclareLaunchArgument(
        'smart_topic',
        default_value='/hobot_hand_lmk_detection',
        description='websocket smart topic')

    mono2d_body_det_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mono2d_body_detection'),
                'launch/mono2d_body_detection.launch.py')),
        launch_arguments={
            'smart_topic': LaunchConfiguration('smart_topic'),
            'mono2d_body_pub_topic': '/hobot_mono2d_body_detection'
        }.items()
    )

    # 人手关键点检测
    hand_lmk_pub_topic_arg = DeclareLaunchArgument(
        'hand_lmk_pub_topic',
        default_value='/hobot_hand_lmk_detection',
        description='hand landmark ai message publish topic')

    hand_lmk_det_node = Node(
        package='hand_lmk_detection',
        executable='hand_lmk_detection',
        output='screen',
        parameters=[
            {"ai_msg_pub_topic_name": LaunchConfiguration(
                'hand_lmk_pub_topic')},
            {"ai_msg_sub_topic_name": "/hobot_mono2d_body_detection"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        web_smart_topic_arg,
        mono2d_body_det_node,
        hand_lmk_pub_topic_arg,
        hand_lmk_det_node
    ])
