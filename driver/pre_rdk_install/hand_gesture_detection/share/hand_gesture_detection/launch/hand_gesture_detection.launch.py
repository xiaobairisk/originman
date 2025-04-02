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
        default_value='/hobot_hand_gesture_detection',
        description='websocket smart topic')
    is_dynamic_gesture_arg = DeclareLaunchArgument(
        'is_dynamic_gesture',
        default_value='false',
        description='true is dynamic gesture, false is static gesture')
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='log level')
    time_interval_sec_arg = DeclareLaunchArgument(
        'time_interval_sec',
        default_value='0.25',
        description='time interval for hand gesture voting')

    hand_lmk_det_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hand_lmk_detection'),
                'launch/hand_lmk_detection.launch.py')),
        launch_arguments={
            'smart_topic': '/hobot_hand_gesture_detection',
            'hand_lmk_pub_topic': '/hobot_hand_lmk_detection',
            'log_level': LaunchConfiguration('log_level')
        }.items()
    )

    # 手势识别算法
    hand_gesture_det_node = Node(
        package='hand_gesture_detection',
        executable='hand_gesture_detection',
        output='screen',
        parameters=[
            {"ai_msg_pub_topic_name": "/hobot_hand_gesture_detection"},
            {"ai_msg_sub_topic_name": "/hobot_hand_lmk_detection"},
            {"is_dynamic_gesture": LaunchConfiguration('is_dynamic_gesture')},
            {"time_interval_sec": LaunchConfiguration('time_interval_sec')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        web_smart_topic_arg,
        is_dynamic_gesture_arg,
        log_level_arg,
        time_interval_sec_arg,
        hand_lmk_det_node,
        hand_gesture_det_node
    ])
