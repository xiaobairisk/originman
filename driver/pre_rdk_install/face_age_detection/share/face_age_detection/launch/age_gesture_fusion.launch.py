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
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    log_level = LaunchConfiguration('log_level')
    max_slide_window_size_cmd = DeclareLaunchArgument(
        "max_slide_window_size", default_value="30",
        description="max_slide_window_size")

    hand_lmk_det_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hand_lmk_detection'),
                'launch/hand_lmk_detection.launch.py')),
        launch_arguments={
            'smart_topic': '/tros_age_gesture_perc_fusion',
            'hand_lmk_pub_topic': '/hobot_hand_lmk_detection'
        }.items()
    )

    # 手势识别算法
    hand_gesture_det_node = Node(
        package='hand_gesture_detection',
        executable='hand_gesture_detection',
        output='screen',
        parameters=[
            {"ai_msg_pub_topic_name": "/hobot_hand_gesture_detection"},
            {"ai_msg_sub_topic_name": "/hobot_hand_lmk_detection"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )
    
    face_age_det_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("face_age_detection"),
                "launch/face_age_det_node.launch.py",
            )
        ),
        launch_arguments={
            "ai_msg_sub_topic_name": "/hobot_mono2d_body_detection",
            "ai_msg_pub_topic_name": "/hobot_face_age_detection",
            "is_shared_mem_sub": "1",
            "max_slide_window_size": LaunchConfiguration("max_slide_window_size"),
            "log_level": log_level,
        }.items(),
    )

    perc_fusion_node = Node(
        package='tros_ai_fusion',
        executable='tros_ai_fusion',
        name='tros_perc_fusion_node',
        output='screen',
        parameters=[
                    {'topic_name_base': '/hobot_hand_gesture_detection'},
                    {'topic_names_fusion': ['/hobot_face_age_detection']},
                    {'pub_fusion_topic_name': '/tros_age_gesture_perc_fusion'},
        ],
       arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription(
        [
            hand_lmk_det_node,
            hand_gesture_det_node,
            face_age_det_node,
            perc_fusion_node
        ]
    )
