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
    mono2d_body_det_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mono2d_body_detection"),
                "launch/mono2d_body_detection.launch.py",
            )
        ),
        launch_arguments={
            "mono2d_body_pub_topic": "/hobot_mono2d_body_detection",
            "smart_topic": "/hobot_face_landmarks_detection",
            "log_level": "error",
        }.items(),
    )

    face_landmarks_det_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("face_landmarks_detection"),
                "launch/face_landmarks_det_node.launch.py",
            )
        ),
        launch_arguments={
            "ai_msg_sub_topic_name": "/hobot_mono2d_body_detection",
            "ai_msg_pub_topic_name": "/hobot_face_landmarks_detection",
            "is_shared_mem_sub": "1",
            "log_level": "info",
        }.items(),
    )

    return LaunchDescription(
        [
            mono2d_body_det_node,
            face_landmarks_det_node,
        ]
    )
