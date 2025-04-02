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


def declare_configurable_parameters(parameters):
    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param["default_value"],
            description=param["description"],
        )
        for param in parameters
    ]


def set_configurable_parameters(parameters):
    return dict(
        [(param["name"], LaunchConfiguration(param["name"])) for param in parameters]
    )


def generate_launch_description():
    model_file_name = os.path.join(
        get_package_share_directory("face_age_detection"),
        "config",
        "faceAge.hbm",
    )

    feed_image_path = os.path.join(
        get_package_share_directory("face_age_detection"),
        "config",
        "image.png",
    )

    node_params = [
        {
            "name": "feed_type",
            "default_value": "0",
            "description": "feed_type",
        },
        {
            "name": "feed_image_path",
            "default_value": feed_image_path,
            "description": "feed_image_path",
        },
        {
            "name": "roi_xyxy",
            "default_value": "398,602,476,685,377,203,476,299,251,242,328,337,547,243,666,355,389,304,489,404,283,343,364,450",
            "description": "roi_xyxy",
        },
        {
            "name": "is_sync_mode",
            "default_value": "0",
            "description": "is_sync_mode",
        },
        {
            "name": "model_file_name",
            "default_value": model_file_name,
            "description": "model_file_name",
        },
        {
            "name": "is_shared_mem_sub",
            "default_value": "1",
            "description": "is_shared_mem_sub",
        },
        {
            "name": "dump_render_img",
            "default_value": "0",
            "description": "dump_render_img",
        },
        {
            "name": "ai_msg_pub_topic_name",
            "default_value": "/face_age_detection",
            "description": "ai_msg_pub_topic_name",
        },
        {
            "name": "max_slide_window_size",
            "default_value": "30",
            "description": "max_slide_window_size",
        },
        {"name": "log_level", "default_value": "info", "description": "log_level"},
    ]

    launch = declare_configurable_parameters(node_params)
    launch.append(
        Node(
            package="face_age_detection",
            executable="face_age_detection",
            output="screen",
            parameters=[set_configurable_parameters(node_params)],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        )
    )

    return LaunchDescription(launch)
