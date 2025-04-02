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
from launch.actions import GroupAction

def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='warn')
    perc_sub_topic_arg = DeclareLaunchArgument(
        'lowpass_perc_sub_topic',
        default_value='/tros_perc_fusion',
        description='lowpass subscribed msg topic')
    perc_pub_topic_arg = DeclareLaunchArgument(
        'lowpass_perc_pub_topic',
        default_value='/tros_perc_render',
        description='lowpass published msg topic')
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(get_package_share_directory('tros_lowpass_filter'), 'params', 'low_pass.json'),
        description='Full path to the ROS2 parameters file to use for all low-pass filters')

    tros_lowpass_filter_node = Node(
        package='tros_lowpass_filter',
        executable='tros_lowpass_filter',
        name='tros_lowpass_filter_node',
        parameters=[
            {'perc_sub_topic': LaunchConfiguration('lowpass_perc_sub_topic')},
            {'perc_pub_topic': LaunchConfiguration('lowpass_perc_pub_topic')},
            {'config_file': LaunchConfiguration('config_file')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    ld = LaunchDescription()
    ld.add_action(log_level_arg)
    ld.add_action(perc_sub_topic_arg)
    ld.add_action(perc_pub_topic_arg)
    ld.add_action(declare_config_file_arg)
    ld.add_action(tros_lowpass_filter_node)
    return ld
