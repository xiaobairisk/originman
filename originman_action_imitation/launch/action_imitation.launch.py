import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # 包含手势检测的launch文件
    hand_gesture_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hand_gesture_detection'),
                'launch/hand_gesture_fusion.launch.py'))
    )

    # Originman动作模仿节点
    action_imitation_node = Node(
        package='originman_action_imitation',
        executable='action_imitation_node',
        output='screen',
        parameters=[
            {"pluse": 1500},
            {"sub_topic": "/tros_perc_fusion"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 返回LaunchDescription
    return LaunchDescription([
        hand_gesture_detection,
        action_imitation_node
    ])