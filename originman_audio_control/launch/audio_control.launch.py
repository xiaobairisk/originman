#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    asr_node = Node(
        package='originman_llm_chat',
        executable='asr_node',
        name='asr_node',
        output='screen',
        emulate_tty=True,
    )

    audio_control_node = Node(
        package='originman_audio_control',
        executable='audio_control_node',
        name='audio_control_node',
        output='screen',
        emulate_tty=True,
    )

    ld.add_action(asr_node)
    ld.add_action(audio_control_node)

    return ld

if __name__ == '__main__':
    generate_launch_description()