#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    tts_node = Node(
        package='originman_llm_chat',
        executable='text_to_speech_node',
        name='text_to_speech_node',
        output='screen',
        emulate_tty=True,
    )

    llm_node = Node(
        package='originman_llm_chat',
        executable='llm_chat_node',
        name='llm_chat_node',
        output='screen',
        emulate_tty=True,
    )

    asr_node = Node(
        package='originman_llm_chat',
        executable='asr_node',
        name='asr_node',
        output='screen',
        emulate_tty=True,
    )

    ld.add_action(tts_node)
    ld.add_action(llm_node)
    ld.add_action(asr_node)

    return ld

if __name__ == '__main__':
    generate_launch_description()