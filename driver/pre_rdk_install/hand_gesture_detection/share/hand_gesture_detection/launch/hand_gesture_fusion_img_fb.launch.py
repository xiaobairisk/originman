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
    max_slide_window_size_cmd = DeclareLaunchArgument(
        "max_slide_window_size", default_value="30",
        description="max_slide_window_size")
    time_interval_sec_arg = DeclareLaunchArgument(
        'time_interval_sec',
        default_value='0.25',
        description='time interval for hand gesture voting')
    pub_fusion_topic_name_arg = DeclareLaunchArgument(
        'pub_fusion_topic_name',
        default_value='/tros_perc_fusion',
        description='tros fusion ai message publish topic')
    
    # 人手关键点检测
    hand_lmk_pub_topic_arg = DeclareLaunchArgument(
        'hand_lmk_pub_topic',
        default_value='/hobot_hand_lmk_detection',
        description='hand landmark ai message publish topic')

    static_gesture_task_num_arg = DeclareLaunchArgument(
        'static_gesture_task_num',
        default_value='4',
        description='static gesture task num')
    dynamic_gesture_task_num_arg = DeclareLaunchArgument(
        'dynamic_gesture_task_num',
        default_value='4',
        description='dynamic gesture task num')
    
    web_smart_topic_arg = DeclareLaunchArgument(
        'smart_topic',
        default_value='/hobot_hand_lmk_detection',
        description='websocket smart topic')

    # local image publish
    feedback_picture_arg = DeclareLaunchArgument(
        'picture',
        default_value='./config/960x544.nv12',
        description='feedback picture')
    fb_image_format_arg = DeclareLaunchArgument(
        'picture_format',
        default_value='nv12',
        description='feedback picture format')
    
    fb_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_image_publisher'),
                'launch/hobot_image_publisher.launch.py')),
        launch_arguments={
            'publish_image_source': LaunchConfiguration('picture'),
            'publish_image_format': LaunchConfiguration('picture_format'),
            'publish_source_image_w': '1920',
            'publish_source_image_h': '1080',
            'publish_output_image_w': '960',
            'publish_output_image_h': '544',
            'publish_message_topic_name': '/img_png',
            'publish_fps': '30',
            'publish_is_shared_mem': 'False',
            'publish_is_compressed_img_pub': 'True'
        }.items()
    )

    # png->nv12
    decoder_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_in_format': 'jpeg',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/hbmem_img',
            'codec_sub_topic': '/img_png'
        }.items()
    )

    # web
    web_smart_topic_arg = DeclareLaunchArgument(
        'smart_topic',
        default_value='/tros_perc_render',
        description='websocket smart topic')
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/img_png',
            'websocket_smart_topic': LaunchConfiguration('smart_topic')
        }.items()
    )

    # mono2d body detection
    mono2d_body_pub_topic_arg = DeclareLaunchArgument(
        'mono2d_body_pub_topic',
        default_value='/hobot_mono2d_body_detection',
        description='mono2d body ai message publish topic')
    mono2d_body_det_node = Node(
        package='mono2d_body_detection',
        executable='mono2d_body_detection',
        output='screen',
        parameters=[
            {"ai_msg_pub_topic_name": LaunchConfiguration(
                'mono2d_body_pub_topic')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    shared_mem_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('hobot_shm'),
                        'launch/hobot_shm.launch.py'))
            )

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

    # 静态手势识别算法
    hand_static_gesture_det_node = Node(
        package='hand_gesture_detection',
        executable='hand_gesture_detection',
        name='hand_static_gesture_detection',
        output='screen',
        parameters=[
            {"ai_msg_pub_topic_name": "/hobot_hand_static_gesture_detection"},
            {"ai_msg_sub_topic_name": "/hobot_hand_lmk_detection"},
            {"is_dynamic_gesture": False},
            {"task_num": LaunchConfiguration('static_gesture_task_num')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 动态手势识别算法
    hand_dynamic_gesture_det_node = Node(
        package='hand_gesture_detection',
        executable='hand_gesture_detection',
        name='hand_dynamic_gesture_detection',
        output='screen',
        parameters=[
            {"ai_msg_pub_topic_name": "/hobot_hand_dynamic_gesture_detection"},
            {"ai_msg_sub_topic_name": "/hobot_hand_lmk_detection"},
            {"is_dynamic_gesture": True},
            {"time_interval_sec": LaunchConfiguration('time_interval_sec')},
            {"threshold": 0.5},
            {"task_num": LaunchConfiguration('static_gesture_task_num')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
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
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
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
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    perc_fusion_node = Node(
        package='tros_ai_fusion',
        executable='tros_ai_fusion',
        name='tros_perc_fusion_node',
        output='screen',
        parameters=[
                    {'topic_name_base': '/hobot_hand_static_gesture_detection'},
                    {'topic_names_fusion': ['/hobot_face_age_detection', 'hobot_face_landmarks_detection', '/hobot_hand_dynamic_gesture_detection']},
                    {'pub_fusion_topic_name': LaunchConfiguration("pub_fusion_topic_name")},
                    {'filter_duplicated_roi': True}
        ],
       arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")]
    )

    tros_lowpass_filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tros_lowpass_filter'),
                         'launch/low_pass.launch.py'))
    )

    group_action_face_lmk = GroupAction([
        face_landmarks_det_node,
        hand_static_gesture_det_node,
    ])

    group_action_batch = GroupAction([
        mono2d_body_pub_topic_arg,
        feedback_picture_arg,
        fb_image_format_arg,
        web_smart_topic_arg,
        hand_lmk_pub_topic_arg,
        
        shared_mem_node,
        fb_node,
        decoder_node,
        web_node,
        mono2d_body_det_node,
        hand_lmk_det_node,
        hand_dynamic_gesture_det_node,
        face_age_det_node,
        perc_fusion_node,
        tros_lowpass_filter_node
        ])

    ld = LaunchDescription()
    ld.add_action(max_slide_window_size_cmd)
    ld.add_action(time_interval_sec_arg)
    ld.add_action(log_level_arg)
    ld.add_action(pub_fusion_topic_name_arg)
    ld.add_action(group_action_face_lmk)
    ld.add_action(group_action_batch)
    return ld
