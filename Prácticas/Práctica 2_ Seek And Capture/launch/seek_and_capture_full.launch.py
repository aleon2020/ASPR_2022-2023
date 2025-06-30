# Copyright 2023 Intelligent Robotics Lab
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

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Node for darknet execution
    darknet_launch_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'darknet_ros', 'darknet_ros.launch.py'],
    )

    # Node for darknet detection
    darknet_detection_asr = Node(
        package='perception_asr',
        executable='darknet_detection',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('input_bbxs_detection', '/darknet_ros/bounding_boxes'),
            ('output_detection_2d', '/detection_2d_array')
        ]
    )

    # Node for depth detection
    depth_detection_asr = Node(
        package='perception_asr',
        executable='detection_2d_to_3d_depth',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('input_depth', '/camera/depth/image_raw'),
            ('camera_info', '/camera/depth/camera_info'),
            ('input_detection_2d', '/detection_2d_array')
            ])

    # Node for debug info
    debug_cmd = Node(
        package='debug_forocoches',
        executable='debug_interface',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ('output_led_1', '/commands/led1'),
            ('output_led_2', '/commands/led2'),
            ('output_sound', '/commands/sound')
        ]
    )

    # Node for seek_and_capture_cmd
    seek_and_capture_cmd = Node(
        package='seek_and_capture_forocoches',
        executable='sac_forocoches',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('output_vel', '/cmd_vel'),
            ('output_detection_3d', '/output_detection_3d'),
            ('input_wheel_drop', '/events/wheel_drop'),
            ('input_battery_state', '/sensors/battery_state')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(darknet_launch_cmd)
    ld.add_action(darknet_detection_asr)
    ld.add_action(depth_detection_asr)
    ld.add_action(debug_cmd)
    ld.add_action(seek_and_capture_cmd)

    return ld
