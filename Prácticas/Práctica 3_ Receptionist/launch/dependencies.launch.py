# Copyright 2021 Intelligent Robotics Lab
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
    # darknet_launch_cmd = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'darknet_ros', 'darknet_ros.launch.py'],
    # )

    # Node for navigation execution
    navigation_launch_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'ir_robots', 'navigation.launch.py'],
    )

    # Node for dialog execution
    dialog_launch_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'gb_dialog', 'gb_dialog_services_soundplay.launch.py'],
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

    ld = LaunchDescription()
    # ld.add_action(darknet_launch_cmd)
    ld.add_action(navigation_launch_cmd)
    ld.add_action(dialog_launch_cmd)
    ld.add_action(darknet_detection_asr)
    ld.add_action(depth_detection_asr)

    return ld
