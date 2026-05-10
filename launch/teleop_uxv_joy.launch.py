"""
Joypad launch file for UXV control.
Sample configuration for DS4 joypad.

May 8, 2026
"""

# Copyright 2026 dotX Automation s.r.l.
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
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('teleop_uxv_joy'),
        'config',
        'teleop_uxv_joy.yaml'
    )

    # Declare launch arguments
    ns = LaunchConfiguration('namespace')
    cf = LaunchConfiguration('cf')
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )
    cf_launch_arg = DeclareLaunchArgument(
        'cf',
        default_value=config
    )
    ld.add_action(ns_launch_arg)
    ld.add_action(cf_launch_arg)

    container = ComposableNodeContainer(
        name='joy_container',
        namespace='',
        package='dua_app_management',
        executable='dua_component_container_mt',
        emulate_tty=True,
        output='both',
        log_cmd=True,
        composable_node_descriptions=[
            ComposableNode(
                package='joy',
                plugin='joy::GameController',
                name='game_controller',
                namespace=ns,
                parameters=[
                    {
                        'device_id': 0,
                        'deadzone': 0.1,
                        'autorepeat_rate': 20.0,
                        'sticky_buttons': False,
                        'coalesce_interval_ms': 1
                    }
                ],
                remappings=[
                    ('joy', 'joy'),
                    ('joy/set_feedback', 'joy/set_feedback')
                ]
            ),
            ComposableNode(
                package='teleop_uxv_joy',
                plugin='teleop_uxv_joy::TeleopUXVJoy',
                name='teleop_uxv_joy',
                namespace=ns,
                parameters=[cf],
                remappings=[
                    ('/joy', 'joy'),
                    ('/cmd_uxv', '/cmd_uxv')
                ]
            )
        ]
    )
    ld.add_action(container)

    return ld
