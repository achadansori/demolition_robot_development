#!/usr/bin/env python3
"""
Real Robot Launch - Connect to STM32 transmitter via serial

Launches:
- Serial bridge to STM32 transmitter
- Joy node for gamepad input
- NO simulation visualization (can be added if needed)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_pkg = get_package_share_directory('demolition_robot_control')

    # Launch arguments
    port = LaunchConfiguration('port', default='/dev/ttyACM0')  # STM32 USB CDC
    baudrate = LaunchConfiguration('baudrate', default='115200')

    # Serial bridge node
    serial_bridge = Node(
        package='demolition_robot_control',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'port': port,
            'baudrate': baudrate
        }]
    )

    # Joy node (gamepad input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Serial port for STM32 transmitter (USB CDC)'),
        DeclareLaunchArgument('baudrate', default_value='115200',
                              description='Serial baudrate'),
        serial_bridge,
        joy_node
    ])
