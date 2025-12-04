#!/usr/bin/env python3
"""
Hardware Launch File for Brokk Demolition Robot
Launches serial bridge to communicate with STM32 transmitter
Use this mode to control real hardware via LoRa
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for STM32 transmitter (USB CDC)'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate'
    )

    # Get launch configurations
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')

    # Serial bridge node (sends commands to STM32 transmitter)
    serial_bridge = Node(
        package='brokk_control',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'port': port,
            'baudrate': baudrate,
            'timeout': 1.0
        }]
    )

    # Joy node (for joystick input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.1,
            'autorepeat_rate': 50.0
        }]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        serial_bridge,
        joy_node
    ])
