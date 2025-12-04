#!/usr/bin/env python3
"""
Hybrid Launch File for Brokk Demolition Robot
Combines simulation visualization with real hardware control
- Serial bridge sends commands to STM32 transmitter (real hardware via LoRa)
- Joint controller updates RViz visualization
- Use this mode to control real hardware while seeing robot visualization
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


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

    # Get package directories
    desc_pkg_share = FindPackageShare('brokk_description').find('brokk_description')

    # URDF file path
    urdf_file = os.path.join(desc_pkg_share, 'urdf', 'brokk_robot.urdf.xacro')

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # RViz config file
    rviz_config_file = os.path.join(desc_pkg_share, 'rviz', 'brokk_view.rviz')

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

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

    # Joint controller node (updates visualization)
    joint_controller = Node(
        package='brokk_control',
        executable='joint_controller',
        name='joint_controller',
        output='screen'
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

    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        robot_state_publisher,
        serial_bridge,
        joint_controller,
        joy_node,
        rviz
    ])
