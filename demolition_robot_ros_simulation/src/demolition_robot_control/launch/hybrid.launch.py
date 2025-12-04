#!/usr/bin/env python3
"""
Hybrid Mode Launch - Real robot + Simulation simultaneously

Launches:
- RViz visualization
- Serial bridge to STM32 transmitter (real robot)
- Joint controller (simulation)
- Joy node for gamepad input

Both real robot and simulation receive same joystick commands!
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    description_pkg = get_package_share_directory('demolition_robot_description')
    control_pkg = get_package_share_directory('demolition_robot_control')

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    baudrate = LaunchConfiguration('baudrate', default='115200')

    # Include robot description launch (visualization)
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'display.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'gui': 'false'}.items()
    )

    # Serial bridge node (real robot control)
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

    # Joint controller node (simulation control)
    joint_controller = Node(
        package='demolition_robot_control',
        executable='joint_controller',
        name='joint_controller',
        output='screen',
        parameters=[os.path.join(control_pkg, 'config', 'robot_params.yaml')]
    )

    # Joy node (gamepad input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='Serial port for STM32 transmitter'),
        DeclareLaunchArgument('baudrate', default_value='115200',
                              description='Serial baudrate'),
        robot_description_launch,
        serial_bridge,
        joint_controller,
        joy_node
    ])
