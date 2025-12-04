#!/usr/bin/env python3
"""
Simulation Only Launch - No hardware required

Launches:
- RViz visualization
- Joint controller for simulation
- Joy node for gamepad input
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

    # Include robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'display.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'gui': 'false'}.items()
    )

    # Joint controller node (controls simulation joints)
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
        robot_description_launch,
        joint_controller,
        joy_node
    ])
