#!/usr/bin/env python3
"""
Simulation Launch File for Brokk Demolition Robot
Launches robot visualization with joint controller (no hardware connection)
Use this mode for testing control logic without real hardware
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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

    # Joint controller node (converts joystick to joint states)
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
        robot_state_publisher,
        joint_controller,
        joy_node,
        rviz
    ])
