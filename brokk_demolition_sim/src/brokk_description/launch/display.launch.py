#!/usr/bin/env python3
"""
Launch file for Brokk demolition robot visualization in RViz
Displays the robot model with joint state publisher GUI
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_share = FindPackageShare('brokk_description').find('brokk_description')

    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'brokk_robot.urdf.xacro')

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # RViz config file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'brokk_view.rviz')

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

    # Joint state publisher GUI node (for manual control in visualization mode)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
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
        joint_state_publisher_gui,
        rviz
    ])
