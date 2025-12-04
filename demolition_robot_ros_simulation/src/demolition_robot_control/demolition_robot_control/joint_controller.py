#!/usr/bin/env python3
"""
Joint Controller Node - Control robot joints in simulation

Subscribes to /joy topic and publishes joint states for simulation visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header
import math


class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Joint states
        self.joint_positions = {
            'boom_joint': 0.0,
            'stick_joint': 0.0,
            'bucket_joint': 0.0,
            'slew_joint': 0.0,
            'outrigger_left_joint': 0.0,
            'outrigger_right_joint': 0.0,
            'track_left_joint': 0.0,
            'track_right_joint': 0.0
        }

        # Subscribe to joystick
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        # Publish joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timer untuk publish joint states
        self.create_timer(0.05, self.publish_joint_states)  # 20Hz

        # Control parameters
        self.deadzone = 0.1
        self.speed_scale = 0.02  # rad/update for revolute joints
        self.linear_scale = 0.01  # m/update for prismatic joints

        self.get_logger().info('Joint Controller Node started!')

    def joy_callback(self, msg):
        """
        Update joint positions based on joystick input

        Mapping (default):
        - Left Stick Y: Bucket (Cylinder 3)
        - Right Stick Y: Stick (Cylinder 2)
        - Right Stick X: Boom (Cylinder 1)
        - Left Stick X: Slew rotation
        - D-Pad Up/Down: Outrigger left
        - D-Pad Left/Right: Outrigger right
        - Triggers: Tracks
        """

        # Bucket control (Cylinder 3) - Left Stick Y
        if len(msg.axes) > 1:
            axis_val = msg.axes[1]
            if abs(axis_val) > self.deadzone:
                self.joint_positions['bucket_joint'] += axis_val * self.speed_scale

        # Stick control (Cylinder 2) - Right Stick Y
        if len(msg.axes) > 3:
            axis_val = msg.axes[3]
            if abs(axis_val) > self.deadzone:
                self.joint_positions['stick_joint'] += axis_val * self.speed_scale

        # Boom control (Cylinder 1) - Right Stick X
        if len(msg.axes) > 2:
            axis_val = msg.axes[2]
            if abs(axis_val) > self.deadzone:
                self.joint_positions['boom_joint'] += axis_val * self.speed_scale

        # Slew rotation - Left Stick X
        if len(msg.axes) > 0:
            axis_val = msg.axes[0]
            if abs(axis_val) > self.deadzone:
                self.joint_positions['slew_joint'] += axis_val * self.speed_scale

        # Outrigger left - D-Pad Up/Down (axes 7 on some controllers)
        if len(msg.axes) > 7:
            axis_val = msg.axes[7]
            if abs(axis_val) > self.deadzone:
                self.joint_positions['outrigger_left_joint'] -= axis_val * self.linear_scale

        # Outrigger right - D-Pad Left/Right (axes 6 on some controllers)
        if len(msg.axes) > 6:
            axis_val = msg.axes[6]
            if abs(axis_val) > self.deadzone:
                self.joint_positions['outrigger_right_joint'] -= axis_val * self.linear_scale

        # Tracks - Triggers (axes 4, 5)
        if len(msg.axes) > 5:
            left_trigger = msg.axes[4] if len(msg.axes) > 4 else 0.0
            right_trigger = msg.axes[5] if len(msg.axes) > 5 else 0.0

            # Track rotation (continuous joints)
            self.joint_positions['track_left_joint'] += left_trigger * self.speed_scale * 2
            self.joint_positions['track_right_joint'] += right_trigger * self.speed_scale * 2

        # Clamp joint limits
        self.joint_positions['boom_joint'] = max(-1.57, min(1.57, self.joint_positions['boom_joint']))
        self.joint_positions['stick_joint'] = max(-2.0, min(2.0, self.joint_positions['stick_joint']))
        self.joint_positions['bucket_joint'] = max(-2.5, min(1.0, self.joint_positions['bucket_joint']))
        self.joint_positions['outrigger_left_joint'] = max(-0.8, min(0.0, self.joint_positions['outrigger_left_joint']))
        self.joint_positions['outrigger_right_joint'] = max(-0.8, min(0.0, self.joint_positions['outrigger_right_joint']))

    def publish_joint_states(self):
        """Publish current joint states for visualization"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Add all joints
        for joint_name, position in self.joint_positions.items():
            msg.name.append(joint_name)
            msg.position.append(position)
            msg.velocity.append(0.0)
            msg.effort.append(0.0)

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
