#!/usr/bin/env python3
"""
Joint Controller Node for Brokk Demolition Robot
Converts joystick input to joint states for RViz visualization
Implements the same mode-based control as the real hardware
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header
import math


class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Subscribe to joystick
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Publish joint states
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Joint state storage
        self.joint_state = JointState()
        self.joint_state.name = [
            'boom_joint',              # Cylinder 1
            'stick_joint',             # Cylinder 2
            'bucket_joint',            # Cylinder 3
            'tool_rotation_joint',     # Cylinder 4
            'tool_jaw_1_joint',        # Tool 1
            'tool_jaw_2_joint',        # Tool 2
            'outrigger_left_joint',    # Outrigger left
            'outrigger_right_joint',   # Outrigger right
            'track_left_joint',        # Track left
            'track_right_joint',       # Track right
            'slew_joint'               # Slew rotation
        ]

        # Initialize joint positions
        self.joint_state.position = [
            0.5,   # boom_joint (raised slightly)
            -0.5,  # stick_joint (bent)
            0.0,   # bucket_joint
            0.0,   # tool_rotation_joint
            0.0,   # tool_jaw_1_joint
            0.0,   # tool_jaw_2_joint
            0.0,   # outrigger_left_joint (retracted)
            0.0,   # outrigger_right_joint (retracted)
            0.0,   # track_left_joint
            0.0,   # track_right_joint
            0.0    # slew_joint
        ]

        # Initialize velocities
        self.joint_state.velocity = [0.0] * len(self.joint_state.name)
        self.joint_state.effort = [0.0] * len(self.joint_state.name)

        # Control parameters
        self.deadzone = 0.1  # Joystick deadzone
        self.update_rate = 50.0  # Hz
        self.dt = 1.0 / self.update_rate

        # Create timer for publishing joint states
        self.timer = self.create_timer(self.dt, self.publish_joint_states)

        # Store last joystick state
        self.last_axes = [0.0, 0.0, 0.0, 0.0]
        self.last_buttons = [0] * 8

        self.get_logger().info('Joint Controller Node started')

    def apply_deadzone(self, value):
        """Apply deadzone to joystick value"""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def joy_callback(self, msg):
        """Process joystick message"""
        if len(msg.axes) < 4:
            return

        # Store axes and buttons
        self.last_axes = msg.axes[:4]
        if len(msg.buttons) >= 8:
            self.last_buttons = msg.buttons[:8]

    def publish_joint_states(self):
        """Update and publish joint states based on joystick input"""

        # Apply deadzone to axes
        left_x = self.apply_deadzone(self.last_axes[0])
        left_y = self.apply_deadzone(self.last_axes[1])
        right_x = self.apply_deadzone(self.last_axes[2])
        right_y = self.apply_deadzone(self.last_axes[3])

        # Get mode switches
        s5_1 = self.last_buttons[4] if len(self.last_buttons) > 4 else 0
        s5_2 = self.last_buttons[5] if len(self.last_buttons) > 5 else 0

        # Determine mode
        mode_upper = (s5_1 == 0 and s5_2 == 0)
        mode_lower = (s5_1 == 1 and s5_2 == 0)

        # Joint indices
        BOOM = 0
        STICK = 1
        BUCKET = 2
        TOOL_ROT = 3
        JAW_1 = 4
        JAW_2 = 5
        OUT_LEFT = 6
        OUT_RIGHT = 7
        TRACK_LEFT = 8
        TRACK_RIGHT = 9
        SLEW = 10

        # Velocity scaling factors
        joint_vel_scale = 0.3  # rad/s for revolute joints
        prismatic_vel_scale = 0.1  # m/s for prismatic joints
        track_vel_scale = 2.0  # rad/s for continuous track joints

        # Initialize velocities to zero
        velocities = [0.0] * len(self.joint_state.name)

        if mode_upper:
            # ===== MODE UPPER - EXCAVATOR CONTROLS =====

            # Left Y: Cylinder 3 (Bucket)
            velocities[BUCKET] = -left_y * joint_vel_scale

            # Left X: Slew rotation
            velocities[SLEW] = left_x * joint_vel_scale

            # Right Y: Cylinder 2 (Stick)
            velocities[STICK] = -right_y * joint_vel_scale

            # Right X: Cylinder 1 (Boom)
            velocities[BOOM] = right_x * joint_vel_scale

        elif mode_lower:
            # ===== MODE LOWER - MOBILITY CONTROLS =====

            # Left Y: Track Left
            velocities[TRACK_LEFT] = left_y * track_vel_scale

            # Left X: Outrigger Left
            velocities[OUT_LEFT] = left_x * prismatic_vel_scale

            # Right Y: Track Right
            velocities[TRACK_RIGHT] = right_y * track_vel_scale

            # Right X: Outrigger Right
            velocities[OUT_RIGHT] = right_x * prismatic_vel_scale

        # Update joint positions based on velocities
        for i in range(len(self.joint_state.position)):
            self.joint_state.position[i] += velocities[i] * self.dt

        # Apply joint limits
        limits = [
            (-0.5, 1.57),      # boom_joint
            (-2.5, 0.5),       # stick_joint
            (-1.57, 1.57),     # bucket_joint
            (-3.14, 3.14),     # tool_rotation_joint
            (-0.5, 0.8),       # tool_jaw_1_joint
            (-0.8, 0.5),       # tool_jaw_2_joint
            (0.0, 0.5),        # outrigger_left_joint
            (0.0, 0.5),        # outrigger_right_joint
            (-100.0, 100.0),   # track_left_joint (continuous, no real limit)
            (-100.0, 100.0),   # track_right_joint (continuous, no real limit)
            (-100.0, 100.0)    # slew_joint (continuous, no real limit)
        ]

        for i, (min_limit, max_limit) in enumerate(limits):
            if self.joint_state.position[i] < min_limit:
                self.joint_state.position[i] = min_limit
            elif self.joint_state.position[i] > max_limit:
                self.joint_state.position[i] = max_limit

        # Update velocities
        self.joint_state.velocity = velocities

        # Publish joint state
        self.joint_state.header = Header()
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.joint_state)


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
