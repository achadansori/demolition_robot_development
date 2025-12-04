#!/usr/bin/env python3
"""
Serial Bridge Node - Connect STM32 Transmitter to ROS 2

Reads joystick data from /joy topic and sends binary packet to STM32 transmitter
via serial port. STM32 transmitter then sends data via LoRa to receiver robot.

This allows control from:
- Real gamepad → ROS 2 → Serial → STM32 TX → LoRa → STM32 RX → Real robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import serial
import struct
import threading


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)

        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value

        # Connect to STM32 transmitter via serial
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=timeout)
            self.get_logger().info(f'Connected to STM32 transmitter on {port} @ {baudrate}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.serial_port = None

        # Subscribe to joystick input
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        # Publish serial TX/RX for debugging
        self.serial_tx_pub = self.create_publisher(String, '/robot/serial_tx', 10)
        self.serial_rx_pub = self.create_publisher(String, '/robot/serial_rx', 10)

        # Timer untuk read dari STM32 (optional - untuk monitoring)
        self.create_timer(0.1, self.read_serial_callback)

        self.get_logger().info('Serial Bridge Node started!')

    def joy_callback(self, msg):
        """
        Convert ROS Joy message to STM32 binary format

        Binary packet format (8 bytes):
        - Byte 0: joy_left_x (0-255)
        - Byte 1: joy_left_y (0-255)
        - Byte 2: joy_right_x (0-255)
        - Byte 3: joy_right_y (0-255)
        - Byte 4: r8 potentiometer (0-255)
        - Byte 5: r1 potentiometer (0-255)
        - Bytes 6-7: switches bit-packed (13 switches)
        """
        if self.serial_port is None or not self.serial_port.is_open:
            return

        # Convert joystick axes from [-1.0, 1.0] to [0, 255]
        # axes[0] = left stick X, axes[1] = left stick Y
        # axes[2] = right stick X, axes[3] = right stick Y
        joy_left_x = int((msg.axes[0] + 1.0) * 127.5) if len(msg.axes) > 0 else 127
        joy_left_y = int((msg.axes[1] + 1.0) * 127.5) if len(msg.axes) > 1 else 127
        joy_right_x = int((msg.axes[2] + 1.0) * 127.5) if len(msg.axes) > 2 else 127
        joy_right_y = int((msg.axes[3] + 1.0) * 127.5) if len(msg.axes) > 3 else 127

        # Clamp values to 0-255
        joy_left_x = max(0, min(255, joy_left_x))
        joy_left_y = max(0, min(255, joy_left_y))
        joy_right_x = max(0, min(255, joy_right_x))
        joy_right_y = max(0, min(255, joy_right_y))

        # Potentiometers (for now, set to 255 - can be mapped to triggers)
        r8 = 255
        r1 = 255

        # Pack switches from buttons (bit-packed into 2 bytes)
        switches = 0
        if len(msg.buttons) > 0:
            for i, button in enumerate(msg.buttons[:13]):  # Up to 13 switches
                if button:
                    switches |= (1 << i)

        switches_low = switches & 0xFF
        switches_high = (switches >> 8) & 0xFF

        # Pack binary data (8 bytes matching STM32 transmitter format)
        packet = struct.pack('BBBBBBBB',
                             joy_left_x, joy_left_y,
                             joy_right_x, joy_right_y,
                             r8, r1,
                             switches_low, switches_high)

        try:
            # Send to STM32 transmitter
            self.serial_port.write(packet)

            # Publish for debugging
            tx_msg = String()
            tx_msg.data = f"TX: JL:{joy_left_x},{joy_left_y} JR:{joy_right_x},{joy_right_y}"
            self.serial_tx_pub.publish(tx_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to send data: {e}')

    def read_serial_callback(self):
        """Read any data from STM32 for monitoring (optional)"""
        if self.serial_port is None or not self.serial_port.is_open:
            return

        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Publish received data for monitoring
                    rx_msg = String()
                    rx_msg.data = f"RX: {line}"
                    self.serial_rx_pub.publish(rx_msg)
                    self.get_logger().debug(f'Received from STM32: {line}')
        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
