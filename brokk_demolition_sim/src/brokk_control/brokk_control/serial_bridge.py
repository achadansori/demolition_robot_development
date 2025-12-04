#!/usr/bin/env python3
"""
Serial Bridge Node for Brokk Demolition Robot
Communicates with STM32 transmitter via USB CDC (ttyACM0)
Subscribes to /joy topic and sends binary packets to STM32
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import struct
import time


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)

        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value

        # Binary packet format
        self.HEADER_1 = 0xAA
        self.HEADER_2 = 0x55

        # Connect to STM32 transmitter via serial
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info(f'Serial port opened: {port} @ {baudrate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.serial_port = None

        # Subscribe to joystick
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Store last joystick state
        self.last_joy = {
            'left_x': 127,
            'left_y': 127,
            'right_x': 127,
            'right_y': 127,
            's5_1': 0,
            's5_2': 0,
            's6_1': 0,
            's6_2': 0
        }

        self.get_logger().info('Serial Bridge Node started')
        self.get_logger().info('Waiting for joystick input on /joy topic...')

    def joy_to_byte(self, value):
        """Convert joystick axis from [-1.0, 1.0] to [0, 255]"""
        return int((value + 1.0) * 127.5)

    def button_to_byte(self, value):
        """Convert button state (0 or 1) to byte"""
        return 1 if value > 0 else 0

    def calculate_checksum(self, data):
        """Calculate XOR checksum of data bytes"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def joy_callback(self, msg):
        """Process joystick message and send to STM32"""

        # Check if we have enough axes and buttons
        if len(msg.axes) < 4 or len(msg.buttons) < 8:
            self.get_logger().warn('Insufficient joystick axes or buttons')
            return

        # Map joystick axes (assuming standard gamepad layout)
        # Axes: 0=left_x, 1=left_y, 2=right_x, 3=right_y
        joy_left_x = self.joy_to_byte(msg.axes[0])
        joy_left_y = self.joy_to_byte(msg.axes[1])
        joy_right_x = self.joy_to_byte(msg.axes[2])
        joy_right_y = self.joy_to_byte(msg.axes[3])

        # Map buttons to switches
        # Buttons 4-7 map to s5_1, s5_2, s6_1, s6_2
        s5_1 = self.button_to_byte(msg.buttons[4]) if len(msg.buttons) > 4 else 0
        s5_2 = self.button_to_byte(msg.buttons[5]) if len(msg.buttons) > 5 else 0
        s6_1 = self.button_to_byte(msg.buttons[6]) if len(msg.buttons) > 6 else 0
        s6_2 = self.button_to_byte(msg.buttons[7]) if len(msg.buttons) > 7 else 0

        # Update state
        self.last_joy = {
            'left_x': joy_left_x,
            'left_y': joy_left_y,
            'right_x': joy_right_x,
            'right_y': joy_right_y,
            's5_1': s5_1,
            's5_2': s5_2,
            's6_1': s6_1,
            's6_2': s6_2
        }

        # Pack data into binary format
        data = struct.pack('8B',
            joy_left_x,
            joy_left_y,
            joy_right_x,
            joy_right_y,
            s5_1,
            s5_2,
            s6_1,
            s6_2
        )

        # Calculate checksum
        checksum = self.calculate_checksum(data)

        # Create complete packet: HEADER(2) + DATA(8) + CHECKSUM(1) = 11 bytes
        packet = struct.pack('2B', self.HEADER_1, self.HEADER_2) + data + struct.pack('B', checksum)

        # Send packet to STM32 transmitter
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(packet)

                # Log occasionally (every 50 messages to avoid spam)
                if not hasattr(self, '_msg_count'):
                    self._msg_count = 0
                self._msg_count += 1

                if self._msg_count % 50 == 0:
                    mode = 'UPPER' if (s5_1 == 0 and s5_2 == 0) else \
                           'LOWER' if (s5_1 == 1 and s5_2 == 0) else \
                           'DUAL' if (s5_1 == 0 and s5_2 == 1) else 'INVALID'

                    self.get_logger().info(
                        f'Sent packet - Mode: {mode}, '
                        f'LX:{joy_left_x} LY:{joy_left_y} RX:{joy_right_x} RY:{joy_right_y}'
                    )

            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')

    def destroy_node(self):
        """Clean up resources"""
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
