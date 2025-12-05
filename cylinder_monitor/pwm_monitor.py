#!/usr/bin/env python3
"""
PWM Monitor for Demolition Robot
Real-time visualization of 20 PWM channels from STM32 Receiver
"""

import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import argparse
import sys


class PWMMonitor:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, history_size=100):
        """
        Initialize PWM Monitor

        Args:
            port: Serial port for STM32 receiver
            baudrate: Serial baudrate
            history_size: Number of samples to keep in history
        """
        self.port = port
        self.baudrate = baudrate
        self.history_size = history_size

        # Serial connection
        self.serial_port = None

        # Data storage - 20 channels
        self.num_channels = 20
        self.channel_names = [
            'CYL1_OUT', 'CYL1_IN', 'CYL2_OUT', 'CYL2_IN',
            'CYL3_OUT', 'CYL3_IN', 'CYL4_OUT', 'CYL4_IN',
            'TOOL1', 'TOOL2', 'SLEW_CW', 'SLEW_CCW',
            'OUT_L_UP', 'OUT_L_DN', 'OUT_R_UP', 'OUT_R_DN',
            'TRK_R_FW', 'TRK_R_BK', 'TRK_L_FW', 'TRK_L_BK'
        ]

        # History buffer for each channel
        self.history = {i: deque([0] * history_size, maxlen=history_size)
                       for i in range(self.num_channels)}

        # Current values
        self.current_values = [0] * self.num_channels

        # Packet format
        self.HEADER_1 = 0xAA
        self.HEADER_2 = 0x55
        self.PACKET_SIZE = 23  # Header(2) + Data(20) + Checksum(1)

    def connect(self):
        """Connect to serial port"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"✓ Connected to {self.port} @ {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to open {self.port}: {e}")
            return False

    def calculate_checksum(self, data):
        """Calculate XOR checksum"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def read_packet(self):
        """Read and parse one packet from serial"""
        if not self.serial_port or not self.serial_port.is_open:
            return None

        try:
            # Look for header
            while True:
                byte1 = self.serial_port.read(1)
                if not byte1:
                    return None

                if byte1[0] == self.HEADER_1:
                    byte2 = self.serial_port.read(1)
                    if byte2 and byte2[0] == self.HEADER_2:
                        # Found header, read rest of packet
                        data = self.serial_port.read(20)  # 20 PWM values
                        if len(data) != 20:
                            continue

                        checksum = self.serial_port.read(1)
                        if not checksum:
                            continue

                        # Verify checksum
                        calculated = self.calculate_checksum(data)
                        if calculated == checksum[0]:
                            # Valid packet, unpack data
                            pwm_values = list(struct.unpack('20B', data))
                            return pwm_values
                        else:
                            print(f"⚠ Checksum error: got {checksum[0]}, expected {calculated}")

        except Exception as e:
            print(f"✗ Read error: {e}")
            return None

    def update_data(self):
        """Read packet and update data"""
        pwm_values = self.read_packet()
        if pwm_values:
            self.current_values = pwm_values
            for i, value in enumerate(pwm_values):
                self.history[i].append(value)
            return True
        return False

    def setup_plot(self):
        """Setup matplotlib figure and axes"""
        # Create figure with subplots
        self.fig, self.axes = plt.subplots(5, 4, figsize=(16, 12))
        self.fig.suptitle('PWM Monitor - 20 Channels (0-100%)', fontsize=16, fontweight='bold')
        self.axes = self.axes.flatten()

        # Setup each subplot
        self.lines = []
        for i in range(self.num_channels):
            ax = self.axes[i]
            ax.set_title(self.channel_names[i], fontsize=10, fontweight='bold')
            ax.set_ylim(-5, 105)
            ax.set_xlim(0, self.history_size)
            ax.grid(True, alpha=0.3)
            ax.set_ylabel('PWM (%)', fontsize=8)

            # Create line
            line, = ax.plot([], [], 'b-', linewidth=2)
            self.lines.append(line)

            # Add value text
            ax.text(0.98, 0.95, '', transform=ax.transAxes,
                   fontsize=12, fontweight='bold',
                   verticalalignment='top', horizontalalignment='right',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        plt.tight_layout(rect=[0, 0.03, 1, 0.97])

    def animate(self, frame):
        """Animation function for matplotlib"""
        # Read new data
        self.update_data()

        # Update plots
        x_data = np.arange(len(self.history[0]))
        for i in range(self.num_channels):
            y_data = list(self.history[i])
            self.lines[i].set_data(x_data, y_data)

            # Update value text
            ax = self.axes[i]
            value_text = ax.texts[0]
            value_text.set_text(f'{self.current_values[i]}%')

            # Color code based on value
            if self.current_values[i] > 0:
                self.lines[i].set_color('green')
                self.lines[i].set_linewidth(2.5)
            else:
                self.lines[i].set_color('blue')
                self.lines[i].set_linewidth(1.5)

        return self.lines

    def run(self):
        """Start monitoring and visualization"""
        if not self.connect():
            return

        print(f"\n{'='*60}")
        print(f"PWM Monitor Running")
        print(f"Port: {self.port} @ {self.baudrate} baud")
        print(f"Channels: {self.num_channels}")
        print(f"{'='*60}\n")

        # Setup plot
        self.setup_plot()

        # Start animation
        ani = FuncAnimation(self.fig, self.animate, interval=50, blit=False, cache_frame_data=False)

        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n\n✓ Monitoring stopped by user")
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                print("✓ Serial port closed")


def main():
    parser = argparse.ArgumentParser(description='PWM Monitor for Demolition Robot')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0',
                       help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='Baudrate (default: 115200)')
    parser.add_argument('-s', '--history', type=int, default=100,
                       help='History size (default: 100)')

    args = parser.parse_args()

    monitor = PWMMonitor(port=args.port, baudrate=args.baudrate, history_size=args.history)
    monitor.run()


if __name__ == '__main__':
    main()
