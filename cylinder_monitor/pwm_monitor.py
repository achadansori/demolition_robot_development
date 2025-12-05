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
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, history_size=50):
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
        # Channel names matching PWM channel order in pwm.h
        self.num_channels = 20
        self.channel_names = [
            # PWM 0-1: Cylinder 1 (Boom)
            'CYL1_OUT', 'CYL1_IN',
            # PWM 2-3: Cylinder 2 (Stick)
            'CYL2_OUT', 'CYL2_IN',
            # PWM 4-5: Cylinder 3 (Bucket)
            'CYL3_OUT', 'CYL3_IN',
            # PWM 6-7: Cylinder 4 (unused in current control logic)
            'CYL4_OUT', 'CYL4_IN',
            # PWM 8-9: Tools (unused in current control logic)
            'TOOL1', 'TOOL2',
            # PWM 10-11: Slew rotation
            'SLEW_CW', 'SLEW_CCW',
            # PWM 12-13: Outrigger Left
            'OUT_L_UP', 'OUT_L_DN',
            # PWM 14-15: Outrigger Right
            'OUT_R_UP', 'OUT_R_DN',
            # PWM 16-17: Track Right
            'TRK_R_FW', 'TRK_R_BK',
            # PWM 18-19: Track Left
            'TRK_L_FW', 'TRK_L_BK'
        ]

        # History buffer for each channel (numpy for faster access)
        self.history = {i: np.zeros(history_size) for i in range(self.num_channels)}
        self.history_index = 0

        # Current values
        self.current_values = [0] * self.num_channels

        # Packet format
        self.HEADER_1 = 0xAA
        self.HEADER_2 = 0x55
        self.PACKET_SIZE = 23  # Header(2) + Data(20) + Checksum(1)

        # X-axis data (pre-computed)
        self.x_data = np.arange(history_size)

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
            # Update history using circular buffer
            for i, value in enumerate(pwm_values):
                self.history[i] = np.roll(self.history[i], -1)
                self.history[i][-1] = value
            return True
        return False

    def setup_plot(self):
        """Setup matplotlib figure and axes"""
        # Use interactive backend for better performance
        plt.ion()

        # Create figure with subplots
        self.fig, self.axes = plt.subplots(5, 4, figsize=(16, 10))
        self.fig.suptitle('PWM Monitor - 20 Channels (0-100%)', fontsize=14, fontweight='bold')
        self.axes = self.axes.flatten()

        # Setup each subplot
        self.lines = []
        self.texts = []

        for i in range(self.num_channels):
            ax = self.axes[i]
            ax.set_title(self.channel_names[i], fontsize=9, fontweight='bold')
            ax.set_ylim(-5, 105)
            ax.set_xlim(0, self.history_size - 1)
            ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
            ax.set_ylabel('PWM (%)', fontsize=7)
            ax.tick_params(labelsize=7)

            # Create line with initial data
            line, = ax.plot(self.x_data, self.history[i], 'b-', linewidth=1.5, animated=True)
            self.lines.append(line)

            # Add value text (animated for better performance)
            text = ax.text(0.98, 0.95, '0%', transform=ax.transAxes,
                          fontsize=11, fontweight='bold',
                          verticalalignment='top', horizontalalignment='right',
                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7),
                          animated=True)
            self.texts.append(text)

        plt.tight_layout(rect=[0, 0.02, 1, 0.98])

        # Draw background once
        self.fig.canvas.draw()
        self.backgrounds = [self.fig.canvas.copy_from_bbox(ax.bbox) for ax in self.axes]

    def animate(self, frame):
        """Animation function for matplotlib - optimized with blitting"""
        # Read new data
        self.update_data()

        # List of artists to return for blitting
        artists = []

        # Update each channel
        for i in range(self.num_channels):
            # Restore background
            self.fig.canvas.restore_region(self.backgrounds[i])

            # Check if channel has any activity
            has_activity = self.current_values[i] > 0 or np.max(self.history[i]) > 0

            # Update line data and visibility
            if has_activity:
                # Show line with actual data
                self.lines[i].set_ydata(self.history[i])
                self.lines[i].set_visible(True)

                # Color based on current state
                if self.current_values[i] > 0:
                    self.lines[i].set_color('#00AA00')  # Green when active
                    self.lines[i].set_linewidth(2.0)
                else:
                    self.lines[i].set_color('#0066CC')  # Blue when fading
                    self.lines[i].set_linewidth(1.5)
            else:
                # No activity - set data to NaN to completely remove line
                self.lines[i].set_ydata(np.full(self.history_size, np.nan))
                self.lines[i].set_visible(True)  # Keep visible but with NaN data

            # Update value text
            self.texts[i].set_text(f'{self.current_values[i]}%')

            # Redraw line and text
            self.axes[i].draw_artist(self.lines[i])
            self.axes[i].draw_artist(self.texts[i])

            # Blit the subplot
            self.fig.canvas.blit(self.axes[i].bbox)

            artists.extend([self.lines[i], self.texts[i]])

        return artists

    def run(self):
        """Start monitoring and visualization"""
        if not self.connect():
            return

        print(f"\n{'='*60}")
        print(f"PWM Monitor Running - Optimized")
        print(f"Port: {self.port} @ {self.baudrate} baud")
        print(f"Channels: {self.num_channels}")
        print(f"Update Rate: 50Hz (20ms interval)")
        print(f"{'='*60}\n")

        # Setup plot
        self.setup_plot()

        # Start animation with faster interval and blitting
        # interval=20 means 50Hz update rate (very responsive)
        ani = FuncAnimation(self.fig, self.animate, interval=20, blit=True, cache_frame_data=False)

        try:
            plt.show(block=True)
        except KeyboardInterrupt:
            print("\n\n✓ Monitoring stopped by user")
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                print("✓ Serial port closed")


def main():
    parser = argparse.ArgumentParser(description='PWM Monitor for Demolition Robot (Optimized)')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0',
                       help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='Baudrate (default: 115200)')
    parser.add_argument('-s', '--history', type=int, default=50,
                       help='History buffer size in samples (default: 50, smaller = faster)')

    args = parser.parse_args()

    monitor = PWMMonitor(port=args.port, baudrate=args.baudrate, history_size=args.history)
    monitor.run()


if __name__ == '__main__':
    main()
