#!/usr/bin/env python3
"""
Simple PWM Monitor (Text-based)
Displays PWM values in terminal without graphical plot
"""

import serial
import struct
import time
import sys
import os


class SimplePWMMonitor:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None

        # Channel names matching PWM channel order in pwm.h
        self.channel_names = [
            # PWM 0-1: Cylinder 1 (Boom)
            'CYL1_OUT', 'CYL1_IN',
            # PWM 2-3: Cylinder 2 (Stick)
            'CYL2_OUT', 'CYL2_IN',
            # PWM 4-5: Cylinder 3 (Bucket)
            'CYL3_OUT', 'CYL3_IN',
            # PWM 6-7: Cylinder 4 (unused)
            'CYL4_OUT', 'CYL4_IN',
            # PWM 8-9: Tools (unused)
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

        # Packet format
        self.HEADER_1 = 0xAA
        self.HEADER_2 = 0x55

        # Statistics
        self.packet_count = 0
        self.error_count = 0

    def connect(self):
        """Connect to serial port"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            print(f"✓ Connected to {self.port} @ {self.baudrate} baud\n")
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
        """Read and parse one packet"""
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
                        # Read data
                        data = self.serial_port.read(20)
                        if len(data) != 20:
                            continue

                        checksum = self.serial_port.read(1)
                        if not checksum:
                            continue

                        # Verify checksum
                        calculated = self.calculate_checksum(data)
                        if calculated == checksum[0]:
                            pwm_values = list(struct.unpack('20B', data))
                            return pwm_values
                        else:
                            self.error_count += 1

        except Exception as e:
            print(f"✗ Read error: {e}")
            return None

    def clear_screen(self):
        """Clear terminal screen"""
        os.system('clear' if os.name == 'posix' else 'cls')

    def create_bar(self, value, width=30):
        """Create a text-based bar graph"""
        filled = int((value / 100.0) * width)
        bar = '█' * filled + '░' * (width - filled)
        return bar

    def display_data(self, pwm_values):
        """Display PWM data in terminal"""
        self.clear_screen()

        print("=" * 80)
        print(f"{'PWM MONITOR - DEMOLITION ROBOT':^80}")
        print("=" * 80)
        print(f"Port: {self.port} @ {self.baudrate} baud")
        print(f"Packets: {self.packet_count} | Errors: {self.error_count}")
        print("=" * 80)
        print()

        # Display in 4 columns
        for i in range(0, 20, 4):
            for j in range(4):
                if i + j < 20:
                    idx = i + j
                    name = self.channel_names[idx]
                    value = pwm_values[idx]
                    bar = self.create_bar(value, width=10)
                    print(f"{name:10s} [{bar}] {value:3d}%", end="  ")
            print()

        print()
        print("=" * 80)
        print("Press Ctrl+C to stop")

    def run(self):
        """Start monitoring"""
        if not self.connect():
            return

        print("Starting monitor...\n")
        time.sleep(1)

        try:
            while True:
                pwm_values = self.read_packet()
                if pwm_values:
                    self.packet_count += 1
                    self.display_data(pwm_values)
                    time.sleep(0.05)  # 20Hz update rate

        except KeyboardInterrupt:
            print("\n\n✓ Monitoring stopped by user")
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                print("✓ Serial port closed")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Simple PWM Monitor (Text)')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0',
                       help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='Baudrate (default: 115200)')

    args = parser.parse_args()

    monitor = SimplePWMMonitor(port=args.port, baudrate=args.baudrate)
    monitor.run()


if __name__ == '__main__':
    main()
