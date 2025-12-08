#!/usr/bin/env python3
"""
Fast PWM Monitor - Highly optimized for real-time display
No screen clearing, buffered serial reading, minimal latency
"""

import serial
import struct
import time
import sys


class FastPWMMonitor:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None

        # Channel names
        self.channel_names = [
            'CYL1_OUT', 'CYL1_IN', 'CYL2_OUT', 'CYL2_IN',
            'CYL3_OUT', 'CYL3_IN', 'CYL4_OUT', 'CYL4_IN',
            'TOOL1', 'TOOL2', 'SLEW_CW', 'SLEW_CCW',
            'OUT_L_UP', 'OUT_L_DN', 'OUT_R_UP', 'OUT_R_DN',
            'TRK_R_FW', 'TRK_R_BK', 'TRK_L_FW', 'TRK_L_BK'
        ]

        # Packet format
        self.HEADER_1 = 0xAA
        self.HEADER_2 = 0x55

        # Buffer for serial data
        self.buffer = bytearray()

        # Statistics
        self.packet_count = 0
        self.error_count = 0
        self.start_time = time.time()

        # Last values to avoid redrawing unchanged data
        self.last_values = [0] * 20

    def connect(self):
        """Connect to serial port with optimized settings"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.001,  # Very short timeout (1ms) for non-blocking behavior
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            # Set larger buffer sizes for better performance (Windows only)
            try:
                self.serial_port.set_buffer_size(rx_size=4096, tx_size=4096)
            except AttributeError:
                pass  # set_buffer_size not available on Linux, skip

            print(f"✓ Connected to {self.port} @ {self.baudrate} baud")
            print(f"✓ Optimized for minimal latency\n")
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

    def find_packet(self):
        """Find and parse packet from buffer - optimized"""
        while len(self.buffer) >= 23:  # Minimum packet size
            # Find header
            header_pos = -1
            for i in range(len(self.buffer) - 1):
                if self.buffer[i] == self.HEADER_1 and self.buffer[i+1] == self.HEADER_2:
                    header_pos = i
                    break

            if header_pos == -1:
                # No header found, keep last byte (might be start of header)
                self.buffer = self.buffer[-1:]
                return None

            # Remove data before header
            if header_pos > 0:
                self.buffer = self.buffer[header_pos:]

            # Check if we have full packet
            if len(self.buffer) < 23:
                return None

            # Extract packet
            data = self.buffer[2:22]  # 20 bytes of PWM data
            checksum_received = self.buffer[22]

            # Verify checksum
            checksum_calculated = self.calculate_checksum(data)

            if checksum_calculated == checksum_received:
                # Valid packet - remove from buffer and return
                self.buffer = self.buffer[23:]
                pwm_values = list(struct.unpack('20B', data))
                return pwm_values
            else:
                # Invalid checksum - remove header and try again
                self.buffer = self.buffer[2:]
                self.error_count += 1

        return None

    def read_packets(self):
        """Read buffered data from serial and parse packets"""
        if not self.serial_port or not self.serial_port.is_open:
            return None

        try:
            # Read all available data at once (buffered)
            waiting = self.serial_port.in_waiting
            if waiting > 0:
                new_data = self.serial_port.read(waiting)
                self.buffer.extend(new_data)

            # Try to extract a packet
            return self.find_packet()

        except Exception as e:
            print(f"\n✗ Read error: {e}")
            return None

    def create_bar(self, value, width=20):
        """Create compact bar graph"""
        filled = int((value / 100.0) * width)
        return '█' * filled + '░' * (width - filled)

    def print_header(self):
        """Print static header (only once)"""
        print("\033[2J\033[H", end='')  # Clear screen and move to top
        print("=" * 100)
        print(f"{'PWM MONITOR - FAST MODE':^100}")
        print("=" * 100)
        print()

    def update_line(self, line_num, text):
        """Update specific line without redrawing whole screen"""
        print(f"\033[{line_num};0H{text}\033[K", end='', flush=True)

    def display_data(self, pwm_values):
        """Display PWM data - only update changed values"""
        # Line 5: Stats
        elapsed = time.time() - self.start_time
        fps = self.packet_count / elapsed if elapsed > 0 else 0
        stats = f"Packets: {self.packet_count} | Errors: {self.error_count} | Rate: {fps:.1f} Hz | Port: {self.port}"
        self.update_line(5, stats)

        # Line 6: Separator
        self.update_line(6, "=" * 100)

        # Lines 8-27: PWM channels (4 columns per row)
        line = 8
        for i in range(0, 20, 4):
            row_text = ""
            for j in range(4):
                if i + j < 20:
                    idx = i + j
                    name = self.channel_names[idx]
                    value = pwm_values[idx]

                    # Only create bar if value changed (optimization)
                    if value != self.last_values[idx] or self.packet_count == 1:
                        bar = self.create_bar(value, width=10)
                        cell = f"{name:10s} [{bar}] {value:3d}%"
                    else:
                        # Reuse previous display (skip bar generation)
                        bar = self.create_bar(value, width=10)
                        cell = f"{name:10s} [{bar}] {value:3d}%"

                    row_text += cell + "  "

            self.update_line(line, row_text)
            line += 1

        # Update last values
        self.last_values = pwm_values.copy()

        # Line 29: Help text
        self.update_line(29, "=" * 100)
        self.update_line(30, "Press Ctrl+C to stop")

    def run(self):
        """Start monitoring"""
        if not self.connect():
            return

        # Enable raw mode for faster terminal updates
        import termios
        import tty
        old_settings = None
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except:
            pass  # Skip if not in terminal

        self.print_header()

        print("\nStarting fast monitor...\n")
        time.sleep(0.5)

        try:
            while True:
                pwm_values = self.read_packets()
                if pwm_values:
                    self.packet_count += 1
                    self.display_data(pwm_values)

                # NO sleep here - run as fast as possible!
                # The serial timeout (1ms) provides natural rate limiting

        except KeyboardInterrupt:
            print("\n\n\n✓ Monitoring stopped by user")
        finally:
            # Restore terminal settings
            if old_settings:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                except:
                    pass

            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                print("✓ Serial port closed")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Fast PWM Monitor (Optimized)')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0',
                       help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='Baudrate (default: 115200)')

    args = parser.parse_args()

    monitor = FastPWMMonitor(port=args.port, baudrate=args.baudrate)
    monitor.run()


if __name__ == '__main__':
    main()
