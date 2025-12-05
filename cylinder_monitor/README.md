# PWM Cylinder Monitor

Monitor dan visualisasi real-time untuk 20 channel PWM output dari STM32 Receiver demolition robot.

## Fitur

- ✅ Real-time monitoring 20 channel PWM (0-100%)
- ✅ Visualisasi grafik dengan matplotlib
- ✅ Mode text-based untuk terminal
- ✅ Komunikasi serial USB CDC dengan STM32
- ✅ Checksum validation
- ✅ Error detection dan packet counting

## Channel List (20 Channels)

```
PWM 1-2:   Cylinder 1 OUT/IN (Boom)
PWM 3-4:   Cylinder 2 OUT/IN (Stick)
PWM 5-6:   Cylinder 3 OUT/IN (Bucket)
PWM 7-8:   Cylinder 4 OUT/IN
PWM 9-10:  Tool 1 / Tool 2
PWM 11-12: Slew CW / CCW
PWM 13-14: Outrigger Left UP/DOWN
PWM 15-16: Outrigger Right UP/DOWN
PWM 17-18: Track Right FORWARD/BACKWARD
PWM 19-20: Track Left FORWARD/BACKWARD
```

## Protokol Komunikasi

### Format Packet (23 bytes)
```
[Header 2B] [PWM Data 20B] [Checksum 1B]

Header: 0xAA 0x55
Data: 20 bytes, setiap byte = PWM duty cycle 0-100%
Checksum: XOR of all 20 data bytes
```

## Instalasi

### Requirements
- Python 3.7+
- pyserial
- matplotlib (untuk mode grafik)
- numpy (untuk mode grafik)

### Install Dependencies
```bash
cd cylinder_monitor

# Install dengan pip
pip3 install -r requirements.txt

# Atau manual
pip3 install pyserial numpy matplotlib
```

### Serial Port Permission
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Logout dan login lagi, atau:
newgrp dialout
```

## Cara Menggunakan

### 1. Mode Grafik (Matplotlib)
Visualisasi 20 channel dengan grafik real-time:

```bash
# Default port /dev/ttyACM0, baudrate 115200
python3 pwm_monitor.py

# Custom port
python3 pwm_monitor.py -p /dev/ttyACM1

# Custom baudrate
python3 pwm_monitor.py -b 9600

# Custom history size
python3 pwm_monitor.py -s 200
```

**Output**:
- Window matplotlib dengan 20 subplot (5 baris x 4 kolom)
- Setiap subplot menampilkan grafik real-time satu channel
- Nilai PWM ditampilkan di pojok kanan atas setiap subplot
- Warna hijau = aktif (PWM > 0), biru = tidak aktif (PWM = 0)

### 2. Mode Text (Terminal)
Monitoring di terminal tanpa GUI:

```bash
# Default port /dev/ttyACM0
python3 pwm_monitor_simple.py

# Custom port dan baudrate
python3 pwm_monitor_simple.py -p /dev/ttyUSB0 -b 9600
```

**Output**:
```
================================================================================
                    PWM MONITOR - DEMOLITION ROBOT
================================================================================
Port: /dev/ttyACM0 @ 115200 baud
Packets: 1234 | Errors: 2
================================================================================

CYL1_OUT   [░░░░░░░░░░]   0%  CYL1_IN    [████░░░░░░]  45%  ...
CYL2_OUT   [███████░░░]  78%  CYL2_IN    [░░░░░░░░░░]   0%  ...
...

================================================================================
Press Ctrl+C to stop
```

## Modifikasi STM32 Receiver

Untuk mengirim data PWM via USB serial, tambahkan di `control_demoliiton_robot/Core/Src/main.c`:

```c
// Di bagian includes, tambahkan
#include "pwm.h"
#include <string.h>

// Di main loop (setelah Control_Update), tambahkan:

// Send PWM data via USB CDC every 100ms
static uint32_t last_usb_send = 0;
if (HAL_GetTick() - last_usb_send >= 100) {
    last_usb_send = HAL_GetTick();

    // Create packet: Header(2) + PWM(20) + Checksum(1)
    uint8_t packet[23];
    packet[0] = 0xAA;  // Header 1
    packet[1] = 0x55;  // Header 2

    // Get all 20 PWM values
    for (int i = 0; i < 20; i++) {
        packet[2 + i] = PWM_GetDutyCycle(i);
    }

    // Calculate checksum
    uint8_t checksum = 0;
    for (int i = 2; i < 22; i++) {
        checksum ^= packet[i];
    }
    packet[22] = checksum;

    // Send via USB CDC
    CDC_Transmit_FS(packet, 23);
}
```

Tambahkan fungsi getter di `pwm.c`:

```c
uint8_t PWM_GetDutyCycle(PWM_Channel_t channel)
{
    if (channel >= PWM_CHANNEL_COUNT) return 0;

    // Return current duty cycle (0-100)
    return pwm_duty_cycle[channel];
}
```

Dan deklarasi di `pwm.h`:

```c
uint8_t PWM_GetDutyCycle(PWM_Channel_t channel);
```

## Troubleshooting

### Serial Port Not Found
```bash
# Check available ports
ls -l /dev/ttyACM* /dev/ttyUSB*

# Check USB devices
lsusb | grep STM

# Try different port
python3 pwm_monitor.py -p /dev/ttyUSB0
```

### Permission Denied
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Logout and login, or:
newgrp dialout
```

### No Data Received
1. Pastikan STM32 receiver sudah diprogram dengan code pengiriman data
2. Check baudrate sama (default 115200)
3. Check STM32 USB CDC sudah terdeteksi
4. Test dengan serial monitor: `screen /dev/ttyACM0 115200`

### Matplotlib Error
```bash
# Install matplotlib dependencies
sudo apt install python3-tk

# Or use simple text monitor
python3 pwm_monitor_simple.py
```

## Command Line Options

### pwm_monitor.py (Grafik)
```
-p, --port      Serial port (default: /dev/ttyACM0)
-b, --baudrate  Baudrate (default: 115200)
-s, --history   History buffer size (default: 100)
-h, --help      Show help message
```

### pwm_monitor_simple.py (Text)
```
-p, --port      Serial port (default: /dev/ttyACM0)
-b, --baudrate  Baudrate (default: 115200)
-h, --help      Show help message
```

## Testing

Untuk test tanpa hardware, bisa pakai serial port virtual:

```bash
# Install socat
sudo apt install socat

# Create virtual serial ports
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# Output: /dev/pts/X <-> /dev/pts/Y

# Monitor di satu terminal
python3 pwm_monitor_simple.py -p /dev/pts/X

# Send test data di terminal lain
echo -ne '\xAA\x55\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00' > /dev/pts/Y
```

## Tips

1. **Mode Grafik** lebih bagus untuk analisis dan debugging
2. **Mode Text** lebih ringan untuk monitoring jangka panjang
3. Gunakan history size besar (200-500) untuk melihat trend jangka panjang
4. Update rate 20Hz (50ms) sudah cukup untuk monitoring real-time
5. Channel yang tidak aktif akan menampilkan 0%

## Integration dengan ROS 2

Monitor ini bisa digabung dengan ROS 2:
- Mode hardware: STM32 → LoRa → STM32 Receiver → USB → Monitor
- Mode hybrid: Joystick → ROS → STM32 Transmitter → LoRa → Receiver → Monitor

Jadi bisa melihat output PWM real-time sambil kontrol dari ROS 2!

## Credits

Developed for demolition robot project with:
- STM32F407/F401 microcontrollers
- 20-channel PWM system (10kHz for TIP122)
- USB CDC Virtual COM Port
- Real-time monitoring and visualization
