# Brokk Demolition Robot ROS 2 Simulation

ROS 2 Humble workspace untuk simulasi dan kontrol robot demolition dengan 10 cylinder hydraulic seperti robot Brokk.

## Arsitektur Sistem

### Hardware Real
- **STM32 Transmitter**: Membaca joystick, mengirim data ke STM32 Receiver via LoRa
- **LoRa E220-900T22D**: Komunikasi wireless 868MHz
- **STM32 Receiver**: Menerima data LoRa, mengontrol 20 channel PWM (10kHz) untuk TIP122 transistors
- **Hydraulic System**: 10 actuator (4 cylinder, 2 tool, slew, 2 outrigger, 2 track)

### Komponen Simulasi

#### 1. **brokk_description** Package
URDF model robot demolition dengan 10 joint hydraulic:
- Cylinder 1: Boom (lengan pertama)
- Cylinder 2: Stick (lengan kedua)
- Cylinder 3: Bucket (bucket tilt)
- Cylinder 4: Tool Rotation (rotasi tool)
- Tool 1 & 2: Gripper/crusher jaws
- Outrigger Left & Right: Stabilizer kiri/kanan
- Track Left & Right: Roda track kiri/kanan
- Slew: Rotasi platform

#### 2. **brokk_control** Package
Node-node kontrol:
- **serial_bridge**: Komunikasi dengan STM32 transmitter via USB CDC (ttyACM0)
- **joint_controller**: Konversi joystick ke joint states untuk visualisasi RViz

## Mode Kontrol

Sistem menggunakan mode switching dengan switch s5_1 dan s5_2 (button 4 & 5 pada joystick):

### Mode UPPER (s5_1=0, s5_2=0) - Excavator Controls
```
Left Stick Y:  Cylinder 3 (Bucket) IN/OUT
Left Stick X:  Slew CW/CCW
Right Stick Y: Cylinder 2 (Stick) IN/OUT
Right Stick X: Cylinder 1 (Boom) IN/OUT
```

### Mode LOWER (s5_1=1, s5_2=0) - Mobility Controls
```
Left Stick Y:  Track Left Forward/Backward
Left Stick X:  Outrigger Left Up/Down
Right Stick Y: Track Right Forward/Backward
Right Stick X: Outrigger Right Up/Down
```

### Mode DUAL (s5_1=0, s5_2=1)
Reserved untuk implementasi masa depan.

## Instalasi

### Requirements
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- pyserial (`pip3 install pyserial`)
- Joystick compatible (tested dengan gamepad standar)

### Build Workspace

```bash
cd ~/demolition_robot_development/brokk_demolition_sim

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install dependencies
sudo apt update
sudo apt install ros-humble-joy ros-humble-xacro \
                 ros-humble-joint-state-publisher-gui \
                 python3-serial

# Build
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Cara Menggunakan

### 1. Simulation Mode (Tanpa Hardware)
Visualisasi robot di RViz dengan kontrol joystick, tanpa koneksi ke hardware asli.

```bash
source install/setup.bash
ros2 launch brokk_control simulation.launch.py
```

**Gunakan untuk**:
- Testing control logic
- Visualisasi pergerakan robot
- Development tanpa hardware

### 2. Hardware Mode (Kontrol Real Robot)
Mengirim command ke STM32 transmitter yang terhubung ke hardware asli via LoRa.

```bash
source install/setup.bash

# Default port /dev/ttyACM0
ros2 launch brokk_control hardware.launch.py

# Custom port
ros2 launch brokk_control hardware.launch.py port:=/dev/ttyACM1
```

**Gunakan untuk**:
- Kontrol hardware asli
- Testing dengan robot fisik
- Operasional real

### 3. Hybrid Mode (Hardware + Visualization)
Kontrol hardware asli sambil melihat visualisasi di RViz.

```bash
source install/setup.bash
ros2 launch brokk_control hybrid.launch.py
```

**Gunakan untuk**:
- Kontrol hardware dengan visual feedback
- Monitoring posisi robot real-time
- Best mode untuk operasional

## Visualisasi Saja (Manual Control)

Untuk visualisasi dengan kontrol manual via GUI slider:

```bash
source install/setup.bash
ros2 launch brokk_description display.launch.py
```

## Joystick Mapping

### Standard Gamepad Layout
```
Axis 0: Left Stick X
Axis 1: Left Stick Y
Axis 2: Right Stick X
Axis 3: Right Stick Y

Button 4: s5_1 (Mode switch bit 1)
Button 5: s5_2 (Mode switch bit 2)
Button 6: s6_1 (Reserved)
Button 7: s6_2 (Reserved)
```

### Testing Joystick
```bash
# Install jstest
sudo apt install joystick

# Test joystick
jstest /dev/input/js0

# Check joystick topic in ROS
ros2 topic echo /joy
```

## Protokol Komunikasi Serial

### Binary Packet Format (11 bytes)
```
[Header 2B] [Data 8B] [Checksum 1B]

Header: 0xAA 0x55
Data:
  - joy_left_x  (0-255)
  - joy_left_y  (0-255)
  - joy_right_x (0-255)
  - joy_right_y (0-255)
  - s5_1        (0 or 1)
  - s5_2        (0 or 1)
  - s6_1        (0 or 1)
  - s6_2        (0 or 1)
Checksum: XOR of all data bytes
```

### Joystick Value Mapping
```
ROS Joy axis: -1.0 to +1.0
STM32 format: 0 to 255 (center = 127)

Conversion: byte_value = (axis_value + 1.0) * 127.5
```

## Troubleshooting

### Serial Port Permission Error
```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

### STM32 Not Detected
```bash
# Check USB device
lsusb | grep STM

# Check serial ports
ls -l /dev/ttyACM*

# Try different port
ros2 launch brokk_control hardware.launch.py port:=/dev/ttyUSB0
```

### Joystick Not Working
```bash
# Check joystick device
ls -l /dev/input/js*

# Test with jstest
jstest /dev/input/js0

# Check ROS joy node
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0
```

### RViz Not Showing Robot
```bash
# Check robot_description topic
ros2 topic list | grep robot_description

# Check joint_states
ros2 topic echo /joint_states

# Check TF frames
ros2 run tf2_tools view_frames
```

## Struktur File

```
brokk_demolition_sim/
├── src/
│   ├── brokk_description/          # Robot URDF model
│   │   ├── urdf/
│   │   │   └── brokk_robot.urdf.xacro
│   │   ├── rviz/
│   │   │   └── brokk_view.rviz
│   │   └── launch/
│   │       └── display.launch.py
│   │
│   └── brokk_control/              # Control nodes
│       ├── brokk_control/
│       │   ├── serial_bridge.py    # STM32 serial communication
│       │   └── joint_controller.py # Joint state publisher
│       └── launch/
│           ├── simulation.launch.py
│           ├── hardware.launch.py
│           └── hybrid.launch.py
│
└── README.md
```

## Kecocokan dengan Hardware Asli

Package ini 100% kompatibel dengan hardware asli karena:

1. **Binary Protocol**: Menggunakan format binary packet yang sama (11 bytes dengan header 0xAA 0x55)
2. **Mode Switching**: Implementasi mode UPPER/LOWER sesuai dengan firmware STM32 receiver
3. **Joystick Mapping**: Mapping yang sama dengan control.c di STM32
4. **USB CDC**: Support ttyACM0 untuk STM32 USB Virtual COM Port
5. **PWM Channels**: Joint mapping sesuai dengan 20 PWM channels hardware

## Development

### Modify Robot Model
Edit URDF file:
```bash
nano src/brokk_description/urdf/brokk_robot.urdf.xacro
```

Rebuild:
```bash
colcon build --packages-select brokk_description
```

### Modify Control Logic
Edit serial bridge atau joint controller:
```bash
nano src/brokk_control/brokk_control/serial_bridge.py
nano src/brokk_control/brokk_control/joint_controller.py
```

Rebuild:
```bash
colcon build --packages-select brokk_control
```

### Monitor Communication
```bash
# Watch serial bridge output
ros2 run brokk_control serial_bridge --ros-args --log-level debug

# Monitor joint states
ros2 topic echo /joint_states

# Monitor joystick
ros2 topic echo /joy
```

## Credits

Developed for demolition robot project with:
- STM32F407/F401 microcontrollers
- LoRa E220-900T22D wireless modules
- 20-channel PWM system (10kHz for TIP122 transistors)
- Hydraulic control system

ROS 2 Humble simulation workspace.
