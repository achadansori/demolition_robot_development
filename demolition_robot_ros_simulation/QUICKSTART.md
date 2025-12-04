# 🚀 Quick Start Guide

Panduan cepat untuk memulai ROS 2 simulation demolition robot.

## 📦 Installation (Ubuntu 22.04)

```bash
# 1. Install ROS 2 Humble (jika belum)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop

# 2. Install dependencies
sudo apt install -y \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-joy \
  python3-serial \
  python3-colcon-common-extensions

# 3. Setup environment (add to ~/.bashrc)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 🔨 Build Workspace

```bash
cd ~/demolition_robot_development/demolition_robot_ros_simulation
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 🎮 Mode 1: Simulation Only (No Hardware)

Perfect untuk testing tanpa hardware apapun.

```bash
# Terminal 1: Launch simulasi
source ~/demolition_robot_development/demolition_robot_ros_simulation/install/setup.bash
ros2 launch demolition_robot_control simulation.launch.py
```

**Expected Output:**
- ✅ RViz window terbuka dengan model robot
- ✅ Joint State Publisher GUI untuk manual control
- ✅ Console menunjukkan "Joint Controller Node started!"

**Test tanpa gamepad:**
- Gunakan sliders di Joint State Publisher GUI untuk menggerakkan joints
- Robot akan bergerak di RViz sesuai slider

**Test dengan gamepad (Xbox/PS4):**
```bash
# Check gamepad detected
ls /dev/input/js*   # Should show /dev/input/js0

# Terminal 2: Test gamepad input
ros2 topic echo /joy
# Gerakkan joystick, harusnya muncul data

# Robot akan otomatis bergerak sesuai joystick!
```

## 🔌 Mode 2: Real Robot (Serial to STM32)

Connect ke STM32 transmitter untuk control robot real.

```bash
# 1. Check serial port
ls /dev/ttyUSB*   # Should show /dev/ttyUSB0 (or similar)

# 2. Give permission
sudo chmod 666 /dev/ttyUSB0

# 3. Launch real robot control
source ~/demolition_robot_development/demolition_robot_ros_simulation/install/setup.bash
ros2 launch demolition_robot_control real_robot.launch.py port:=/dev/ttyUSB0
```

**Expected Output:**
```
[serial_bridge]: Connected to STM32 transmitter on /dev/ttyUSB0 @ 115200
[serial_bridge]: Serial Bridge Node started!
[joy_node]: Opened joystick: /dev/input/js0
```

**Test:**
```bash
# Terminal 2: Monitor serial TX data
ros2 topic echo /robot/serial_tx

# Gerakkan joystick, harusnya muncul:
# data: 'TX: JL:127,130 JR:127,127'

# Terminal 3: Check joystick input
ros2 topic echo /joy
```

**Troubleshooting:**
- **No /dev/ttyUSB0**: Colok ulang USB cable ke STM32
- **Permission denied**: Jalankan `sudo chmod 666 /dev/ttyUSB0`
- **No data sent**: Check joystick connected di `/dev/input/js0`

## 🔄 Mode 3: Hybrid (Real + Simulation)

**BEST MODE!** Control real robot sambil lihat simulasi di RViz.

```bash
# Terminal 1: Launch hybrid mode
source ~/demolition_robot_development/demolition_robot_ros_simulation/install/setup.bash
ros2 launch demolition_robot_control hybrid.launch.py port:=/dev/ttyUSB0
```

**Expected Output:**
- ✅ RViz window dengan model robot
- ✅ Serial bridge connected ke STM32
- ✅ Joystick input ready
- ✅ **Satu joystick control KEDUA robot (real + virtual)!**

**Test:**
- Gerakkan joystick → Real robot bergerak via LoRa
- Simulasi di RViz juga bergerak bersamaan
- Perfect untuk development & testing!

## 🎯 Joystick Mapping (Default Xbox/PS4)

| Control | Joystick Input | Function |
|---------|----------------|----------|
| **Left Stick Y** | Axes[1] | Cylinder 3 (Bucket) |
| **Right Stick Y** | Axes[3] | Cylinder 2 (Stick) |
| **Right Stick X** | Axes[2] | Cylinder 1 (Boom) |
| **Left Stick X** | Axes[0] | Slew Rotation |
| **D-Pad Up/Down** | Axes[7] | Outrigger Left |
| **D-Pad Left/Right** | Axes[6] | Outrigger Right |
| **Left Trigger** | Axes[4] | Track Left |
| **Right Trigger** | Axes[5] | Track Right |

## 📊 Useful Commands

```bash
# List all topics
ros2 topic list

# Monitor joystick input
ros2 topic echo /joy

# Monitor serial TX to STM32
ros2 topic echo /robot/serial_tx

# Monitor serial RX from STM32
ros2 topic echo /robot/serial_rx

# Monitor joint states (simulation)
ros2 topic echo /joint_states

# Check node status
ros2 node list

# Visualize topic flow
rqt_graph
```

## 🔧 Configuration

Edit `~/demolition_robot_development/demolition_robot_ros_simulation/src/demolition_robot_control/config/robot_params.yaml`:

```yaml
serial:
  port: "/dev/ttyUSB0"    # Change if different
  baudrate: 115200        # Match STM32 settings

control:
  deadzone: 0.1           # Joystick deadzone (0.0 - 1.0)
  speed_scale: 0.02       # Joint speed multiplier
```

Rebuild after config change:
```bash
colcon build
source install/setup.bash
```

## ❌ Troubleshooting

### Problem: RViz doesn't show robot model
**Solution:**
```bash
# Check robot_description topic
ros2 topic echo /robot_description --once

# Restart launch
ros2 launch demolition_robot_control simulation.launch.py
```

### Problem: Joystick not detected
**Solution:**
```bash
# Install jstest-gtk for testing
sudo apt install jstest-gtk
jstest-gtk   # Test gamepad

# Check device
ls -l /dev/input/js0
sudo chmod 666 /dev/input/js0  # If permission denied
```

### Problem: Serial port not found
**Solution:**
```bash
# List USB devices
lsusb

# Check dmesg for USB events
dmesg | grep tty

# Add user to dialout group (for permanent access)
sudo usermod -a -G dialout $USER
# Logout and login again
```

### Problem: Robot moving in simulation but not real
**Solution:**
1. Check serial bridge connected: `ros2 topic echo /robot/serial_tx`
2. Verify STM32 transmitter powered ON
3. Check LoRa link between TX and RX
4. Monitor STM32 receiver PWM outputs with oscilloscope

## 📚 Next Steps

1. **Calibrate joystick mapping** - Edit `joint_controller.py` untuk custom mapping
2. **Add more sensors** - Extend serial protocol untuk IMU, GPS, cameras
3. **Record data** - Use `ros2 bag record` untuk logging
4. **Autonomous mode** - Implement path planning dengan Nav2
5. **Multi-robot** - Control multiple robots simultaneously

## 🎥 Example Session

```bash
# Complete workflow example:

# 1. Build workspace
cd ~/demolition_robot_development/demolition_robot_ros_simulation
colcon build
source install/setup.bash

# 2. Connect STM32 transmitter
sudo chmod 666 /dev/ttyUSB0

# 3. Launch hybrid mode
ros2 launch demolition_robot_control hybrid.launch.py port:=/dev/ttyUSB0

# 4. In another terminal, monitor data
ros2 topic echo /robot/serial_tx

# 5. Grab gamepad and control robot!
# - Left stick Y: Move bucket up/down
# - Right stick: Control boom and stick
# - Watch simulation and real robot move together!

# 6. Record session for playback
ros2 bag record -a   # Press Ctrl+C to stop

# 7. Playback later
ros2 bag play <bag_file>
```

Enjoy controlling your demolition robot! 🎮🤖
