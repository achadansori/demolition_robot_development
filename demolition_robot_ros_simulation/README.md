# Demolition Robot ROS 2 Simulation

ROS 2 Humble workspace untuk simulasi dan kontrol demolition robot dengan opsi hybrid:
- **Real Robot**: Control STM32 via serial (LoRa transmitter)
- **Simulation**: Visualisasi di RViz/Gazebo
- **Hybrid**: Kedua mode bisa berjalan bersamaan

## 📋 Requirements

```bash
# ROS 2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop

# Dependencies
sudo apt install -y \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-joy \
  python3-serial \
  python3-colcon-common-extensions
```

## 🏗️ Workspace Structure

```
demolition_robot_ros_simulation/
├── src/
│   ├── demolition_robot_description/    # URDF, meshes, launch files
│   ├── demolition_robot_control/        # Control nodes
│   └── demolition_robot_bringup/        # Launch files untuk startup
├── install/
├── build/
└── log/
```

## 🚀 Quick Start

### 1. Build Workspace

```bash
cd ~/demolition_robot_development/demolition_robot_ros_simulation
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 2. Run Simulation Only (No Hardware)

```bash
# Terminal 1: Launch visualization
ros2 launch demolition_robot_bringup simulation.launch.py

# Terminal 2: Control dengan keyboard/joystick
ros2 run joy joy_node  # Jika pakai gamepad
```

### 3. Run Real Robot (Serial to STM32)

```bash
# Terminal 1: Serial bridge ke STM32 transmitter
ros2 launch demolition_robot_bringup real_robot.launch.py port:=/dev/ttyUSB0

# Terminal 2 (Optional): Monitor data
ros2 topic echo /robot/state
```

### 4. Run Hybrid Mode (Real + Simulation)

```bash
# Kedua mode aktif - joystick control ke real robot DAN simulasi
ros2 launch demolition_robot_bringup hybrid.launch.py port:=/dev/ttyUSB0
```

## 📡 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | sensor_msgs/Joy | Joystick input |
| `/robot/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/robot/joint_states` | sensor_msgs/JointState | Joint positions |
| `/robot/serial_tx` | std_msgs/String | Data sent to STM32 |
| `/robot/serial_rx` | std_msgs/String | Data from STM32 |
| `/robot/cylinder_1` | std_msgs/Float64 | Cylinder 1 control (-1 to 1) |
| `/robot/cylinder_2` | std_msgs/Float64 | Cylinder 2 control (-1 to 1) |
| `/robot/cylinder_3` | std_msgs/Float64 | Cylinder 3 control (-1 to 1) |
| `/robot/cylinder_4` | std_msgs/Float64 | Cylinder 4 control (-1 to 1) |

## 🎮 Joystick Mapping

Default mapping (Xbox/PS4 controller):

- **Left Stick Y**: Cylinder 3 (Bucket)
- **Right Stick Y**: Cylinder 2 (Stick)
- **Left Stick X**: Slew rotation
- **Right Stick X**: Cylinder 1 (Boom)
- **D-Pad Up/Down**: Outrigger left
- **D-Pad Left/Right**: Outrigger right
- **Triggers**: Track movement

## 🔧 Configuration

Edit `demolition_robot_control/config/robot_params.yaml`:

```yaml
serial:
  port: "/dev/ttyUSB0"
  baudrate: 115200
  timeout: 0.1

control:
  deadzone: 10
  joystick_center: 127
  update_rate: 100  # Hz
```

## 📊 Architecture

```
Gamepad → joy_node → control_bridge → ┬→ serial_node → STM32 (Real)
                                       └→ simulation → RViz (Virtual)
```
