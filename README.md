# Differential Drive Robot - ROS 2

A complete ROS 2 differential drive robot system with description, control, and bringup packages.

## Packages

### Robot Platform
- **diffbot_description** - Robot URDF and visualization
- **diffbot_control** - ros2_control with differential drive controller
- **diffbot_bringup** - Central launch point for all components

### LiDAR SLAM System (NEW)
- **raspi_bringup** - Lightweight package for Raspberry Pi (YD LiDAR + ESP32 motor control)
- **laptop_slam** - SLAM Toolbox integration and RViz visualization for mapping

See [QUICKSTART.md](QUICKSTART.md) for LiDAR SLAM setup and usage.

## Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage

### 1. Launch the Robot (Full System)

```bash
ros2 launch diffbot_bringup diffbot_bringup.launch.py
```

Options:
- `use_rviz:=false` - Launch without RViz
- `use_mock_hardware:=false` - Use real hardware (requires hardware plugin)

### 2. View Robot (Visualization Only)

```bash
ros2 launch diffbot_bringup view_robot.launch.py
```

### 3. Control the Robot (Keyboard Teleop)

```bash
# In a separate terminal, source the workspace first
source ~/ros2_ws/install/setup.bash

# Run teleop (publishes to /cmd_vel which is remapped to the controller)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> **Note:** Make sure your keyboard focus is on the teleop terminal window when pressing keys!

**Controls:**
- `i` - Forward
- `k` - Stop  
- `,` - Backward
- `j` - Turn left
- `l` - Turn right
- `u` / `o` - Forward + turn
- `m` / `.` - Backward + turn
- `q` / `z` - Increase/decrease max speeds
- `w` / `x` - Increase/decrease linear speed only
- `e` / `c` - Increase/decrease angular speed only

## Robot Specifications

- **Wheel separation**: 0.45 m
- **Wheel radius**: 0.1 m
- **Max linear velocity**: 1.0 m/s
- **Max angular velocity**: 2.0 rad/s

## Topics

- `/diff_drive_controller/cmd_vel` - Velocity commands (input)
- `/diff_drive_controller/odom` - Odometry (output)
- `/joint_states` - Joint states (output)
- `/tf` - Transforms (output)

## LiDAR SLAM System

### Quick Start

**On Raspberry Pi (Robot):**
```bash
ros2 launch raspi_bringup raspi_robot.launch.py
```

**On Laptop:**
```bash
# Terminal 1: SLAM & Visualization
ros2 launch laptop_slam laptop_slam.launch.py

# Terminal 2: Teleop Control
ros2 launch laptop_slam teleop.launch.py
```

### Documentation
- [Quick Start Guide](QUICKSTART.md) - Essential commands
- [Raspberry Pi Setup](src/raspi_bringup/README.md) - Detailed Pi configuration
- [Laptop Setup](src/laptop_slam/README.md) - SLAM setup
- [ESP32 Firmware](src/raspi_bringup/esp32_firmware/README.md) - Motor control

### System Architecture
```
Raspberry Pi (Robot)          Laptop (Remote)
├── YD LiDAR (/scan)    ────▶ ├── SLAM Toolbox
├── ESP32 Bridge        ◀──── ├── Teleop Keyboard
└── Motors                     └── RViz Visualization
```

## License

Apache-2.0

