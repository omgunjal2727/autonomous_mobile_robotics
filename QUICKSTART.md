# Quick Start Guide - LiDAR SLAM System

## 🚀 Quick Setup

### On Raspberry Pi (Robot)
```bash
# 1. Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 2. Launch robot nodes
ros2 launch raspi_bringup raspi_robot.launch.py

# If you need to specify ports:
ros2 launch raspi_bringup raspi_robot.launch.py \
  lidar_port:=/dev/ttyUSB0 \
  esp32_port:=/dev/ttyUSB1
```

### On Laptop

**Terminal 1 - SLAM & Visualization:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch laptop_slam laptop_slam.launch.py
```

**Terminal 2 - Robot Control:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch laptop_slam teleop.launch.py
```

**Keyboard Controls:**
- `i` = Forward
- `j` = Turn Left  
- `k` = Stop
- `l` = Turn Right
- `,` = Backward

## 💾 Save Your Map

```bash
# Create maps directory
mkdir -p ~/maps

# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

## 🔍 Troubleshooting Commands

```bash
# Check if scan data is coming from Raspberry Pi
ros2 topic hz /scan
ros2 topic echo /scan --once

# List all topics (should see same on both devices)
ros2 topic list

# Check ROS_DOMAIN_ID (must match on both devices)
echo $ROS_DOMAIN_ID

# Test motor control
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Stop robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## 📦 Package Locations

- **Raspberry Pi Package**: `/home/omg/ros2_ws/src/raspi_bringup`
- **Laptop Package**: `/home/omg/ros2_ws/src/laptop_slam`
- **ESP32 Firmware**: `/home/omg/ros2_ws/src/raspi_bringup/esp32_firmware`

## 📚 Documentation

- [Raspberry Pi Setup Guide](file:///home/omg/ros2_ws/src/raspi_bringup/README.md)
- [Laptop Setup Guide](file:///home/omg/ros2_ws/src/laptop_slam/README.md)
- [ESP32 Firmware Guide](file:///home/omg/ros2_ws/src/raspi_bringup/esp32_firmware/README.md)
- [Complete Walkthrough](file:///home/omg/.gemini/antigravity/brain/d0a61b12-1639-475b-b747-6dccf64d8dbf/walkthrough.md)

## ⚙️ Network Setup

**CRITICAL**: Set the same ROS_DOMAIN_ID on both devices!

```bash
# On both Raspberry Pi and Laptop:
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

## 🔧 First-Time Setup Checklist

### Raspberry Pi
- [ ] ROS2 Humble installed
- [ ] YD LiDAR driver installed
- [ ] `raspi_bringup` package built
- [ ] User added to `dialout` group
- [ ] ROS_DOMAIN_ID set to 42
- [ ] ESP32 programmed with firmware

### Laptop  
- [ ] ROS2 Humble Desktop installed
- [ ] SLAM Toolbox installed
- [ ] `laptop_slam` package built
- [ ] ROS_DOMAIN_ID set to 42
- [ ] Can ping Raspberry Pi

### ESP32
- [ ] ArduinoJson library installed
- [ ] Pin configuration matches motor driver
- [ ] Firmware uploaded successfully
- [ ] Motors tested and working

## 🎯 Tips for Best Results

1. **Drive Slowly**: ~0.2 m/s for best map quality
2. **Overlap Paths**: Revisit areas for loop closure
3. **Close Loops**: Return to start to improve accuracy
4. **Avoid Dynamic Objects**: People, pets, moving items
5. **Good Lighting**: Clear LiDAR line of sight

## 🆘 Common Issues

| Problem | Solution |
|---------|----------|
| Can't see `/scan` topic | Check ROS_DOMAIN_ID, network, firewall |
| Motors not moving | Check ESP32 serial, power, connections |
| Poor map quality | Drive slower, adjust SLAM params |
| "No transform" in RViz | Normal at start - drive robot to initialize |

---

**Ready to map? Start with the Raspberry Pi, then launch SLAM on your laptop!** 🗺️
