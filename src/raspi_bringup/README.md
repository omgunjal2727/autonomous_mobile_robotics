# Raspberry Pi Robot Setup Guide

This package runs on the Raspberry Pi mounted on your robot. It handles:
- **YD LiDAR**: Publishes `/scan` data for mapping
- **ESP32 Motor Control**: Receives `/cmd_vel` commands and controls motors via serial

## Prerequisites

### Hardware
- Raspberry Pi (3B+ or newer recommended)
- YD LiDAR (X2, X4, or compatible model)
- ESP32 connected via USB for motor control
- Two available USB ports

### Software
- Ubuntu 22.04 (for ROS2 Humble)
- ROS2 Humble installed

## Installation

### 1. Install ROS2 Humble on Raspberry Pi

```bash
# Add ROS2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble (base version for Raspberry Pi)
sudo apt update
sudo apt install -y ros-humble-ros-base

# Install additional tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo apt install -y python3-serial  # For ESP32 communication
```

### 2. Install YD LiDAR ROS2 Driver

```bash
# Install dependencies
sudo apt install -y cmake pkg-config

# Clone and build YD LiDAR driver
cd ~/ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ~/ros2_ws
colcon build --packages-select ydlidar_ros2_driver
```

### 3. Build This Package

```bash
cd ~/ros2_ws
colcon build --packages-select raspi_bringup
source install/setup.bash
```

## Configuration

### 1. Identify Serial Ports

Connect your devices and identify which USB port is which:

```bash
# List all USB devices
ls -l /dev/ttyUSB*

# Check device info
udevadm info -a -n /dev/ttyUSB0 | grep -E 'ATTRS{idVendor}|ATTRS{idProduct}'
```

**Typical setup:**
- YD LiDAR: `/dev/ttyUSB0`
- ESP32: `/dev/ttyUSB1`

### 2. Set USB Permissions

```bash
# Add your user to dialout group (required for serial access)
sudo usermod -a -G dialout $USER

# Log out and log back in for changes to take effect
```

### 3. (Optional) Create Persistent Device Names

Create udev rules for consistent device names:

```bash
# Create udev rules file
sudo nano /etc/udev/rules.d/99-robot-devices.rules
```

Add these lines (adjust ATTRS based on your devices):

```
# YD LiDAR
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ydlidar"

# ESP32
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="esp32"
```

Reload udev rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Now you can use `/dev/ydlidar` and `/dev/esp32` instead of `/dev/ttyUSB*`.

### 4. Configure Network for ROS2

Set the same ROS_DOMAIN_ID on both Raspberry Pi and laptop:

```bash
# Add to ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

## Usage

### Start Robot Nodes

```bash
# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch with default ports
ros2 launch raspi_bringup raspi_robot.launch.py

# Or specify custom ports
ros2 launch raspi_bringup raspi_robot.launch.py lidar_port:=/dev/ttyUSB0 esp32_port:=/dev/ttyUSB1

# Or use persistent device names
ros2 launch raspi_bringup raspi_robot.launch.py lidar_port:=/dev/ydlidar esp32_port:=/dev/esp32
```

### Verify Everything is Working

**In separate terminals:**

```bash
# Check if scan data is being published
ros2 topic hz /scan
ros2 topic echo /scan --once

# Test motor control
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## Auto-Start on Boot (Optional)

Create a systemd service to start the robot nodes automatically:

```bash
sudo nano /etc/systemd/system/robot.service
```

Add:

```ini
[Unit]
Description=Robot ROS2 Nodes
After=network.target

[Service]
Type=simple
User=YOUR_USERNAME
Environment="ROS_DOMAIN_ID=42"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/YOUR_USERNAME/ros2_ws/install/setup.bash && ros2 launch raspi_bringup raspi_robot.launch.py"
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable robot.service
sudo systemctl start robot.service

# Check status
sudo systemctl status robot.service
```

## Troubleshooting

### LiDAR Not Publishing

```bash
# Check if device is connected
ls -l /dev/ttyUSB*

# Check permissions
groups  # Should include 'dialout'

# Check LiDAR node logs
ros2 node info /ydlidar_node
```

### ESP32 Not Responding

```bash
# Test serial connection
sudo apt install -y minicom
minicom -D /dev/ttyUSB1 -b 115200

# Check if ESP32 is receiving data (check ESP32 serial monitor)
```

### Network Issues

```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # Should be same on both devices

# Check if topics are visible from laptop
ros2 topic list

# Test network connectivity
ping YOUR_LAPTOP_IP
```

## ESP32 Firmware

The ESP32 should be programmed to:
1. Receive JSON commands via serial: `{"linear": 0.5, "angular": 0.2}`
2. Parse the JSON and control motors accordingly
3. (Optional) Send IMU data back

Example Arduino code structure:

```cpp
#include <ArduinoJson.h>

void setup() {
  Serial.begin(115200);
  // Initialize motor pins
}

void loop() {
  if (Serial.available()) {
    String json = Serial.readStringUntil('\n');
    StaticJsonDocument<200> doc;
    deserializeJson(doc, json);
    
    float linear = doc["linear"];
    float angular = doc["angular"];
    
    // Convert to motor speeds and control motors
    controlMotors(linear, angular);
  }
}
```

## Next Steps

Once the Raspberry Pi is running:
1. Connect your laptop to the same network
2. Set the same `ROS_DOMAIN_ID` on your laptop
3. Follow the laptop setup guide to run SLAM and teleop

## License

Apache-2.0
