# 🚀 Quick Commands - LiDAR SLAM System

## On Raspberry Pi

### First Time Setup
```bash
# 1. Unplug and replug LiDAR to reset it

# 2. Clone/update repository
cd ~
git clone https://github.com/omgunjal2727/autonomous_mobile_robotics.git ros2_ws
# OR if already cloned:
cd ~/ros2_ws && git pull origin main

# 3. Build
cd ~/ros2_ws
colcon build --packages-select raspi_bringup ydlidar_ros2_driver
source install/setup.bash

# 4. Set ROS_DOMAIN_ID (add to ~/.bashrc for permanent)
export ROS_DOMAIN_ID=42
```

### Every Time - Launch Robot
```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=42

# Launch (without ESP32)
ros2 launch raspi_bringup raspi_robot.launch.py esp32_port:=/dev/null
```

---

## On Laptop

### First Time Setup
```bash
# 1. Install dependencies
sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server
sudo apt install ros-humble-teleop-twist-keyboard xterm

# 2. Clone/update repository
cd ~
git clone https://github.com/omgunjal2727/autonomous_mobile_robotics.git ros2_ws
# OR if already cloned:
cd ~/ros2_ws && git pull origin main

# 3. Build
cd ~/ros2_ws
colcon build --packages-select laptop_slam
source install/setup.bash

# 4. Set ROS_DOMAIN_ID (add to ~/.bashrc for permanent)
export ROS_DOMAIN_ID=42
```

### Every Time - Launch SLAM

**Terminal 1: SLAM & Visualization**
```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch laptop_slam laptop_slam.launch.py
```

**Terminal 2: Teleop Control (optional)**
```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch laptop_slam teleop.launch.py
```

### Save Map
```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

---

## Troubleshooting

### LiDAR Not Working
```bash
# Check USB port
ls -l /dev/ttyUSB*

# Give permissions
sudo chmod 666 /dev/ttyUSB0

# Unplug and replug LiDAR if it fails to start
```

### Can't See /scan on Laptop
```bash
# Check ROS_DOMAIN_ID matches on both devices
echo $ROS_DOMAIN_ID  # Should be 42

# Check network connectivity
ping RASPBERRY_PI_IP

# Verify scan is publishing on Raspberry Pi
ros2 topic hz /scan
```

### Transform Errors in RViz
- Normal at startup - move the robot to initialize SLAM
- Errors should stop after a few seconds

---

## What You Should See

**In RViz:**
- ✅ Red dots = Laser scan data
- ✅ Gray/white map = Generated map
- ✅ Black areas = Obstacles/walls
- ✅ White areas = Free space

**Move the robot** (push by hand or use teleop) and watch the map build! 🗺️
