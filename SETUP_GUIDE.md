# Raspberry Pi Setup - Quick Commands

## On Raspberry Pi

### 1. Clone the Repository
```bash
cd ~
git clone https://github.com/omgunjal2727/autonomous_mobile_robotics.git ros2_ws
cd ros2_ws
```

### 2. Install YD LiDAR SDK
```bash
cd ~/ros2_ws/src
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir -p build && cd build
cmake ..
make -j4
sudo make install
```

### 3. Install YD LiDAR ROS2 Driver
```bash
cd ~/ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
```

### 4. Build Only Raspberry Pi Package
```bash
cd ~/ros2_ws
colcon build --packages-select raspi_bringup ydlidar_ros2_driver
source install/setup.bash
```

### 5. Set Permissions
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in, then:
sudo chmod 666 /dev/ttyUSB0  # For LiDAR
sudo chmod 666 /dev/ttyUSB1  # For ESP32
```

### 6. Launch Robot
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch raspi_bringup raspi_robot.launch.py
```

---

## On Laptop

### 1. Clone the Repository
```bash
cd ~
git clone https://github.com/omgunjal2727/autonomous_mobile_robotics.git ros2_ws
cd ros2_ws
```

### 2. Install Dependencies
```bash
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server
sudo apt install ros-humble-teleop-twist-keyboard xterm
```

### 3. Build Only Laptop Package
```bash
cd ~/ros2_ws
colcon build --packages-select laptop_slam
source install/setup.bash
```

### 4. Set ROS_DOMAIN_ID (must match Raspberry Pi)
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

### 5. Launch SLAM
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch laptop_slam laptop_slam.launch.py
```

### 6. Launch Teleop (separate terminal)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch laptop_slam teleop.launch.py
```

### 7. Save Map
```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

---

## Network Setup (Both Devices)

```bash
# On both Raspberry Pi and Laptop
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

## Verify Communication

```bash
# On laptop, check if you can see /scan from Raspberry Pi
ros2 topic list
ros2 topic hz /scan
```
