# Simple LiDAR SLAM - Quick Start Guide

Connect your YD LiDAR directly to your laptop, visualize scan data in RViz, and generate maps using SLAM Toolbox. Control your robot manually via teleop.

## 🎯 What This Does

- **Connects** YD LiDAR to your laptop via USB
- **Visualizes** laser scan data in RViz (red dots)
- **Generates** a 2D map as you move the robot
- **Provides** keyboard teleop to send movement commands
- **Saves** maps for later use

## 📋 Prerequisites

### Hardware
- Laptop with Ubuntu 22.04
- YD LiDAR (X2, X4, or compatible model)
- USB cable to connect LiDAR to laptop
- Robot with motors (you'll move it manually or via teleop)

### Software
```bash
# Install ROS2 Humble Desktop
sudo apt install ros-humble-desktop

# Install required packages
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install xterm

# Install YD LiDAR driver
cd ~/ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ~/ros2_ws
colcon build --packages-select ydlidar_ros2_driver
```

## 🚀 Quick Start (3 Steps)

### 1. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select lidar_slam_simple
source install/setup.bash
```

### 2. Connect YD LiDAR

```bash
# Plug in your YD LiDAR via USB

# Find the port (usually /dev/ttyUSB0)
ls /dev/ttyUSB*

# Give yourself permission
sudo chmod 666 /dev/ttyUSB0
# OR add yourself to dialout group (one-time, requires logout):
sudo usermod -a -G dialout $USER
```

### 3. Launch Everything

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch lidar_slam_simple slam.launch.py
```

**That's it!** This single command starts:
- ✅ YD LiDAR node (publishes `/scan`)
- ✅ SLAM Toolbox (builds map)
- ✅ RViz (visualization)
- ✅ Teleop keyboard (robot control)

## 🎮 Using the System

### What You'll See

**RViz Window:**
- **Red dots** = Laser scan data from LiDAR
- **Gray/white map** = Generated map (builds as you move)
- **Black areas** = Obstacles/walls
- **White areas** = Free space

**Teleop Window (xterm):**
- Keyboard controls for robot movement

### Keyboard Controls

Focus on the **teleop window** and use these keys:

```
   u    i    o
   j    k    l
   m    ,    .
```

- `i` = Move forward
- `,` = Move backward
- `j` = Turn left
- `l` = Turn right
- `k` = Stop
- `u/o/m/.` = Diagonal movements
- `q/z` = Increase/decrease max speeds

### Building a Map

1. **Start the system** (launch command above)
2. **Wait** for RViz to open and show laser scans
3. **Move the robot** slowly using teleop keys
4. **Watch** the map build in RViz (gray areas)
5. **Explore** your environment - drive around slowly
6. **Return** to starting point for loop closure (improves map)

**Tips:**
- Move slowly (~0.2 m/s) for best results
- Overlap your paths for better mapping
- Avoid dynamic objects (people, pets)

## 💾 Save Your Map

Once you're happy with the map:

```bash
# In a new terminal
source ~/ros2_ws/install/setup.bash

# Create maps directory
mkdir -p ~/maps

# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

This creates:
- `my_map.pgm` - Map image
- `my_map.yaml` - Map metadata

## 🔧 Troubleshooting

### LiDAR Not Working

```bash
# Check if LiDAR is connected
ls /dev/ttyUSB*

# Check permissions
ls -l /dev/ttyUSB0

# If permission denied, run:
sudo chmod 666 /dev/ttyUSB0

# Check if scan data is publishing
ros2 topic hz /scan
ros2 topic echo /scan --once
```

### No Map Appearing

- **Move the robot!** The map only builds when you move
- Check that laser scans are visible in RViz (red dots)
- Ensure SLAM Toolbox is running: `ros2 node list | grep slam`

### Teleop Not Working

- Make sure the **teleop window** has focus (click on it)
- Check if commands are being published: `ros2 topic echo /cmd_vel`
- Note: Teleop publishes `/cmd_vel` - your robot must subscribe to this topic

### Wrong LiDAR Port

If your LiDAR is on a different port:

```bash
# Launch with custom port
ros2 launch lidar_slam_simple slam.launch.py lidar_port:=/dev/ttyUSB1
```

### RViz Shows "No transform from [laser_link] to [map]"

- This is **normal** at startup
- Move the robot a bit to initialize SLAM
- The error will disappear once SLAM starts

## 📊 Topics Reference

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | Laser scan data from LiDAR |
| `/map` | nav_msgs/OccupancyGrid | Generated map |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands (from teleop) |

## ⚙️ Configuration

### Adjust LiDAR Settings

Edit `config/ydlidar.yaml` to match your LiDAR model:
- Change `port` if using different USB port
- Adjust `frequency`, `min_range`, `max_range` for your model

### Adjust SLAM Settings

Edit `config/slam.yaml`:
- `minimum_travel_distance: 0.2` - Decrease for slower robots
- `resolution: 0.05` - Map resolution (5cm default)
- `max_laser_range: 8.0` - Match your LiDAR's max range

## 🎓 Next Steps

Once you have a map:
- Use Nav2 for autonomous navigation
- Add more sensors (IMU, camera)
- Implement waypoint following
- Create multi-floor maps

## 📝 Notes

- **No Raspberry Pi needed** - Everything runs on laptop
- **No ESP32 needed** - Just visualizing and mapping
- **Manual robot movement** - Push robot by hand or use existing motor control
- **Simple setup** - One launch file, one command

## 🆘 Still Having Issues?

Check these:
1. ROS2 Humble is installed: `ros2 --version`
2. YD LiDAR driver is built: `ros2 pkg list | grep ydlidar`
3. SLAM Toolbox is installed: `ros2 pkg list | grep slam_toolbox`
4. LiDAR is spinning (motor should be running)
5. USB cable is good quality (data + power)

## License

Apache-2.0

---

**Happy Mapping! 🗺️**
