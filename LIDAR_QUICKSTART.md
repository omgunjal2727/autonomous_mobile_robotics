# 🚀 LiDAR SLAM - Ultra Quick Start

## One Command to Rule Them All

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch lidar_slam_simple slam.launch.py
```

This starts **everything**:
- ✅ YD LiDAR
- ✅ SLAM mapping
- ✅ RViz visualization  
- ✅ Teleop keyboard

## Before First Run

```bash
# 1. Install YD LiDAR driver (one-time)
cd ~/ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ~/ros2_ws
colcon build

# 2. Give USB permission
sudo chmod 666 /dev/ttyUSB0
```

## Keyboard Controls

```
   u    i    o
   j    k    l
   m    ,    .
```

- **i** = Forward
- **,** = Backward
- **j** = Left
- **l** = Right
- **k** = Stop

## Save Map

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

## Troubleshooting

**No scan data?**
```bash
ls /dev/ttyUSB*  # Find your port
sudo chmod 666 /dev/ttyUSB0
```

**Different USB port?**
```bash
ros2 launch lidar_slam_simple slam.launch.py lidar_port:=/dev/ttyUSB1
```

**No map appearing?**
- Move the robot! Map only builds when moving
- Check red dots (laser scans) appear in RViz

---

📖 **Full Guide:** [README.md](src/lidar_slam_simple/README.md)
