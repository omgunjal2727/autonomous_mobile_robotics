# Laptop SLAM Setup Guide

This package runs on your laptop and provides SLAM mapping and visualization for the robot.

## Prerequisites

### Hardware
- Laptop with Ubuntu 22.04
- Network connection to Raspberry Pi (WiFi or Ethernet)

### Software
- ROS2 Humble installed
- SSH access to Raspberry Pi

## Installation

### 1. Install ROS2 Humble

If not already installed:

```bash
# Add ROS2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop (includes RViz)
sudo apt update
sudo apt install -y ros-humble-desktop

# Install additional packages
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-nav2-map-server
sudo apt install -y ros-humble-teleop-twist-keyboard
sudo apt install -y xterm  # For teleop in separate window
```

### 2. Build This Package

```bash
cd ~/ros2_ws
colcon build --packages-select laptop_slam
source install/setup.bash
```

## Network Configuration

### 1. Set ROS_DOMAIN_ID

**CRITICAL**: Must match the Raspberry Pi's ROS_DOMAIN_ID

```bash
# Add to ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

### 2. Verify Network Connection

```bash
# Ping Raspberry Pi (replace with your Pi's IP)
ping RASPBERRY_PI_IP

# Check if you can see topics from Raspberry Pi
ros2 topic list

# You should see /scan if Raspberry Pi is running
```

### 3. (Optional) Configure ROS2 DDS

For better network performance, you can configure FastDDS:

```bash
# Create FastDDS config
mkdir -p ~/.ros
cat > ~/.ros/fastdds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomUDPTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>CustomUDPTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF

# Set environment variable
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml" >> ~/.bashrc
source ~/.bashrc
```

## Usage

### Quick Start

**Terminal 1 - SLAM & Visualization:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch laptop_slam laptop_slam.launch.py
```

This will:
- Start SLAM Toolbox for mapping
- Open RViz with map visualization
- Display robot model and laser scans

**Terminal 2 - Teleop Control:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch laptop_slam teleop.launch.py
```

This opens a separate window for keyboard control:
- `i` - Move forward
- `j` - Turn left
- `k` - Stop
- `l` - Turn right
- `,` - Move backward
- `u/o/m/.` - Diagonal movements
- `q/z` - Increase/decrease max speeds

### Saving the Map

Once you've finished mapping:

```bash
# Save the map to a file
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# This creates two files:
# - my_map.pgm (the map image)
# - my_map.yaml (map metadata)
```

### Loading a Saved Map

To use SLAM Toolbox in localization mode with a saved map:

1. Edit `config/slam_params.yaml`:
   ```yaml
   mode: localization  # Change from 'mapping'
   map_file_name: /home/YOUR_USERNAME/maps/my_map
   map_start_pose: [0.0, 0.0, 0.0]  # [x, y, yaw]
   ```

2. Launch as normal:
   ```bash
   ros2 launch laptop_slam laptop_slam.launch.py
   ```

## SSH Workflow

### Option 1: Run Everything via SSH

```bash
# SSH into Raspberry Pi
ssh pi@RASPBERRY_PI_IP

# On Raspberry Pi: Start robot nodes
cd ~/ros2_ws
source install/setup.bash
ros2 launch raspi_bringup raspi_robot.launch.py
```

Then on your laptop, run SLAM and teleop as shown above.

### Option 2: X11 Forwarding (Not Recommended)

You can forward RViz over SSH, but it will be slow:

```bash
# SSH with X11 forwarding
ssh -X pi@RASPBERRY_PI_IP

# Run RViz remotely (will be laggy)
ros2 launch laptop_slam laptop_slam.launch.py
```

**Recommended**: Run robot nodes on Raspberry Pi, SLAM/RViz on laptop.

## Troubleshooting

### Can't See /scan Topic

```bash
# Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # Should be same on both devices

# Check network connectivity
ping RASPBERRY_PI_IP

# List all topics
ros2 topic list

# Check topic info
ros2 topic info /scan
```

### SLAM Not Building Map

```bash
# Check if scan data is being received
ros2 topic hz /scan
ros2 topic echo /scan --once

# Check TF tree
ros2 run tf2_tools view_frames
# Opens frames.pdf showing TF tree

# Verify SLAM Toolbox is running
ros2 node list | grep slam
```

### RViz Shows "No transform from [laser_link] to [map]"

This is normal at startup. Drive the robot a bit to initialize SLAM.

```bash
# Publish a test velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once
```

### Teleop Not Controlling Robot

```bash
# Check if cmd_vel is being published
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel

# Check if ESP32 bridge on Raspberry Pi is receiving commands
# (check Raspberry Pi terminal logs)
```

### Map Quality Issues

Adjust SLAM parameters in `config/slam_params.yaml`:

- **Map too sparse**: Decrease `minimum_travel_distance` (e.g., 0.1)
- **Map too dense**: Increase `minimum_travel_distance` (e.g., 0.3)
- **Poor loop closure**: Increase `loop_search_maximum_distance`
- **Slow performance**: Increase `throttle_scans` or `map_update_interval`

## Advanced Usage

### Custom SLAM Parameters

```bash
# Launch with custom parameters file
ros2 launch laptop_slam laptop_slam.launch.py slam_params_file:=/path/to/custom_params.yaml
```

### Without RViz

```bash
# Launch SLAM only (no visualization)
ros2 launch laptop_slam laptop_slam.launch.py use_rviz:=false
```

### Monitor Topics

```bash
# Terminal 1: Monitor scan rate
ros2 topic hz /scan

# Terminal 2: Monitor map updates
ros2 topic hz /map

# Terminal 3: Monitor velocity commands
ros2 topic echo /cmd_vel
```

## Tips for Good Maps

1. **Drive Slowly**: Move at ~0.2 m/s for best results
2. **Overlap Scans**: Revisit areas for loop closure
3. **Avoid Dynamic Objects**: People, moving furniture, etc.
4. **Good Lighting**: Ensure LiDAR has clear line of sight
5. **Close Loops**: Return to starting position to improve map accuracy

## Next Steps

- Use the map for autonomous navigation with Nav2
- Add more sensors (camera, IMU) for better localization
- Implement waypoint navigation
- Create multi-floor maps

## License

Apache-2.0
