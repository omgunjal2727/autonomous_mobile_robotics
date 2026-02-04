# Differential Drive Robot Control

This package provides ros2_control configuration and controllers for the differential drive robot.

## Overview

This package implements:
- **ros2_control** hardware interface with mock components
- **Differential Drive Controller** for velocity-based control
- **Joint State Broadcaster** for publishing joint states
- Complete controller configuration with velocity limits and safety features

## Package Structure

```
diffbot_control/
├── config/
│   ├── diffbot_control.urdf       # URDF with ros2_control tags
│   └── diffbot_controllers.yaml   # Controller parameters
├── launch/
│   └── diffbot_control.launch.py  # Main launch file
├── CMakeLists.txt
└── package.xml
```

## Controllers

### 1. Differential Drive Controller
- **Type**: `diff_drive_controller/DiffDriveController`
- **Subscribes to**: `/cmd_vel` (geometry_msgs/Twist)
- **Publishes**: 
  - `/odom` (nav_msgs/Odometry)
  - `/tf` (odom → base_footprint transform)
- **Controls**: Left and right wheel velocities

### 2. Joint State Broadcaster
- **Type**: `joint_state_broadcaster/JointStateBroadcaster`
- **Publishes**: `/joint_states` (sensor_msgs/JointState)

## Configuration Parameters

### Robot Dimensions
- **Wheel separation**: 0.45 m
- **Wheel radius**: 0.1 m

### Velocity Limits
- **Linear velocity**: ±1.0 m/s
- **Angular velocity**: ±2.0 rad/s
- **Linear acceleration**: ±1.0 m/s²
- **Angular acceleration**: ±1.5 rad/s²

### Safety Features
- **Command timeout**: 0.5 seconds (robot stops if no command received)
- **Velocity limiting**: Enforces max velocity and acceleration
- **Preserve turning radius**: Maintains accurate turning geometry

## Usage

### 1. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select diffbot_control diffbot_description
source install/setup.bash
```

### 2. Launch the Control System

```bash
# Launch with RViz (default)
ros2 launch diffbot_control diffbot_control.launch.py

# Launch without RViz
ros2 launch diffbot_control diffbot_control.launch.py use_rviz:=false
```

This will:
- Start the `ros2_control_node` with mock hardware
- Spawn the joint state broadcaster
- Spawn the differential drive controller
- Launch robot_state_publisher for TF
- Optionally open RViz2

### 3. Control the Robot

Send velocity commands to move the robot:

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Move in a circle
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Or use keyboard teleop:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 4. Monitor the System

Check controller status:
```bash
ros2 control list_controllers
```

View odometry:
```bash
ros2 topic echo /odom
```

View joint states:
```bash
ros2 topic echo /joint_states
```

Check TF tree:
```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

## Hardware Interface

The package currently uses **mock_components/GenericSystem** for simulation/testing. This mirrors commanded velocities to the state interfaces, allowing you to test the control system without real hardware.

### Switching to Real Hardware

To use with real hardware:

1. **Create a hardware plugin** (e.g., `diffbot_hardware/DiffBotHardware`)
2. **Update the URDF** (`config/diffbot_control.urdf`):
   ```xml
   <hardware>
     <plugin>diffbot_hardware/DiffBotHardware</plugin>
     <param name="serial_port">/dev/ttyUSB0</param>
     <!-- Add your hardware-specific parameters -->
   </hardware>
   ```
3. **Implement the hardware interface** following the ros2_control hardware interface API

## Topics

### Subscribed
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

### Published
- `/odom` (nav_msgs/Odometry) - Robot odometry
- `/joint_states` (sensor_msgs/JointState) - Joint positions and velocities
- `/tf` - Transform tree (odom → base_footprint → base_link → wheels)

## TF Frames

```
odom (published by diff_drive_controller)
└── base_footprint
    └── base_link (published by robot_state_publisher)
        ├── left_wheel
        ├── right_wheel
        ├── caster_wheel
        └── laser_link
```

## Integration with Navigation

This control package is ready for Nav2 integration:

1. The differential drive controller publishes odometry on `/odom`
2. TF transforms are properly configured
3. Velocity limits are enforced
4. The `/cmd_vel` topic is standard for Nav2

To use with Nav2, you'll need to:
- Configure Nav2 parameters
- Set up costmaps
- Configure planners and controllers
- Add sensor data (laser scans, etc.)

## Customization

### Modify Controller Parameters

Edit [`config/diffbot_controllers.yaml`](file:///home/omg/ros2_ws/src/diffbot_control/config/diffbot_controllers.yaml):
- Adjust velocity/acceleration limits
- Change wheel separation or radius
- Modify odometry covariance
- Tune controller behavior

### Add More Controllers

You can add additional controllers (e.g., for a manipulator):

1. Add controller configuration to `diffbot_controllers.yaml`
2. Add spawner nodes in the launch file
3. Update the URDF with additional joints/interfaces

## Troubleshooting

### Controllers not loading
```bash
# Check controller manager
ros2 control list_controllers

# Check hardware interface
ros2 control list_hardware_interfaces
```

### Robot not moving
- Verify controllers are active: `ros2 control list_controllers`
- Check if commands are being received: `ros2 topic echo /cmd_vel`
- Verify joint states are published: `ros2 topic echo /joint_states`

### TF errors
- Ensure all nodes are running
- Check TF tree: `ros2 run rqt_tf_tree rqt_tf_tree`
- Verify `enable_odom_tf: true` in controller config

## Dependencies

- `hardware_interface` - ros2_control hardware interface
- `controller_manager` - Controller lifecycle management
- `diff_drive_controller` - Differential drive controller
- `joint_state_broadcaster` - Joint state publishing
- `mock_components` - Mock hardware for testing
- `robot_state_publisher` - TF publishing
- `diffbot_description` - Robot URDF

## License

Apache-2.0
