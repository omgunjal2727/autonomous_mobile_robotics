# Differential Drive Robot Description

This package contains the URDF description for a simple differential drive robot.

## Robot Description

The robot consists of:
- **Base Link**: A rectangular base (0.6m x 0.4m x 0.2m)
- **Two Driven Wheels**: Left and right wheels (0.1m radius, 0.05m width)
  - Wheel separation: 0.45m
  - Continuous joints for differential drive control
- **Caster Wheel**: Front spherical caster (0.05m radius) for stability
- **Laser Scanner**: Mounted on top of the base for navigation

## Package Structure

```
diffbot_description/
├── urdf/
│   └── diffbot.urdf          # Main robot URDF file
├── launch/
│   └── display.launch.py     # Launch file for RViz visualization
├── rviz/
│   └── diffbot.rviz          # RViz configuration
├── meshes/                   # (Empty - for future custom meshes)
├── CMakeLists.txt
└── package.xml
```

## Usage

### 1. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select diffbot_description
source install/setup.bash
```

### 2. Visualize in RViz

Launch the robot visualization:

```bash
ros2 launch diffbot_description display.launch.py
```

This will:
- Start the `robot_state_publisher` to publish the robot's TF tree
- Launch `joint_state_publisher_gui` to control the wheel joints
- Open RViz2 with a pre-configured view

### 3. Check the Robot Description

View the robot description topic:

```bash
ros2 topic echo /robot_description
```

### 4. Visualize TF Tree

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

## Robot Specifications

### Dimensions
- Base: 0.6m (length) x 0.4m (width) x 0.2m (height)
- Wheel radius: 0.1m
- Wheel width: 0.05m
- Wheel separation: 0.45m
- Caster radius: 0.05m

### Mass Properties
- Base mass: 10.0 kg
- Wheel mass: 1.0 kg each
- Caster mass: 0.5 kg
- Laser mass: 0.2 kg

### Coordinate Frames
- `base_footprint`: Ground projection of the robot (for navigation)
- `base_link`: Main robot body frame
- `left_wheel`: Left wheel frame
- `right_wheel`: Right wheel frame
- `caster_wheel`: Front caster frame
- `laser_link`: Laser scanner frame

## Next Steps

This robot description can be used with:
- **Gazebo**: For physics simulation (requires adding Gazebo plugins)
- **Nav2**: For autonomous navigation
- **ros2_control**: For hardware interface and controller management
- **Custom controllers**: For differential drive control

## Customization

To modify the robot:
1. Edit `urdf/diffbot.urdf` to change dimensions, add sensors, or modify links
2. Update `rviz/diffbot.rviz` if you add new visualization elements
3. Rebuild the package: `colcon build --packages-select diffbot_description`

## License

Apache-2.0
