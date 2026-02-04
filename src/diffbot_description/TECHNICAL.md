# Differential Drive Robot - Technical Overview

## Robot Structure Diagram

```
                    [laser_link]
                         |
    ┌────────────────────┼────────────────────┐
    │                    │                    │
    │              [base_link]                │
    │           (Main Body Frame)             │
    │                                         │
    └─────────────────────────────────────────┘
              |                    |
         [left_wheel]         [right_wheel]
         (continuous)         (continuous)
              
              [caster_wheel]
              (front support)
                   
         [base_footprint]
         (ground projection)
```

## TF Tree Structure

```
base_footprint (root)
└── base_link
    ├── left_wheel
    ├── right_wheel
    ├── caster_wheel
    └── laser_link
```

## Joint Types

| Joint Name | Type | Parent | Child | Axis |
|------------|------|--------|-------|------|
| base_footprint_joint | fixed | base_footprint | base_link | - |
| left_wheel_joint | continuous | base_link | left_wheel | Y (0 1 0) |
| right_wheel_joint | continuous | base_link | right_wheel | Y (0 1 0) |
| caster_wheel_joint | fixed | base_link | caster_wheel | - |
| laser_joint | fixed | base_link | laser_link | - |

## Physical Parameters

### Base Link
- **Geometry**: Box (0.6m × 0.4m × 0.2m)
- **Mass**: 10.0 kg
- **Color**: Blue (RGBA: 0.2, 0.2, 0.8, 1.0)
- **Position**: 0.1m above base_footprint

### Left & Right Wheels
- **Geometry**: Cylinder (radius: 0.1m, length: 0.05m)
- **Mass**: 1.0 kg each
- **Color**: Black (RGBA: 0.1, 0.1, 0.1, 1.0)
- **Separation**: 0.45m (center to center)
- **Position**: ±0.225m from base_link center

### Caster Wheel
- **Geometry**: Sphere (radius: 0.05m)
- **Mass**: 0.5 kg
- **Color**: Gray (RGBA: 0.5, 0.5, 0.5, 1.0)
- **Position**: 0.25m forward, 0.05m below base_link

### Laser Scanner
- **Geometry**: Cylinder (radius: 0.05m, height: 0.04m)
- **Mass**: 0.2 kg
- **Color**: Red (RGBA: 0.8, 0.2, 0.2, 1.0)
- **Position**: 0.2m forward, 0.12m above base_link

## Differential Drive Kinematics

The robot uses a differential drive configuration:

- **Wheel Base (L)**: 0.45m (distance between wheels)
- **Wheel Radius (r)**: 0.1m
- **Linear Velocity**: v = (v_left + v_right) / 2
- **Angular Velocity**: ω = (v_right - v_left) / L

Where:
- v_left = ω_left × r (left wheel linear velocity)
- v_right = ω_right × r (right wheel linear velocity)
- ω_left, ω_right = angular velocities of left and right wheels

## Use Cases

This robot description is suitable for:

1. **Navigation Development**: Test Nav2 algorithms
2. **SLAM**: Mapping and localization with the laser scanner
3. **Control Development**: Implement differential drive controllers
4. **Simulation**: Gazebo physics simulation (with plugins)
5. **Education**: Learn ROS 2 robot description and URDF

## Adding Gazebo Support

To use this robot in Gazebo, you'll need to add:

1. **Gazebo plugins** in the URDF:
   - `libgazebo_ros_diff_drive.so` for wheel control
   - `libgazebo_ros_ray_sensor.so` for laser scanner
   - `libgazebo_ros_joint_state_publisher.so` for joint states

2. **Gazebo-specific properties**:
   - Friction coefficients for wheels
   - Sensor configurations
   - Material properties

3. **Launch file** for Gazebo simulation

## Next Development Steps

1. **Add Gazebo plugins** for simulation
2. **Create ros2_control configuration** for hardware interface
3. **Add IMU sensor** for better odometry
4. **Implement camera** for vision-based navigation
5. **Create navigation launch files** for Nav2 integration
6. **Add URDF macros** for easier customization

## File Locations

- **URDF**: `src/diffbot_description/urdf/diffbot.urdf`
- **Launch**: `src/diffbot_description/launch/display.launch.py`
- **RViz Config**: `src/diffbot_description/rviz/diffbot.rviz`
- **Package**: `src/diffbot_description/package.xml`
- **Build**: `src/diffbot_description/CMakeLists.txt`
