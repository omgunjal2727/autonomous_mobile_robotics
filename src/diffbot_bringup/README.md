# Diffbot Bringup

Central launch package for the differential drive robot. This package provides convenient launch files to bring up the entire robot system.

## Launch Files

### diffbot_bringup.launch.py
Main launch file that starts the complete robot system including:
- ros2_control node with mock/real hardware
- Differential drive controller
- Joint state broadcaster
- Robot state publisher
- Optional RViz visualization

**Usage:**
```bash
ros2 launch diffbot_bringup diffbot_bringup.launch.py
```

**Arguments:**
- `use_rviz:=true/false` - Launch RViz (default: true)
- `use_mock_hardware:=true/false` - Use mock hardware (default: true)
- `use_sim_time:=true/false` - Use simulation time (default: false)

### view_robot.launch.py
Visualization-only launch for viewing the robot in RViz without controllers.

**Usage:**
```bash
ros2 launch diffbot_bringup view_robot.launch.py
```

### teleop.launch.py
Keyboard teleoperation for controlling the robot.

**Usage:**
```bash
ros2 launch diffbot_bringup teleop.launch.py
```

## Directory Structure

```
diffbot_bringup/
├── launch/          # Launch files
├── config/          # Configuration files (add your configs here)
├── params/          # Parameter files (add your params here)
└── README.md
```

## Adding New Components

This package is designed to be extended. To add new components:

1. Add new launch files to `launch/`
2. Add configuration files to `config/`
3. Add parameter files to `params/`
4. Update dependencies in `package.xml`

## Examples

```bash
# Launch with RViz
ros2 launch diffbot_bringup diffbot_bringup.launch.py

# Launch without RViz
ros2 launch diffbot_bringup diffbot_bringup.launch.py use_rviz:=false

# View robot only
ros2 launch diffbot_bringup view_robot.launch.py

# Keyboard control
ros2 launch diffbot_bringup teleop.launch.py
```
