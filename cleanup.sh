#!/bin/bash

echo "Cleaning up old ROS processes..."

# Kill all diffbot and controller processes
pkill -f "diffbot_bringup"
pkill -f "ros2_control_node"
pkill -f "controller_manager"
pkill -f "robot_state_publisher"
pkill -f "rviz2"
pkill -f "teleop_twist_keyboard"

sleep 2

echo "✓ Cleanup complete"
echo ""
echo "Now run:"
echo "  ros2 launch diffbot_bringup diffbot_bringup.launch.py"
