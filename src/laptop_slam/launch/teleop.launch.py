#!/usr/bin/env python3
"""
Teleop Launch File
Launches keyboard teleoperation for robot control
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Topic to publish velocity commands to'
        )
    )

    # Initialize arguments
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    # Teleop twist keyboard node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # Run in separate terminal window
        remappings=[
            ('/cmd_vel', cmd_vel_topic),
        ],
    )

    # Create the launch description
    ld = LaunchDescription(declared_arguments)
    ld.add_action(teleop_node)
    
    return ld
