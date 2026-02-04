#!/usr/bin/env python3
"""
Main bringup launch file for the differential drive robot.
This file brings up all the core components needed to run the robot.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='true',
            description='Use mock hardware interface (for testing without real robot)'
        )
    )

    # Initialize arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')

    # Include the control launch file
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('diffbot_control'),
                'launch',
                'diffbot_control.launch.py'
            ])
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_mock_hardware': use_mock_hardware,
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)
    
    # Add the control system
    ld.add_action(control_launch)
    
    return ld
