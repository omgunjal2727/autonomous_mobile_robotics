#!/usr/bin/env python3
"""
Launch file for visualizing the robot in RViz without controllers.
Useful for checking the robot description and URDF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    # Initialize arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include the display launch file from diffbot_description
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('diffbot_description'),
                'launch',
                'display.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Create the launch description
    ld = LaunchDescription(declared_arguments)
    ld.add_action(display_launch)
    
    return ld
