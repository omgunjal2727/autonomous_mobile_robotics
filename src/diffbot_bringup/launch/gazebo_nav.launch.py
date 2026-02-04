#!/usr/bin/env python3
"""
Gazebo Navigation Launch File
Launches Gazebo simulation with Nav2 for autonomous navigation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load'
        )
    )
    
    # Initialize arguments
    map_yaml_file = LaunchConfiguration('map')
    
    # Include Gazebo simulation
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('diffbot_bringup'),
                'launch',
                'gazebo_sim.launch.py'
            ])
        ])
    )
    
    # Nav2 parameters
    nav2_params = PathJoinSubstitution([
        FindPackageShare('diffbot_bringup'),
        'config',
        'nav2_params.yaml'
    ])
    
    # Nav2 bringup launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items()
    )
    
    return LaunchDescription(declared_arguments + [gazebo_sim, nav2_bringup_launch])
