#!/usr/bin/env python3
"""
Gazebo SLAM Launch File
Launches Gazebo simulation with SLAM Toolbox for mapping
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz for visualization'
        )
    )
    
    # Initialize arguments
    use_rviz = LaunchConfiguration('use_rviz')
    
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
    
    # SLAM Toolbox configuration
    slam_config = PathJoinSubstitution([
        FindPackageShare('diffbot_bringup'),
        'config',
        'slam_toolbox.yaml'
    ])
    
    # SLAM Toolbox node (online async)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': True}],
        remappings=[
            ('/scan', '/scan'),
        ]
    )
    
    # RViz configuration for SLAM
    rviz_config = PathJoinSubstitution([
        FindPackageShare('diffbot_description'),
        'rviz',
        'nav2_slam.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz)
    )
    
    nodes = [
        gazebo_sim,
        slam_node,
        rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
