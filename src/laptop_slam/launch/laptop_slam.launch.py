#!/usr/bin/env python3
"""
Laptop SLAM Launch File
Launches SLAM Toolbox, RViz, and robot state publisher for mapping
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('laptop_slam'),
                'config',
                'slam_params.yaml'
            ]),
            description='Full path to SLAM parameters file'
        )
    )
    
    # Initialize arguments
    use_rviz = LaunchConfiguration('use_rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # Robot description
    urdf_file = PathJoinSubstitution([
        FindPackageShare('laptop_slam'),
        'urdf',
        'robot.urdf'
    ])
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['cat ', urdf_file]),
            'use_sim_time': False
        }]
    )
    
    # SLAM Toolbox node (online async)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
        remappings=[
            ('/scan', '/scan'),
        ]
    )
    
    # RViz configuration
    rviz_config = PathJoinSubstitution([
        FindPackageShare('laptop_slam'),
        'rviz',
        'slam_config.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz)
    )
    
    nodes = [
        robot_state_publisher_node,
        slam_node,
        rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
