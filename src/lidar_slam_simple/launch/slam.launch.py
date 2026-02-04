#!/usr/bin/env python3
"""
Simple LiDAR SLAM Launch File
Connects YD LiDAR to laptop, runs SLAM, and visualizes in RViz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for YD LiDAR'
        )
    )
    
    # Initialize arguments
    lidar_port = LaunchConfiguration('lidar_port')
    
    # YD LiDAR configuration
    ydlidar_config = PathJoinSubstitution([
        FindPackageShare('lidar_slam_simple'),
        'config',
        'ydlidar.yaml'
    ])
    
    # SLAM configuration
    slam_config = PathJoinSubstitution([
        FindPackageShare('lidar_slam_simple'),
        'config',
        'slam.yaml'
    ])
    
    # RViz configuration
    rviz_config = PathJoinSubstitution([
        FindPackageShare('lidar_slam_simple'),
        'rviz',
        'slam.rviz'
    ])
    
    # YD LiDAR node
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_node',
        output='screen',
        parameters=[
            ydlidar_config,
            {'port': lidar_port}
        ]
    )
    
    # Static transform: base_link -> laser_link
    # Adjust if your LiDAR is mounted at a different position
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link']
    )
    
    # SLAM Toolbox node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config]
    )
    
    # Teleop keyboard node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e'
    )
    
    nodes = [
        ydlidar_node,
        base_to_laser_tf,
        slam_node,
        rviz_node,
        teleop_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
