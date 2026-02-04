#!/usr/bin/env python3
"""
Sensors Launch File
Launches YDLidar X2 and ESP32 bridge for hardware integration
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
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for ESP32'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for YDLidar X2'
        )
    )
    
    # Initialize arguments
    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')
    
    # YDLidar X2 configuration file
    ydlidar_config = PathJoinSubstitution([
        FindPackageShare('diffbot_bringup'),
        'config',
        'ydlidar_x2.yaml'
    ])
    
    # YDLidar X2 node
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_node',
        output='screen',
        parameters=[ydlidar_config],
        remappings=[
            ('/scan', '/scan'),
        ]
    )
    
    # ESP32 Bridge node
    esp32_bridge_node = Node(
        package='diffbot_bringup',
        executable='esp32_bridge.py',
        name='esp32_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': 115200,
            'timeout': 1.0
        }]
    )
    
    nodes = [
        ydlidar_node,
        esp32_bridge_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
