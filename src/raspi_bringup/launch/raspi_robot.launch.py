#!/usr/bin/env python3
"""
Raspberry Pi Robot Launch File
Launches YD LiDAR and ESP32 motor bridge for robot operation
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
            description='Serial port for YD LiDAR (use "ls /dev/ttyUSB*" to find)'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'esp32_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for ESP32 (use "ls /dev/ttyUSB*" to find)'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'lidar_baud',
            default_value='115200',
            description='Baud rate for YD LiDAR'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'esp32_baud',
            default_value='115200',
            description='Baud rate for ESP32'
        )
    )
    
    # Initialize arguments
    lidar_port = LaunchConfiguration('lidar_port')
    esp32_port = LaunchConfiguration('esp32_port')
    lidar_baud = LaunchConfiguration('lidar_baud')
    esp32_baud = LaunchConfiguration('esp32_baud')
    
    # YD LiDAR configuration file
    ydlidar_config = PathJoinSubstitution([
        FindPackageShare('raspi_bringup'),
        'config',
        'ydlidar.yaml'
    ])
    
    # YD LiDAR node
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_node',
        output='screen',
        parameters=[
            ydlidar_config,
            {'port': lidar_port, 'baudrate': lidar_baud}
        ]
    )
    
    # ESP32 Motor Bridge node
    esp32_bridge_node = Node(
        package='raspi_bringup',
        executable='esp32_motor_bridge.py',
        name='esp32_motor_bridge',
        output='screen',
        parameters=[{
            'serial_port': esp32_port,
            'baud_rate': esp32_baud,
            'timeout': 1.0,
            'cmd_vel_timeout': 0.5
        }]
    )

    # Static transform: base_link -> laser_link
    # Enables SLAM to know where the LiDAR is on the robot
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link']
    )
    
    nodes = [
        ydlidar_node,
        esp32_bridge_node,
        static_tf_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
