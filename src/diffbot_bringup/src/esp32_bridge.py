#!/usr/bin/env python3
"""
ESP32 Serial Bridge Node
Handles bidirectional communication between ROS2 and ESP32 via serial port.
- Subscribes to velocity commands from diff_drive_controller
- Publishes IMU data from ESP32
- Sends motor commands to ESP32
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Imu
import serial
import json
import struct
from threading import Thread


class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        
        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=timeout
            )
            self.get_logger().info(f'Connected to ESP32 on {serial_port} at {baud_rate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            self.serial_conn = None
        
        # Subscribe to velocity commands from controller
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            '/diff_drive_controller/cmd_vel_out',
            self.cmd_vel_callback,
            10
        )
        
        # Publish IMU data
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Start serial reading thread
        if self.serial_conn:
            self.running = True
            self.read_thread = Thread(target=self.read_serial_loop, daemon=True)
            self.read_thread.start()
        
        self.get_logger().info('ESP32 Bridge node started')
    
    def cmd_vel_callback(self, msg):
        """
        Receive velocity commands and send to ESP32
        Format: JSON with linear and angular velocities
        Example: {"linear": 0.5, "angular": 0.2}
        """
        if not self.serial_conn:
            return
        
        try:
            # Extract velocities
            linear_vel = msg.twist.linear.x
            angular_vel = msg.twist.angular.z
            
            # Create JSON message
            cmd_data = {
                'linear': round(linear_vel, 3),
                'angular': round(angular_vel, 3)
            }
            
            # Send to ESP32
            json_str = json.dumps(cmd_data) + '\n'
            self.serial_conn.write(json_str.encode('utf-8'))
            
            self.get_logger().debug(f'Sent to ESP32: {json_str.strip()}')
            
        except Exception as e:
            self.get_logger().error(f'Error sending velocity command: {e}')
    
    def read_serial_loop(self):
        """
        Continuously read from serial port and publish IMU data
        Expected format from ESP32: JSON with IMU data
        Example: {"ax": 0.1, "ay": 0.0, "az": 9.8, "gx": 0.0, "gy": 0.0, "gz": 0.0}
        """
        while self.running and rclpy.ok():
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    
                    if line:
                        # Parse JSON data from ESP32
                        data = json.loads(line)
                        
                        # Create IMU message
                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = 'imu_link'
                        
                        # Linear acceleration (m/s^2)
                        imu_msg.linear_acceleration.x = data.get('ax', 0.0)
                        imu_msg.linear_acceleration.y = data.get('ay', 0.0)
                        imu_msg.linear_acceleration.z = data.get('az', 0.0)
                        
                        # Angular velocity (rad/s)
                        imu_msg.angular_velocity.x = data.get('gx', 0.0)
                        imu_msg.angular_velocity.y = data.get('gy', 0.0)
                        imu_msg.angular_velocity.z = data.get('gz', 0.0)
                        
                        # Set covariance (adjust based on your sensor)
                        imu_msg.linear_acceleration_covariance = [
                            0.01, 0.0, 0.0,
                            0.0, 0.01, 0.0,
                            0.0, 0.0, 0.01
                        ]
                        imu_msg.angular_velocity_covariance = [
                            0.01, 0.0, 0.0,
                            0.0, 0.01, 0.0,
                            0.0, 0.0, 0.01
                        ]
                        
                        # Publish IMU data
                        self.imu_pub.publish(imu_msg)
                        
                        self.get_logger().debug(f'Published IMU data: ax={data.get("ax")}, gz={data.get("gz")}')
                        
            except json.JSONDecodeError:
                self.get_logger().warn(f'Invalid JSON from ESP32: {line}')
            except Exception as e:
                self.get_logger().error(f'Error reading serial: {e}')
    
    def destroy_node(self):
        """Clean up on shutdown"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
