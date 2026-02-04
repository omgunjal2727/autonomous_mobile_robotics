#!/usr/bin/env python3
"""
ESP32 Motor Bridge Node for Raspberry Pi
Subscribes to /cmd_vel and sends velocity commands to ESP32 via serial
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json
from threading import Lock


class ESP32MotorBridge(Node):
    def __init__(self):
        super().__init__('esp32_motor_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)  # Stop if no command for 0.5s
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        
        # Initialize serial connection
        self.serial_conn = None
        self.serial_lock = Lock()
        
        try:
            self.serial_conn = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=timeout
            )
            self.get_logger().info(f'✓ Connected to ESP32 on {serial_port} at {baud_rate} baud')
        except Exception as e:
            self.get_logger().error(f'✗ Failed to connect to ESP32: {e}')
            self.get_logger().warn('Motor control will not be available')
        
        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Safety timer - send stop command if no messages received
        self.last_cmd_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('ESP32 Motor Bridge node started')
        self.get_logger().info(f'Subscribed to: /cmd_vel')
    
    def cmd_vel_callback(self, msg):
        """
        Receive velocity commands and send to ESP32
        Format: JSON with linear and angular velocities
        Example: {"linear": 0.5, "angular": 0.2}
        """
        if not self.serial_conn:
            return
        
        # Update last command time
        self.last_cmd_time = self.get_clock().now()
        
        try:
            # Extract velocities
            linear_vel = msg.linear.x
            angular_vel = msg.angular.z
            
            # Send to ESP32
            self.send_velocity_command(linear_vel, angular_vel)
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel callback: {e}')
    
    def send_velocity_command(self, linear, angular):
        """Send velocity command to ESP32 via serial"""
        try:
            with self.serial_lock:
                # Create JSON message
                cmd_data = {
                    'linear': round(linear, 3),
                    'angular': round(angular, 3)
                }
                
                # Send to ESP32
                json_str = json.dumps(cmd_data) + '\n'
                self.serial_conn.write(json_str.encode('utf-8'))
                
                self.get_logger().debug(f'→ ESP32: {json_str.strip()}')
                
        except Exception as e:
            self.get_logger().error(f'Error sending to ESP32: {e}')
    
    def safety_check(self):
        """
        Safety check - send stop command if no velocity commands received
        for longer than cmd_vel_timeout
        """
        if not self.serial_conn:
            return
        
        time_since_last_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_last_cmd > self.cmd_vel_timeout:
            # Send stop command
            self.send_velocity_command(0.0, 0.0)
    
    def destroy_node(self):
        """Clean up on shutdown"""
        # Send stop command before closing
        if self.serial_conn:
            self.get_logger().info('Sending stop command before shutdown...')
            self.send_velocity_command(0.0, 0.0)
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32MotorBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
