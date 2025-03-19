#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
import time
import numpy as np

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
        # Robot state
        self.wheel_pos = 0.0
        self.wheel_vel = 0.0
        self.steering_angle = 0.0
        self.target_steering_angle = 0.0
        
        # Robot parameters
        self.wheel_radius = 0.05  # From URDF
        self.wheel_separation = 0.15  # Approximate value based on URDF
        self.max_steering_angle = 0.5  # From URDF limits
        
        # Current velocity command
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        self.get_logger().info('Joint state publisher for SLAM initialized')

    def cmd_vel_callback(self, msg):
        # Update velocity commands
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        
        # Calculate target steering angle based on angular velocity
        # Simple Ackermann steering approximation
        if abs(self.linear_x) > 0.001:  # Only steer when moving
            # Convert angular velocity to steering angle (simplified)
            self.target_steering_angle = np.clip(
                self.angular_z * 0.5,  # Scale factor for responsiveness
                -self.max_steering_angle, 
                self.max_steering_angle
            )
        else:
            # When not moving forward, can still set steering position
            self.target_steering_angle = np.clip(
                self.angular_z * 0.2,
                -self.max_steering_angle,
                self.max_steering_angle
            )
            
        self.get_logger().debug(f'Cmd vel: linear={self.linear_x}, angular={self.angular_z}, target_steering={self.target_steering_angle}')

    def timer_callback(self):
        # Update wheel position based on velocity
        wheel_vel = self.linear_x / self.wheel_radius  # w = v/r
        self.wheel_pos += wheel_vel * 0.05  # dt = 0.05s
        
        # Gradually update steering angle (smooth transition)
        if abs(self.target_steering_angle - self.steering_angle) > 0.01:
            # Move steering angle toward target at a fixed rate
            direction = 1 if self.target_steering_angle > self.steering_angle else -1
            self.steering_angle += direction * 0.05  # 0.05 rad per update
            self.steering_angle = np.clip(self.steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        # Create message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Add all joint names and positions
        msg.name = [
            'front_left_wheel_joint', 
            'front_right_wheel_joint', 
            'left_rear_wheel_joint', 
            'right_rear_wheel_joint', 
            'steering_control_joint', 
            'front_left_pivot_joint', 
            'front_right_pivot_joint'
        ]
        
        # Set joint positions
        # Wheels rotate based on velocity, steering based on angular command
        msg.position = [
            self.wheel_pos,  # front_left_wheel
            self.wheel_pos,  # front_right_wheel
            self.wheel_pos,  # left_rear_wheel
            self.wheel_pos,  # right_rear_wheel
            self.steering_angle,  # steering_control
            self.steering_angle,  # front_left_pivot (mimic joint)
            self.steering_angle   # front_right_pivot (mimic joint)
        ]
        
        # Set velocities (optional)
        wheel_vels = [wheel_vel] * 4 + [0.0] * 3
        msg.velocity = wheel_vels
        msg.effort = [0.0] * 7
        
        # Publish message
        self.publisher.publish(msg)

def main():
    print('Starting joint state publisher for robot SLAM...')
    rclpy.init()
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 