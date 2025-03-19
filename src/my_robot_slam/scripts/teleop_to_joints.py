#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Set up QoS profiles
        latching_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )
        
        # Publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # TF broadcaster for robot position
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing joint states and updating position
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
        # Robot state
        self.wheel_pos = 0.0
        self.steering_angle = 0.0
        self.target_steering_angle = 0.0
        
        # Robot position (x, y, theta) in map frame
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Robot parameters from URDF
        self.wheel_radius = 0.05
        self.wheel_base = 0.22  # Distance between front and rear axles
        self.track_width = 0.15  # Distance between left and right wheels
        self.max_steering_angle = 0.5  # radians
        
        # Velocity commands
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # Speed and steering control factors
        self.speed_scale = 0.3       # Scale factor to reduce linear velocity (was implicitly 1.0)
        self.steering_response = 1.2  # For steering angle calculation
        self.steering_rate = 0.1      # For steering angle updates
        
        self.get_logger().info('Robot controller initialized with reduced speed')
    
    def cmd_vel_callback(self, msg):
        # Store velocity commands with speed scaling
        self.linear_x = msg.linear.x * self.speed_scale  # Apply speed reduction
        
        # Fix: Invert angular_z for correct steering direction
        # In ROS, positive angular.z is counterclockwise (left), but our steering might need adjustment
        self.angular_z = -msg.angular.z  # Inverted to fix the steering direction
        
        # Calculate target steering angle from angular velocity
        if abs(self.linear_x) > 0.01:
            # Use angular velocity to determine steering angle
            # Simple approximation for Ackermann steering
            self.target_steering_angle = np.clip(
                self.angular_z * self.steering_response,
                -self.max_steering_angle,
                self.max_steering_angle
            )
        else:
            # When not moving, we can still set steering position with higher response
            self.target_steering_angle = np.clip(
                self.angular_z * 0.5,
                -self.max_steering_angle,
                self.max_steering_angle
            )
            
        self.get_logger().debug(f'CMD: lin={self.linear_x:.2f}, ang={self.angular_z:.2f}, target_steer={self.target_steering_angle:.2f}')
    
    def update_robot_position(self, dt):
        """Update robot position based on kinematics model"""
        if abs(self.linear_x) < 0.001:
            return  # No movement
        
        # Simple bicycle model (Ackermann steering approximation)
        v = self.linear_x
        if abs(self.steering_angle) > 0.001:
            # Calculate turning radius based on steering angle
            # Add a small offset to prevent division by zero
            steering_angle = self.steering_angle if abs(self.steering_angle) > 0.01 else 0.01
            turning_radius = self.wheel_base / math.tan(abs(steering_angle)) * (1.0 if steering_angle >= 0 else -1.0)
            
            # Calculate change in orientation (fix sign for correct turning direction)
            delta_theta = (v * dt) / turning_radius
            
            # Update position based on circular motion
            self.theta += delta_theta
            self.x += turning_radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y += turning_radius * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
        else:
            # Straight line motion
            self.x += v * dt * math.cos(self.theta)
            self.y += v * dt * math.sin(self.theta)
    
    def publish_robot_transform(self):
        """Publish the robot's position as a TF transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'
        
        # Set transform
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert theta to quaternion (rotation around Z axis)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)
    
    def publish_joint_states(self):
        """Publish joint states for visualization"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Include all joint names from the URDF
        msg.name = [
            'front_left_wheel_joint', 
            'front_right_wheel_joint', 
            'left_rear_wheel_joint', 
            'right_rear_wheel_joint', 
            'steering_control_joint', 
            'front_left_pivot_joint', 
            'front_right_pivot_joint'
        ]
        
        # Calculate wheel rotation based on distance traveled
        wheel_rotation = self.wheel_pos
        
        # Set all joint positions
        msg.position = [
            wheel_rotation,  # front_left_wheel
            wheel_rotation,  # front_right_wheel
            wheel_rotation,  # left_rear_wheel
            wheel_rotation,  # right_rear_wheel
            self.steering_angle,  # steering_control
            self.steering_angle,  # front_left_pivot (mimic joint)
            self.steering_angle   # front_right_pivot (mimic joint)
        ]
        
        # Set velocities
        wheel_vel = self.linear_x / self.wheel_radius if self.wheel_radius != 0 else 0
        msg.velocity = [wheel_vel] * 4 + [0.0] * 3
        msg.effort = [0.0] * 7
        
        # Publish message
        self.joint_state_pub.publish(msg)
    
    def timer_callback(self):
        # Update time
        dt = 0.05  # seconds (fixed time step)
        
        # Update wheel position based on linear velocity
        self.wheel_pos += (self.linear_x / self.wheel_radius) * dt if self.wheel_radius != 0 else 0
        
        # Update steering angle (increased rate for smoother, faster response)
        if abs(self.target_steering_angle - self.steering_angle) > 0.01:
            direction = 1 if self.target_steering_angle > self.steering_angle else -1
            self.steering_angle += direction * self.steering_rate  # Increased from 0.05
            self.steering_angle = np.clip(self.steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        # Update robot position in the world
        self.update_robot_position(dt)
        
        # Publish transform for robot position
        self.publish_robot_transform()
        
        # Publish joint states
        self.publish_joint_states()
        
        # Debug info
        if abs(self.linear_x) > 0.001 or abs(self.angular_z) > 0.001:
            self.get_logger().info(
                f'x:{self.x:.2f}, y:{self.y:.2f}, θ:{self.theta:.2f}, '
                f'steering:{self.steering_angle:.2f}, vel:{self.linear_x:.2f}'
            )

def main():
    print('Starting robot controller for terrain navigation with reduced speed...')
    rclpy.init()
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 