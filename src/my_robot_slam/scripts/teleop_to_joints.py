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
        # Extract raw velocity commands
        raw_linear_x = msg.linear.x
        raw_linear_y = msg.linear.y  # Will be non-zero for lateral movement keys
        raw_angular_z = msg.angular.z
        
        # Log raw commands for debugging
        self.get_logger().info(f'TELEOP RAW: x={raw_linear_x:.2f}, y={raw_linear_y:.2f}, rot={raw_angular_z:.2f}')
        
        # For car-like (Ackermann) vehicles:
        # 1. Handle forward/backward movement with linear.x
        # 2. Map angular.z to steering angle
        # 3. For diagonal movements, combine linear.x with angular.z
        
        # First, process forward/backward motion with speed scaling
        self.linear_x = raw_linear_x * self.speed_scale
        
        # If there's lateral movement requested, convert it to a more intuitive
        # combined forward/turning movement for car-like vehicles
        if abs(raw_linear_y) > 0.05:
            # If moving sideways is attempted, add a gentle turning component
            # while maintaining forward motion
            side_movement_dir = 1.0 if raw_linear_y > 0 else -1.0
            
            # Only apply this if we're also moving forward
            if abs(raw_linear_x) > 0.05:
                # Blend the lateral movement into angular steering
                # This makes diagonal keys (u, o, m, .) work more intuitively
                additional_steering = side_movement_dir * 0.5  # Half of max steering
                raw_angular_z = raw_angular_z + additional_steering
                
                # Show feedback about the conversion
                self.get_logger().info(f'Converting lateral y={raw_linear_y:.2f} to additional steering={additional_steering:.2f}')
            else:
                # If trying to move purely sideways, give a helpful message
                self.get_logger().warn(f"Note: Car-like robots can't move sideways directly. Use i/k for forward/back and j/l for turning.")
        
        # Now handle rotation/steering:
        # We invert the sign because positive angular.z (counter-clockwise) should map
        # to negative steering angle (left turn) for a car
        self.angular_z = -raw_angular_z
        
        # Compute target steering angle based on angular velocity command
        if abs(self.linear_x) > 0.01:
            # Active driving mode - use normal steering response
            self.target_steering_angle = np.clip(
                self.angular_z * self.steering_response,
                -self.max_steering_angle,
                self.max_steering_angle
            )
            self.get_logger().info(f'DRIVING MODE: forward={self.linear_x:.2f}, steer={self.target_steering_angle:.2f}')
        else:
            # Static turning mode - allow steering in place with reduced response
            # (Note: real cars can't turn in place, but we allow it for better usability)
            self.target_steering_angle = np.clip(
                self.angular_z * 0.5,
                -self.max_steering_angle,
                self.max_steering_angle
            )
            # Add a small amount of forward motion to help visualize steering direction
            if abs(self.angular_z) > 0.05:
                self.linear_x = 0.05 * self.speed_scale  # Very slow forward motion while turning in place
            self.get_logger().info(f'STATIC MODE: steer={self.target_steering_angle:.2f}, creep={self.linear_x:.2f}')
            
        # Final command summary
        self.get_logger().info(f'FINAL CMD: speed={self.linear_x:.2f}, steering={self.target_steering_angle:.2f}')
    
    def update_robot_position(self, dt):
        """Update robot position based on kinematics model"""
        # Exit early if no movement
        if abs(self.linear_x) < 0.001:
            return
            
        # Simple bicycle/Ackermann model
        v = self.linear_x  # Forward/backward velocity
        
        # Handle turning
        if abs(self.steering_angle) > 0.01:  # Non-zero steering angle
            # Compute turning radius (avoid division by zero with min threshold)
            # For a car-like vehicle with Ackermann steering:
            # - Positive steering angle = wheels point right = turn right = negative turning radius
            # - Negative steering angle = wheels point left = turn left = positive turning radius
            effective_angle = max(abs(self.steering_angle), 0.01) * (-1.0 if self.steering_angle > 0 else 1.0)
            turning_radius = self.wheel_base / math.tan(abs(self.steering_angle))
            
            # Direction of rotation depends on steering angle sign and velocity direction
            turning_radius *= effective_angle / abs(effective_angle)
            
            # Handling backward motion - changes turning behavior
            if v < 0:
                turning_radius *= -1.0  # Reverse direction when moving backward
            
            # Change in orientation depends on speed, turning radius and time
            delta_theta = (v * dt) / turning_radius
            
            # Update orientation
            self.theta += delta_theta
            
            # Update position (arc motion)
            self.x += turning_radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y += turning_radius * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
            
            # Log turning movement
            self.get_logger().info(f'TURNING: radius={turning_radius:.2f}m, delta_θ={delta_theta:.4f}rad, v={v:.2f}m/s')
        else:
            # Straight line motion
            dx = v * dt * math.cos(self.theta)
            dy = v * dt * math.sin(self.theta)
            
            # Update position
            self.x += dx
            self.y += dy
            
            # Log straight movement
            self.get_logger().info(f'STRAIGHT: dx={dx:.3f}m, dy={dy:.3f}m, v={v:.2f}m/s')
        
        # Normalize theta to keep it within [-π, π]
        while self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        while self.theta < -math.pi:
            self.theta += 2.0 * math.pi
    
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