#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.wheel_pos = 0.0
        self.angle = 0.0
        self.angle_dir = 1

    def timer_callback(self):
        # Update positions
        self.wheel_pos += 0.1
        self.angle += 0.05 * self.angle_dir
        if self.angle > 0.4:
            self.angle_dir = -1
        elif self.angle < -0.4:
            self.angle_dir = 1

        # Create message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['front_left_wheel_joint', 'front_right_wheel_joint', 'left_rear_wheel_joint', 'right_rear_wheel_joint', 
                   'steering_control_joint', 'front_left_pivot_joint', 'front_right_pivot_joint']
        msg.position = [self.wheel_pos, self.wheel_pos, self.wheel_pos, self.wheel_pos, self.angle, self.angle, self.angle]
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        
        # Publish message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint states: wheel={self.wheel_pos:.2f}, steering={self.angle:.2f}')

def main():
    print('Starting robot animation...')
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