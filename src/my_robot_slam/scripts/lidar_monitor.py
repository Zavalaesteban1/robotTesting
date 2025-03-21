#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarMonitor(Node):
    def __init__(self):
        super().__init__('lidar_monitor')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        self.get_logger().info('LiDAR monitor started - waiting for laser scan data...')
        
    def laser_callback(self, msg):
        # Convert to numpy for easier analysis
        ranges = np.array(msg.ranges)
        
        # Filter out invalid values
        valid_ranges = ranges[~np.isnan(ranges) & ~np.isinf(ranges) & (ranges < msg.range_max)]
        
        if len(valid_ranges) > 0:
            min_dist = np.min(valid_ranges)
            max_dist = np.max(valid_ranges)
            avg_dist = np.mean(valid_ranges)
            
            # Get the directions of closest obstacles
            min_indices = np.where(ranges == min_dist)[0]
            min_angles = [(msg.angle_min + i * msg.angle_increment) * 180.0 / np.pi for i in min_indices]
            
            self.get_logger().info(f'LiDAR Stats - Min: {min_dist:.2f}m, Max: {max_dist:.2f}m, Avg: {avg_dist:.2f}m')
            self.get_logger().info(f'Closest obstacles at angles: {[f"{a:.1f}°" for a in min_angles]}')
        else:
            self.get_logger().info('No valid range measurements detected')

def main(args=None):
    rclpy.init(args=args)
    lidar_monitor = LidarMonitor()
    rclpy.spin(lidar_monitor)
    lidar_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 