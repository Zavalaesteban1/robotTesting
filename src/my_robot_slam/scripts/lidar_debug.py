#!/usr/bin/env python3
# lidar_debug.py - Visualizes LiDAR scan data to debug wall detection

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
import numpy as np
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class LidarDebugger(Node):
    def __init__(self):
        super().__init__('lidar_debugger')
        
        # Subscribe to scan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # Setup TF listener for robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Store current scan for visualization
        self.current_scan = None
        
        # Setup timer for visualization
        self.vis_timer = self.create_timer(0.5, self.visualize_scan)
        
        self.get_logger().info('LiDAR debugger started')
    
    def scan_callback(self, msg):
        self.current_scan = msg
        # Log scan summary
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[~np.isnan(ranges) & ~np.isinf(ranges)]
        
        if len(valid_ranges) > 0:
            min_range = np.min(valid_ranges)
            max_range = np.max(valid_ranges)
            mean_range = np.mean(valid_ranges)
            
            # Count ranges below 90% of max range (potential obstacle detections)
            obstacles = np.sum(valid_ranges < msg.range_max * 0.9)
            
            self.get_logger().info(f'Scan summary: min={min_range:.2f}m, max={max_range:.2f}m, mean={mean_range:.2f}m, obstacles={obstacles}/{len(valid_ranges)}')
    
    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # Extract orientation (yaw) using a simplified approach
            q = transform.transform.rotation
            # Calculate yaw from quaternion (simplified)
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            return (x, y, z, yaw)
        except Exception as e:
            self.get_logger().warning(f'Failed to get robot position: {e}')
            return None
    
    def visualize_scan(self):
        if self.current_scan is None:
            return
            
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return
            
        # Extract scan data
        ranges = np.array(self.current_scan.ranges)
        angles = np.linspace(self.current_scan.angle_min, self.current_scan.angle_max, len(ranges))
        
        # Filter out invalid readings
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # Create marker array for visualization
        marker_array = MarkerArray()
        
        # Add point cloud visualization for all scan points
        cloud_marker = Marker()
        cloud_marker.header.frame_id = 'map'
        cloud_marker.header.stamp = self.get_clock().now().to_msg()
        cloud_marker.ns = 'lidar_scan_cloud'
        cloud_marker.id = 0
        cloud_marker.type = Marker.POINTS
        cloud_marker.action = Marker.ADD
        cloud_marker.pose.orientation.w = 1.0
        cloud_marker.scale.x = 0.05  # Point size
        cloud_marker.scale.y = 0.05
        cloud_marker.lifetime.sec = 1  # Short lifetime
        
        # Set default color (gradient based on distance)
        cloud_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.5)  # White base color
        
        # Add points to point cloud
        cloud_marker.points = []
        cloud_marker.colors = []
        
        x, y, z, yaw = robot_pos
        
        # Create color gradient
        for range_val, angle in zip(valid_ranges, valid_angles):
            # Skip points at max range (no obstacle detected)
            if range_val >= self.current_scan.range_max * 0.99:
                continue
                
            # Calculate point position in map frame
            point_x = x + range_val * math.cos(yaw + angle)
            point_y = y + range_val * math.sin(yaw + angle)
            
            point = Point(x=point_x, y=point_y, z=z + 0.1)
            cloud_marker.points.append(point)
            
            # Color based on distance (red=close, green=medium, blue=far)
            distance_factor = min(1.0, range_val / self.current_scan.range_max)
            if distance_factor < 0.3:  # Close obstacles (red)
                color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            elif distance_factor < 0.6:  # Medium obstacles (yellow)
                color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            else:  # Far obstacles (green)
                color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                
            cloud_marker.colors.append(color)
        
        # Publish point cloud marker if we have points
        if cloud_marker.points:
            self.marker_pub.publish(cloud_marker)
            
        # Add a visualization of the robot position and orientation
        robot_marker = Marker()
        robot_marker.header.frame_id = 'map'
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = 'robot_position'
        robot_marker.id = 0
        robot_marker.type = Marker.ARROW
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = x
        robot_marker.pose.position.y = y
        robot_marker.pose.position.z = z + 0.2
        
        # Set orientation from yaw
        robot_marker.pose.orientation.x = 0.0
        robot_marker.pose.orientation.y = 0.0
        robot_marker.pose.orientation.z = math.sin(yaw/2)
        robot_marker.pose.orientation.w = math.cos(yaw/2)
        
        # Set scale
        robot_marker.scale.x = 0.5  # Arrow length
        robot_marker.scale.y = 0.1  # Arrow width
        robot_marker.scale.z = 0.1  # Arrow height
        
        # Set color
        robot_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue
        
        # Set lifetime
        robot_marker.lifetime.sec = 1
        
        # Publish robot marker
        self.marker_pub.publish(robot_marker)

def main(args=None):
    rclpy.init(args=args)
    
    node = LidarDebugger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 