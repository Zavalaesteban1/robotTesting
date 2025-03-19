#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
import random

class ObstaclePublisher(Node):
    """Node that publishes obstacles as visualization markers."""
    
    def __init__(self):
        super().__init__('obstacle_publisher')
        
        # Publisher for obstacles - publish both as MarkerArray and individual Markers for compatibility
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Timer for publishing obstacles - increase frequency
        self.timer = self.create_timer(0.2, self.publish_obstacles)  # Publish 5 times per second
        
        # Create a variety of obstacles with different shapes
        self.obstacles = self.generate_maze_obstacles()
        
        # Initialize counter for logging
        self._count = 0
        
        self.get_logger().info(f'Obstacle publisher initialized with {len(self.obstacles)} obstacles')
    
    def generate_maze_obstacles(self):
        """Generate a maze-like environment with obstacles."""
        markers = []
        marker_id = 0
        
        # Create walls for a maze - increase height for better visibility
        wall_height = 0.5
        wall_thickness = 0.1
        
        # Define colors with high alpha for visibility
        wall_color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0)  # Gray
        obstacle_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Bright Red
        column_color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Bright Blue
        
        # Create outer boundary walls
        boundary_size = 5.0
        wall_positions = [
            # Outer walls (x, y, width, height)
            (-boundary_size/2, 0, wall_thickness, boundary_size),  # Left wall
            (boundary_size/2, 0, wall_thickness, boundary_size),   # Right wall
            (0, -boundary_size/2, boundary_size, wall_thickness),  # Bottom wall
            (0, boundary_size/2, boundary_size, wall_thickness)    # Top wall
        ]
        
        # Add interior maze walls
        interior_walls = [
            # Horizontal walls (x, y, width, height)
            (-1.5, -1.0, 2.0, wall_thickness),
            (1.5, 1.0, 2.0, wall_thickness),
            (-2.0, 2.0, 2.0, wall_thickness),
            (0.0, -2.0, 3.0, wall_thickness),
            
            # Vertical walls (x, y, width, height)
            (-1.0, 1.5, wall_thickness, 2.0),
            (2.0, -1.5, wall_thickness, 2.0),
            (1.0, 0.0, wall_thickness, 3.0),
            (-2.0, -1.0, wall_thickness, 1.5)
        ]
        
        wall_positions.extend(interior_walls)
        
        # Add walls
        for x, y, width, height in wall_positions:
            marker = Marker()
            marker.header.frame_id = 'map'  # Ensure the frame_id is correct
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'walls'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = wall_height / 2.0
            
            # Set orientation (no rotation)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = width
            marker.scale.y = height
            marker.scale.z = wall_height
            
            # Set color
            marker.color = wall_color
            
            markers.append(marker)
        
        # Add some cylindrical obstacles (columns)
        column_positions = [
            (-1.5, 1.5),
            (1.5, -1.5),
            (0.0, 0.0),
            (-2.0, -2.0),
            (2.0, 2.0)
        ]
        
        for x, y in column_positions:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'columns'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = wall_height / 2.0
            
            # Set orientation (no rotation)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = wall_height
            
            # Set color
            marker.color = column_color
            
            markers.append(marker)
        
        # Add some small cube obstacles
        for i in range(10):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position (ensure they don't overlap with walls)
            marker.pose.position.x = random.uniform(-boundary_size/2 + 0.5, boundary_size/2 - 0.5)
            marker.pose.position.y = random.uniform(-boundary_size/2 + 0.5, boundary_size/2 - 0.5)
            marker.pose.position.z = wall_height / 2.0
            
            # Set orientation (no rotation)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Set scale - make them a bit larger for visibility
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = wall_height
            
            # Set color
            marker.color = obstacle_color
            
            markers.append(marker)
        
        # Make markers persistent
        for marker in markers:
            marker.lifetime.sec = 0  # Persistent
        
        return markers
    
    def publish_obstacles(self):
        """Publish the obstacles as marker array."""
        marker_array = MarkerArray()
        
        # Get current time for header stamps
        now = self.get_clock().now().to_msg()
        
        # Update timestamp on all markers
        for marker in self.obstacles:
            marker.header.stamp = now
            # Also publish each marker individually
            self.marker_pub.publish(marker)
        
        marker_array.markers = self.obstacles
        self.marker_array_pub.publish(marker_array)
        
        # Only log once to avoid flooding
        if self._count == 0:
            self.get_logger().info(f'Published {len(self.obstacles)} obstacles')
        self._count += 1
        if self._count >= 10:
            self._count = 0

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 