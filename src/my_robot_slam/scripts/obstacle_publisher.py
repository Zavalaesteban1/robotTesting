#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
import random
import os

class ObstaclePublisher(Node):
    """Node that publishes obstacles as visualization markers for the competition field."""
    
    def __init__(self):
        super().__init__('obstacle_publisher')
        
        # Publisher for obstacles - publish both as MarkerArray and individual Markers for compatibility
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Timer for publishing obstacles - increase frequency
        self.timer = self.create_timer(0.2, self.publish_obstacles)  # Publish 5 times per second
        
        # Create a variety of obstacles for the competition field
        self.obstacles = self.generate_competition_field()
        
        # Initialize counter for logging
        self._count = 0
        
        # Print working directory for debugging
        self.get_logger().info(f'Current working directory: {os.getcwd()}')
        self.get_logger().info(f'Competition field initialized with {len(self.obstacles)} obstacles')
    
    def generate_competition_field(self):
        """Generate a competition field with boundary walls, tree obstacles, and challenging terrain."""
        markers = []
        marker_id = 0
        
        # Units conversion: ft to meters (1 foot = 0.3048 meters, 1 inch = 0.0254 meters)
        field_size = 8 * 0.3048  # 8 ft in meters
        wall_height = 5 * 0.0254  # 5 inches in meters
        obstacle_diameter = 1 * 0.0254  # 1 inch diameter in meters
        min_path_width = 1.1 * 0.3048  # 1.1 ft in meters
        terrain_elevation = 0.5 * 0.0254  # 0.5 inches in meters
        
        # Define colors with high alpha for visibility
        wall_color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0)  # Gray walls
        obstacle_color = ColorRGBA(r=0.6, g=0.4, b=0.2, a=1.0)  # Brown tree obstacles
        sand_color = ColorRGBA(r=0.9, g=0.8, b=0.6, a=1.0)  # Sand terrain
        gravel_color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)  # Gravel terrain
        fire_zone_color = ColorRGBA(r=1.0, g=0.4, b=0.4, a=0.3)  # Red for potential fire zones
        start_zone_color = ColorRGBA(r=0.4, g=1.0, b=0.4, a=0.3)  # Green for robot start zone
        
        # Create boundary walls - 8ft x 8ft square
        wall_thickness = 0.05  # 5cm thick walls
        half_field = field_size / 2
        
        # Boundary walls (x, y, width, height)
        wall_positions = [
            (-half_field, 0, wall_thickness, field_size),  # Left wall
            (half_field, 0, wall_thickness, field_size),   # Right wall
            (0, -half_field, field_size, wall_thickness),  # Bottom wall
            (0, half_field, field_size, wall_thickness)    # Top wall
        ]
        
        # Add walls
        for x, y, width, height in wall_positions:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'walls'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(wall_height / 2.0)
            
            # Set orientation (no rotation)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = float(width)
            marker.scale.y = float(height)
            marker.scale.z = float(wall_height)
            
            # Set color
            marker.color = wall_color
            
            markers.append(marker)
        
        # Create four corner zones (potential fire or start locations)
        zone_size = field_size / 4  # Each zone is 2ft x 2ft
        corner_positions = [
            (half_field - zone_size/2, half_field - zone_size/2),     # Top-right
            (half_field - zone_size/2, -half_field + zone_size/2),    # Bottom-right
            (-half_field + zone_size/2, half_field - zone_size/2),    # Top-left
            (-half_field + zone_size/2, -half_field + zone_size/2)    # Bottom-left
        ]
        
        # Randomly select one corner for fire and another for start
        fire_corner_idx = random.randint(0, 3)
        start_corner_idx = (fire_corner_idx + 2) % 4  # Use opposite corner for start
        
        # Create corner zones
        for i, (x, y) in enumerate(corner_positions):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            
            if i == fire_corner_idx:
                marker.ns = 'fire_zone'
                marker.color = fire_zone_color
            elif i == start_corner_idx:
                marker.ns = 'start_zone'
                marker.color = start_zone_color
            else:
                marker.ns = 'corner_zone'
                marker.color = ColorRGBA(r=0.5, g=0.5, b=1.0, a=0.2)  # Blue for other corners
                
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position (slightly above ground)
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(0.01)  # Just above ground
            
            # Set orientation (no rotation)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = float(zone_size)
            marker.scale.y = float(zone_size)
            marker.scale.z = float(0.01)  # Very thin
            
            markers.append(marker)
            
            # Add a simulated fire in the fire zone (just for visualization)
            if i == fire_corner_idx:
                fire_marker = Marker()
                fire_marker.header.frame_id = 'map'
                fire_marker.header.stamp = self.get_clock().now().to_msg()
                fire_marker.ns = 'fire'
                fire_marker.id = marker_id
                marker_id += 1
                fire_marker.type = Marker.CYLINDER
                fire_marker.action = Marker.ADD
                
                # Set position
                fire_marker.pose.position.x = float(x)
                fire_marker.pose.position.y = float(y)
                fire_marker.pose.position.z = float(wall_height / 2.0)
                
                # Set orientation (no rotation)
                fire_marker.pose.orientation.x = 0.0
                fire_marker.pose.orientation.y = 0.0
                fire_marker.pose.orientation.z = 0.0
                fire_marker.pose.orientation.w = 1.0
                
                # Set scale
                fire_marker.scale.x = float(0.3)
                fire_marker.scale.y = float(0.3)
                fire_marker.scale.z = float(wall_height)
                
                # Set color (orange-red for fire)
                fire_marker.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.8)
                
                markers.append(fire_marker)
        
        # Create tree obstacles (dowels/PVC pipes)
        # Create a grid of potential positions inside field excluding corner zones
        grid_spacing = min_path_width  # Minimum path width between obstacles
        grid_points_x = int((field_size - 2*zone_size) / grid_spacing) + 1
        grid_points_y = int((field_size - 2*zone_size) / grid_spacing) + 1
        
        tree_positions = []
        
        # Create a maze-like pattern with tree obstacles that ensures pathways
        # Start with a grid pattern and then remove some to create paths
        for i in range(grid_points_x):
            for j in range(grid_points_y):
                x = -half_field + zone_size + i * grid_spacing
                y = -half_field + zone_size + j * grid_spacing
                
                # Skip positions too close to corner zones
                too_close_to_corner = False
                for corner_x, corner_y in corner_positions:
                    distance = math.sqrt((x - corner_x)**2 + (y - corner_y)**2)
                    if distance < zone_size:
                        too_close_to_corner = True
                        break
                
                if not too_close_to_corner:
                    # Add some randomness to positions to make it look more natural
                    x += random.uniform(-0.1, 0.1)
                    y += random.uniform(-0.1, 0.1)
                    
                    # Only keep some trees to create paths (70% chance)
                    if random.random() < 0.7:
                        tree_positions.append((x, y))
        
        # Remove some trees to ensure there are navigable paths
        # Create main paths by removing trees that block direct routes
        for i in range(len(tree_positions)):
            # Check if this tree blocks major paths
            x, y = tree_positions[i]
            
            # Skip trees that are along major pathways
            on_major_path = False
            
            # Main X and Y axis paths
            if abs(x) < min_path_width/2 or abs(y) < min_path_width/2:
                on_major_path = True
                
            # Diagonal paths
            if abs(abs(x) - abs(y)) < min_path_width/2:
                on_major_path = True
                
            if on_major_path:
                tree_positions[i] = None
        
        # Filter out None values
        tree_positions = [pos for pos in tree_positions if pos is not None]
        
        # Add tree obstacles
        for x, y in tree_positions:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'trees'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(wall_height / 2.0)
            
            # Set orientation (no rotation)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Set scale - diameter of obstacles is 1 inch
            marker.scale.x = float(obstacle_diameter)
            marker.scale.y = float(obstacle_diameter)
            marker.scale.z = float(wall_height)
            
            # Set color - brown for trees
            marker.color = obstacle_color
            
            markers.append(marker)
        
        # Add challenging terrain areas (sand and gravel)
        # Create several patches of different terrains
        terrain_patches = [
            # x, y, width, height, type
            (-half_field/2, half_field/3, half_field/2, half_field/3, 'sand'),
            (half_field/3, -half_field/2, half_field/3, half_field/2, 'gravel'),
        ]
        
        for x, y, width, height, terrain_type in terrain_patches:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = terrain_type
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position - slightly above ground level
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(terrain_elevation / 2.0)
            
            # Set orientation (no rotation)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = float(width)
            marker.scale.y = float(height)
            marker.scale.z = float(terrain_elevation)
            
            # Set color based on terrain type
            if terrain_type == 'sand':
                marker.color = sand_color
            else:  # gravel
                marker.color = gravel_color
            
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
        
        # Print periodic update for debugging
        if self._count % 10 == 0:
            self.get_logger().info(f'Publishing {len(self.obstacles)} competition field elements')
        
        # Update timestamp on all markers and publish individually
        for marker in self.obstacles:
            marker.header.stamp = now
            # Also publish each marker individually
            self.marker_pub.publish(marker)
        
        # Create a new list for the marker array - important to create a copy
        marker_array.markers = list(self.obstacles)
        self.marker_array_pub.publish(marker_array)
        
        # Only log once to avoid flooding
        if self._count == 0:
            namespaces = set([m.ns for m in self.obstacles])
            self.get_logger().info(f'Publishing competition field with namespaces: {namespaces}')
        self._count += 1
        if self._count >= 50:
            self._count = 0

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 