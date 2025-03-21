#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
import numpy as np
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class WallDetectorDebug(Node):
    def __init__(self):
        super().__init__('wall_detector_debug')
        
        # Subscribe to visualization markers
        self.marker_sub = self.create_subscription(
            Marker,
            'visualization_marker',
            self.marker_callback,
            10
        )
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # Setup TF listener for robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Store walls and obstacles
        self.walls = []
        self.obstacles = []
        
        # Visualization timer
        self.vis_timer = self.create_timer(1.0, self.visualize_walls)
        
        self.get_logger().info('Wall detector debug node started')
    
    def marker_callback(self, msg):
        """Process incoming markers to identify walls"""
        # Check namespace to identify what this marker represents
        if msg.ns == 'walls':
            pos = (msg.pose.position.x, msg.pose.position.y)
            scale = (msg.scale.x, msg.scale.y, msg.scale.z)
            
            # Check if this is already in our list
            for i, (p, s) in enumerate(self.walls):
                if math.sqrt((p[0]-pos[0])**2 + (p[1]-pos[1])**2) < 0.1:
                    # Update existing wall
                    self.walls[i] = (pos, scale)
                    return
            
            # Add new wall
            self.walls.append((pos, scale))
            self.get_logger().info(f'Detected wall at {pos} with scale {scale}')
        
        elif msg.ns == 'trees':
            pos = (msg.pose.position.x, msg.pose.position.y)
            scale = (msg.scale.x, msg.scale.y, msg.scale.z)
            
            # Check if this is already in our list
            for i, (p, s) in enumerate(self.obstacles):
                if math.sqrt((p[0]-pos[0])**2 + (p[1]-pos[1])**2) < 0.1:
                    # Update existing obstacle
                    self.obstacles[i] = (pos, scale)
                    return
            
            # Add new obstacle
            self.obstacles.append((pos, scale))
    
    def get_robot_position(self):
        """Get the current robot position from TF"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract orientation (yaw) using a simplified approach
            q = transform.transform.rotation
            # Calculate yaw from quaternion (simplified)
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warning(f'Failed to get robot position: {e}')
            return None
    
    def visualize_walls(self):
        """Visualize detected walls and generate debugging info"""
        if not self.walls:
            self.get_logger().warning('No walls detected yet - environment may not be loaded properly')
            return
        
        robot_pos = self.get_robot_position()
        if not robot_pos:
            return
        
        rx, ry, ryaw = robot_pos
        
        # Log the total walls detected
        self.get_logger().info(f'Currently tracking {len(self.walls)} walls and {len(self.obstacles)} obstacles')
        
        # Create visualization for walls
        marker_array = MarkerArray()
        
        # Add wall bounding box visualizations
        for i, (pos, scale) in enumerate(self.walls):
            # Create marker for the wall perimeter
            wall_marker = Marker()
            wall_marker.header.frame_id = 'map'
            wall_marker.header.stamp = self.get_clock().now().to_msg()
            wall_marker.ns = 'wall_perimeters'
            wall_marker.id = i
            wall_marker.type = Marker.LINE_STRIP
            wall_marker.action = Marker.ADD
            wall_marker.pose.orientation.w = 1.0
            wall_marker.scale.x = 0.05  # Line width
            wall_marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)  # Magenta
            wall_marker.lifetime.sec = 2
            
            # Calculate the corners of the wall
            half_width = scale[0] / 2.0
            half_height = scale[1] / 2.0
            
            # Add points for the perimeter (closing the loop)
            corners = [
                Point(x=pos[0]-half_width, y=pos[1]-half_height, z=0.05),
                Point(x=pos[0]+half_width, y=pos[1]-half_height, z=0.05),
                Point(x=pos[0]+half_width, y=pos[1]+half_height, z=0.05),
                Point(x=pos[0]-half_width, y=pos[1]+half_height, z=0.05),
                Point(x=pos[0]-half_width, y=pos[1]-half_height, z=0.05)
            ]
            wall_marker.points = corners
            
            marker_array.markers.append(wall_marker)
            
            # Calculate distances from robot to each wall edge
            # Check if robot is too close to this wall
            distance_to_center = math.sqrt((rx-pos[0])**2 + (ry-pos[1])**2)
            
            # Check distance to each edge
            edges = [
                ((pos[0]-half_width, pos[1]-half_height), (pos[0]+half_width, pos[1]-half_height)),  # Bottom
                ((pos[0]+half_width, pos[1]-half_height), (pos[0]+half_width, pos[1]+half_height)),  # Right
                ((pos[0]+half_width, pos[1]+half_height), (pos[0]-half_width, pos[1]+half_height)),  # Top
                ((pos[0]-half_width, pos[1]+half_height), (pos[0]-half_width, pos[1]-half_height))   # Left
            ]
            
            # Calculate min distance to wall edges
            min_edge_dist = float('inf')
            for edge in edges:
                # Distance from point to line segment
                p1, p2 = edge
                edge_len_squared = (p2[0]-p1[0])**2 + (p2[1]-p1[1])**2
                
                if edge_len_squared == 0:  # Handle degenerate case
                    dist = math.sqrt((rx-p1[0])**2 + (ry-p1[1])**2)
                else:
                    # Project point onto line
                    t = max(0, min(1, ((rx-p1[0])*(p2[0]-p1[0]) + (ry-p1[1])*(p2[1]-p1[1])) / edge_len_squared))
                    proj_x = p1[0] + t * (p2[0] - p1[0])
                    proj_y = p1[1] + t * (p2[1] - p1[1])
                    
                    # Distance to projection
                    dist = math.sqrt((rx-proj_x)**2 + (ry-proj_y)**2)
                
                min_edge_dist = min(min_edge_dist, dist)
            
            # Log if robot is too close to wall
            if min_edge_dist < 1.0:
                self.get_logger().warn(f'Robot is close to wall {i}: {min_edge_dist:.2f}m')
                
                # Draw a line connecting robot to the closest wall
                robot_to_wall = Marker()
                robot_to_wall.header.frame_id = 'map'
                robot_to_wall.header.stamp = self.get_clock().now().to_msg()
                robot_to_wall.ns = 'robot_wall_distance'
                robot_to_wall.id = i
                robot_to_wall.type = Marker.LINE_STRIP
                robot_to_wall.action = Marker.ADD
                robot_to_wall.pose.orientation.w = 1.0
                robot_to_wall.scale.x = 0.02  # Line width
                
                # Color based on distance (red=close, yellow=medium)
                if min_edge_dist < 0.3:
                    robot_to_wall.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
                else:
                    robot_to_wall.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
                    
                robot_to_wall.lifetime.sec = 1
                
                # Add points for the line
                robot_to_wall.points = [
                    Point(x=rx, y=ry, z=0.1),
                    Point(x=pos[0], y=pos[1], z=0.1)
                ]
                
                marker_array.markers.append(robot_to_wall)
        
        # Publish all markers
        if marker_array.markers:
            self.marker_array_pub.publish(marker_array)
            
        # Simulate virtual LiDAR from robot to walls
        self.simulate_virtual_lidar(robot_pos)
    
    def simulate_virtual_lidar(self, robot_pos):
        """Simulate LiDAR rays from robot to verify walls are detected correctly"""
        if not robot_pos:
            return
        
        rx, ry, ryaw = robot_pos
        
        # Create a marker array for virtual LiDAR
        marker_array = MarkerArray()
        
        # Send rays in all directions
        num_rays = 36  # One ray every 10 degrees
        max_range = 3.0  # Max range for virtual LiDAR
        
        # Combine walls and obstacles for ray tracing
        all_obstacles = self.walls + self.obstacles
        
        # Add visualization for robot position
        robot_marker = Marker()
        robot_marker.header.frame_id = 'map'
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = 'virtual_lidar_robot'
        robot_marker.id = 0
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = rx
        robot_marker.pose.position.y = ry
        robot_marker.pose.position.z = 0.1
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale.x = 0.2
        robot_marker.scale.y = 0.2
        robot_marker.scale.z = 0.2
        robot_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red
        robot_marker.lifetime.sec = 1
        
        marker_array.markers.append(robot_marker)
        
        # Add rays from the robot
        for i in range(num_rays):
            angle = i * (2 * math.pi / num_rays)
            direction = (math.cos(angle + ryaw), math.sin(angle + ryaw))
            
            # Find distance to nearest obstacle
            min_dist = max_range
            hit_obstacle = False
            
            # Check intersection with each wall/obstacle
            for pos, scale in all_obstacles:
                # Determine if this is a wall (rectangular) or obstacle (circular)
                is_wall = (scale[0] > 5 * scale[1]) or (scale[1] > 5 * scale[0])
                
                if is_wall:
                    # For walls, check intersection with each edge
                    half_width = scale[0] / 2.0
                    half_height = scale[1] / 2.0
                    
                    # Wall corners
                    corners = [
                        (pos[0] - half_width, pos[1] - half_height),
                        (pos[0] + half_width, pos[1] - half_height),
                        (pos[0] + half_width, pos[1] + half_height),
                        (pos[0] - half_width, pos[1] + half_height)
                    ]
                    
                    # Wall edges
                    edges = [
                        (corners[0], corners[1]),  # Bottom
                        (corners[1], corners[2]),  # Right
                        (corners[2], corners[3]),  # Top
                        (corners[3], corners[0])   # Left
                    ]
                    
                    # Check intersection with each edge
                    for p1, p2 in edges:
                        # Ray-line segment intersection
                        edge_vector = (p2[0] - p1[0], p2[1] - p1[1])
                        
                        # Cross product determinant
                        det = edge_vector[1] * direction[0] - edge_vector[0] * direction[1]
                        
                        # If det is zero, lines are parallel
                        if abs(det) < 1e-6:
                            continue
                        
                        # Calculate intersection parameters
                        t1 = ((p1[1] - ry) * direction[0] - (p1[0] - rx) * direction[1]) / det
                        t2 = (edge_vector[1] * (p1[0] - rx) - edge_vector[0] * (p1[1] - ry)) / det
                        
                        # Check if intersection is within line segment and ray
                        if 0 <= t1 <= 1 and t2 >= 0:
                            dist = t2  # Distance along ray
                            
                            if 0 <= dist < min_dist:
                                min_dist = dist
                                hit_obstacle = True
                else:
                    # For circular obstacles
                    # Vector from robot to obstacle center
                    to_center = (pos[0] - rx, pos[1] - ry)
                    
                    # Project to_center vector onto ray direction
                    dot_product = to_center[0] * direction[0] + to_center[1] * direction[1]
                    
                    # Skip if obstacle is behind the ray
                    if dot_product < 0:
                        continue
                    
                    # Find closest point on ray to obstacle center
                    proj_x = rx + dot_product * direction[0]
                    proj_y = ry + dot_product * direction[1]
                    
                    # Distance from this point to obstacle center
                    dist_to_center = math.sqrt((proj_x - pos[0])**2 + (proj_y - pos[1])**2)
                    
                    # Use average of scale.x and scale.y as radius
                    radius = (scale[0] + scale[1]) / 4.0  # Scale is diameter, divide by 4
                    
                    if dist_to_center <= radius:
                        # Calculate actual intersection distance
                        intersection_dist = dot_product - math.sqrt(radius**2 - dist_to_center**2)
                        
                        if 0 <= intersection_dist < min_dist:
                            min_dist = intersection_dist
                            hit_obstacle = True
            
            # Create ray marker
            ray_marker = Marker()
            ray_marker.header.frame_id = 'map'
            ray_marker.header.stamp = self.get_clock().now().to_msg()
            ray_marker.ns = 'virtual_lidar_rays'
            ray_marker.id = i
            ray_marker.type = Marker.ARROW
            ray_marker.action = Marker.ADD
            ray_marker.pose.position.x = rx
            ray_marker.pose.position.y = ry
            ray_marker.pose.position.z = 0.1
            
            # Set orientation from direction
            angle_to_target = math.atan2(direction[1], direction[0])
            ray_marker.pose.orientation.z = math.sin(angle_to_target/2)
            ray_marker.pose.orientation.w = math.cos(angle_to_target/2)
            
            # Set scale based on hit distance
            ray_marker.scale.x = min_dist  # Arrow length
            ray_marker.scale.y = 0.03  # Arrow width
            ray_marker.scale.z = 0.03  # Arrow height
            
            # Set color based on hit (green=hit, blue=miss)
            if hit_obstacle:
                ray_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # Green
            else:
                ray_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5)  # Blue
                
            ray_marker.lifetime.sec = 1
            
            marker_array.markers.append(ray_marker)
        
        # Publish all visualizations
        if marker_array.markers:
            self.marker_array_pub.publish(marker_array)
            
        # Count hits
        hit_count = sum(1 for m in marker_array.markers if m.ns == 'virtual_lidar_rays' and m.color.g > 0)
        self.get_logger().info(f'Virtual LiDAR: {hit_count}/{num_rays} rays hit obstacles')

def main(args=None):
    rclpy.init(args=args)
    
    node = WallDetectorDebug()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 