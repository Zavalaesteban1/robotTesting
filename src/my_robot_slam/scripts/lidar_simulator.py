#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
import math
import numpy as np
import tf2_ros
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy
import random

class LiDARSimulator(Node):
    def __init__(self):
        super().__init__('lidar_simulator')
        
        # Store node startup time for diagnostics
        self._node_start_time = time.time()
        
        # Declare parameters
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('angle_min', -3.14159)
        self.declare_parameter('angle_max', 3.14159)
        self.declare_parameter('angle_increment', 0.01)
        self.declare_parameter('scan_time', 0.1)
        self.declare_parameter('publish_rate', 10.0)
        
        # Get parameters
        self.scan_topic = self.get_parameter('scan_topic').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.scan_time = self.get_parameter('scan_time').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Log current parameters for debugging
        self.get_logger().info(f'LiDAR Simulator initialized with:')
        self.get_logger().info(f'  scan_topic: {self.scan_topic}')
        self.get_logger().info(f'  laser_frame: {self.laser_frame}')
        self.get_logger().info(f'  range_max: {self.range_max}')
        self.get_logger().info(f'  publish_rate: {self.publish_rate}')
        
        # Setup publisher for laser scan with reliable QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self.scan_publisher = self.create_publisher(LaserScan, self.scan_topic, qos_profile)
        
        # Setup publisher for debug markers
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # Setup subscriber for obstacle markers
        # Using QoS with durability to ensure we get all obstacle markers
        latching_qos = QoSProfile(depth=20)
        latching_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        
        self.marker_sub = self.create_subscription(
            Marker, 
            'visualization_marker',
            self.marker_callback,
            qos_profile=latching_qos
        )
        
        # Add hardcoded walls if we're having trouble detecting them
        self.add_default_walls()
        
        # Store obstacles
        self.obstacles = []
        self.walls = []
        self.markers_received = False
        
        # Setup TF listener for robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publish static transforms if needed
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Ensure base_link to laser transform exists
        self.publish_base_to_laser_transform()
        
        # Setup timer for publishing scan
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_scan)
        
        # Add a diagnostic timer to ensure we are receiving markers
        self.diagnostic_timer = self.create_timer(5.0, self.check_marker_status)
        
        # Add a timer to publish empty scans until real data is available
        self.startup_timer = self.create_timer(0.1, self.publish_empty_scan)
        
        self.get_logger().info('LiDAR simulator started')
    
    def add_default_walls(self):
        """Add default walls around the perimeter in case detection fails"""
        # Create a 5m x 5m square arena
        self.get_logger().warn("Adding default walls to ensure obstacle detection")
        
        # Left wall - make walls thicker (increased from 0.1 to 0.3)
        self.walls.append(((-2.5, 0.0, 0.0), (0.3, 5.0, 0.5)))
        
        # Right wall - make walls thicker
        self.walls.append(((2.5, 0.0, 0.0), (0.3, 5.0, 0.5)))
        
        # Bottom wall - make walls thicker
        self.walls.append(((0.0, -2.5, 0.0), (5.0, 0.3, 0.5)))
        
        # Top wall - make walls thicker
        self.walls.append(((0.0, 2.5, 0.0), (5.0, 0.3, 0.5)))
        
        # Add some obstacles - make them larger too
        self.obstacles.append(((1.0, 1.0, 0.0), (0.7, 0.7, 0.5)))
        self.obstacles.append(((-1.0, -1.0, 0.0), (0.7, 0.7, 0.5)))
        
        self.get_logger().info(f"Added {len(self.walls)} default walls and {len(self.obstacles)} default obstacles")
        self.markers_received = True
    
    def publish_base_to_laser_transform(self):
        """Publish static transform from base_link to laser frame"""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = self.laser_frame
        
        # Laser is typically mounted on top of the robot
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.1
        
        # No rotation
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        
        self.tf_static_broadcaster.sendTransform(tf)
        self.get_logger().info(f'Published static transform: {tf.header.frame_id} -> {tf.child_frame_id}')
        
        # Also add a debug timer to republish this transform periodically
        self.create_timer(5.0, self.republish_transform)
    
    def republish_transform(self):
        """Periodically republish the transform to ensure it's available"""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = self.laser_frame
        
        # Laser is typically mounted on top of the robot
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.1
        
        # No rotation
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        
        self.tf_static_broadcaster.sendTransform(tf)
        self.get_logger().debug(f'Republished static transform: {tf.header.frame_id} -> {tf.child_frame_id}')
    
    def publish_empty_scan(self):
        """Publish an empty scan to ensure the topic is available"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.laser_frame
        
        # Set scan parameters
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Set all ranges to max range
        num_readings = int(round((self.angle_max - self.angle_min) / self.angle_increment))
        scan.ranges = [self.range_max] * num_readings
        
        # Publish scan
        self.scan_publisher.publish(scan)
        self.get_logger().info(f'Published empty scan on {self.scan_topic} with {len(scan.ranges)} points')
        
        # Keep publishing empty scans for 30 seconds to ensure data is available during startup
        if time.time() - self._node_start_time < 30.0:
            self.get_logger().info('Still in startup phase, continuing to publish empty scans')
        elif self.markers_received:
            self.startup_timer.cancel()
            self.get_logger().info('Stopping empty scan publishing - real data now available')
    
    def check_marker_status(self):
        """Diagnostic check to ensure markers are being received"""
        if not self.walls and not self.obstacles:
            self.get_logger().warn('No walls or obstacles received yet. Make sure the maze is loaded properly.')
            
            # Print available topics to help diagnose the issue
            try:
                from rclpy.topic_endpoint_info import TopicEndpointTypeEnum
                topics = self.get_topic_names_and_types()
                self.get_logger().info(f'Available topics: {topics}')
            except Exception as e:
                self.get_logger().error(f'Error getting topics: {e}')
        else:
            self.get_logger().info(f'Current environment: {len(self.walls)} walls and {len(self.obstacles)} obstacles')
            
        # Try to get transforms to help diagnose issues
        try:
            self.tf_buffer.lookup_transform('map', self.laser_frame, rclpy.time.Time())
            self.get_logger().info('Transform from map to laser is working')
        except Exception as e:
            self.get_logger().warn(f'Transform from map to laser failed: {e}')
            
            # Try other frame combinations
            try:
                self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                self.get_logger().info('Transform from map to base_link is working')
            except Exception as e:
                self.get_logger().warn(f'Transform from map to base_link failed: {e}')
                
            try:
                self.tf_buffer.lookup_transform('base_link', self.laser_frame, rclpy.time.Time())
                self.get_logger().info('Transform from base_link to laser is working')
            except Exception as e:
                self.get_logger().warn(f'Transform from base_link to laser failed: {e}')
    
    def marker_callback(self, msg):
        # Process markers to extract obstacles
        try:
            # Log all marker namespaces for debugging
            self.get_logger().debug(f'Received marker with namespace: {msg.ns}, id: {msg.id}, type: {msg.type}')
            
            # More aggressive wall detection
            if msg.ns in ['trees', 'walls', 'tree', 'wall', 'obstacles', 'maze', 'environment']:
                # Store position and radius of the obstacle
                position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
                scale = (msg.scale.x, msg.scale.y, msg.scale.z)
                
                # Log details for debugging
                self.get_logger().info(f'Processing marker: ns={msg.ns}, id={msg.id}, type={msg.type}, ' + 
                                      f'pos=({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}), ' +
                                      f'scale=({scale[0]:.2f}, {scale[1]:.2f}, {scale[2]:.2f})')
                
                # Determine which collection to add to based on namespace and shape
                is_wall = False
                
                # Check namespace - expanded to catch more potential wall markers
                if msg.ns in ['walls', 'wall', 'maze', 'environment']:
                    is_wall = True
                    self.get_logger().info(f'Identified wall by namespace: {msg.ns}')
                # Also check shape - cubes are likely walls, cylinders are likely trees
                elif msg.type == Marker.CUBE:
                    # Check if one dimension is much larger (typical for walls)
                    if (scale[0] > 2 * scale[1]) or (scale[1] > 2 * scale[0]):
                        is_wall = True
                        self.get_logger().info(f'Identified wall by shape: CUBE with uneven dimensions')
                # Also detect walls by color (pink/purple)
                elif msg.color.r > 0.7 and msg.color.b > 0.7 and msg.color.g < 0.3:
                    is_wall = True
                    self.get_logger().info(f'Identified wall by color: pink/purple')
                
                # Add to appropriate collection
                if is_wall:
                    # Check if this wall is already in our list (by position)
                    for i, (pos, _) in enumerate(self.walls):
                        if math.sqrt((pos[0]-position[0])**2 + (pos[1]-position[1])**2) < 0.1:
                            # Update existing wall
                            self.walls[i] = (position, scale)
                            self.get_logger().debug(f'Updated wall at {position}')
                            return
                            
                    # Add new wall
                    self.walls.append((position, scale))
                    self.get_logger().info(f'Added wall at {position} with scale {scale}')
                else:
                    # Check if this obstacle is already in our list
                    for i, (pos, _) in enumerate(self.obstacles):
                        if math.sqrt((pos[0]-position[0])**2 + (pos[1]-position[1])**2) < 0.1:
                            # Update existing obstacle
                            self.obstacles[i] = (position, scale)
                            self.get_logger().debug(f'Updated obstacle at {position}')
                            return
                            
                    # Add new obstacle
                    self.obstacles.append((position, scale))
                    self.get_logger().info(f'Added obstacle at {position} with scale {scale}')
                
                self.markers_received = True
                
                # Log total counts
                self.get_logger().info(f'Current environment: {len(self.walls)} walls and {len(self.obstacles)} obstacles')
        except Exception as e:
            # Catch any errors in marker processing to avoid crashing
            self.get_logger().error(f'Error processing marker: {e}')
    
    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', self.laser_frame, rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            return (x, y, z)
        except Exception as e:
            self.get_logger().warning(f'Failed to get robot position: {e}')
            return None
    
    def calculate_distance_to_obstacle(self, origin, direction, obstacles):
        """Raytrace to find distance to nearest obstacle"""
        # Default to max range
        min_distance = self.range_max
        
        # Check each obstacle
        for obstacle in obstacles:
            try:
                pos, scale = obstacle
                
                # Increase the effective size of all obstacles by adding a safety margin
                # This makes walls and obstacles "appear" larger to the LiDAR
                safety_margin = 0.2  # 20cm safety margin
                effective_scale = (scale[0] + safety_margin, scale[1] + safety_margin, scale[2])
                
                # Check if this is from trees (cylindrical) or walls (rectangular)
                # We can determine this by examining the scale - walls usually have one dimension much larger
                is_wall = (scale[0] > 3 * scale[1]) or (scale[1] > 3 * scale[0])
                
                if is_wall:
                    # For walls, use line-segment intersection with the 4 wall edges
                    # Calculate wall corners with the enhanced scale
                    half_width = effective_scale[0] / 2.0
                    half_height = effective_scale[1] / 2.0
                    
                    # Wall corners
                    corners = [
                        (pos[0] - half_width, pos[1] - half_height),  # Bottom-left
                        (pos[0] + half_width, pos[1] - half_height),  # Bottom-right
                        (pos[0] + half_width, pos[1] + half_height),  # Top-right
                        (pos[0] - half_width, pos[1] + half_height)   # Top-left
                    ]
                    
                    # Wall edges as line segments
                    edges = [
                        (corners[0], corners[1]),  # Bottom
                        (corners[1], corners[2]),  # Right
                        (corners[2], corners[3]),  # Top
                        (corners[3], corners[0])   # Left
                    ]
                    
                    # Check intersection with each edge
                    for edge in edges:
                        p1, p2 = edge
                        
                        # Ray-line segment intersection
                        # Vector representing the line segment
                        edge_vector = (p2[0] - p1[0], p2[1] - p1[1])
                        
                        # Cross product determinant for intersection calculation
                        det = edge_vector[1] * direction[0] - edge_vector[0] * direction[1]
                        
                        # Skip if lines are parallel (det near zero)
                        if abs(det) < 1e-6:
                            continue
                        
                        # Calculate intersection parameters
                        t1 = ((p1[1] - origin[1]) * direction[0] - (p1[0] - origin[0]) * direction[1]) / det
                        t2 = (edge_vector[1] * (p1[0] - origin[0]) - edge_vector[0] * (p1[1] - origin[1])) / det
                        
                        # Check if intersection is within the line segment and in front of the ray origin
                        if 0 <= t1 <= 1 and t2 >= 0:
                            dist = t2  # Distance along ray to intersection
                            
                            if dist < min_distance:
                                min_distance = dist
                                if random.random() < 0.01:  # Only log occasionally to avoid spam
                                    self.get_logger().debug(f"Wall hit at distance {dist:.2f}m")
                else:
                    # For circular obstacles like trees, use circle-ray intersection
                    # Vector from origin to obstacle center (in x-y plane)
                    to_center = (pos[0] - origin[0], pos[1] - origin[1])
                    
                    # Project onto ray direction to find closest approach
                    dot_product = to_center[0] * direction[0] + to_center[1] * direction[1]
                    
                    # Skip if obstacle is behind the ray
                    if dot_product < 0:
                        continue
                    
                    # Find closest point on ray to obstacle center
                    proj_x = origin[0] + dot_product * direction[0]
                    proj_y = origin[1] + dot_product * direction[1]
                    
                    # Distance from this point to obstacle center
                    dist_to_center = math.sqrt((proj_x - pos[0])**2 + (proj_y - pos[1])**2)
                    
                    # Use average of scale.x and scale.y as radius (assuming scale is diameter)
                    radius = (scale[0] + scale[1]) / 4.0  # Divide by 4 since scale is diameter
                    
                    if dist_to_center <= radius:
                        # Calculate actual intersection distance
                        # Using Pythagorean theorem: d = sqrt(dot_product^2 - dist_to_center^2)
                        discriminant = radius**2 - dist_to_center**2
                        if discriminant >= 0:  # Ensure ray hits the circle
                            intersection_dist = dot_product - math.sqrt(discriminant)
                            
                            # Update min_distance if this is closer
                            if 0 <= intersection_dist < min_distance:
                                min_distance = intersection_dist
            except Exception as e:
                # Catch any errors in intersection calculation
                self.get_logger().error(f"Error in raytrace calculation: {e}")
        
        return min_distance
    
    def publish_scan(self):
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            self.get_logger().warn("Failed to get robot position, skipping scan generation")
            return
        
        # Create LaserScan message
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.laser_frame
        
        # Set scan parameters
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Calculate number of rays
        num_readings = int(round((self.angle_max - self.angle_min) / self.angle_increment))
        scan.ranges = [self.range_max] * num_readings
        
        # Raytrace for each angle if we have obstacles to detect
        all_obstacles = self.walls + self.obstacles
        if len(all_obstacles) == 0:
            self.get_logger().warning("No obstacles found for LiDAR simulation - scan will return max range")
            if self.markers_received:
                self.get_logger().warning("Markers were received but no valid obstacles were extracted")
            
            # Visualize to help debug what's happening
            self.visualize_environment(robot_pos)
        else:
            # Ensure we have the right coordinate frame
            x, y, z = robot_pos
            robot_yaw = 0.0
            
            # Try to get the orientation if we can
            try:
                transform = self.tf_buffer.lookup_transform('map', self.laser_frame, rclpy.time.Time())
                q = transform.transform.rotation
                robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            except Exception as e:
                self.get_logger().warning(f"Failed to get robot orientation: {e}")
            
            # Process each ray
            for i in range(num_readings):
                angle = self.angle_min + i * self.angle_increment
                # Direction vector in map frame (accounting for robot orientation)
                direction = (math.cos(angle + robot_yaw), math.sin(angle + robot_yaw))
                
                # Calculate distance to nearest obstacle
                distance = self.calculate_distance_to_obstacle(
                    (x, y),  # Robot position
                    direction,
                    all_obstacles
                )
                
                # Add random noise to simulate real sensor
                noise = np.random.normal(0, 0.01)  # Mean 0, std 1cm
                distance += noise
                
                # Ensure distance is within range limits
                if distance < self.range_min:
                    distance = self.range_min
                
                scan.ranges[i] = distance
            
            # Visualize detected obstacles for debugging
            if random.random() < 0.2:  # Only visualize periodically
                self.visualize_environment(robot_pos)
        
        # Calculate percentage of rays that hit something
        hits = sum(1 for r in scan.ranges if r < self.range_max * 0.99)
        hit_percentage = 100 * hits / len(scan.ranges)
        self.get_logger().info(f"Scan completed: {hit_percentage:.1f}% of rays hit obstacles")
        
        # Publish scan
        self.scan_publisher.publish(scan)
    
    def visualize_environment(self, robot_pos):
        """Visualize the environment for debugging"""
        if not robot_pos:
            return
            
        x, y, z = robot_pos
        
        # Create marker array for environment visualization
        marker_array = MarkerArray()
        
        # Add marker for robot position
        robot_marker = Marker()
        robot_marker.header.frame_id = 'map'
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = 'lidar_sim_robot'
        robot_marker.id = 0
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = x
        robot_marker.pose.position.y = y
        robot_marker.pose.position.z = 0.2
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale.x = 0.2
        robot_marker.scale.y = 0.2
        robot_marker.scale.z = 0.2
        robot_marker.color.r = 0.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 1.0
        robot_marker.color.a = 1.0
        robot_marker.lifetime.sec = 1
        
        marker_array.markers.append(robot_marker)
        
        # Add markers for walls
        for i, (pos, scale) in enumerate(self.walls):
            wall_marker = Marker()
            wall_marker.header.frame_id = 'map'
            wall_marker.header.stamp = self.get_clock().now().to_msg()
            wall_marker.ns = 'lidar_sim_walls'
            wall_marker.id = i
            wall_marker.type = Marker.CUBE
            wall_marker.action = Marker.ADD
            wall_marker.pose.position.x = pos[0]
            wall_marker.pose.position.y = pos[1]
            wall_marker.pose.position.z = pos[2]
            wall_marker.pose.orientation.w = 1.0
            wall_marker.scale.x = scale[0]
            wall_marker.scale.y = scale[1]
            wall_marker.scale.z = scale[2]
            wall_marker.color.r = 1.0
            wall_marker.color.g = 0.0
            wall_marker.color.b = 0.0
            wall_marker.color.a = 0.3
            wall_marker.lifetime.sec = 1
            
            marker_array.markers.append(wall_marker)
        
        # Add markers for obstacles
        for i, (pos, scale) in enumerate(self.obstacles):
            obs_marker = Marker()
            obs_marker.header.frame_id = 'map'
            obs_marker.header.stamp = self.get_clock().now().to_msg()
            obs_marker.ns = 'lidar_sim_obstacles'
            obs_marker.id = i
            obs_marker.type = Marker.CYLINDER
            obs_marker.action = Marker.ADD
            obs_marker.pose.position.x = pos[0]
            obs_marker.pose.position.y = pos[1]
            obs_marker.pose.position.z = pos[2]
            obs_marker.pose.orientation.w = 1.0
            obs_marker.scale.x = scale[0]
            obs_marker.scale.y = scale[1]
            obs_marker.scale.z = scale[2]
            obs_marker.color.r = 0.0
            obs_marker.color.g = 1.0
            obs_marker.color.b = 0.0
            obs_marker.color.a = 0.3
            obs_marker.lifetime.sec = 1
            
            marker_array.markers.append(obs_marker)
        
        # Publish visualization
        if marker_array.markers:
            self.marker_array_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LiDARSimulator()
    rclpy.spin(node)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 