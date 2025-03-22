#!/usr/bin/env python3
# round1_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32, ColorRGBA, Float32MultiArray
from sensor_msgs.msg import LaserScan
import tf2_ros
import math
import time
import numpy as np
# Use the quaternion utilities from geometry and tf2
import tf2_geometry_msgs
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

class Round1Controller(Node):
    def __init__(self):
        super().__init__('round1_controller')
        
        # Declare parameters with defaults
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('force_move_interval', 5.0)  # Force move every 5 seconds
        
        # Get parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.force_move_interval = self.get_parameter('force_move_interval').value
        
        # Log key parameters
        self.get_logger().info(f'Parameters loaded:')
        self.get_logger().info(f'  cmd_vel_topic: {self.cmd_vel_topic}')
        self.get_logger().info(f'  scan_topic: {self.scan_topic}')
        self.get_logger().info(f'  robot_frame: {self.robot_frame}')
        self.get_logger().info(f'  world_frame: {self.world_frame}')
        self.get_logger().info(f'  safe_distance: {self.safe_distance}')
        self.get_logger().info(f'  force_move_interval: {self.force_move_interval}')
        
        # Create publisher
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # Create subscribers with reliable QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile=qos_profile
        )
        
        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State variables
        self.scan_data = None
        self.last_scan_time = None
        self.obstacle_count = 0
        self.move_direction = 0
        self.force_move_count = 0
        
        # Initialize movement tracking
        self.last_cmd = None
        self.last_cmd_time = None
        
        # Create timer for periodic checks
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Create timer for forcing movement occasionally
        self.force_move_timer = self.create_timer(self.force_move_interval, self.force_move)
        
        # Test movement at startup
        self.startup_test_timer = self.create_timer(2.0, self.test_movement)
        self.movement_tested = False
        
        # Declare parameters
        self.declare_parameter('obstacle_threshold', 0.8)  # Distance in meters to consider as obstacle
        self.declare_parameter('min_safe_distance', 0.5)   # Minimum safe distance to obstacles
        self.declare_parameter('scan_sectors', 16)         # Number of sectors to divide laser scan into
        self.declare_parameter('wall_safety_multiplier', 1.5)  # Multiply safety distance for walls
        
        # Get parameters
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.scan_sectors = self.get_parameter('scan_sectors').value
        self.wall_safety_multiplier = self.get_parameter('wall_safety_multiplier').value
        
        # Setup publishers and subscribers
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.marker_sub = self.create_subscription(Marker, 'visualization_marker', self.marker_callback, 10)
        self.metrics_pub = self.create_publisher(Float32MultiArray, 'round1_metrics', 10)
        self.obstacle_point_pub = self.create_publisher(PointStamped, 'obstacle_point', 10)
        
        # For recording fire position
        self.fire_position = None
        self.start_position = None
        self.obstacles_detected = 0
        self.closest_to_fire = float('inf')
        
        # For obstacle avoidance - get parameters
        self.laser_data = None
        self.danger_zones = []  # Store detected obstacles 
        self.closest_obstacle_point = None  # Track the closest obstacle
        self.detected_walls = []  # Store detected walls
        
        # For exploration
        self.exploration_targets = []
        self.current_target = None
        self.target_reached_threshold = 0.3
        self.random_walk_timer = 0
        self.random_walk_duration = 5.0  # seconds
        self.exploration_strategy = "random_walk"  # random_walk, frontier_based
        self.visited_positions = []
        self.position_visit_threshold = 0.5  # meters
        
        # State machine
        self.state = "INITIALIZE"  # INITIALIZE, EXPLORE, RETURN, FINISHED
        self.start_time = time.time()
        self.return_path = []
        
        # Visualization markers
        self.path_markers = []
        self.visited_markers = []
        self.obstacle_markers = []
        
        # For stability
        self.transform_ready = False
        self.initialization_delay = 5.0  # seconds to wait for transforms to be ready
        self.init_start_time = time.time()
        
        # Add a timer to check transforms periodically instead
        self.tf_check_timer = self.create_timer(2.0, self.check_transforms)
        
        # Add a timer to check if we're receiving laser data
        self.laser_check_timer = self.create_timer(5.0, self.check_laser_data)
        self.last_laser_time = None
        
        self.get_logger().info("Round 1 controller initialized")
    
    def scan_callback(self, msg):
        if msg is None:
            # This is called for diagnostics
            from rclpy.topic_endpoint_info import TopicEndpointTypeEnum
            # Debug the laser data issue
            self.get_logger().warn("Laser data diagnostics:")
            try:
                # List topics to see if scan is being published
                topics = self.get_topic_names_and_types()
                self.get_logger().info(f"Available topics: {topics}")
                
                # Check if scan topic exists
                scan_topics = [t for t in topics if 'scan' in t[0]]
                if scan_topics:
                    self.get_logger().info(f"Found scan topics: {scan_topics}")
                else:
                    self.get_logger().error("No scan topics found!")
            except Exception as e:
                self.get_logger().error(f"Error getting topics: {e}")
            return
            
        self.laser_data = msg
        self.last_laser_time = time.time()
        
        # Process the LiDAR data
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[~np.isnan(ranges) & ~np.isinf(ranges)]
        
        if len(valid_ranges) > 0:
            # Count obstacles - more sophisticated approach with clustering
            obstacles = np.sum(valid_ranges < self.obstacle_threshold)
            
            if obstacles > 2:  # At least 2 points to consider a new obstacle
                self.obstacles_detected += 1
            
            # Find the closest obstacle
            min_dist_idx = np.argmin(valid_ranges)
            min_dist = valid_ranges[min_dist_idx]
            
            # Process the scan data for obstacle detection and visualization
            self.process_scan_data(msg)
            
            # Publish obstacle point for visualization
            self.publish_closest_obstacle_point(msg, min_dist_idx)
    
    def process_scan_data(self, scan_msg):
        """Process the scan data to identify danger zones"""
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Handle inf and nan values
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # Clear previous danger zones
        self.danger_zones = []
        
        # If we have valid data
        if len(valid_ranges) > 0:
            # Divide the scan into sectors for better decision making
            sector_size = 2 * np.pi / self.scan_sectors
            sector_ranges = [[] for _ in range(self.scan_sectors)]
            
            for i, (r, a) in enumerate(zip(valid_ranges, valid_angles)):
                sector_idx = int((a + np.pi) / sector_size) % self.scan_sectors
                sector_ranges[sector_idx].append(r)
            
            # Calculate average distance per sector and find danger zones
            for i, sector in enumerate(sector_ranges):
                if sector:  # If we have measurements in this sector
                    avg_dist = np.mean(sector)
                    angle = -np.pi + (i + 0.5) * sector_size  # Center angle of the sector
                    
                    if avg_dist < self.obstacle_threshold:
                        self.danger_zones.append({
                            'angle': angle,
                            'distance': avg_dist,
                            'sector': i
                        })
    
    def publish_closest_obstacle_point(self, scan_msg, min_idx):
        """Publish the closest obstacle point for visualization"""
        if not self.laser_data or min_idx >= len(scan_msg.ranges):
            return
        
        robot_pos = self.get_robot_position()
        if not robot_pos:
            return
            
        r = scan_msg.ranges[min_idx]
        angle = scan_msg.angle_min + min_idx * scan_msg.angle_increment
        
        # Calculate the position in the map frame
        x = robot_pos[0] + r * math.cos(robot_pos[2] + angle)
        y = robot_pos[1] + r * math.sin(robot_pos[2] + angle)
        
        # Store for visualization
        self.closest_obstacle_point = (x, y)
        
        # Create and publish PointStamped message
        point_msg = PointStamped()
        point_msg.header.frame_id = "map"
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.1
        
        self.obstacle_point_pub.publish(point_msg)
    
    def publish_visualizations(self):
        """Publish various visualizations for RViz"""
        if not self.transform_ready:
            return
            
        # Visualize danger zones (obstacles)
        self.visualize_obstacles()
        
        # Visualize the laser scan data
        if self.laser_data:
            self.publish_laser_visualization(self.laser_data)
    
    def visualize_obstacles(self):
        """Visualize the detected obstacles"""
        if not self.danger_zones or not self.get_robot_position():
            return
            
        marker_array = MarkerArray()
        robot_pos = self.get_robot_position()
        
        for i, zone in enumerate(self.danger_zones):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "danger_zones"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Calculate position in map frame
            marker.pose.position.x = robot_pos[0] + zone['distance'] * math.cos(robot_pos[2] + zone['angle'])
            marker.pose.position.y = robot_pos[1] + zone['distance'] * math.sin(robot_pos[2] + zone['angle'])
            marker.pose.position.z = 0.1
            
            # Size based on distance (closer = bigger)
            size = 0.1 + (self.obstacle_threshold - zone['distance']) * 0.4 / self.obstacle_threshold
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = size
            
            # Color based on distance (red = close, yellow = medium)
            danger_factor = 1.0 - (zone['distance'] / self.obstacle_threshold)
            marker.color = ColorRGBA(r=1.0, g=danger_factor, b=0.0, a=0.7)
            
            # Short lifetime to prevent clutter
            marker.lifetime.sec = 1
            
            marker_array.markers.append(marker)
        
        # Publish the marker array
        self.marker_array_pub.publish(marker_array)
    
    def publish_laser_visualization(self, scan_msg):
        """Visualize the laser scan data as points"""
        if not self.get_robot_position():
            return
            
        robot_pos = self.get_robot_position()
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Create a marker array for the scan points
        marker_array = MarkerArray()
        
        # Filter out invalid readings
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # Create markers for a subsample of points to avoid overwhelming visualization
        sample_step = max(1, len(valid_ranges) // 100)  # Visualize up to 100 points
        
        for i in range(0, len(valid_ranges), sample_step):
            r = valid_ranges[i]
            angle = valid_angles[i]
            
            # Skip points that are too far or too close
            if r > 8.0 or r < 0.1:
                continue
                
            # Calculate the position in the map frame
            x = robot_pos[0] + r * math.cos(robot_pos[2] + angle)
            y = robot_pos[1] + r * math.sin(robot_pos[2] + angle)
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "laser_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.05
            
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            # Color based on distance: close=red, far=blue
            distance_factor = min(1.0, r / 5.0)  # Normalize to 0-1 range
            marker.color = ColorRGBA(r=1.0-distance_factor, g=0.0, b=distance_factor, a=0.8)
            
            # Short lifetime
            marker.lifetime.sec = 1
            
            marker_array.markers.append(marker)
        
        # Publish the marker array
        if marker_array.markers:
            self.marker_array_pub.publish(marker_array)

    def marker_callback(self, msg):
        # Log marker information for debugging
        self.get_logger().debug(f"Received marker with namespace: {msg.ns}")
        
        # Listen for fire markers
        if msg.ns == 'fire' and self.fire_position is None:
            self.fire_position = (msg.pose.position.x, msg.pose.position.y)
            self.get_logger().info(f"Fire detected at {self.fire_position}")
            
            # Create a special marker to highlight the detected fire
            self.publish_detection_marker(self.fire_position)
            
            # Don't switch to return mode immediately, continue exploring until 
            # initialization is complete and we have a valid position
            if self.transform_ready and self.get_robot_position():
                self.get_logger().info("Switching to RETURN mode after fire detection")
                self.state = "RETURN"
                
                # Calculate a path back to start that avoids obstacles
                self.plan_return_path()
        
        # Track walls for improved obstacle avoidance
        elif msg.ns == 'walls' or msg.ns == 'wall':
            wall_pos = (msg.pose.position.x, msg.pose.position.y)
            wall_size = (msg.scale.x, msg.scale.y)
            
            # Check if this wall is already tracked
            for i, (pos, size) in enumerate(self.detected_walls):
                if math.sqrt((pos[0]-wall_pos[0])**2 + (pos[1]-wall_pos[1])**2) < 0.2:
                    # Update existing wall
                    self.detected_walls[i] = (wall_pos, wall_size)
                    return
            
            # Add new wall
            self.detected_walls.append((wall_pos, wall_size))
            self.get_logger().info(f"Added wall at {wall_pos} with size {wall_size}")
    
    def publish_detection_marker(self, position):
        # Create a marker to show you've detected the fire
        detection_marker = Marker()
        detection_marker.header.frame_id = "map"
        detection_marker.header.stamp = self.get_clock().now().to_msg()
        detection_marker.ns = "fire_detection"
        detection_marker.id = 0
        detection_marker.type = Marker.SPHERE
        detection_marker.action = Marker.ADD
        detection_marker.pose.position.x = position[0]
        detection_marker.pose.position.y = position[1]
        detection_marker.pose.position.z = 0.5
        detection_marker.scale.x = 0.5
        detection_marker.scale.y = 0.5
        detection_marker.scale.z = 0.5
        detection_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        self.marker_pub.publish(detection_marker)
    
    def get_robot_position(self):
        """Get the robot's position in the map frame with robust retry logic"""
        retry_count = 0
        max_retries = 5
        retry_delay = 0.1  # seconds
        
        while retry_count < max_retries:
            try:
                # Try to get the transform from map to base_footprint
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_footprint',  # Changed from 'base_link' to 'base_footprint'
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                
                # Extract yaw from quaternion
                q = transform.transform.rotation
                _, _, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
                
                self.transform_ready = True
                
                # Log successful position tracking
                if retry_count > 0:
                    self.get_logger().info(f"Recovered robot position after {retry_count+1} attempts")
                
                return (x, y, yaw)
                
            except Exception as e:
                retry_count += 1
                if retry_count == 1:  # Only log the first attempt to reduce spam
                    self.get_logger().warning(f"Failed to get robot position: {e}")
                
                # Try alternate transform paths
                if retry_count == 2:
                    try:
                        # Try map to base_link instead
                        transform = self.tf_buffer.lookup_transform(
                            'map', 
                            'base_link',
                            rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(seconds=0.5)
                        )
                        
                        self.get_logger().info("Used base_link for position (fallback 1)")
                        x = transform.transform.translation.x
                        y = transform.transform.translation.y
                        
                        # Extract yaw from quaternion
                        q = transform.transform.rotation
                        _, _, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
                        
                        self.transform_ready = True
                        return (x, y, yaw)
                    except:
                        pass
                
                # Another fallback attempt with different frames
                if retry_count == 3:
                    try:
                        # Try odom to base_footprint
                        transform = self.tf_buffer.lookup_transform(
                            'odom', 
                            'base_footprint',
                            rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(seconds=0.5)
                        )
                        
                        self.get_logger().info("Used odom frame for position (fallback 2)")
                        x = transform.transform.translation.x
                        y = transform.transform.translation.y
                        
                        # Extract yaw from quaternion
                        q = transform.transform.rotation
                        _, _, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
                        
                        self.transform_ready = True
                        return (x, y, yaw)
                    except:
                        pass
                
                if retry_count < max_retries:
                    self.get_logger().warning(f"Retrying to get robot position (attempt {retry_count+1}/{max_retries})")
                    time.sleep(retry_delay)
                else:
                    self.get_logger().error("Failed to get robot position after multiple attempts")
                    if not self.transform_ready:
                        self.get_logger().error("Transform system not ready - check TF tree connections")
        
        # If we reach here, we failed to get the position
        return None
    
    def plan_return_path(self):
        # In a real implementation, this would use a path planning algorithm like A*
        # For simplicity, we'll just set a direct path back to start
        robot_pos = self.get_robot_position()
        if robot_pos and self.start_position:
            # For now, just add the start position as the target
            self.return_path = [self.start_position]
            self.get_logger().info(f"Return path planned: {self.return_path}")
    
    def check_obstacles(self):
        """Check for obstacles using laser scan and return a safe velocity"""
        if not self.laser_data:
            self.get_logger().warn("No laser data available, forced stop for safety")
            # Complete stop and report obstacle to caller
            return True, 0.0, 0.0
        
        ranges = np.array(self.laser_data.ranges)
        angles = np.linspace(self.laser_data.angle_min, self.laser_data.angle_max, len(ranges))
        
        # Improved handling of inf and nan values
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges)
        ranges_filtered = np.copy(ranges)
        
        # If we have too few valid readings, stop for safety
        valid_count = np.sum(valid_mask)
        if valid_count < len(ranges) * 0.2:  # If less than 20% of readings are valid
            self.get_logger().warn(f"Too few valid laser readings: {valid_count}/{len(ranges)}, stopping for safety")
            return True, 0.0, 0.0  # Safety stop
            
        # Set invalid readings to a large but finite value
        ranges_filtered[~valid_mask] = self.obstacle_threshold * 2.0  # Set to double threshold
        
        # Log significant range data for debugging
        if np.any(valid_mask):
            min_range = np.min(ranges_filtered[valid_mask])
            min_range_idx = np.argmin(ranges_filtered)
            min_range_angle = angles[min_range_idx]
            self.get_logger().info(f"Min range: {min_range:.2f}m at angle: {min_range_angle:.2f}rad")
            
            # Extremely aggressive emergency stop if anything is close - INCREASED
            # Increased from 0.5*min_safe_distance to 3.0*min_safe_distance
            if min_range < self.min_safe_distance * 3.0:
                self.get_logger().error(f"EMERGENCY STOP - obstacle at {min_range:.2f}m")
                # Completely stop and report obstacle detected
                return True, 0.0, 0.0
        
        # Check obstacles in all directions with smaller angle ranges for better precision
        # Front is split into narrow segments for more precise control
        center_front_mask = np.abs(angles) < np.pi/10  # 18 degrees straight ahead
        front_mask = np.abs(angles) < np.pi/6  # 30 degrees to left and right (narrower front view)
        front_right_mask = (angles < 0) & (angles > -np.pi/3)  # 60 degrees to right
        front_left_mask = (angles > 0) & (angles < np.pi/3)  # 60 degrees to left
        left_mask = (angles > np.pi/6) & (angles < np.pi/2)  # Side left
        right_mask = (angles < -np.pi/6) & (angles > -np.pi/2)  # Side right
        
        # Rear checks for reverse safety
        rear_mask = np.abs(angles) > 3*np.pi/4  # Rear 90 degrees
        
        # Get minimum distances in each direction (with additional validity checks)
        center_front_valid = center_front_mask & valid_mask
        front_valid = front_mask & valid_mask
        front_right_valid = front_right_mask & valid_mask
        front_left_valid = front_left_mask & valid_mask
        left_valid = left_mask & valid_mask
        right_valid = right_mask & valid_mask
        rear_valid = rear_mask & valid_mask
        
        center_front_min = np.min(ranges[center_front_valid]) if np.any(center_front_valid) else 10.0
        front_min = np.min(ranges[front_valid]) if np.any(front_valid) else 10.0
        front_right_min = np.min(ranges[front_right_valid]) if np.any(front_right_valid) else 10.0
        front_left_min = np.min(ranges[front_left_valid]) if np.any(front_left_valid) else 10.0
        left_min = np.min(ranges[left_valid]) if np.any(left_valid) else 10.0
        right_min = np.min(ranges[right_valid]) if np.any(right_valid) else 10.0
        rear_min = np.min(ranges[rear_valid]) if np.any(rear_valid) else 10.0
        
        # Enhanced logging to help determine if walls are being detected
        self.get_logger().info(f"Distances - center: {center_front_min:.2f}, front: {front_min:.2f}, front-left: {front_left_min:.2f}, front-right: {front_right_min:.2f}, left: {left_min:.2f}, right: {right_min:.2f}, rear: {rear_min:.2f}")
        
        # MUCH more aggressive thresholds - INCREASED VALUES FOR WALL DETECTION 
        # Use an AGGRESSIVE threshold that gets larger as the robot moves faster
        # This provides a larger safety margin at higher speeds
        dynamic_threshold = self.obstacle_threshold * 5.0  # Increased from 3x to 5x higher than base threshold
        
        # Use a longer lookahead for the center front area
        center_threshold = self.obstacle_threshold * 6.0  # Increased from 4x to 6x for direct front
        
        # SIMPLE REVERSAL BEHAVIOR - If any obstacle is too close, just back up
        # Check if anything is dangerously close in front
        critical_distance = self.min_safe_distance * 5.0  # Increased from 3.5 to 5.0 for more safety margin
        
        # If there's anything close in front, REVERSE!
        if front_min < critical_distance or center_front_min < critical_distance:
            self.get_logger().warn(f"WALL/OBSTACLE TOO CLOSE: {front_min:.2f}m - REVERSING!")
            
            # Only reverse if there's space behind us
            if rear_min > critical_distance:
                # Reverse movement with slight rotation to avoid getting stuck
                linear_x = -0.1  # Increased from -0.06 for faster escape
                
                # Random rotation to prevent getting stuck in the same position
                if front_left_min < front_right_min:
                    angular_z = -1.0  # Increased from -0.6 for more aggressive turning
                else:
                    angular_z = 1.0  # Increased from 0.6 for more aggressive turning
            else:
                # If blocked behind too, just rotate in place
                self.get_logger().warn("BLOCKED BEHIND TOO - ROTATING IN PLACE")
                linear_x = 0.0
                angular_z = 1.5  # Increased from 1.2 for faster rotation
                
            return True, linear_x, angular_z
        
        # Detect if there's an obstacle we need to avoid
        obstacle_center = center_front_min < center_threshold
        obstacle_ahead = front_min < dynamic_threshold
        obstacle_front_right = front_right_min < dynamic_threshold
        obstacle_front_left = front_left_min < dynamic_threshold
        obstacle_left = left_min < dynamic_threshold
        obstacle_right = right_min < dynamic_threshold
        
        # Calculate repulsive forces from obstacles
        linear_x = 0.0
        angular_z = 0.0
        
        # Handle central obstacles - high priority
        if obstacle_center:
            # Decide which way to turn based on more open space (left or right)
            if left_min > right_min:
                # More space to the left, make sharp left turn
                angular_z = 2.0  # Increased from 1.2 for very aggressive turning
                linear_x = -0.05  # Reverse slowly while turning instead of stopping
                self.get_logger().error("WALL DEAD AHEAD - turning sharp left and backing up")
            else:
                # More space to the right, make sharp right turn
                angular_z = -2.0  # Increased from -1.2 for very aggressive turning
                linear_x = -0.05  # Reverse slowly while turning
                self.get_logger().error("WALL DEAD AHEAD - turning sharp right and backing up")
            return True, linear_x, angular_z
        
        # Handle frontal obstacles
        if obstacle_ahead or obstacle_front_left or obstacle_front_right:
            # Determine which way to turn based on more open space
            if front_left_min > front_right_min and not obstacle_left:
                # More space to the left, turn left
                angular_z = 1.5  # Increased from 1.0 for more aggressive turning
                linear_x = 0.0  # Full stop while turning instead of slow forward
                self.get_logger().error("WALL AHEAD - turning left")
            elif not obstacle_right:
                # More space to the right, turn right
                angular_z = -1.5  # Increased from -1.0
                linear_x = 0.0  # Full stop
                self.get_logger().error("WALL AHEAD - turning right")
            else:
                # Both sides blocked, rotate in place to find open space
                angular_z = 2.0  # Increased from 1.5
                linear_x = -0.05  # Slight backward movement while rotating
                self.get_logger().error("WALLS ON ALL SIDES - rotating rapidly to find an exit")
                
            return True, linear_x, angular_z
        
        # Handle side obstacles
        if obstacle_left and not obstacle_right:
            # Obstacle on left, veer right
            angular_z = -0.8  # Increased from -0.7
            linear_x = 0.03  # Reduced from 0.05
            self.get_logger().info("Obstacle on left, veering right")
            return True, linear_x, angular_z
            
        if obstacle_right and not obstacle_left:
            # Obstacle on right, veer left
            angular_z = 0.8  # Increased from 0.7
            linear_x = 0.03  # Reduced from 0.05
            self.get_logger().info("Obstacle on right, veering left")
            return True, linear_x, angular_z
        
        # No immediate obstacles that require action
        return False, 0.0, 0.0
    
    def update_exploration_target(self):
        # This would implement frontier-based exploration in a more complex system
        # For now, we'll use a simple random target selection
        robot_pos = self.get_robot_position()
        if not robot_pos:
            return
        
        # Add current position to visited list
        current_pos = (robot_pos[0], robot_pos[1])
        
        # Check if we're near a previously visited position
        visited = False
        for pos in self.visited_positions:
            dist = math.sqrt((current_pos[0] - pos[0])**2 + (current_pos[1] - pos[1])**2)
            if dist < self.position_visit_threshold:
                visited = True
                break
        
        if not visited:
            self.visited_positions.append(current_pos)
            self.visualize_visited_position(current_pos)
        
        # Random walk exploration
        if self.random_walk_timer <= 0 or self.current_target is None:
            # Generate new random direction
            random_angle = np.random.uniform(-np.pi, np.pi)
            random_distance = np.random.uniform(1.0, 2.0)
            
            self.current_target = (
                robot_pos[0] + random_distance * math.cos(random_angle),
                robot_pos[1] + random_distance * math.sin(random_angle)
            )
            
            self.random_walk_timer = self.random_walk_duration
            self.get_logger().debug(f"New exploration target: {self.current_target}")
        else:
            self.random_walk_timer -= 0.1  # Decrease timer
    
    def visualize_visited_position(self, position):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "visited_positions"
        marker.id = len(self.visited_markers)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.05
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5)
        marker.lifetime.sec = 300  # 5 minutes
        
        self.marker_pub.publish(marker)
        self.visited_markers.append(marker)
    
    def move_to_target(self, target_x, target_y):
        """Move the robot toward a target position while avoiding obstacles"""
        robot_pos = self.get_robot_position()
        if not robot_pos:
            return None, False
        
        x, y, yaw = robot_pos
        
        # Calculate distance and angle to target
        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference (-pi to pi)
        angle_diff = target_angle - yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Check for obstacles - this is our priority
        obstacle_detected, obs_linear_x, obs_angular_z = self.check_obstacles()
        
        cmd = Twist()
        
        if obstacle_detected:
            # Use obstacle avoidance velocity - this MUST override navigation
            cmd.linear.x = obs_linear_x
            cmd.angular.z = obs_angular_z
            self.get_logger().info("Obstacle avoidance active, overriding navigation")
        else:
            # ULTRA-CONSERVATIVE MOVEMENT: drastically reduced speeds
            if abs(angle_diff) > 0.2:  # First align with the target (reduced threshold)
                cmd.linear.x = 0.0  # Complete stop when turning (reduced from 0.01)
                cmd.angular.z = 0.15 * angle_diff  # Even slower turning (reduced from 0.2)
                self.get_logger().info("Aligning with target - rotation only")
            else:
                # Move toward the target - use much lower max velocity
                cmd.linear.x = min(0.03, distance * 0.3)  # Further reduced from 0.04, more conservative proportional control
                cmd.angular.z = 0.15 * angle_diff  # Reduced from 0.2 for gentler corrections
                
                # Additional check - if target is far, move even slower to have more reaction time
                if distance > 1.0:
                    cmd.linear.x = min(cmd.linear.x, 0.02)  # Cap speed for distant targets
                    self.get_logger().info(f"Long distance movement - reduced speed to {cmd.linear.x:.3f}")
        
        return cmd, distance < self.target_reached_threshold
    
    def control_loop(self):
        """Main control loop - runs at 10Hz"""
        # Early exit during initialization delay
        if time.time() - self.init_start_time < self.initialization_delay:
            # During initialization, send a small movement command to ensure robot is responsive
            self.send_move_command(0.02, 0.1)  # Very small linear movement with rotation
            return
        
        # If we don't have laser data yet, try to send a small movement command to stimulate data
        if not self.laser_data:
            self.get_logger().warn("No laser data available in control loop, sending small movement to stimulate data")
            self.send_move_command(0.05, 0.2)  # Small forward movement with rotation
            return
            
        # Get current robot position
        robot_pos = self.get_robot_position()
        
        # Handle state machine transitions
        if self.state == "INITIALIZE":
            self.start_position = robot_pos[:2]
            self.state = "EXPLORE"
            self.get_logger().info(f"Starting exploration from {robot_pos[:2]}")
            
        elif self.state == "EXPLORE":
            # Update exploration target if needed
            self.update_exploration_target()
            
            # Move toward the current exploration target
            if self.current_target:
                cmd, target_reached = self.move_to_target(self.current_target[0], self.current_target[1])
                
                if cmd:
                    self.cmd_vel_pub.publish(cmd)
                
                # If target reached, reset for new target
                if target_reached:
                    self.current_target = None
            
            # Update closest distance to fire if fire is found
            if self.fire_position:
                dist = math.sqrt((robot_pos[0] - self.fire_position[0])**2 + (robot_pos[1] - self.fire_position[1])**2)
                if dist < self.closest_to_fire:
                    self.closest_to_fire = dist
                    self.get_logger().info(f"New closest distance to fire: {self.closest_to_fire:.2f}m")
            
        elif self.state == "RETURN":
            # Return to start point
            if not self.return_path:
                self.plan_return_path()  # Ensure we have a return path
            
            if self.return_path:
                target = self.return_path[0]
                cmd, target_reached = self.move_to_target(target[0], target[1])
                
                if cmd:
                    self.cmd_vel_pub.publish(cmd)
                
                if target_reached:
                    self.return_path.pop(0)  # Remove the reached target
                    
                    if not self.return_path:  # No more targets in the path
                        # We've reached the start position, mission complete
                        self.get_logger().info("Mission complete! Returned to start.")
                        elapsed_time = time.time() - self.start_time
                        
                        # Calculate final distance to start
                        start_dist = math.sqrt((robot_pos[0] - self.start_position[0])**2 + (robot_pos[1] - self.start_position[1])**2)
                        
                        # Publish metrics
                        self.publish_metrics(elapsed_time, self.closest_to_fire, start_dist)
                        
                        # End mission
                        cmd = Twist()
                        cmd.linear.x = 0.0
                        cmd.angular.z = 0.0
                        self.cmd_vel_pub.publish(cmd)
                        self.state = "FINISHED"
    
    def publish_metrics(self, elapsed_time, fire_dist, start_dist):
        # Publish the performance metrics
        metrics_msg = Float32MultiArray()
        metrics_msg.data = [float(elapsed_time), float(fire_dist), float(start_dist), float(self.obstacles_detected)]
        self.metrics_pub.publish(metrics_msg)
        
        self.get_logger().info(f"Round 1 Metrics: Time={elapsed_time:.2f}s, Closest to fire={fire_dist:.2f}m, Final distance to start={start_dist:.2f}m, Obstacles detected={self.obstacles_detected}")

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert quaternion to euler angles (roll, pitch, yaw)
        """
        # Roll (rotation around x-axis)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (rotation around y-axis)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
            
        # Yaw (rotation around z-axis)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return (roll, pitch, yaw)

    def tf_callback(self, msg):
        # Log transform information for debugging
        self.get_logger().debug(f"Received transform from {msg.header.frame_id} to {msg.child_frame_id}")

    def check_transforms(self):
        """Check if transforms are available and log for debugging"""
        try:
            # Try to get the transform from map to base_footprint
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            self.transform_ready = True
            self.get_logger().info("Transform from map to base_footprint is available")
        except Exception as e:
            self.get_logger().warning(f"Transform from map to base_footprint not available: {e}")
            
            # Try alternate transform paths
            try:
                # Try map to base_link instead
                transform = self.tf_buffer.lookup_transform(
                    'map', 
                    'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                self.transform_ready = True
                self.get_logger().info("Transform from map to base_link is available")
            except Exception as e:
                self.get_logger().warning(f"Transform from map to base_link not available: {e}")

    def check_laser_data(self):
        # Check if we're receiving laser data
        if self.last_laser_time and time.time() - self.last_laser_time > 5.0:
            self.get_logger().warn("No laser data received for 5 seconds, checking laser data")
            self.scan_callback(None)
        self.last_laser_time = time.time()

    def send_move_command(self, linear_x, angular_z):
        """Send a move command to the robot"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)

    def test_movement(self):
        """Send test movements to verify robot can move"""
        self.movement_tested = True
        
        # Alternate between forward/backward and rotation
        if self.movement_tested:
            # Forward then backward
            self.get_logger().info("Movement test: Forward/backward")
            cmd = Twist()
            cmd.linear.x = 0.05  # Small forward movement
            self.cmd_vel_pub.publish(cmd)
            
            # Schedule a stop command after a short delay
            self.create_timer(0.5, lambda: self.send_stop_command(), one_shot=True)
        else:
            # Rotate left then right
            self.get_logger().info("Movement test: Rotation")
            cmd = Twist()
            cmd.angular.z = 0.3  # Small rotation
            self.cmd_vel_pub.publish(cmd)
            
            # Schedule a stop command after a short delay
            self.create_timer(0.5, lambda: self.send_stop_command(), one_shot=True)
    
    def send_stop_command(self):
        """Send a command to stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Stop command sent")

    def force_move(self):
        """Force the robot to move to avoid getting stuck"""
        self.get_logger().info(f"Force move triggered (count: {self.force_move_count})")
        
        # Check for obstacles first - don't force movement if obstacles are too close
        obstacle_detected, _, _ = self.check_obstacles()
        if obstacle_detected:
            self.get_logger().warn("Force move canceled - obstacles detected")
            return
        
        # Create a twist message with MUCH gentler movement
        twist = Twist()
        
        # Use extremely gentle movements that respect obstacles
        if self.force_move_count % 2 == 0:
            # Move forward very slowly
            twist.linear.x = 0.05  # Reduced from 0.2 to be much gentler
            twist.angular.z = 0.0
            self.get_logger().info("Forcing gentle forward movement")
        else:
            # Turn very slowly
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # Reduced from 0.5 for gentler rotation
            self.get_logger().info("Forcing gentle rotation")
        
        # Publish the twist message
        self.cmd_vel_pub.publish(twist)
        self.last_cmd = twist
        self.last_cmd_time = self.get_clock().now()
        
        # Schedule a stop after 0.5 seconds for safety
        self.create_timer(0.5, self.send_stop_command, one_shot=True)
        
        # Increment the force move count
        self.force_move_count += 1

def main(args=None):
    rclpy.init(args=args)
    controller = Round1Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
