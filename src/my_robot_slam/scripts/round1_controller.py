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

class Round1Controller(Node):
    def __init__(self):
        super().__init__('round1_controller')
        
        # Setup publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.marker_sub = self.create_subscription(Marker, 'visualization_marker', self.marker_callback, 10)
        self.metrics_pub = self.create_publisher(Float32MultiArray, 'round1_metrics', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.obstacle_point_pub = self.create_publisher(PointStamped, 'obstacle_point', 10)
        
        # For recording fire position
        self.fire_position = None
        self.start_position = None
        self.obstacles_detected = 0
        self.closest_to_fire = float('inf')
        
        # For obstacle avoidance
        self.laser_data = None
        self.obstacle_threshold = 0.5  # Distance in meters to consider as obstacle
        self.min_safe_distance = 0.3   # Minimum safe distance to keep from obstacles
        self.danger_zones = []  # Store detected obstacles 
        self.scan_sectors = 16  # Number of sectors to divide the scan into
        self.closest_obstacle_point = None  # Track the closest obstacle
        
        # For exploration
        self.exploration_targets = []
        self.current_target = None
        self.target_reached_threshold = 0.3
        self.random_walk_timer = 0
        self.random_walk_duration = 5.0  # seconds
        self.exploration_strategy = "random_walk"  # random_walk, frontier_based
        self.visited_positions = []
        self.position_visit_threshold = 0.5  # meters
        
        # Setup TF listener for position tracking
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # State machine
        self.state = "INITIALIZE"  # INITIALIZE, EXPLORE, RETURN, FINISHED
        self.start_time = time.time()
        self.return_path = []
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.visualization_timer = self.create_timer(0.5, self.publish_visualizations)
        
        # Visualization markers
        self.path_markers = []
        self.visited_markers = []
        self.obstacle_markers = []
        
        # For stability
        self.transform_ready = False
        self.initialization_delay = 5.0  # seconds to wait for transforms to be ready
        self.init_start_time = time.time()
        
        self.get_logger().info("Round 1 controller initialized")
    
    def scan_callback(self, msg):
        self.laser_data = msg
        
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
            self.get_logger().error(f"Failed to get robot position: {e}")
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
            return False, 0.0, 0.0
        
        ranges = np.array(self.laser_data.ranges)
        angles = np.linspace(self.laser_data.angle_min, self.laser_data.angle_max, len(ranges))
        
        # Handle inf and nan values
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges)
        ranges_filtered = np.copy(ranges)
        ranges_filtered[~valid_mask] = 10.0  # Set to a large value
        
        # Check obstacles in different directions
        front_mask = np.abs(angles) < np.pi/4  # 45 degrees to left and right
        front_right_mask = (angles < 0) & (angles > -np.pi/3)
        front_left_mask = (angles > 0) & (angles < np.pi/3)
        left_mask = (angles > np.pi/6) & (angles < np.pi/2)
        right_mask = (angles < -np.pi/6) & (angles > -np.pi/2)
        
        # Get minimum distances in each direction
        front_min = np.min(ranges_filtered[front_mask]) if np.any(front_mask) else 10.0
        front_right_min = np.min(ranges_filtered[front_right_mask]) if np.any(front_right_mask) else 10.0
        front_left_min = np.min(ranges_filtered[front_left_mask]) if np.any(front_left_mask) else 10.0
        left_min = np.min(ranges_filtered[left_mask]) if np.any(left_mask) else 10.0
        right_min = np.min(ranges_filtered[right_mask]) if np.any(right_mask) else 10.0
        
        # Detect if there's an obstacle we need to avoid
        obstacle_ahead = front_min < self.obstacle_threshold
        obstacle_front_right = front_right_min < self.obstacle_threshold
        obstacle_front_left = front_left_min < self.obstacle_threshold
        obstacle_left = left_min < self.obstacle_threshold
        obstacle_right = right_min < self.obstacle_threshold
        
        # Calculate repulsive forces from obstacles
        linear_x = 0.0
        angular_z = 0.0
        
        if obstacle_ahead or obstacle_front_left or obstacle_front_right:
            # Determine which way to turn based on more open space
            if front_left_min > front_right_min and not obstacle_left:
                # More space to the left, turn left
                angular_z = 0.7
                linear_x = 0.05
                self.get_logger().debug("Obstacle ahead, turning left")
            elif not obstacle_right:
                # More space to the right, turn right
                angular_z = -0.7
                linear_x = 0.05
                self.get_logger().debug("Obstacle ahead, turning right")
            else:
                # Both sides blocked, rotate in place to find open space
                angular_z = 0.8
                linear_x = 0.0
                self.get_logger().debug("Obstacles on all sides, rotating in place")
                
            return True, linear_x, angular_z
        
        # Handle side obstacles
        if obstacle_left and not obstacle_right:
            # Obstacle on left, veer right
            angular_z = -0.3
            linear_x = 0.1
            self.get_logger().debug("Obstacle on left, veering right")
            return True, linear_x, angular_z
            
        if obstacle_right and not obstacle_left:
            # Obstacle on right, veer left
            angular_z = 0.3
            linear_x = 0.1
            self.get_logger().debug("Obstacle on right, veering left")
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
            return None
        
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
        
        # Check for obstacles
        obstacle_detected, obs_linear_x, obs_angular_z = self.check_obstacles()
        
        cmd = Twist()
        
        if obstacle_detected:
            # Use obstacle avoidance velocity
            cmd.linear.x = obs_linear_x
            cmd.angular.z = obs_angular_z
            self.get_logger().debug("Obstacle avoidance active")
        else:
            # Normal navigation to target
            if abs(angle_diff) > 0.3:  # First align with the target
                cmd.linear.x = 0.05  # Move slowly when turning
                cmd.angular.z = 0.5 * angle_diff  # Proportional control for rotation
            else:
                # Move toward the target
                cmd.linear.x = min(0.2, distance)  # Proportional speed control
                cmd.angular.z = 0.3 * angle_diff  # Keep correcting angle while moving
        
        return cmd, distance < self.target_reached_threshold
    
    def control_loop(self):
        # Wait for initialization delay
        if time.time() - self.init_start_time < self.initialization_delay:
            if time.time() - self.init_start_time > (self.initialization_delay - 1.0) and not self.transform_ready:
                self.get_logger().info("Initialization delay complete, starting navigation...")
                self.transform_ready = True
            return
        
        robot_pos = self.get_robot_position()
        if not robot_pos:
            self.get_logger().warn("Cannot get robot position, waiting...")
            return
            
        x, y, yaw = robot_pos
        current_pos = (x, y)
            
        if self.state == "INITIALIZE":
            self.start_position = current_pos
            self.state = "EXPLORE"
            self.get_logger().info(f"Starting exploration from {current_pos}")
            
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
                dist = math.sqrt((x - self.fire_position[0])**2 + (y - self.fire_position[1])**2)
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
                        start_dist = math.sqrt((x - self.start_position[0])**2 + (y - self.start_position[1])**2)
                        
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

def main(args=None):
    rclpy.init(args=args)
    controller = Round1Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
