#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
import tf2_ros
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        
        # Declare parameters
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('obstacle_threshold', 0.8)  # Distance to consider as obstacle
        
        # Get parameters
        self.scan_topic = self.get_parameter('scan_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        
        # Create subscribers
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
        
        # Create publishers
        self.marker_pub = self.create_publisher(Marker, 'collected_data_markers', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'collected_data_array', 10)
        self.data_pub = self.create_publisher(Float32MultiArray, 'collected_data', 10)
        
        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize data collection
        self.robot_positions = []  # Store robot positions over time
        self.obstacle_positions = []  # Store detected obstacle positions
        self.last_scan_time = None
        self.collected_data_points = 0
        
        # Create timer for periodic publishing of collected data
        self.timer = self.create_timer(1.0, self.publish_collected_data)
        
        self.get_logger().info('Data collector initialized')
    
    def scan_callback(self, msg):
        if self.last_scan_time is None:
            self.last_scan_time = time.time()
        
        current_time = time.time()
        # Only process scans at a reasonable rate (1 Hz)
        if current_time - self.last_scan_time < 1.0:
            return
            
        self.last_scan_time = current_time
        
        # Get robot position
        robot_pos = self.get_robot_position()
        if not robot_pos:
            self.get_logger().warn("Failed to get robot position, skipping data collection")
            return
            
        x, y, yaw = robot_pos
        
        # Add to robot positions list
        self.robot_positions.append((x, y, yaw))
        
        # Add a marker for the current robot position
        self.mark_current_position(x, y)
        
        # Process the LiDAR data to find obstacles
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter out invalid readings
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        obstacles_detected = 0
        
        # Find obstacles (points significantly closer than surroundings)
        for i in range(len(valid_ranges)):
            range_val = valid_ranges[i]
            angle = valid_angles[i]
            
            # Skip points that are too far
            if range_val > self.obstacle_threshold:
                continue
                
            # Convert to world coordinates
            obstacle_x = x + range_val * math.cos(yaw + angle)
            obstacle_y = y + range_val * math.sin(yaw + angle)
            
            # Skip if we already have a point very close to this one
            is_new_point = True
            for ox, oy in self.obstacle_positions:
                if math.sqrt((ox-obstacle_x)**2 + (oy-obstacle_y)**2) < 0.2:
                    is_new_point = False
                    break
                    
            if is_new_point:
                self.obstacle_positions.append((obstacle_x, obstacle_y))
                obstacles_detected += 1
                self.collected_data_points += 1
                
                # Visualize this point
                self.visualize_point(obstacle_x, obstacle_y, len(self.obstacle_positions))
        
        # Log data collection progress
        if obstacles_detected > 0:
            self.get_logger().info(f"Collected {obstacles_detected} new obstacle points. Total: {self.collected_data_points}")
    
    def visualize_point(self, x, y, marker_id):
        """Create a visualization marker for a collected data point"""
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "collected_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        marker.color.r = 0.3
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.8
        
        # Long lifetime for persistent visualization
        marker.lifetime.sec = 600  # 10 minutes
        
        self.marker_pub.publish(marker)
    
    def publish_collected_data(self):
        """Periodically publish summary of collected data"""
        # Publish robot trajectory markers
        if len(self.robot_positions) > 1:
            # Create a line strip for the robot path
            path_marker = Marker()
            path_marker.header.frame_id = self.world_frame
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = "robot_path"
            path_marker.id = 0
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            
            # Add each position as a point in the line strip
            for x, y, _ in self.robot_positions:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.01
                path_marker.points.append(point)
            
            # Style the line
            path_marker.scale.x = 0.05  # Line width
            path_marker.color.r = 0.0
            path_marker.color.g = 0.0
            path_marker.color.b = 1.0
            path_marker.color.a = 0.8
            
            # Long lifetime
            path_marker.lifetime.sec = 600  # 10 minutes
            
            # Publish path marker
            self.marker_pub.publish(path_marker)
            
            # Also create sphere markers for each position (blue balls)
            marker_array = MarkerArray()
            for i, (x, y, _) in enumerate(self.robot_positions):
                # Only add a marker every few positions to avoid cluttering
                if i % 3 != 0:  # Add every 3rd position
                    continue
                    
                ball_marker = Marker()
                ball_marker.header.frame_id = self.world_frame
                ball_marker.header.stamp = self.get_clock().now().to_msg()
                ball_marker.ns = "robot_path_balls"
                ball_marker.id = i
                ball_marker.type = Marker.SPHERE
                ball_marker.action = Marker.ADD
                
                ball_marker.pose.position.x = x
                ball_marker.pose.position.y = y
                ball_marker.pose.position.z = 0.05
                ball_marker.pose.orientation.w = 1.0
                
                # Small blue balls
                ball_marker.scale.x = 0.08
                ball_marker.scale.y = 0.08
                ball_marker.scale.z = 0.08
                
                # Blue color
                ball_marker.color.r = 0.0
                ball_marker.color.g = 0.3
                ball_marker.color.b = 1.0
                ball_marker.color.a = 0.8
                
                # Long lifetime
                ball_marker.lifetime.sec = 600  # 10 minutes
                
                marker_array.markers.append(ball_marker)
            
            # Publish all ball markers at once
            if marker_array.markers:
                self.marker_array_pub.publish(marker_array)
        
        # Publish statistics
        self.get_logger().info(f"Collection stats: {len(self.robot_positions)} robot positions, {len(self.obstacle_positions)} obstacle points")
    
    def get_robot_position(self):
        """Get the robot's position in the world frame"""
        try:
            # Try to get the transform from world frame to robot frame
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract yaw from quaternion
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            return (x, y, yaw)
            
        except Exception as e:
            self.get_logger().warning(f"Failed to get robot position: {e}")
            
            # Try alternate frames
            try:
                # Try base_footprint instead
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    'base_footprint',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                
                # Extract yaw from quaternion
                q = transform.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                
                return (x, y, yaw)
            except:
                return None
            
        return None

    def mark_current_position(self, x, y):
        """Create a marker for the current robot position"""
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "current_position"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # Long lifetime for persistent visualization
        marker.lifetime.sec = 600  # 10 minutes
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    rclpy.spin(node)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 