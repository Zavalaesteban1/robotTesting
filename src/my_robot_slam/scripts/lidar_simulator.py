#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
import numpy as np
import tf2_ros
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class LiDARSimulator(Node):
    def __init__(self):
        super().__init__('lidar_simulator')
        
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
        
        # Setup publisher for laser scan
        self.scan_publisher = self.create_publisher(LaserScan, self.scan_topic, 10)
        
        # Setup subscriber for obstacle markers
        # Using QoS with durability to ensure we get all obstacle markers
        latching_qos = QoSProfile(depth=10)
        latching_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        
        self.marker_sub = self.create_subscription(
            Marker, 
            'visualization_marker',
            self.marker_callback,
            qos_profile=latching_qos
        )
        
        # Store obstacles
        self.obstacles = []
        self.walls = []
        
        # Setup TF listener for robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Setup timer for publishing scan
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_scan)
        
        self.get_logger().info('LiDAR simulator started')
        
    def marker_callback(self, msg):
        # Process markers to extract obstacles
        if msg.ns in ['trees', 'walls']:
            # Store position and radius of the obstacle
            position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            scale = (msg.scale.x, msg.scale.y, msg.scale.z)
            
            # Determine which collection to add to
            if msg.ns == 'trees':
                self.obstacles.append((position, scale))
                self.get_logger().debug(f'Added tree obstacle at {position}')
            elif msg.ns == 'walls':
                self.walls.append((position, scale))
                self.get_logger().debug(f'Added wall at {position}')
    
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
            pos, scale = obstacle
            
            # Simple circle-ray intersection for cylinders (trees)
            # This is a simplification - a proper implementation would use
            # proper 3D ray-cylinder intersection
            
            # Vector from origin to obstacle center (just use x-y plane)
            to_center = (pos[0] - origin[0], pos[1] - origin[1])
            
            # Project to_center vector onto ray direction
            dot_product = to_center[0] * direction[0] + to_center[1] * direction[1]
            
            # Skip if obstacle is behind the ray
            if dot_product < 0:
                continue
            
            # Find closest point on ray to obstacle center
            proj_x = origin[0] + dot_product * direction[0]
            proj_y = origin[1] + dot_product * direction[1]
            
            # Distance from this point to obstacle center
            dist_to_center = math.sqrt((proj_x - pos[0])**2 + (proj_y - pos[1])**2)
            
            # If this distance is less than obstacle radius, ray intersects
            # Use average of scale.x and scale.y as radius
            radius = (scale[0] + scale[1]) / 4.0  # Divide by 4 because scale is diameter
            
            if dist_to_center <= radius:
                # Calculate actual intersection distance
                # Using Pythagorean theorem: d = sqrt(dot_product^2 - dist_to_center^2)
                intersection_dist = dot_product - math.sqrt(radius**2 - dist_to_center**2)
                
                # Update min_distance if this is closer
                if 0 <= intersection_dist < min_distance:
                    min_distance = intersection_dist
        
        return min_distance
    
    def publish_scan(self):
        robot_pos = self.get_robot_position()
        if robot_pos is None:
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
        
        # Raytrace for each angle
        all_obstacles = self.obstacles + self.walls
        for i in range(num_readings):
            angle = self.angle_min + i * self.angle_increment
            direction = (math.cos(angle), math.sin(angle))
            
            # Calculate distance to nearest obstacle
            distance = self.calculate_distance_to_obstacle(robot_pos, direction, all_obstacles)
            
            # Add random noise to simulate real sensor
            noise = np.random.normal(0, 0.01)  # Mean 0, std 1cm
            distance += noise
            
            # Ensure distance is within range limits
            if distance < self.range_min:
                distance = self.range_min
            
            scan.ranges[i] = distance
        
        # Publish scan
        self.scan_publisher.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = LiDARSimulator()
    rclpy.spin(node)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 