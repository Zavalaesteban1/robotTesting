#!/usr/bin/env python3
# round1_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, ColorRGBA
import tf2_ros
import math
import time

class Round1Controller(Node):
    def __init__(self):
        super().__init__('round1_controller')
        
        # Setup publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_sub = self.create_subscription(Marker, 'visualization_marker', self.marker_callback, 10)
        self.metrics_pub = self.create_publisher(Float32MultiArray, 'round1_metrics', 10)
        
        # For recording fire position
        self.fire_position = None
        self.start_position = None
        self.obstacles_detected = 0
        self.closest_to_fire = float('inf')
        
        # Setup TF listener for position tracking
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # State machine
        self.state = "INITIALIZE"  # INITIALIZE, EXPLORE, RETURN
        self.start_time = time.time()
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Round 1 controller initialized")
    
    def marker_callback(self, msg):
        # Listen for fire markers
        if msg.ns == 'fire' and self.fire_position is None:
            self.fire_position = (msg.pose.position.x, msg.pose.position.y)
            self.get_logger().info(f"Fire detected at {self.fire_position}")
            
            # Create a special marker to highlight the detected fire
            self.publish_detection_marker(self.fire_position)
            
            # Switch to return mode after finding fire
            self.state = "RETURN"
    
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
            return (transform.transform.translation.x, transform.transform.translation.y)
        except Exception as e:
            self.get_logger().error(f"Failed to get robot position: {e}")
            return None
    
    def control_loop(self):
        robot_pos = self.get_robot_position()
        if not robot_pos:
            return
            
        if self.state == "INITIALIZE":
            self.start_position = robot_pos
            self.state = "EXPLORE"
            self.get_logger().info(f"Starting exploration from {robot_pos}")
            
        elif self.state == "EXPLORE":
            # Simple exploration strategy (you can make this more sophisticated)
            cmd = Twist()
            cmd.linear.x = 0.2  # Forward speed
            cmd.angular.z = 0.1  # Small angular velocity for exploration
            self.cmd_vel_pub.publish(cmd)
            
            # Update closest distance to fire if fire is found
            if self.fire_position:
                dist = math.sqrt((robot_pos[0] - self.fire_position[0])**2 + 
                                (robot_pos[1] - self.fire_position[1])**2)
                self.closest_to_fire = min(self.closest_to_fire, dist)
            
        elif self.state == "RETURN":
            # Return to start point
            if self.start_position:
                dx = self.start_position[0] - robot_pos[0]
                dy = self.start_position[1] - robot_pos[1]
                
                distance = math.sqrt(dx**2 + dy**2)
                angle = math.atan2(dy, dx)
                
                cmd = Twist()
                
                if distance > 0.2:  # Not at start yet
                    # Calculate robot heading in map frame
                    try:
                        # Calculate heading by looking at orientation in map frame
                        cmd.linear.x = 0.2  # Move forward
                        cmd.angular.z = angle * 0.5  # Turn toward start
                    except:
                        cmd.angular.z = 0.5  # Just turn
                else:
                    # We've reached the start position, mission complete
                    self.get_logger().info("Mission complete! Returned to start.")
                    elapsed_time = time.time() - self.start_time
                    
                    # Publish metrics
                    self.publish_metrics(elapsed_time, self.closest_to_fire, distance)
                    
                    # End mission
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.state = "FINISHED"
                
                self.cmd_vel_pub.publish(cmd)
    
    def publish_metrics(self, time, fire_dist, start_dist):
        # Publish the performance metrics
        self.get_logger().info(f"Round 1 Metrics: Time={time:.2f}s, Closest to fire={fire_dist:.2f}m, Final distance to start={start_dist:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    controller = Round1Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
