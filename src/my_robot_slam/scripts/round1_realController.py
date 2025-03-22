#!/usr/bin/env python3
# round1_realController.py - A* Navigation for Fire Detection Competition

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
import heapq
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

class AStar:
    """A* pathfinding algorithm implementation"""
    
    class Node:
        """Node for A* search"""
        def __init__(self, x, y, cost=0, heuristic=0):
            self.x = x
            self.y = y
            self.cost = cost          # g-score (cost from start)
            self.heuristic = heuristic # h-score (heuristic to goal)
            self.parent = None
            
        def __lt__(self, other):
            # For priority queue comparison
            return (self.cost + self.heuristic) < (other.cost + other.heuristic)
            
        def __eq__(self, other):
            if other is None:
                return False
            return self.x == other.x and self.y == other.y
            
        def __hash__(self):
            return hash((self.x, self.y))

    def __init__(self, grid_resolution=0.2):
        """Initialize A* with grid resolution (m per cell)"""
        self.grid_resolution = grid_resolution
        self.obstacles = set()  # Set of (x, y) tuples representing obstacle positions
        self.grid_bounds = {"min_x": -10, "max_x": 10, "min_y": -10, "max_y": 10}
        
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int(round(x / self.grid_resolution))
        grid_y = int(round(y / self.grid_resolution))
        return grid_x, grid_y
        
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        world_x = grid_x * self.grid_resolution
        world_y = grid_y * self.grid_resolution
        return world_x, world_y
    
    def add_obstacle(self, x, y, inflation_radius=0.3):
        """Add obstacle at world coordinates (x,y) with inflation radius"""
        # Convert to grid coordinates
        grid_x, grid_y = self.world_to_grid(x, y)
        
        # Add the obstacle cell and inflate it
        inflation_cells = int(round(inflation_radius / self.grid_resolution))
        
        for dx in range(-inflation_cells, inflation_cells + 1):
            for dy in range(-inflation_cells, inflation_cells + 1):
                # Check if point is within inflation radius
                if dx**2 + dy**2 <= inflation_cells**2:
                    self.obstacles.add((grid_x + dx, grid_y + dy))
    
    def clear_obstacles(self):
        """Clear all obstacles"""
        self.obstacles = set()
    
    def get_neighbors(self, node):
        """Get valid neighbor cells for the given node"""
        neighbors = []
        # 8-connected grid (including diagonals)
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), 
                      (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            nx, ny = node.x + dx, node.y + dy
            
            # Check bounds
            if (nx < self.grid_bounds["min_x"] / self.grid_resolution or 
                nx > self.grid_bounds["max_x"] / self.grid_resolution or
                ny < self.grid_bounds["min_y"] / self.grid_resolution or
                ny > self.grid_bounds["max_y"] / self.grid_resolution):
                continue
                
            # Skip obstacles
            if (nx, ny) in self.obstacles:
                continue
                
            # Calculate cost (diagonal movement costs more)
            cost = 1.0 if dx == 0 or dy == 0 else 1.414
            
            neighbors.append((nx, ny, cost))
            
        return neighbors
    
    def heuristic(self, x1, y1, x2, y2):
        """Calculate heuristic (Euclidean distance)"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def find_path(self, start_x, start_y, goal_x, goal_y):
        """Find path from start to goal using A* algorithm
        
        Args:
            start_x, start_y: World coordinates of start position
            goal_x, goal_y: World coordinates of goal position
            
        Returns:
            List of (x,y) world coordinates for the path, or None if no path found
        """
        # Convert to grid coordinates
        start_grid_x, start_grid_y = self.world_to_grid(start_x, start_y)
        goal_grid_x, goal_grid_y = self.world_to_grid(goal_x, goal_y)
        
        # Check if start or goal is in an obstacle
        if (start_grid_x, start_grid_y) in self.obstacles:
            print(f"Start position {start_x},{start_y} is in an obstacle!")
            # Try to find nearest non-obstacle cell
            for radius in range(1, 10):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        if dx**2 + dy**2 <= radius**2:
                            if (start_grid_x + dx, start_grid_y + dy) not in self.obstacles:
                                start_grid_x, start_grid_y = start_grid_x + dx, start_grid_y + dy
                                print(f"Moved start to {self.grid_to_world(start_grid_x, start_grid_y)}")
                                break
                    else:
                        continue
                    break
                else:
                    continue
                break
        
        if (goal_grid_x, goal_grid_y) in self.obstacles:
            print(f"Goal position {goal_x},{goal_y} is in an obstacle!")
            # Try to find nearest non-obstacle cell for goal
            for radius in range(1, 10):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        if dx**2 + dy**2 <= radius**2:
                            if (goal_grid_x + dx, goal_grid_y + dy) not in self.obstacles:
                                goal_grid_x, goal_grid_y = goal_grid_x + dx, goal_grid_y + dy
                                print(f"Moved goal to {self.grid_to_world(goal_grid_x, goal_grid_y)}")
                                break
                    else:
                        continue
                    break
                else:
                    continue
                break

        # Start A* search
        start_node = self.Node(start_grid_x, start_grid_y)
        goal_node = self.Node(goal_grid_x, goal_grid_y)
        
        # Initialize the open and closed sets
        open_set = []
        heapq.heappush(open_set, start_node)
        open_set_dict = {(start_node.x, start_node.y): start_node}
        closed_set = {}  # (x,y) -> node
        
        # Main A* loop
        while open_set:
            # Get node with lowest f-score
            current = heapq.heappop(open_set)
            
            # Remove from dictionary too
            del open_set_dict[(current.x, current.y)]
            
            # Check if we reached the goal
            if current.x == goal_node.x and current.y == goal_node.y:
                # Reconstruct path
                path = []
                while current:
                    # Convert grid coordinates back to world coordinates
                    world_x, world_y = self.grid_to_world(current.x, current.y)
                    path.append((world_x, world_y))
                    current = current.parent
                    
                path.reverse()  # Reverse to get start-to-goal order
                return path
                
            # Add to closed set
            closed_set[(current.x, current.y)] = current
            
            # Check all neighbors
            for nx, ny, move_cost in self.get_neighbors(current):
                # Skip if already in closed set
                if (nx, ny) in closed_set:
                    continue
                    
                # Calculate cost to this neighbor
                tentative_cost = current.cost + move_cost
                
                # Check if this node is already in open set
                if (nx, ny) in open_set_dict:
                    neighbor = open_set_dict[(nx, ny)]
                    if tentative_cost < neighbor.cost:
                        # Found a better path to this node, update it
                        neighbor.cost = tentative_cost
                        neighbor.parent = current
                        # Reheapify (remove and add again)
                        open_set.remove(neighbor)
                        heapq.heapify(open_set)
                        heapq.heappush(open_set, neighbor)
                else:
                    # New node, add to open set
                    h = self.heuristic(nx, ny, goal_node.x, goal_node.y)
                    neighbor = self.Node(nx, ny, tentative_cost, h)
                    neighbor.parent = current
                    heapq.heappush(open_set, neighbor)
                    open_set_dict[(nx, ny)] = neighbor
        
        # If we get here, no path was found
        print("No path found!")
        return None


class Round1RealController(Node):
    """ROS2 node for Round 1 competition controller using A* navigation"""
    
    def __init__(self):
        super().__init__('round1_real_controller')
        
        # Declare parameters with defaults
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('obstacle_threshold', 0.8)
        self.declare_parameter('grid_resolution', 0.2)
        self.declare_parameter('inflation_radius', 0.3)
        self.declare_parameter('max_velocity_linear', 0.2)
        self.declare_parameter('max_velocity_angular', 0.5)
        
        # Get parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.max_vel_linear = self.get_parameter('max_velocity_linear').value
        self.max_vel_angular = self.get_parameter('max_velocity_angular').value
        
        # Log key parameters
        self.get_logger().info(f'Parameters loaded:')
        self.get_logger().info(f'  grid_resolution: {self.grid_resolution}')
        self.get_logger().info(f'  inflation_radius: {self.inflation_radius}')
        self.get_logger().info(f'  max_vel_linear: {self.max_vel_linear}')
        self.get_logger().info(f'  max_vel_angular: {self.max_vel_angular}')
        
        # Initialize A* planner
        self.astar = AStar(grid_resolution=self.grid_resolution)
        
        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.path_pub = self.create_publisher(MarkerArray, 'path_markers', 10)
        self.grid_pub = self.create_publisher(MarkerArray, 'grid_markers', 10)
        
        # Create QoS profile for better scan reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile=qos_profile
        )
        
        self.marker_sub = self.create_subscription(
            Marker,
            'visualization_marker',
            self.marker_callback,
            10
        )
        
        # Initialize state variables
        self.robot_position = None
        self.start_position = None
        self.fire_position = None
        self.current_path = None
        self.path_index = 0
        self.target_position = None
        self.obstacles_detected = 0
        self.laser_data = None
        self.grid_updated = False
        self.map_bounds = {"min_x": -10, "max_x": 10, "min_y": -10, "max_y": 10}
        
        # State machine states
        self.STATE_INIT = "INITIALIZE"
        self.STATE_EXPLORE = "EXPLORE" 
        self.STATE_RETURN = "RETURN"
        self.STATE_FINISHED = "FINISHED"
        self.state = self.STATE_INIT
        
        # Exploration variables
        self.exploration_targets = []
        self.current_exploration_target = None
        self.exploration_radius = 5.0  # meters
        self.exploration_points = 8   # number of points around the perimeter
        
        # Performance metrics
        self.start_time = time.time()
        self.closest_to_fire = float('inf')
        
        # Create timers
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        self.path_planning_timer = self.create_timer(1.0, self.path_planning_loop)  # 1Hz planning
        self.grid_update_timer = self.create_timer(2.0, self.update_grid)  # Update grid every 2 seconds
        self.visualization_timer = self.create_timer(0.5, self.publish_visualizations)  # 2Hz visualization
        
        # Transform check timer
        self.tf_check_timer = self.create_timer(1.0, self.check_transforms)
        self.transform_ready = False
        
        self.get_logger().info("Round 1 Real Controller with A* Navigation initialized")

    def check_transforms(self):
        """Check if transforms are available and log debugging info"""
        try:
            # Try to get the transform from map to base_footprint
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            self.transform_ready = True
            self.get_logger().debug("Transform from map to base_footprint is available")
        except Exception as e:
            # Try alternate transform paths if main one fails
            try:
                # Try map to base_link instead
                transform = self.tf_buffer.lookup_transform(
                    'map', 
                    'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                self.transform_ready = True
                self.get_logger().debug("Transform from map to base_link is available")
            except Exception as e2:
                self.get_logger().warning(f"Transform not available: {e2}")
                self.transform_ready = False
    
    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        self.laser_data = msg
        
        # Don't process scan if we don't have a valid robot position
        if not self.transform_ready or not self.get_robot_position():
            return
            
        # Process the scan data to identify obstacles
        self.process_scan_data(msg)
        
        # Grid has been updated with new laser data
        self.grid_updated = True
    
    def process_scan_data(self, scan_msg):
        """Process laser scan data to identify obstacles"""
        if not self.get_robot_position():
            return
            
        robot_pos = self.get_robot_position()
        robot_x, robot_y, robot_yaw = robot_pos
        
        # Convert laser scan to obstacle points in world frame
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Filter out invalid readings
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges) & (ranges < scan_msg.range_max) & (ranges > scan_msg.range_min)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        obstacles = []
        
        # Process each valid laser reading
        for i, (r, angle) in enumerate(zip(valid_ranges, valid_angles)):
            # Skip points that are too far or too close
            if r > 5.0 or r < 0.1:
                continue
                
            # Check if this is an obstacle (within threshold)
            if r < self.obstacle_threshold:
                # Calculate the position in the world frame
                obstacle_x = robot_x + r * math.cos(robot_yaw + angle)
                obstacle_y = robot_y + r * math.sin(robot_yaw + angle)
                
                obstacles.append((obstacle_x, obstacle_y))
                self.obstacles_detected += 1
        
        # Update the A* grid with obstacles
        for obs_x, obs_y in obstacles:
            self.astar.add_obstacle(obs_x, obs_y, self.inflation_radius)
            
            # Update map bounds if needed
            self.map_bounds["min_x"] = min(self.map_bounds["min_x"], obs_x - 5.0)
            self.map_bounds["max_x"] = max(self.map_bounds["max_x"], obs_x + 5.0)
            self.map_bounds["min_y"] = min(self.map_bounds["min_y"], obs_y - 5.0)
            self.map_bounds["max_y"] = max(self.map_bounds["max_y"], obs_y + 5.0)
        
        # Update A* grid bounds
        self.astar.grid_bounds = self.map_bounds
    
    def marker_callback(self, msg):
        """Process incoming visualization markers"""
        # Check for fire markers - this would be provided by a fire detection node
        if msg.ns == 'fire' and not self.fire_position:
            self.fire_position = (msg.pose.position.x, msg.pose.position.y)
            self.get_logger().info(f"Fire detected at position: {self.fire_position}")
            
            # Create a special marker to highlight the detected fire
            self.publish_fire_marker(self.fire_position)
            
            # Switch to return mode if fire is found
            if self.state == self.STATE_EXPLORE:
                self.state = self.STATE_RETURN
                self.get_logger().info("Fire found! Switching to RETURN mode")
    
    def get_robot_position(self):
        """Get the robot's position in the world frame with retry logic"""
        if not self.transform_ready:
            return None
            
        try:
            # Get transform from map to base_footprint
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract yaw from quaternion
            q = transform.transform.rotation
            _, _, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
            
            # Store robot position
            self.robot_position = (x, y, yaw)
            return self.robot_position
            
        except Exception as e:
            # Try alternate transform (map to base_link)
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 
                    'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                
                # Extract yaw from quaternion
                q = transform.transform.rotation
                _, _, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
                
                # Store robot position
                self.robot_position = (x, y, yaw)
                return self.robot_position
                
            except Exception as e2:
                self.get_logger().warning(f"Failed to get robot position: {e2}")
                return None
    
    def euler_from_quaternion(self, x, y, z, w):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
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
        
        return roll, pitch, yaw
    
    def update_grid(self):
        """Update the A* grid with current knowledge"""
        if not self.transform_ready or not self.get_robot_position():
            return
            
        # No need to update if we haven't received new laser data
        if not self.grid_updated:
            return
            
        # Grid updated flag reset
        self.grid_updated = False
        
        # Log grid status
        obstacle_count = len(self.astar.obstacles)
        self.get_logger().debug(f"Grid updated: {obstacle_count} obstacles")
        
        # If we're in exploration mode, generate new exploration targets
        if self.state == self.STATE_EXPLORE and not self.exploration_targets:
            self.generate_exploration_targets()
    
    def generate_exploration_targets(self):
        """Generate exploration targets in a circular pattern around the robot"""
        if not self.get_robot_position():
            return
            
        robot_x, robot_y, _ = self.get_robot_position()
        
        # Clear previous targets
        self.exploration_targets = []
        
        # Generate points in a circle around the current position
        for i in range(self.exploration_points):
            angle = 2.0 * math.pi * i / self.exploration_points
            target_x = robot_x + self.exploration_radius * math.cos(angle)
            target_y = robot_y + self.exploration_radius * math.sin(angle)
            
            # Check if the target is in an obstacle
            grid_x, grid_y = self.astar.world_to_grid(target_x, target_y)
            if (grid_x, grid_y) not in self.astar.obstacles:
                self.exploration_targets.append((target_x, target_y))
        
        if self.exploration_targets:
            self.get_logger().info(f"Generated {len(self.exploration_targets)} exploration targets")
            # Visualize the exploration targets
            self.visualize_exploration_targets()
        else:
            # If all targets are in obstacles, reduce the radius and try again
            self.exploration_radius *= 0.8
            if self.exploration_radius < 1.0:
                self.exploration_radius = 1.0
            self.get_logger().info(f"All targets in obstacles, reducing radius to {self.exploration_radius}")
    
    def publish_fire_marker(self, position):
        """Publish a marker indicating that fire has been detected"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fire_detected"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.5
        
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        # Green to indicate detection (not the fire itself)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        
        marker.lifetime.sec = 0  # 0 = forever
        
        self.marker_pub.publish(marker)
    
    def visualize_exploration_targets(self):
        """Visualize exploration targets in RViz"""
        marker_array = MarkerArray()
        
        for i, (target_x, target_y) in enumerate(self.exploration_targets):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "exploration_targets"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = target_x
            marker.pose.position.y = target_y
            marker.pose.position.z = 0.1
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # Blue color for exploration targets
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)
            
            marker.lifetime.sec = 10  # 10 seconds
            
            marker_array.markers.append(marker)
            
        if marker_array.markers:
            self.marker_array_pub.publish(marker_array)

    def path_planning_loop(self):
        """Main path planning loop - runs at 1Hz"""
        if not self.transform_ready or not self.get_robot_position():
            return
            
        robot_pos = self.get_robot_position()
        robot_x, robot_y, _ = robot_pos
        
        # If this is first run, store the start position
        if self.start_position is None:
            self.start_position = (robot_x, robot_y)
            self.get_logger().info(f"Start position recorded: {self.start_position}")
        
        # State machine
        if self.state == self.STATE_INIT:
            # Initialization phase
            if self.transform_ready:
                self.state = self.STATE_EXPLORE
                self.get_logger().info("Switching to EXPLORE mode")
        
        elif self.state == self.STATE_EXPLORE:
            # Exploration phase - find the fire
            
            # If fire is found, switch to return mode
            if self.fire_position:
                self.state = self.STATE_RETURN
                self.get_logger().info("Fire found during exploration! Switching to RETURN mode")
                # Reset the path to force a new path calculation
                self.current_path = None
                return
                
            # If we have a current path and haven't reached the end, continue following it
            if self.current_path and self.path_index < len(self.current_path):
                # Update distance to fire if we know where it is
                if self.fire_position:
                    dist_to_fire = math.sqrt((robot_x - self.fire_position[0])**2 + 
                                            (robot_y - self.fire_position[1])**2)
                    if dist_to_fire < self.closest_to_fire:
                        self.closest_to_fire = dist_to_fire
                        self.get_logger().info(f"New closest distance to fire: {self.closest_to_fire:.2f}m")
                        
                # Check if we've reached the current target
                target = self.current_path[self.path_index]
                dist_to_target = math.sqrt((robot_x - target[0])**2 + (robot_y - target[1])**2)
                
                if dist_to_target < 0.2:  # Target reached threshold
                    self.path_index += 1
                    self.get_logger().info(f"Reached waypoint {self.path_index}/{len(self.current_path)}")
                    
                    if self.path_index >= len(self.current_path):
                        # Reached the end of the current path
                        self.get_logger().info("Reached end of current path")
                        self.current_path = None
                        self.path_index = 0
                
            # If we don't have a path or have reached the end of the current one,
            # choose a new exploration target
            else:
                # If we have exploration targets, choose the next one
                if self.exploration_targets:
                    # Sort targets by distance to robot
                    self.exploration_targets.sort(key=lambda t: 
                        math.sqrt((t[0] - robot_x)**2 + (t[1] - robot_y)**2))
                    
                    # Select the nearest target
                    target = self.exploration_targets.pop(0)
                    self.get_logger().info(f"Selected exploration target: {target}")
                    
                    # Find a path to the target
                    path = self.astar.find_path(robot_x, robot_y, target[0], target[1])
                    
                    if path:
                        self.current_path = path
                        self.path_index = 0
                        self.target_position = target
                        self.get_logger().info(f"Path found with {len(path)} waypoints")
                        # Visualize the path
                        self.visualize_path(path)
                    else:
                        self.get_logger().warning(f"Could not find path to target {target}")
                        # Try a different target next time
                
                # If we have no more exploration targets, generate new ones
                else:
                    self.generate_exploration_targets()
        
        elif self.state == self.STATE_RETURN:
            # Return to start phase after finding the fire
            
            # If we've already reached the start position
            if self.current_path is None or self.path_index >= len(self.current_path):
                # Calculate path from current position to start
                if self.start_position:
                    self.get_logger().info(f"Planning return path to start: {self.start_position}")
                    
                    path = self.astar.find_path(robot_x, robot_y, 
                                               self.start_position[0], self.start_position[1])
                    
                    if path:
                        self.current_path = path
                        self.path_index = 0
                        self.get_logger().info(f"Return path found with {len(path)} waypoints")
                        # Visualize the return path
                        self.visualize_path(path, is_return=True)
                    else:
                        self.get_logger().warning("Could not find path back to start!")
                        # Try a simpler direct path
                        direct_path = [(robot_x, robot_y), self.start_position]
                        self.current_path = direct_path
                        self.path_index = 0
                        self.get_logger().warning("Using direct path to start (no obstacle avoidance)")
                
            # Check if we've reached the start position
            if self.start_position:
                dist_to_start = math.sqrt((robot_x - self.start_position[0])**2 + 
                                         (robot_y - self.start_position[1])**2)
                                         
                if dist_to_start < 0.2:  # Close enough to start
                    self.get_logger().info(f"Reached start position! Mission complete!")
                    self.state = self.STATE_FINISHED
                    
                    # Calculate final metrics
                    elapsed_time = time.time() - self.start_time
                    self.get_logger().info(f"Mission completed in {elapsed_time:.1f} seconds")
                    self.get_logger().info(f"Closest distance to fire: {self.closest_to_fire:.2f}m")
                    self.get_logger().info(f"Final distance to start: {dist_to_start:.2f}m")
                    self.get_logger().info(f"Obstacles detected: {self.obstacles_detected}")
    
    def control_loop(self):
        """Robot control loop - runs at 10Hz"""
        if not self.transform_ready or not self.get_robot_position():
            # Not ready for control yet
            return
            
        robot_pos = self.get_robot_position()
        robot_x, robot_y, robot_yaw = robot_pos
        
        # Initialize motion command
        cmd = Twist()
        
        # State machine for control
        if self.state == self.STATE_INIT:
            # In initialization, do a small rotation to help localization
            cmd.angular.z = 0.2
        
        elif self.state == self.STATE_FINISHED:
            # Mission complete, stop the robot
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        elif self.current_path and self.path_index < len(self.current_path):
            # We have a path to follow
            target = self.current_path[self.path_index]
            
            # Calculate distance and angle to target
            dx = target[0] - robot_x
            dy = target[1] - robot_y
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            
            # Calculate angle difference (-pi to pi)
            angle_diff = target_angle - robot_yaw
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
                
            # Check if we've reached the current waypoint
            if distance < 0.2:  # Target reached threshold
                self.path_index += 1
                self.get_logger().info(f"Reached waypoint {self.path_index}/{len(self.current_path)}")
                return  # Let the planning loop handle the next waypoint
            
            # Different motion strategy based on angle to target
            if abs(angle_diff) > 0.3:  # ~17 degrees
                # First align with the target
                cmd.linear.x = 0.0  # Stop forward motion when turning
                cmd.angular.z = 0.3 * angle_diff  # Turn proportionally
                self.get_logger().debug(f"Aligning with target, angle diff: {angle_diff:.2f}")
            else:
                # Move toward the target
                # Proportional control based on distance and angle
                cmd.linear.x = min(self.max_vel_linear, 0.5 * distance)
                cmd.angular.z = 0.3 * angle_diff  # Smaller steering correction
                
                # Slow down when approaching the target
                if distance < 0.5:
                    cmd.linear.x *= distance / 0.5
                
                self.get_logger().debug(f"Moving to target, distance: {distance:.2f}, speed: {cmd.linear.x:.2f}")
        
        else:
            # No path to follow, just stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        # Apply obstacle avoidance override if needed
        cmd = self.apply_obstacle_avoidance(cmd)
        
        # Publish command to the robot
        self.cmd_vel_pub.publish(cmd)
    
    def apply_obstacle_avoidance(self, cmd):
        """Override commanded velocity with obstacle avoidance behavior if needed"""
        if not self.laser_data:
            return cmd  # No laser data yet
            
        # Get the laser ranges
        ranges = np.array(self.laser_data.ranges)
        angles = np.linspace(self.laser_data.angle_min, self.laser_data.angle_max, len(ranges))
        
        # Filter out invalid readings
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) == 0:
            return cmd  # No valid readings
            
        # Check minimum distances in different sectors
        # Forward sector (within 30 degrees of straight ahead)
        forward_mask = np.abs(valid_angles) < (30 * math.pi / 180)
        if np.any(forward_mask):
            forward_min = np.min(valid_ranges[forward_mask])
            
            # Emergency stop for obstacles directly ahead
            if forward_min < self.safe_distance:
                self.get_logger().warn(f"Obstacle ahead at {forward_min:.2f}m - emergency stop")
                
                # Stop forward motion
                cmd.linear.x = 0.0
                
                # Turn away from the obstacle
                if cmd.angular.z == 0.0:
                    # If not already turning, choose a random direction
                    cmd.angular.z = 0.5 if np.random.random() > 0.5 else -0.5
                
                return cmd
        
        # Check left and right sectors for closer obstacles during turns
        if abs(cmd.angular.z) > 0.1:  # We're turning
            # Check the side we're turning toward
            if cmd.angular.z > 0:  # Turning left
                left_mask = (valid_angles > 0) & (valid_angles < (90 * math.pi / 180))
                if np.any(left_mask):
                    left_min = np.min(valid_ranges[left_mask])
                    if left_min < self.safe_distance:
                        self.get_logger().warn(f"Obstacle on left at {left_min:.2f}m - reversing turn")
                        cmd.angular.z = -0.3  # Turn right instead
            else:  # Turning right
                right_mask = (valid_angles < 0) & (valid_angles > (-90 * math.pi / 180))
                if np.any(right_mask):
                    right_min = np.min(valid_ranges[right_mask])
                    if right_min < self.safe_distance:
                        self.get_logger().warn(f"Obstacle on right at {right_min:.2f}m - reversing turn")
                        cmd.angular.z = 0.3  # Turn left instead
        
        return cmd
    
    def publish_visualizations(self):
        """Publish visualization markers for RViz"""
        if not self.transform_ready or not self.get_robot_position():
            return
            
        # Visualize the current path if available
        if self.current_path:
            self.visualize_path(self.current_path)
            
        # Visualize the grid (obstacle map) - this is expensive, so do it less frequently
        if np.random.random() < 0.2:  # Only update 20% of the time to reduce load
            self.visualize_grid()
            
        # Visualize the robot's position
        self.visualize_robot_position()
        
        # If we know the fire position, visualize it
        if self.fire_position:
            self.publish_fire_marker(self.fire_position)
    
    def visualize_path(self, path, is_return=False):
        """Visualize a path in RViz"""
        marker_array = MarkerArray()
        
        # Path line strip
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "path_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        line_marker.scale.x = 0.05  # Line width
        
        # Green for exploration, red for return
        if is_return:
            line_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        else:
            line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            
        line_marker.lifetime.sec = 1  # 1 second lifetime
        
        # Add points to the line strip
        for x, y in path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            line_marker.points.append(p)
            
        marker_array.markers.append(line_marker)
        
        # Waypoint markers
        for i, (x, y) in enumerate(path):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "path_waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.05
            
            # Current target is larger
            if i == self.path_index:
                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.15
                # Current target is yellow
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            else:
                marker.scale.x = 0.08
                marker.scale.y = 0.08
                marker.scale.z = 0.08
                # Other waypoints inherit path color (red or green)
                if is_return:
                    marker.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.8)
                else:
                    marker.color = ColorRGBA(r=0.3, g=1.0, b=0.3, a=0.8)
            
            marker.lifetime.sec = 1  # 1 second lifetime
            
            marker_array.markers.append(marker)
            
        self.path_pub.publish(marker_array)
    
    def visualize_grid(self):
        """Visualize the A* grid (obstacles) in RViz"""
        marker_array = MarkerArray()
        
        # Convert grid cells to world coordinates
        grid_cells = list(self.astar.obstacles)
        
        # Limit the number of cells to visualize to avoid overloading RViz
        if len(grid_cells) > 1000:
            # Randomly sample cells
            indices = np.random.choice(len(grid_cells), 1000, replace=False)
            grid_cells = [grid_cells[i] for i in indices]
        
        for i, (grid_x, grid_y) in enumerate(grid_cells):
            world_x, world_y = self.astar.grid_to_world(grid_x, grid_y)
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "grid_obstacles"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = 0.05
            
            # Size matches grid resolution
            marker.scale.x = self.grid_resolution
            marker.scale.y = self.grid_resolution
            marker.scale.z = 0.1
            
            # Red for obstacles
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            
            marker.lifetime.sec = 2  # 2 second lifetime
            
            marker_array.markers.append(marker)
            
        self.grid_pub.publish(marker_array)
    
    def visualize_robot_position(self):
        """Visualize the robot's current position and orientation"""
        robot_pos = self.get_robot_position()
        if not robot_pos:
            return
            
        robot_x, robot_y, robot_yaw = robot_pos
        
        # Create a marker for the robot
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_position"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose.position.x = robot_x
        marker.pose.position.y = robot_y
        marker.pose.position.z = 0.1
        
        # Set orientation from yaw
        cy = math.cos(robot_yaw * 0.5)
        sy = math.sin(robot_yaw * 0.5)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = sy
        marker.pose.orientation.w = cy
        
        # Arrow size
        marker.scale.x = 0.3  # Length
        marker.scale.y = 0.05  # Width
        marker.scale.z = 0.05  # Height
        
        # Blue for robot
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    controller = Round1RealController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
