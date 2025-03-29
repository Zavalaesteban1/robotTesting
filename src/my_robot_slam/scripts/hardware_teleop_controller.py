#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
import threading

# HARDWARE IMPORTS - MODIFY FOR YOUR SPECIFIC HARDWARE
# Example options (uncomment or add what you need):
# import RPi.GPIO as GPIO  # For Raspberry Pi GPIO
# from adafruit_motorkit import MotorKit  # For Adafruit Motor HAT
# import board  # For Adafruit boards
# import busio  # For I2C/SPI communication
# import serial  # For serial communication with motor controllers

class HardwareRobotController(Node):
    def __init__(self):
        super().__init__('hardware_robot_controller')
        
        # Get parameters from ROS
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_radius', 0.05),
                ('wheel_base', 0.22),
                ('track_width', 0.15),
                ('max_steering_angle', 0.5),
                ('speed_scale', 0.3),
                ('steering_response', 1.2),
                ('steering_rate', 0.1),
                ('motor_max_pwm', 255),
                ('motor_min_pwm', 50),
                ('hardware_type', 'simulation'),  # Options: 'simulation', 'gpio', 'adafruit', 'arduino', 'esp'
                ('serial_port', '/dev/ttyUSB0'),
                ('serial_baud', 115200),
                ('esp_command_format', 'M,{left},{right}'),  # Format for ESP commands
                ('debug_level', 'info'),  # Options: 'debug', 'info', 'warn', 'error'
                ('odom_frame', 'odom'),   # Frame ID for odometry
                ('base_frame', 'base_footprint'),  # Base frame ID
                ('use_esp_odometry', True),  # Whether to use odometry data from ESP32
                ('esp_odometry_format', 'O,{x},{y},{theta},{vx},{vtheta}')  # Format for ESP odometry data
            ]
        )
        
        # Load parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.speed_scale = self.get_parameter('speed_scale').value
        self.steering_response = self.get_parameter('steering_response').value
        self.steering_rate = self.get_parameter('steering_rate').value
        self.motor_max_pwm = self.get_parameter('motor_max_pwm').value
        self.motor_min_pwm = self.get_parameter('motor_min_pwm').value
        self.hardware_type = self.get_parameter('hardware_type').value
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value
        self.esp_command_format = self.get_parameter('esp_command_format').value
        self.debug_level = self.get_parameter('debug_level').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.use_esp_odometry = self.get_parameter('use_esp_odometry').value
        self.esp_odometry_format = self.get_parameter('esp_odometry_format').value
        
        # Set logging level based on parameter
        if self.debug_level == 'debug':
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        elif self.debug_level == 'info':
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        elif self.debug_level == 'warn':
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        elif self.debug_level == 'error':
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        
        # Set up QoS profiles
        latching_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )
        
        # Publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Add odometry publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Subscribe to cmd_vel from terminal-based teleop
        # Using a reliable QoS for better terminal input reliability
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10)
        
        # TF broadcaster for robot position
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing joint states and updating position - 20Hz update rate
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Robot state
        self.wheel_pos = 0.0
        self.left_wheel_pos = 0.0  # Track individual wheel positions
        self.right_wheel_pos = 0.0
        self.steering_angle = 0.0
        self.target_steering_angle = 0.0
        
        # Robot position (x, y, theta) in map frame - estimated from encoders/odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Velocity commands
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # Initialize hardware
        self.init_hardware()
        
        self.get_logger().info('Hardware Robot controller initialized with:')
        self.get_logger().info(f'  Hardware type: {self.hardware_type}')
        self.get_logger().info(f'  Wheel radius: {self.wheel_radius} m')
        self.get_logger().info(f'  Track width: {self.track_width} m')
        self.get_logger().info(f'  Speed scale: {self.speed_scale}')
    
    def init_hardware(self):
        """Initialize hardware motors and encoders based on hardware_type parameter"""
        try:
            if self.hardware_type == 'simulation':
                self.hardware_ready = True
                self.get_logger().info('Simulation mode: No hardware will be controlled')
                
            elif self.hardware_type == 'gpio':
                # Raspberry Pi GPIO initialization
                try:
                    import RPi.GPIO as GPIO
                    
                    # Motor A - Left
                    self.LEFT_MOTOR_PIN1 = 17  # GPIO pin numbers - ADJUST FOR YOUR HARDWARE
                    self.LEFT_MOTOR_PIN2 = 18
                    self.LEFT_MOTOR_PWM_PIN = 12
                    # Motor B - Right
                    self.RIGHT_MOTOR_PIN1 = 22
                    self.RIGHT_MOTOR_PIN2 = 23
                    self.RIGHT_MOTOR_PWM_PIN = 13
                    
                    # Setup GPIO
                    GPIO.setmode(GPIO.BCM)
                    GPIO.setup(self.LEFT_MOTOR_PIN1, GPIO.OUT)
                    GPIO.setup(self.LEFT_MOTOR_PIN2, GPIO.OUT)
                    GPIO.setup(self.LEFT_MOTOR_PWM_PIN, GPIO.OUT)
                    GPIO.setup(self.RIGHT_MOTOR_PIN1, GPIO.OUT)
                    GPIO.setup(self.RIGHT_MOTOR_PIN2, GPIO.OUT)
                    GPIO.setup(self.RIGHT_MOTOR_PWM_PIN, GPIO.OUT)
                    
                    # Setup PWM
                    self.left_motor_pwm = GPIO.PWM(self.LEFT_MOTOR_PWM_PIN, 1000)  # 1000 Hz frequency
                    self.right_motor_pwm = GPIO.PWM(self.RIGHT_MOTOR_PWM_PIN, 1000)
                    self.left_motor_pwm.start(0)  # Start with 0% duty cycle
                    self.right_motor_pwm.start(0)
                    
                    # Register GPIO cleanup on node shutdown
                    self.hardware_ready = True
                    self.get_logger().info('GPIO initialized successfully')
                    
                except ImportError:
                    self.get_logger().error('Failed to import RPi.GPIO. Is this running on a Raspberry Pi?')
                    self.hardware_ready = False
                    
            elif self.hardware_type == 'adafruit':
                # Adafruit Motor HAT initialization
                try:
                    from adafruit_motorkit import MotorKit
                    
                    self.kit = MotorKit()  # Initialize motor HAT
                    self.left_motor = self.kit.motor1  # Motor 1 is left
                    self.right_motor = self.kit.motor2  # Motor 2 is right
                    
                    self.hardware_ready = True
                    self.get_logger().info('Adafruit Motor HAT initialized successfully')
                    
                except ImportError:
                    self.get_logger().error('Failed to import adafruit_motorkit. Is the library installed?')
                    self.hardware_ready = False
                    
            elif self.hardware_type == 'arduino':
                # Arduino Serial communication
                try:
                    import serial
                    
                    self.get_logger().info(f'Connecting to Arduino on {self.serial_port} at {self.serial_baud} baud')
                    self.arduino = serial.Serial(self.serial_port, self.serial_baud, timeout=1)
                    time.sleep(2)  # Wait for Arduino to reset
                    
                    self.hardware_ready = True
                    self.get_logger().info('Arduino serial connection established')
                    
                except (ImportError, serial.SerialException) as e:
                    self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
                    self.hardware_ready = False
            elif self.hardware_type == 'esp':
                # ESP Serial communication
                try:
                    import serial
                    
                    self.get_logger().info(f'Connecting to ESP on {self.serial_port} at {self.serial_baud} baud')
                    self.esp = serial.Serial(self.serial_port, self.serial_baud, timeout=1)
                    time.sleep(2)  # Wait for ESP to reset
                    
                    # Initialize ESP32 odometry variables
                    self.esp_odom_x = 0.0
                    self.esp_odom_y = 0.0
                    self.esp_odom_theta = 0.0
                    self.esp_odom_vx = 0.0
                    self.esp_odom_vtheta = 0.0
                    self.esp_odom_last_time = self.get_clock().now()
                    
                    # Start a thread to read from serial port if using ESP odometry
                    if self.use_esp_odometry:
                        self.running = True
                        self.serial_thread = threading.Thread(target=self.read_from_esp)
                        self.serial_thread.daemon = True
                        self.serial_thread.start()
                        self.get_logger().info('ESP odometry reading thread started')
                    
                    self.hardware_ready = True
                    self.get_logger().info('ESP serial connection established')
                    
                except (ImportError, serial.SerialException) as e:
                    self.get_logger().error(f'Failed to connect to ESP: {str(e)}')
                    self.hardware_ready = False
            else:
                self.get_logger().error(f'Unknown hardware type: {self.hardware_type}')
                self.hardware_ready = False
                
        except Exception as e:
            self.hardware_ready = False
            self.get_logger().error(f'Failed to initialize hardware: {str(e)}')
            self.get_logger().warn('Running in simulation mode only')
    
    def send_motor_commands(self):
        """Send commands to hardware motors based on hardware_type"""
        if not hasattr(self, 'hardware_ready') or not self.hardware_ready:
            return
        
        try:
            # Calculate motor speeds from linear_x and angular_z (differential drive)
            left_power = self.linear_x - (self.angular_z * self.track_width / 2.0)
            right_power = self.linear_x + (self.angular_z * self.track_width / 2.0)
            
            # Scale to motor PWM range
            left_pwm = self.scale_to_pwm(left_power)
            right_pwm = self.scale_to_pwm(right_power)
            
            # Send commands based on hardware type
            if self.hardware_type == 'simulation':
                # Just log the commands
                self.get_logger().debug(f'Simulation motor commands: L={left_pwm:.1f}, R={right_pwm:.1f}')
                
            elif self.hardware_type == 'gpio':
                import RPi.GPIO as GPIO
                
                # Set motor directions
                if left_pwm >= 0:
                    GPIO.output(self.LEFT_MOTOR_PIN1, GPIO.HIGH)
                    GPIO.output(self.LEFT_MOTOR_PIN2, GPIO.LOW)
                else:
                    GPIO.output(self.LEFT_MOTOR_PIN1, GPIO.LOW)
                    GPIO.output(self.LEFT_MOTOR_PIN2, GPIO.HIGH)
                    
                if right_pwm >= 0:
                    GPIO.output(self.RIGHT_MOTOR_PIN1, GPIO.HIGH)
                    GPIO.output(self.RIGHT_MOTOR_PIN2, GPIO.LOW)
                else:
                    GPIO.output(self.RIGHT_MOTOR_PIN1, GPIO.LOW)
                    GPIO.output(self.RIGHT_MOTOR_PIN2, GPIO.HIGH)
                
                # Set PWM values (use absolute value since direction is set by pins)
                self.left_motor_pwm.ChangeDutyCycle(abs(left_pwm))
                self.right_motor_pwm.ChangeDutyCycle(abs(right_pwm))
                
                self.get_logger().debug(f'GPIO motor commands: L={left_pwm:.1f}, R={right_pwm:.1f}')
                
            elif self.hardware_type == 'adafruit':
                # Motor values should be between -1.0 and 1.0
                self.left_motor.throttle = left_pwm / 100.0  # Convert percentage to -1.0 to 1.0 range
                self.right_motor.throttle = right_pwm / 100.0
                
                self.get_logger().debug(f'Adafruit motor commands: L={left_pwm/100.0:.2f}, R={right_pwm/100.0:.2f}')
                
            elif self.hardware_type == 'arduino':
                # Send command as a string: "M,left_pwm,right_pwm\n"
                command = f"M,{int(left_pwm)},{int(right_pwm)}\n"
                self.arduino.write(command.encode())
                
                self.get_logger().debug(f'Arduino motor commands: {command.strip()}')
            
            elif self.hardware_type == 'esp':
                # Send command as a string: "M,left_pwm,right_pwm\n"
                command = self.esp_command_format.format(left=int(left_pwm), right=int(right_pwm))
                self.esp.write(command.encode())
                
                self.get_logger().debug(f'ESP motor commands: {command.strip()}')
            
            # For any real hardware, log at info level when actually moving
            if self.hardware_type != 'simulation' and (abs(left_pwm) > 0 or abs(right_pwm) > 0):
                self.get_logger().info(f'Motor commands: L={left_pwm:.1f}, R={right_pwm:.1f}')
            
        except Exception as e:
            self.get_logger().error(f'Error sending motor commands: {str(e)}')
    
    def cleanup_hardware(self):
        """Cleanup hardware resources on shutdown"""
        if not hasattr(self, 'hardware_ready') or not self.hardware_ready:
            return
            
        try:
            if self.hardware_type == 'gpio':
                import RPi.GPIO as GPIO
                # Stop motors
                if hasattr(self, 'left_motor_pwm'):
                    self.left_motor_pwm.ChangeDutyCycle(0)
                if hasattr(self, 'right_motor_pwm'):
                    self.right_motor_pwm.ChangeDutyCycle(0)
                # Clean up GPIO
                GPIO.cleanup()
                self.get_logger().info('GPIO cleanup completed')
                
            elif self.hardware_type == 'adafruit':
                # Stop motors
                if hasattr(self, 'left_motor'):
                    self.left_motor.throttle = 0
                if hasattr(self, 'right_motor'):
                    self.right_motor.throttle = 0
                self.get_logger().info('Adafruit motors stopped')
                
            elif self.hardware_type == 'arduino':
                # Send stop command
                if hasattr(self, 'arduino'):
                    self.arduino.write(b"M,0,0\n")
                    self.arduino.close()
                    self.get_logger().info('Arduino connection closed')
                    
            elif self.hardware_type == 'esp':
                # Stop the serial reading thread
                if hasattr(self, 'running'):
                    self.running = False
                    if hasattr(self, 'serial_thread') and self.serial_thread.is_alive():
                        # Give thread time to exit gracefully
                        time.sleep(0.5)
                        self.get_logger().info('ESP serial thread stopped')
                
                # Send stop command
                if hasattr(self, 'esp'):
                    self.esp.write(b"M,0,0\n")
                    self.esp.close()
                    self.get_logger().info('ESP connection closed')
                    
        except Exception as e:
            self.get_logger().error(f'Error during hardware cleanup: {str(e)}')
    
    def scale_to_pwm(self, power):
        """Scale normalized power (-1.0 to 1.0) to PWM values"""
        # Apply deadzone and scale to PWM range
        normalized_power = max(-1.0, min(1.0, power))  # Clamp to -1.0 to 1.0
        
        # Apply deadzone (motors don't move until certain threshold)
        if abs(normalized_power) < 0.1:
            return 0
        
        # Scale to PWM range with minimum PWM to overcome stiction
        if normalized_power > 0:
            pwm = self.motor_min_pwm + (self.motor_max_pwm - self.motor_min_pwm) * normalized_power
        elif normalized_power < 0:
            pwm = -self.motor_min_pwm + (self.motor_min_pwm - self.motor_max_pwm) * normalized_power
        else:
            pwm = 0
            
        return pwm
    
    def read_encoders(self):
        """Read encoder values to track wheel movement - MODIFY FOR YOUR HARDWARE"""
        # This is a placeholder - implement with your actual encoder reading code
        # For now, we'll simulate encoder readings based on commanded velocity
        if hasattr(self, 'hardware_ready') and self.hardware_ready:
            # In a real implementation, you would read actual encoder values here
            # For now, we'll estimate based on commanded velocity
            dt = 0.05  # seconds (time step)
            wheel_circumference = 2 * math.pi * self.wheel_radius
            
            # Calculate theoretical wheel movement based on commanded velocities
            left_wheel_travel = (self.linear_x - (self.angular_z * self.track_width / 2.0)) * dt
            right_wheel_travel = (self.linear_x + (self.angular_z * self.track_width / 2.0)) * dt
            
            # Convert to wheel rotation (radians)
            left_wheel_rotation = left_wheel_travel / self.wheel_radius
            right_wheel_rotation = right_wheel_travel / self.wheel_radius
            
            # Update wheel positions
            self.left_wheel_pos += left_wheel_rotation
            self.right_wheel_pos += right_wheel_rotation
            
            # Use average for overall wheel position (for backwards compatibility)
            self.wheel_pos = (self.left_wheel_pos + self.right_wheel_pos) / 2.0
    
    def update_odometry(self):
        """Update the robot's odometry based on encoder data"""
        # Calculate the robot's movement since the last update
        dt = 0.05  # seconds (fixed time step)
        
        # In a real implementation, you would use actual encoder readings
        # For differential drive, calculate movement from left and right wheel rotations
        left_travel = (self.left_wheel_pos - self.prev_left_wheel_pos) * self.wheel_radius
        right_travel = (self.right_wheel_pos - self.prev_right_wheel_pos) * self.wheel_radius
        
        # Save current positions for next update
        self.prev_left_wheel_pos = self.left_wheel_pos
        self.prev_right_wheel_pos = self.right_wheel_pos
        
        # Calculate robot's linear and angular movement
        center_travel = (left_travel + right_travel) / 2.0
        rotation = (right_travel - left_travel) / self.track_width
        
        # Update robot's position and orientation
        self.theta += rotation
        self.x += center_travel * math.cos(self.theta)
        self.y += center_travel * math.sin(self.theta)
        
        # Normalize theta to keep it within [-π, π]
        while self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        while self.theta < -math.pi:
            self.theta += 2.0 * math.pi
    
    def cmd_vel_callback(self, msg):
        """Process incoming teleop commands from terminal or other sources"""
        # Extract raw velocity commands
        raw_linear_x = msg.linear.x
        raw_linear_y = msg.linear.y
        raw_angular_z = msg.angular.z
        
        # Log raw commands for debugging
        self.get_logger().debug(f'TELEOP RAW: x={raw_linear_x:.2f}, y={raw_linear_y:.2f}, rot={raw_angular_z:.2f}')
        
        # Process forward/backward motion with speed scaling
        self.linear_x = raw_linear_x * self.speed_scale
        
        # Handle lateral movement by converting to turning
        if abs(raw_linear_y) > 0.05 and abs(raw_linear_x) > 0.05:
            side_movement_dir = 1.0 if raw_linear_y > 0 else -1.0
            additional_steering = side_movement_dir * 0.5
            raw_angular_z = raw_angular_z + additional_steering
        
        # Process rotation/steering (invert sign for car-like steering)
        self.angular_z = -raw_angular_z
        
        # Compute target steering angle
        if abs(self.linear_x) > 0.01:
            # Active driving mode
            self.target_steering_angle = np.clip(
                self.angular_z * self.steering_response,
                -self.max_steering_angle,
                self.max_steering_angle
            )
        else:
            # Static turning mode with reduced response
            self.target_steering_angle = np.clip(
                self.angular_z * 0.5,
                -self.max_steering_angle,
                self.max_steering_angle
            )
            # Add small forward motion to visualize steering
            if abs(self.angular_z) > 0.05:
                self.linear_x = 0.05 * self.speed_scale
        
        # Log processed commands when they're substantial
        if abs(self.linear_x) > 0.01 or abs(self.angular_z) > 0.01:
            self.get_logger().info(f'Processed cmd_vel: linear={self.linear_x:.2f} m/s, angular={self.angular_z:.2f} rad/s')
        
        # Send commands to hardware motors
        self.send_motor_commands()
    
    def publish_joint_states(self):
        """Publish joint states for visualization in RViz"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Include all joint names from the URDF
        msg.name = [
            'front_left_wheel_joint', 
            'front_right_wheel_joint', 
            'left_rear_wheel_joint', 
            'right_rear_wheel_joint', 
            'steering_control_joint', 
            'front_left_pivot_joint', 
            'front_right_pivot_joint'
        ]
        
        # Use separate values for left and right wheels if available
        msg.position = [
            self.left_wheel_pos,   # front_left_wheel
            self.right_wheel_pos,  # front_right_wheel
            self.left_wheel_pos,   # left_rear_wheel
            self.right_wheel_pos,  # right_rear_wheel
            self.steering_angle,   # steering_control
            self.steering_angle,   # front_left_pivot
            self.steering_angle    # front_right_pivot
        ]
        
        # Set velocities
        left_wheel_vel = self.linear_x - (self.angular_z * self.track_width / 2.0)
        right_wheel_vel = self.linear_x + (self.angular_z * self.track_width / 2.0)
        left_wheel_vel = left_wheel_vel / self.wheel_radius if self.wheel_radius != 0 else 0
        right_wheel_vel = right_wheel_vel / self.wheel_radius if self.wheel_radius != 0 else 0
        
        msg.velocity = [
            left_wheel_vel,   # front_left_wheel
            right_wheel_vel,  # front_right_wheel
            left_wheel_vel,   # left_rear_wheel
            right_wheel_vel,  # right_rear_wheel
            0.0,             # steering_control
            0.0,             # front_left_pivot
            0.0              # front_right_pivot
        ]
        
        msg.effort = [0.0] * 7
        
        # Publish message
        self.joint_state_pub.publish(msg)
    
    def publish_robot_transform(self):
        """Publish the robot's position as a TF transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'
        
        # Set transform
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert theta to quaternion (rotation around Z axis)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)
    
    def timer_callback(self):
        """Main control loop"""
        # Initialize previous wheel positions on first run
        if not hasattr(self, 'prev_left_wheel_pos'):
            self.prev_left_wheel_pos = self.left_wheel_pos
            self.prev_right_wheel_pos = self.right_wheel_pos
        
        # Read encoder values (or simulate them)
        self.read_encoders()
        
        # Update robot's odometry position
        self.update_odometry()
        
        # Update steering angle
        if abs(self.target_steering_angle - self.steering_angle) > 0.01:
            direction = 1 if self.target_steering_angle > self.steering_angle else -1
            self.steering_angle += direction * self.steering_rate
            self.steering_angle = np.clip(self.steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        # Publish transform for robot position
        self.publish_robot_transform()
        
        # Publish joint states
        self.publish_joint_states()
        
        # Debug info - only log when actually moving
        if abs(self.linear_x) > 0.001 or abs(self.angular_z) > 0.001:
            self.get_logger().debug(
                f'Position: x={self.x:.2f}, y={self.y:.2f}, θ={self.theta:.2f}, '
                f'steering={self.steering_angle:.2f}, vel={self.linear_x:.2f}'
            )
    
    def read_from_esp(self):
        """Thread function to read odometry data from ESP32"""
        import serial
        
        self.get_logger().info('ESP serial reading thread started')
        
        while self.running and hasattr(self, 'esp') and self.esp.is_open:
            try:
                # Read line from ESP
                if self.esp.in_waiting > 0:
                    line = self.esp.readline().decode('utf-8').strip()
                    
                    # Process odometry data
                    if line.startswith('O,'):  # 'O' for Odometry
                        self.process_esp_odometry(line)
                    elif line:
                        self.get_logger().debug(f'ESP data: {line}')
                
                # Small sleep to prevent CPU overuse
                time.sleep(0.001)
                
            except Exception as e:
                self.get_logger().error(f'Error reading from ESP: {str(e)}')
                time.sleep(1.0)  # Wait before retrying
        
        self.get_logger().info('ESP serial reading thread stopped')
    
    def process_esp_odometry(self, data):
        """Process odometry data received from ESP32"""
        try:
            # Parse odometry data from ESP format: 'O,x,y,theta,vx,vtheta'
            parts = data.split(',')
            if len(parts) >= 6:  # 'O' + 5 values
                self.esp_odom_x = float(parts[1])
                self.esp_odom_y = float(parts[2])
                self.esp_odom_theta = float(parts[3])
                self.esp_odom_vx = float(parts[4])
                self.esp_odom_vtheta = float(parts[5])
                
                # Publish odometry
                self.publish_esp_odometry()
                
                # Debug info
                self.get_logger().debug(f'ESP Odometry: x={self.esp_odom_x:.2f}, y={self.esp_odom_y:.2f}, θ={self.esp_odom_theta:.2f}')
                
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Error parsing ESP odometry data "{data}": {str(e)}')
    
    def publish_esp_odometry(self):
        """Publish odometry data from ESP32 to ROS"""
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Set position
        odom.pose.pose.position.x = self.esp_odom_x
        odom.pose.pose.position.y = self.esp_odom_y
        odom.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion (rotation around Z axis)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.esp_odom_theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.esp_odom_theta / 2)
        
        # Set velocity
        odom.twist.twist.linear.x = self.esp_odom_vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.esp_odom_vtheta
        
        # Publish the odometry message
        self.odom_pub.publish(odom)
        
        # Also publish as a TF transform if not using robot_localization
        # This is useful for visualization in RViz
        if hasattr(self, 'tf_broadcaster'):
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            
            # Set transform
            t.transform.translation.x = self.esp_odom_x
            t.transform.translation.y = self.esp_odom_y
            t.transform.translation.z = 0.0
            
            # Set rotation
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(self.esp_odom_theta / 2)
            t.transform.rotation.w = math.cos(self.esp_odom_theta / 2)
            
            # Broadcast transform
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareRobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure proper hardware cleanup
        node.cleanup_hardware()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 