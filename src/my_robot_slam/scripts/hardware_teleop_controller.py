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
        
        # Set up QoS profiles
        latching_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )
        
        # Publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # TF broadcaster for robot position
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing joint states and updating position
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
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
        
        # Robot parameters from URDF - ADJUST THESE TO MATCH YOUR PHYSICAL ROBOT
        self.wheel_radius = 0.05  # in meters
        self.wheel_base = 0.22  # Distance between front and rear axles
        self.track_width = 0.15  # Distance between left and right wheels
        self.max_steering_angle = 0.5  # radians
        
        # Velocity commands
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # Speed and steering control factors
        self.speed_scale = 0.3       # Scale factor to reduce linear velocity
        self.steering_response = 1.2  # For steering angle calculation
        self.steering_rate = 0.1      # For steering angle updates
        
        # Hardware control parameters
        self.motor_max_pwm = 255     # Maximum PWM value for motors (0-255 for Arduino PWM)
        self.motor_min_pwm = 50      # Minimum PWM to overcome stiction
        
        # Initialize hardware
        self.init_hardware()
        
        self.get_logger().info('Hardware Robot controller initialized')
    
    def init_hardware(self):
        """Initialize hardware motors and encoders - MODIFY FOR YOUR HARDWARE"""
        try:
            # === HARDWARE INITIALIZATION CODE - MODIFY FOR YOUR SETUP ===
            
            # EXAMPLE 1: For Raspberry Pi GPIO direct control
            """
            import RPi.GPIO as GPIO
            # Motor A - Left
            self.LEFT_MOTOR_PIN1 = 17  # GPIO pin numbers
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
            """
            
            # EXAMPLE 2: For Adafruit Motor HAT
            """
            from adafruit_motorkit import MotorKit
            
            self.kit = MotorKit()  # Initialize motor HAT
            self.left_motor = self.kit.motor1  # Motor 1 is left
            self.right_motor = self.kit.motor2  # Motor 2 is right
            """
            
            # EXAMPLE 3: For Serial Arduino Communication
            """
            import serial
            self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            """
            
            # EXAMPLE 4: For ROS2 Arduino Bridge
            # (In this case, just publish to cmd_vel and use a bridge node)
            
            # === ENCODER INITIALIZATION - IF USING ENCODERS ===
            """
            # Set up encoder pins
            self.LEFT_ENCODER_A = 5
            self.LEFT_ENCODER_B = 6
            self.RIGHT_ENCODER_A = 26
            self.RIGHT_ENCODER_B = 19
            
            GPIO.setup(self.LEFT_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.LEFT_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.RIGHT_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.RIGHT_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Set up encoder callbacks
            GPIO.add_event_detect(self.LEFT_ENCODER_A, GPIO.RISING, callback=self.left_encoder_callback)
            GPIO.add_event_detect(self.RIGHT_ENCODER_A, GPIO.RISING, callback=self.right_encoder_callback)
            
            # Encoder tracking variables
            self.left_encoder_count = 0
            self.right_encoder_count = 0
            self.encoder_resolution = 12  # Ticks per revolution (adjust for your encoder)
            """
            
            # FOR NOW, WE'LL USE PLACEHOLDERS - MODIFY THIS WITH YOUR ACTUAL HARDWARE CODE
            self.hardware_ready = True
            self.get_logger().info('Hardware initialized successfully - PLACEHOLDER')
            
        except Exception as e:
            self.hardware_ready = False
            self.get_logger().error(f'Failed to initialize hardware: {str(e)}')
            self.get_logger().warn('Running in simulation mode only')
    
    def send_motor_commands(self):
        """Send commands to hardware motors - MODIFY FOR YOUR HARDWARE"""
        if not hasattr(self, 'hardware_ready') or not self.hardware_ready:
            return
        
        try:
            # Calculate motor speeds from linear_x and angular_z (differential drive)
            # For differential drive robots (most common)
            left_power = self.linear_x - (self.angular_z * self.track_width / 2.0)
            right_power = self.linear_x + (self.angular_z * self.track_width / 2.0)
            
            # For Ackermann steering (car-like robots)
            # This would need a more complex model with servo for steering
            # and motor control for drive wheels
            
            # Scale to motor PWM range
            left_pwm = self.scale_to_pwm(left_power)
            right_pwm = self.scale_to_pwm(right_power)
            
            # === SEND COMMANDS TO HARDWARE - MODIFY FOR YOUR SETUP ===
            
            # EXAMPLE 1: Raspberry Pi GPIO
            """
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
            """
            
            # EXAMPLE 2: Adafruit Motor HAT
            """
            # Motor values should be between -1.0 and 1.0
            self.left_motor.throttle = left_pwm / 100.0  # Convert percentage to -1.0 to 1.0 range
            self.right_motor.throttle = right_pwm / 100.0
            """
            
            # EXAMPLE 3: Arduino Serial
            """
            # Send command as a string: "M,left_pwm,right_pwm\n"
            command = f"M,{int(left_pwm)},{int(right_pwm)}\n"
            self.arduino.write(command.encode())
            """
            
            # PLACEHOLDER - LOG COMMANDS TO VERIFY LOGIC
            self.get_logger().info(f'Motor commands: L={left_pwm:.1f}, R={right_pwm:.1f}')
            
        except Exception as e:
            self.get_logger().error(f'Error sending motor commands: {str(e)}')
    
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
        """Process incoming teleop commands"""
        # Extract raw velocity commands
        raw_linear_x = msg.linear.x
        raw_linear_y = msg.linear.y
        raw_angular_z = msg.angular.z
        
        # Log raw commands for debugging
        self.get_logger().info(f'TELEOP RAW: x={raw_linear_x:.2f}, y={raw_linear_y:.2f}, rot={raw_angular_z:.2f}')
        
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
        
        # Debug info
        if abs(self.linear_x) > 0.001 or abs(self.angular_z) > 0.001:
            self.get_logger().info(
                f'x:{self.x:.2f}, y:{self.y:.2f}, θ:{self.theta:.2f}, '
                f'steering:{self.steering_angle:.2f}, vel:{self.linear_x:.2f}'
            )

def main(args=None):
    rclpy.init(args=args)  # This allows passing ROS args
    node = HardwareRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 