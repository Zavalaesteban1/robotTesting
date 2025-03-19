#!/bin/bash

# This script will animate your robot by publishing joint states

# Source your workspace
source ~/ros2_ws/install/setup.bash

# Function to publish joint states
publish_joint_state() {
    local angle=$1
    local wheel_pos=$2
    
    # Publish joint state message with wheel rotation
    ros2 topic pub --once /joint_states sensor_msgs/msg/JointState "{
        header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
        name: ['front_left_wheel_joint', 'front_right_wheel_joint', 'left_rear_wheel_joint', 'right_rear_wheel_joint', 'steering_control_joint', 'front_left_pivot_joint', 'front_right_pivot_joint'],
        position: [$wheel_pos, $wheel_pos, $wheel_pos, $wheel_pos, $angle, $angle, $angle],
        velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }"
}

echo "Moving robot wheels and steering. Press Ctrl+C to stop..."

# Loop to animate the robot
wheel_pos=0
angle=0
angle_direction=1

while true; do
    # Update wheel position (continuously increasing for rotation)
    wheel_pos=$(echo "$wheel_pos + 0.1" | bc)
    
    # Update steering angle (oscillating between -0.4 and 0.4)
    angle=$(echo "$angle + 0.05 * $angle_direction" | bc)
    if (( $(echo "$angle > 0.4" | bc -l) )); then
        angle_direction=-1
    elif (( $(echo "$angle < -0.4" | bc -l) )); then
        angle_direction=1
    fi
    
    # Publish the joint state
    publish_joint_state $angle $wheel_pos
    
    # Sleep to control animation speed
    sleep 0.1
done 