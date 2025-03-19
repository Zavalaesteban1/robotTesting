# ROS 2 Robot Maze Simulation

This repository contains a ROS 2 project for simulating a robot navigating through a maze environment within RViz. The project includes a configurable robot model, maze obstacle visualization, and keyboard teleoperation.

## Overview

This project demonstrates:
- Custom robot modeling with URDF
- Visualization of a robot model in RViz
- Generation and visualization of maze obstacles
- Teleoperation control of the robot
- Robot movement simulation

## Features

- **Robot Model**: A custom robot with wheels and steering mechanism
- **Maze Environment**: Procedurally generated maze with walls, columns, and obstacles
- **Visualization**: Custom RViz configuration for optimal visualization
- **Control**: Keyboard-based teleoperation for navigating the robot

## Prerequisites

- ROS 2 (Jazzy Jalisco)
- Python 3
- Colcon build tools
- Required ROS 2 packages:
  - robot_state_publisher
  - joint_state_publisher
  - teleop_twist_keyboard
  - rviz2
  - tf2_ros
  - xacro

## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/ros2-robot-maze.git
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_description my_robot_slam
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Launch the Robot Maze Simulation

```bash
ros2 launch my_robot_slam maze_sim.launch.py
```

This will:
- Start the robot state publisher
- Launch the obstacle publisher to create the maze
- Start RViz with the custom configuration
- Launch the teleop keyboard for robot control

### Control the Robot

Use the teleop keyboard window to navigate the robot:
- `i` - Move forward
- `k` - Stop
- `,` - Move backward
- `j` - Turn left
- `l` - Turn right
- `q` - Quit

### Customize the Maze

You can customize the maze by modifying the obstacle publisher script:
```
~/ros2_ws/src/my_robot_slam/scripts/obstacle_publisher.py
```

Parameters that can be adjusted include:
- Wall positions
- Column positions
- Obstacle positions, sizes, and colors
- Maze dimensions

## Project Structure

- **my_robot_description/**
  - **urdf/**: Robot URDF model
  - **rviz/**: RViz configuration files
  - **launch/**: Launch files for visualization

- **my_robot_slam/**
  - **launch/**: Launch files for simulation
  - **scripts/**: Python scripts for control and visualization
    - **obstacle_publisher.py**: Creates and publishes the maze obstacles
    - **teleop_to_joints.py**: Converts teleop commands to joint states

## Troubleshooting

### Common Issues

1. **Obstacles not visible in RViz**
   - Check that the Fixed Frame in RViz is set to "map"
   - Ensure both Marker and MarkerArray displays are enabled
   - Verify the obstacle publisher is running with `ros2 node list | grep obstacle`

2. **Robot not moving**
   - Check teleop terminal is active
   - Ensure the robot controller is running
   - Verify joint states are being published with `ros2 topic echo /joint_states`

3. **Transforms issues**
   - Check TF tree in RViz for missing transforms
   - Ensure static transform publishers are running

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ROS 2 community for documentation and examples
- Open source robotics tools that made this project possible 