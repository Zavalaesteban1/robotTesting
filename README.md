# ROS 2 Robot Maze Simulation

This repository contains a ROS 2 project for simulating a robot navigating through a competition terrain environment within RViz. The project includes a configurable robot model, obstacle visualization, and keyboard teleoperation.

## Overview

This project demonstrates:
- Custom robot modeling with URDF
- Visualization of a robot model in RViz
- Generation and visualization of a competition terrain with various obstacles
- Teleoperation control of the robot
- Robot movement simulation

## Features

- **Robot Model**: A custom robot with wheels and steering mechanism
- **Competition Terrain**: Procedurally generated environment with:
  - Boundary walls forming an 8ft x 8ft field
  - Tree/pole obstacles creating a maze-like structure
  - Corner zones designated as start and fire zones
  - Varied terrain areas (sand and gravel patches)
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

### Launch the Competition Terrain Simulation

```bash
ros2 launch my_robot_slam maze_sim.launch.py
```

This will:
- Start the robot state publisher
- Launch the obstacle publisher to create the competition terrain
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

### Understanding the Competition Terrain

The competition terrain includes:

1. **Boundary Walls**: Gray walls forming the perimeter of the 8ft x 8ft field
2. **Corner Zones**:
   - **Start Zone** (Green): Where the robot begins
   - **Fire Zone** (Red): Target area for firefighting scenario
   - **Other Corners** (Blue): Additional reference areas
3. **Tree Obstacles**: Brown cylinders representing poles/PVC pipes arranged to form paths and barriers
4. **Terrain Types**:
   - **Sand Areas** (Tan): Representing challenging sandy terrain
   - **Gravel Areas** (Gray): Representing rocky/gravel terrain

### Exploring the Competition Terrain

1. **Path Planning**: Navigate through the tree obstacles by finding clear paths
2. **Terrain Navigation**: Practice moving across different terrain types
3. **Fire Zone Challenge**: Attempt to navigate from the start zone to the fire zone
4. **Complete Circuit**: Try to navigate around the perimeter of the field
5. **Time Trials**: Measure how quickly you can navigate specific routes

### Customizing the Competition Terrain

You can customize the competition terrain by modifying the obstacle publisher script:
```
~/ros2_ws/src/my_robot_slam/scripts/obstacle_publisher.py
```

Parameters that can be adjusted include:
- Field dimensions and wall height
- Number and placement of tree obstacles
- Location and size of terrain patches
- Corner zone designations
- Obstacle diameters and spacing

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

3. **Python assertions related to geometry_msgs**
   - If you encounter errors like `geometry_msgs__msg__point__convert_from_py: Assertion 'PyFloat_Check(field)' failed`,
     make sure all coordinates in obstacle_publisher.py are correctly typed as floats

4. **Transforms issues**
   - Check TF tree in RViz for missing transforms
   - Ensure static transform publishers are running

## Project Structure

- **my_robot_description/**
  - **urdf/**: Robot URDF model
  - **rviz/**: RViz configuration files
  - **launch/**: Launch files for visualization

- **my_robot_slam/**
  - **launch/**: Launch files for simulation
  - **scripts/**: Python scripts for control and visualization
    - **obstacle_publisher.py**: Creates and publishes the competition terrain elements
    - **teleop_to_joints.py**: Converts teleop commands to joint states
    - **animate_robot.py**: Animates robot movement for visualization

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ROS 2 community for documentation and examples
- Open source robotics tools that made this project possible 