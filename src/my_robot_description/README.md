# My Robot Description

This ROS2 package contains a simple robot model created for learning purposes. The robot consists of a base, a body, two wheels, and casters for stability.

## Robot Components

- Base (blue): The main platform of the robot
- Body (red): The upper part of the robot mounted on the base
- Wheels (green): Two wheels on either side that can rotate
- Casters (grey): Front and back casters for stability

## Dependencies

- ROS2 (Tested on Humble)
- robot_state_publisher
- joint_state_publisher_gui
- rviz2

## Building the Package

Clone this repository into your ROS2 workspace's `src` directory and then build:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
```

## Visualizing the Robot

To visualize the robot in RViz2, run:

```bash
ros2 launch my_robot_description display.launch.py
```

This will open RViz2 with our robot model loaded. You can use the joint_state_publisher_gui to move the robot's wheels.

## Understanding URDF

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe robots. In this project, we've defined:

- **Links**: The physical parts of the robot (base, body, wheels, casters)
- **Joints**: The connections between links (fixed, continuous)
- **Visual elements**: How the robot appears in RViz
- **Collision elements**: Used for collision detection
- **Inertial properties**: Mass and inertia for physics simulation

To learn more about URDF, visit the [ROS URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html).

## Further Learning

This is a basic example to get started with robot modeling in ROS2. To expand on this project, you could:

1. Add sensors like cameras or LiDAR
2. Create more complex joint types
3. Implement controllers for movement
4. Add more detailed meshes instead of primitive shapes
5. Simulate the robot in Gazebo

## License

This project is licensed under the MIT License - see the LICENSE file for details. 