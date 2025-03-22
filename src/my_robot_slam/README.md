# My Robot SLAM Package

This ROS2 package contains a robot simulation with autonomous navigation and LiDAR-based obstacle avoidance capabilities for firefighting competitions. The robot can explore environments, detect fires, and find its way back to the starting position.

## Robot Components

- Base (blue): The main platform of the robot
- Body (red): The upper part of the robot mounted on the base
- Wheels (green): Two wheels on either side that can rotate
- Casters (grey): Front and back casters for stability
- LiDAR: Simulated laser scanner for obstacle detection and avoidance

## Features

### Autonomous Navigation
- Random exploration with coverage tracking
- Path planning for returning to starting position
- Obstacle avoidance using LiDAR data
- Fire detection and response
- Performance metrics tracking (time, distance, obstacles detected)

### Enhanced Obstacle Avoidance
- Sector-based LiDAR data processing (divides scan into 16 sectors)
- Intelligent navigation decisions based on open space detection
- Gradual speed adjustments for smooth movement
- Context-aware obstacle responses (front vs. side obstacles)
- Visualization of danger zones and closest obstacles

### Advanced Visualization
- Real-time LiDAR data visualization with color coding by distance
- Obstacle visualization with size and color reflecting distance/danger
- Exploration history markers showing visited positions
- Fire detection markers
- Dedicated RViz configuration for comprehensive visualization

## Dependencies

- ROS2 (Tested on Jazzy)
- robot_state_publisher
- joint_state_publisher_gui
- rviz2
- numpy

## Building the Package

Clone this repository into your ROS2 workspace's `src` directory and then build:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_slam
source install/setup.bash
```

## Running the Simulation

To run the full simulation with advanced visualization:

```bash
ros2 launch my_robot_slam round1_sim.launch.py
```

This will launch:
1. The maze simulation environment
2. The LiDAR simulator node
3. The autonomous robot controller
4. RViz with the advanced visualization configuration

## Visualizing LiDAR Data and Obstacles

The enhanced visualization includes:
- **Laser Scan Points**: Rainbow-colored points showing raw LiDAR data
- **Danger Zones**: Red/yellow spheres highlighting detected obstacles
- **Closest Obstacle**: Green marker indicating the nearest obstacle
- **Exploration History**: Blue spheres marking visited positions
- **Fire Detection**: Green marker showing detected fire position

## Understanding the Robot Controller

The `round1_controller.py` script contains the main robot control logic:
- State machine for initialization, exploration, and return phases
- LiDAR data processing through sector-based analysis
- Obstacle avoidance algorithm with contextual awareness
- Random exploration with position tracking
- Fire detection and response mechanism

## Customization

You can adjust various parameters in the controller:
- `obstacle_threshold`: Distance to consider as an obstacle (default: 0.5m)
- `min_safe_distance`: Minimum safe distance from obstacles (default: 0.3m)
- `scan_sectors`: Number of sectors to divide LiDAR scan (default: 16)
- `random_walk_duration`: How long to follow each random target (default: 5.0s)

## Further Development

Potential areas for enhancement:
1. Implement frontier-based exploration instead of random walk
2. Add A* or RRT path planning for more efficient navigation
3. Integrate SLAM for simultaneous mapping
4. Improve fire detection with thermal camera simulation
5. Optimize obstacle clustering for better perception

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Teleop Control Guide

The robot uses a modified teleop keyboard controller that maps keyboard inputs to appropriate movement commands for a car-like vehicle with Ackermann steering.

### Key Mappings

| Key | Movement |
| --- | --- |
| `i` | Drive forward |
| `,` | Drive backward |
| `j` | Turn left |
| `l` | Turn right |
| `u` | Forward + left turn |
| `o` | Forward + right turn |
| `.` | Backward + left turn |
| `m` | Backward + right turn |
| `k` | Stop |

### Speed Control

| Key | Action |
| --- | --- |
| `q` | Increase speed and turn rate by 10% |
| `z` | Decrease speed and turn rate by 10% |
| `w` | Increase speed by 10% |
| `x` | Decrease speed by 10% |
| `e` | Increase turn rate by 10% |
| `c` | Decrease turn rate by 10% |

### Notes on Car-Like Robot Movement

Unlike omnidirectional robots, car-like vehicles with Ackermann steering have specific movement constraints:

1. They can only move forward and backward along their longitudinal axis
2. They cannot move sideways (no lateral/strafing movement)
3. Turning requires coordination of forward/backward movement and steering angle

The teleop controller has been specially configured to:
- Convert diagonal key presses into appropriate forward/backward movement + steering
- Provide small "creep" forward motion when turning in place for better visualization
- Adapt turning behavior based on direction of travel (forward vs reverse)

## Running the Simulation

```bash
# Launch the robot with teleop control
ros2 launch my_robot_slam round1_teleop.launch.py
```

## Troubleshooting

If the robot doesn't move as expected:
- Check the terminal running the teleop keyboard node for key mapping information
- Ensure you have an active terminal window with keyboard focus
- Try using gentle, incremental commands rather than full-speed movements
- Watch the robot controller logs for debugging information about commands received 