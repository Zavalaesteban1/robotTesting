# My Robot SLAM and Navigation System

This package contains the necessary components for SLAM (Simultaneous Localization and Mapping) and navigation for a firefighter robot.

## Overview

The system provides:

- SLAM mapping capabilities using slam_toolbox
- Map saving and management
- Navigation support with Nav2

## Prerequisites

Make sure you have the following packages installed:

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server
```

## Building the Package

1. Build the packages:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

2. Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### 1. Starting SLAM for Mapping

To start SLAM with the physical robot:

```bash
ros2 launch my_robot_slam slam_only.launch.py use_sim_time:=false
```

If you want to visualize with RViz:

```bash
ros2 run rviz2 rviz2
```

Add the following displays in RViz:
- TF
- LaserScan (topic: /scan)  
- Map (topic: /map)

### 2. Saving the Map

After you've created a map by driving around the environment, save it:

```bash
ros2 launch my_robot_slam save_map_simple.launch.py map_file_name:=my_map
```

This will save the map in the `maps` directory with the specified name.

### 3. Using the Map for Navigation

To use the created map for navigation with Nav2:

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map:=/path/to/my_map.yaml
```

### 4. Transferring Maps Between Computers

To use the maps created on your development computer with your Raspberry Pi:

1. Copy the map files (`.pgm` and `.yaml`) from your development computer to the Raspberry Pi
   ```bash
   scp ~/ros2_ws/src/my_robot_slam/maps/my_map.* pi@raspberry_pi_ip:/home/pi/ros2_ws/src/my_robot_slam/maps/
   ```

2. Update your launch file to use the map:
   ```bash
   ros2 launch nav2_bringup navigation_launch.py map:=/home/pi/ros2_ws/src/my_robot_slam/maps/my_map.yaml
   ```

## SLAM Configuration

You can customize the SLAM Toolbox parameters in `config/slam_toolbox_params.yaml`:
- Adjust `resolution` for map detail (default is 0.05 meters per pixel)
- Modify `max_laser_range` based on your LiDAR's capabilities
- Fine-tune loop closure parameters for better map consistency

## Troubleshooting

### Transform Issues
If you encounter transform-related errors, check:
- The `odom_frame`, `map_frame`, and `base_frame` parameters in the SLAM configuration
- Ensure your robot is publishing the correct TF tree

### SLAM Quality Issues
If map quality is poor:
- Ensure your robot moves slowly enough for accurate mapping
- Adjust the scan matcher parameters in the configuration
- Make sure the LiDAR data is clean and has minimal noise 