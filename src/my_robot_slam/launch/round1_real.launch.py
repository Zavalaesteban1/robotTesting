#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Base paths
    my_robot_description_dir = get_package_share_directory('my_robot_description')
    my_robot_slam_dir = get_package_share_directory('my_robot_slam')
    source_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src')
    
    # Paths to scripts
    real_controller_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'round1_realController.py')
    lidar_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'lidar_simulator.py')
    wall_detector_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'wall_detector_debug.py')
    
    # Paths to configuration files
    rviz_config = os.path.join(source_dir, 'my_robot_slam', 'config', 'maze_config.rviz')
    
    # Create launch description
    ld = LaunchDescription()
    
    # First launch maze_sim.launch.py to set up the environment
    maze_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(my_robot_slam_dir, 'launch'), '/maze_sim.launch.py'])
    )
    
    # Add a LiDAR simulator node to publish laser scan data
    lidar_simulator_node = ExecuteProcess(
        cmd=['python3', lidar_script],
        output='screen'
    )
    
    # Add static transform publishers to ensure TF tree is complete
    static_tf_base_link_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )
    
    static_tf_laser_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser_to_base',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )
    
    # Add additional transforms for better coverage
    static_tf_odom_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    static_tf_map_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'map', 'laser_in_map'],
        output='screen'
    )
    
    # Add a wall detector debug node to help visualize walls
    wall_detector_node = ExecuteProcess(
        cmd=['python3', wall_detector_script],
        output='screen'
    )
    
    # Launch RViz with our custom configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Launch the A* navigation controller
    astar_controller_node = ExecuteProcess(
        cmd=['python3', real_controller_script],
        output='screen'
    )
    
    # Add all nodes to launch sequence
    ld.add_action(maze_sim_cmd)
    ld.add_action(static_tf_base_link_to_base_footprint)
    ld.add_action(static_tf_laser_to_base_link)
    ld.add_action(static_tf_odom_to_map)
    ld.add_action(static_tf_map_to_laser)
    ld.add_action(lidar_simulator_node)
    ld.add_action(wall_detector_node)
    ld.add_action(rviz_node)
    ld.add_action(astar_controller_node)
    
    return ld
