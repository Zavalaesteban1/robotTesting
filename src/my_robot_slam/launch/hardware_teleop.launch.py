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
    
    # Paths to scripts and configuration
    hardware_controller_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'hardware_teleop_controller.py')
    rviz_config = os.path.join(source_dir, 'my_robot_slam', 'config', 'advanced_lidar_view.rviz')
    urdf_file = os.path.join(my_robot_description_dir, 'urdf', 'my_robot.urdf')
    
    # Read the URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
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
    
    # Add teleop node for keyboard control
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard_hardware',
        prefix='xterm -e',
        output='screen',
        parameters=[
            {'key_timeout': 0.3},  # More responsive key handling
            {'scale_linear': 0.5},  # Reduce default linear speed
            {'scale_angular': 0.8}  # Slightly reduce angular speed
        ]
    )
    
    # Add hardware controller node
    hardware_controller_node = ExecuteProcess(
        cmd=['python3', hardware_controller_script],
        output='screen'
    )
    
    # Launch RViz with our custom configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_hardware',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Add all nodes to launch sequence
    ld.add_action(robot_state_publisher_node)
    ld.add_action(static_tf_base_link_to_base_footprint)
    ld.add_action(static_tf_laser_to_base_link)
    ld.add_action(teleop_node)
    ld.add_action(hardware_controller_node)
    ld.add_action(rviz_node)
    
    return ld 