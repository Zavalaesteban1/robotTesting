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
    lidar_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'lidar_simulator.py')
    wall_detector_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'wall_detector_debug.py')
    teleop_to_joints_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'teleop_to_joints.py')
    data_collector_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'data_collector.py')
    
    # Paths to configuration files
    rviz_config = os.path.join(source_dir, 'my_robot_slam', 'config', 'advanced_lidar_view.rviz')
    slam_params_file = os.path.join(source_dir, 'my_robot_slam', 'config', 'slam_toolbox_params.yaml')
    
    # Start the competition field
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
    
    # Add teleop node for keyboard control with proper configuration
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'key_timeout': 0.3},  # More responsive key handling
            {'scale_linear': 0.5},  # Reduce default linear speed
            {'scale_angular': 0.8}  # Slightly reduce angular speed
        ],
        # Ensure proper topic mapping
        remappings=[
            ('/cmd_vel', '/cmd_vel')  # Map to expected topic
        ]
    )
    
    # Add teleop-to-joints controller
    teleop_to_joints_node = ExecuteProcess(
        cmd=['python3', teleop_to_joints_script],
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
    
    # Add a simple data collector node that subscribes to LiDAR data
    # and records information about detected walls/obstacles
    data_collector_node = ExecuteProcess(
        cmd=['python3', data_collector_script],
        output='screen'
    )
    
    # Add SLAM Toolbox node for mapping
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )
    
    # Add all nodes to launch sequence
    ld.add_action(maze_sim_cmd)
    ld.add_action(static_tf_base_link_to_base_footprint)
    ld.add_action(static_tf_laser_to_base_link)
    ld.add_action(lidar_simulator_node)
    ld.add_action(teleop_node)
    ld.add_action(teleop_to_joints_node)
    ld.add_action(wall_detector_node)
    ld.add_action(data_collector_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)
    
    return ld 