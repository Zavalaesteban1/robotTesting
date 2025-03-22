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
    teleop_to_joints_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'teleop_to_joints.py')
    
    # Paths to configuration files
    rviz_config = os.path.join(source_dir, 'my_robot_slam', 'config', 'advanced_lidar_view.rviz')
    
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
    
    # Add only essential static transforms
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
    
    # Add teleop node with modified parameters
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'scale_linear': 0.3},  # Slow speed for better mapping
            {'scale_angular': 0.6}  # Slow turns for better mapping
        ]
    )
    
    # Add teleop-to-joints controller
    teleop_to_joints_node = ExecuteProcess(
        cmd=['python3', teleop_to_joints_script],
        output='screen'
    )
    
    # SLAM Toolbox node with properly configured parameters
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'scan_topic': 'scan',
            'mode': 'mapping',
            'max_laser_range': 10.0,
            'resolution': 0.05,
            'map_update_interval': 1.0,
            'loop_closure_enabled': True
        }]
    )
    
    # Launch RViz with map visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Add all nodes to launch sequence
    ld.add_action(maze_sim_cmd)
    ld.add_action(static_tf_base_link_to_base_footprint)
    ld.add_action(static_tf_laser_to_base_link)
    ld.add_action(lidar_simulator_node)
    ld.add_action(teleop_node)
    ld.add_action(teleop_to_joints_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)
    
    return ld 