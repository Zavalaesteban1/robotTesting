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
    round1_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'round1_controller.py')
    lidar_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'lidar_simulator.py')
    wall_detector_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'wall_detector_debug.py')
    
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
    
    # Then add the Round 1 controller
    round1_controller_node = ExecuteProcess(
        cmd=['python3', round1_script],
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
    
    ld.add_action(maze_sim_cmd)
    ld.add_action(static_tf_base_link_to_base_footprint)  # Add static transform publisher
    ld.add_action(static_tf_laser_to_base_link)  # Add static transform publisher
    ld.add_action(lidar_simulator_node)
    ld.add_action(round1_controller_node)
    ld.add_action(wall_detector_node)
    ld.add_action(rviz_node)
    
    return ld
