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
    
    # Paths to configuration files
    rviz_config = os.path.join(source_dir, 'my_robot_slam', 'config', 'advanced_lidar_view.rviz')
    
    # Start the competition field
    ld = LaunchDescription()
    
    # First launch maze_sim.launch.py to set up the environment
    maze_sim_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'my_robot_slam', 'maze_sim.launch.py'],
        output='screen'
    )
    
    # Add a LiDAR simulator node to publish laser scan data
    lidar_simulator_cmd = ExecuteProcess(
        cmd=['python3', lidar_script],
        output='screen'
    )
    
    # Then add the Round 1 controller
    round1_cmd = ExecuteProcess(
        cmd=['python3', round1_script],
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
    ld.add_action(lidar_simulator_cmd)  # Add the LiDAR simulator
    ld.add_action(round1_cmd)
    ld.add_action(rviz_node)
    
    return ld
