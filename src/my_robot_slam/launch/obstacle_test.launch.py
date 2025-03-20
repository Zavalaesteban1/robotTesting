import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    my_robot_description_dir = get_package_share_directory('my_robot_description')
    
    # Source directory (not installed)
    source_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src')
    
    # Path to RViz config
    rviz_config = os.path.join(my_robot_description_dir, 'rviz', 'maze_config.rviz')
    
    # Create the static transform publishers for the map frame
    static_transform_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    
    # Start RViz with the maze-specific config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Start obstacle publisher for maze visualization
    obstacle_script_path = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'obstacle_publisher.py')
    obstacle_cmd = Node(  # Using Node instead of ExecuteProcess for better stability
        package='my_robot_slam',
        executable='scripts/obstacle_publisher.py',
        name='obstacle_publisher',
        output='screen'
    )
    
    # Create and return the launch description
    ld = LaunchDescription()
    
    # Add actions in order
    ld.add_action(static_transform_map)
    ld.add_action(obstacle_cmd)  # Start obstacles before RViz
    ld.add_action(rviz_node)
    
    return ld 