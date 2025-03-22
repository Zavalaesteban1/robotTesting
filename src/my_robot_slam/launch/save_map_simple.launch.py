import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    my_robot_slam_dir = get_package_share_directory('my_robot_slam')
    
    # Create the launch configuration variables
    map_file_name = LaunchConfiguration('map_file_name')
    map_save_folder = LaunchConfiguration('map_save_folder')
    
    # Declare the launch arguments
    declare_map_file_name_cmd = DeclareLaunchArgument(
        'map_file_name',
        default_value='my_map',
        description='Name of the map file to save')
    
    declare_map_save_folder_cmd = DeclareLaunchArgument(
        'map_save_folder',
        default_value=os.path.join(my_robot_slam_dir, 'maps'),
        description='Path to the directory where the map will be saved')
    
    # Save the map using Map Saver
    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', [map_save_folder, '/', map_file_name]])
    
    # Create and return the launch description
    ld = LaunchDescription()
    
    # Declare the launch arguments
    ld.add_action(declare_map_file_name_cmd)
    ld.add_action(declare_map_save_folder_cmd)
    
    # Add the map saver node
    ld.add_action(map_saver_node)
    
    return ld 