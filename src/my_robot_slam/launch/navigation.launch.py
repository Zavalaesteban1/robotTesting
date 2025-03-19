import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    my_robot_slam_dir = get_package_share_directory('my_robot_slam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_map_yaml_file_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(my_robot_slam_dir, 'maps', 'my_map.yaml'),
        description='Full path to map file to load')
    
    # Include Nav2 Bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file
        }.items()
    )
    
    # Create and return the launch description
    ld = LaunchDescription()
    
    # Declare the launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_file_cmd)
    
    # Include Nav2 Bringup
    ld.add_action(nav2_bringup_launch)
    
    return ld 