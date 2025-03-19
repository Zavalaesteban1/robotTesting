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
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(my_robot_slam_dir, 'config', 'slam_toolbox_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the SLAM Toolbox node')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Set to false for real robot
        description='Use simulation clock if true')
    
    # Start the SLAM Toolbox online async node
    slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    # Create and return the launch description
    ld = LaunchDescription()
    
    # Declare the launch arguments
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add the SLAM Toolbox node
    ld.add_action(slam_toolbox_node)
    
    return ld 