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
    my_robot_simulation_dir = get_package_share_directory('my_robot_simulation')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')
    
    # Include the simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_simulation_dir, 'launch', 'gazebo_simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Include the SLAM launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_slam_dir, 'launch', 'slam_mapping.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Create and return launch description
    ld = LaunchDescription()
    
    # Declare the launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Include launches
    ld.add_action(simulation_launch)
    ld.add_action(slam_launch)
    
    return ld 