import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Base paths
    my_robot_slam_dir = get_package_share_directory('my_robot_slam')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Include the teleop launch file (which includes most of your setup)
    teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(my_robot_slam_dir, 'launch'), '/round1_teleop.launch.py'])
    )
    
    # Set up slam_toolbox in online synchronous mode
    slam_params_file = os.path.join(my_robot_slam_dir, 'config', 'mapper_params_online_sync.yaml')
    
    # Create slam_toolbox node for mapping
    slam_toolbox_node = Node(
        parameters=[slam_params_file],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[
            ('scan', 'scan')  # Make sure this matches your scan topic
        ]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add teleop launch (includes your robot setup)
    ld.add_action(teleop_cmd)
    
    # Add the slam_toolbox node
    ld.add_action(slam_toolbox_node)
    
    return ld