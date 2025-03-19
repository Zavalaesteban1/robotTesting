from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare the launch argument
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # Teleop twist keyboard
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/my_robot/cmd_vel')]
    )
    
    # Create and return the launch description
    ld = LaunchDescription()
    
    # Declare the launch argument
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add the teleop node
    ld.add_action(teleop_node)
    
    return ld 