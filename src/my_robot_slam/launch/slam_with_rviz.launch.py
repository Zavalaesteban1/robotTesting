import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    my_robot_slam_dir = get_package_share_directory('my_robot_slam')
    my_robot_description_dir = get_package_share_directory('my_robot_description')
    
    # Source directory (not installed)
    source_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src')
    
    # Create the launch configuration variables
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Path to URDF file
    urdf_file = os.path.join(my_robot_description_dir, 'urdf', 'my_robot.urdf')
    
    # Read the URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Path to RViz config
    rviz_config = os.path.join(my_robot_description_dir, 'rviz', 'slam_config.rviz')
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(my_robot_description_dir, 'rviz', 'urdf_config.rviz')
    
    # Declare the launch arguments
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(my_robot_slam_dir, 'config', 'slam_toolbox_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the SLAM Toolbox node')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Since we're not using Gazebo
        description='Use simulation clock if true')
    
    # Start the robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # Start the joint state publisher GUI for manual joint control
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
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
    
    # Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Start teleop node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen'
    )
    
    # Create and execute the Python script for joint state publishing
    script_path = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'animate_robot.py')
    joint_state_publisher_cmd = ExecuteProcess(
        cmd=['python3', script_path],
        output='screen'
    )
    
    # Create and return the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)  # Added this for manual control
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)
    ld.add_action(teleop_node)
    ld.add_action(joint_state_publisher_cmd)
    
    return ld 