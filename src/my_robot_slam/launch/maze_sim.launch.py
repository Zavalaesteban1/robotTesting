import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    my_robot_description_dir = get_package_share_directory('my_robot_description')
    
    # Source directory (not installed)
    source_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src')
    
    # Path to URDF file
    urdf_file = os.path.join(my_robot_description_dir, 'urdf', 'my_robot.urdf')
    
    # Read the URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Path to RViz config - use the competition-specific config
    rviz_config = os.path.join(my_robot_description_dir, 'rviz', 'maze_config.rviz')
    
    # Create the static transform publishers for the terrain
    # This will create a ground plane relative to the map frame
    static_transform_terrain = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_terrain',
        arguments=['0', '0', '-0.05', '0', '0', '0', 'map', 'terrain']
    )
    
    # Start the robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # Start RViz with the competition-specific config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Start teleop node for robot control
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen'
    )
    
    # Start robot controller to convert teleop commands to joint states
    script_path = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'teleop_to_joints.py')
    controller_cmd = ExecuteProcess(
        cmd=['python3', script_path],
        output='screen'
    )
    
    # Start competition field publisher - using direct execution of script
    # This creates the 8ft x 8ft field with tree obstacles and challenging terrain
    obstacle_script_path = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'obstacle_publisher.py')
    field_publisher_cmd = ExecuteProcess(
        cmd=['python3', obstacle_script_path],
        output='screen'
    )
    
    # Add transforms for the challenging terrain areas
    # This will create a separate reference frame for the elevated terrain patches
    static_transform_sand = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_sand',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'sand_terrain']
    )
    
    static_transform_gravel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_gravel',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'gravel_terrain']
    )
    
    # Create and return the launch description
    ld = LaunchDescription()
    
    # Add transformations for the environment
    ld.add_action(static_transform_terrain)
    ld.add_action(static_transform_sand)
    ld.add_action(static_transform_gravel)
    
    # Add robot and field elements in the correct order
    ld.add_action(robot_state_publisher_node)
    ld.add_action(field_publisher_cmd)  # Start field publisher before RViz
    ld.add_action(rviz_node)            # Then start RViz so it can see the markers
    
    # Add control elements
    ld.add_action(teleop_node)
    ld.add_action(controller_cmd)
    
    return ld 