import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Base paths
    my_robot_description_dir = get_package_share_directory('my_robot_description')
    source_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src')
    
    # Paths to scripts
    round1_script = os.path.join(source_dir, 'my_robot_slam', 'scripts', 'round1_controller.py')
    
    # Start the competition field
    ld = LaunchDescription()
    
    # First launch maze_sim.launch.py to set up the environment
    maze_sim_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'my_robot_slam', 'maze_sim.launch.py'],
        output='screen'
    )
    
    # Then add the Round 1 controller
    round1_cmd = ExecuteProcess(
        cmd=['python3', round1_script],
        output='screen'
    )
    
    ld.add_action(maze_sim_cmd)
    ld.add_action(round1_cmd)
    
    return ld
