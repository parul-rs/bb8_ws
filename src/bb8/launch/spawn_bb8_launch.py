import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import xacro
import os 

def generate_launch_description():
    urdf_filepath = os.path.join(get_package_share_directory('bb8'), 'urdf','bb8.xacro')
    doc = xacro.process_file(urdf_filepath)
    robot_description_config = doc.toxml()

    return LaunchDescription([
        DeclareLaunchArgument('robot_description', default_value=robot_description_config,
                              description='URDF of the robot'),
        
        # Node to publish the robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('robot_description')}],
        ),
        
        # Spawn the robot in Gazebo using the URDF
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'bb8'],
            output='screen',
        ),

        # Node execution: Run the spawn_bb8 Python script
        Node(
            package='bb8',                        # Package name
            executable='spawn_bb8',                # Executable name (this is the Python script)
            name='spawn_bb8_node',                 # Node name (can be anything)
            output='screen',                       # Print logs to screen
            parameters=[{'use_sim_time': False}],  # Example of passing parameters (optional)
        ),
    ])

