import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro
import os 

# def generate_launch_description():
#     urdf_filepath = os.path.join(get_package_share_directory('bb8'), 'urdf','bb8.xacro')
#     doc = xacro.process_file(urdf_filepath)
#     robot_description_config = doc.toxml()

#     return LaunchDescription([
#         DeclareLaunchArgument('robot_description', default_value=robot_description_config,
#                               description='URDF of the robot'),
        
#         # Node to publish the robot state
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[{'robot_description': LaunchConfiguration('robot_description')}],
#         ),
        
#         # Spawn the robot in Gazebo using the URDF
#         Node(
#             package='gazebo_ros',
#             executable='spawn_entity.py',
#             arguments=['-topic', 'robot_description', '-entity', 'bb8'],
#             output='screen',
#         ),

#         # Node execution: Run the spawn_bb8 Python script
#         Node(
#             package='bb8',                        # Package name
#             executable='spawn_bb8',                # Executable name (this is the Python script)
#             name='spawn_bb8_node',                 # Node name (can be anything)
#             output='screen',                       # Print logs to screen
#             parameters=[{'use_sim_time': False}],  # Example of passing parameters (optional)
#         ),
#     ])

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Define the path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('bb8'),
        'urdf/bb8.xacro'
    )
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # world_file = os.path.join(
    #     get_package_share_directory('bb8'),
    #     'worlds', 'empty.world'
    # )
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        #launch_arguments={'world': world_file}.items()
    )

    return LaunchDescription([
        #gazebo_launch,
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', '/robot_description', '-entity', 'bb8', '-x', '0', '-y', '0', '-z', '1'],
            output='screen',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_raw}],
            arguments=[xacro_file]),
                # Node execution: Run the spawn_bb8 Python script
        # Node(
        #     package='bb8',                        # Package name
        #     executable='test_command',                # Executable name (this is the Python script)
        #     name='test_command_node',                 # Node name (can be anything)
        #     output='screen',                       # Print logs to screen
        #     parameters=[{'use_sim_time': False}],  # Example of passing parameters (optional)
        # ),
    ])
