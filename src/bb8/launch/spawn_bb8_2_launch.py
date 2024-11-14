from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Define the path to the Xacro file
    xacro_file = os.path.join(
        get_package_share_directory('bb8'),
        'urdf2',
        'bb8.gazebo.xacro'
    )

    # Command to convert the Xacro file to URDF
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()
    #robot_description_content = Command(['xacro', xacro_file])
    #robot_description = {'robot_description': robot_description_content}
    # Include the Gazebo ROS launch file (this ensures Gazebo is launched with the necessary plugins)
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    return LaunchDescription([
        # Use Gazebo simulation time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Publish the robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description_config,
            }]
        ),

        # Spawn the robot in Gazebo using /spawn_entity
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'bb_8',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),

        # # Launch Gazebo
        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        #     output='screen'
        # Launch Gazebo with ROS 2 integration
        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', '--ros', '2', '--sdf'],
        #     output='screen'
        IncludeLaunchDescription(
            gazebo_launch_file,
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        ),
    ])
