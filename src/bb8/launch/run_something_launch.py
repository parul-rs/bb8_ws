import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare optional launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),

        # Log message to indicate that the launch has started
        LogInfo(condition=None, msg="Starting the spawn_bb8 node..."),

        # Node execution: Run the spawn_bb8 Python script
        Node(
            package='bb8',                        # Package name
            executable='run_something',                # Executable name (this is the Python script)
            name='run_something_node',                 # Node name (can be anything)
            output='screen',                       # Print logs to screen
            parameters=[{'use_sim_time': False}],  # Example of passing parameters (optional)
        ),
    ])
