import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from ament_index_python.packages import get_package_prefix
import xacro
import os 

# README: 
# source /opt/ros/humble/setup.bash
# source install/setup.bash
# source /usr/share/gazebo/setup.sh
# Terminal 1: ros2 launch bb8 spawn_bb8_launch.py world:=src/bb8/worlds/redcube_world.world
# Should run test_command.py

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Define the path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('bb8'),
        'urdf/bb8.xacro'
    )
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # COPIED FROM START WORLD LAUNCHFILE ----------------------------------------------
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_bot_gazebo = get_package_share_directory('bb8')

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "bb8"
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_box_bot_gazebo package
    gazebo_models_path = os.path.join(pkg_box_bot_gazebo, 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # Get the install directory of your package
    install_dir = get_package_prefix('bb8')

    # Get the path to your world files
    gazebo_worlds_path = os.path.join(pkg_box_bot_gazebo, 'worlds')

    # Add to GAZEBO_WORLD_PATH if not already set
    if 'GAZEBO_WORLD_PATH' in os.environ:
        os.environ['GAZEBO_WORLD_PATH'] = os.environ['GAZEBO_WORLD_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_worlds_path
    else:
        os.environ['GAZEBO_WORLD_PATH'] = install_dir + '/share' + ':' + gazebo_worlds_path

    # Print the new GAZEBO_WORLD_PATH for debugging
    print("GAZEBO WORLD PATH==" + str(os.environ["GAZEBO_WORLD_PATH"]))

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # world_file = os.path.join(pkg_box_bot_gazebo, 'worlds', 'wiggly_world.world')

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'world',
        #     default_value=[world_file, ''],
        #     description='SDF world file'
        # ),
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_box_bot_gazebo, 'worlds', 'redcube_world.world'), ''],
          description='SDF world file'),
        gazebo,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_raw}],
            arguments=[xacro_file]),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', '/robot_description', '-entity', 'bb8', '-x', '0', '-y', '0', '-z', '1'],
            output='screen',
        ),
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='false',
        #     description='Use simulation (Gazebo) clock if true'),
        # Node(
        #     package='bb8',                        # Package name
        #     executable='wiggle_world',         # Executable name (this is the Python script)
        #     name='world_wiggler',             # Node name (can be anything)
        #     output='screen',                     # Print logs to screen
        #     parameters=[{'use_sim_time': False}], # Example of passing parameters (optional)
        # ),
        Node(
            package='bb8',                        # Package name
            executable='test_opti_7',                # Executable name (this is the Python script)
            name='test_command_node',                 # Node name (can be anything)
            output='screen',                       # Print logs to screen
            parameters=[{'use_sim_time': False}],  # Example of passing parameters (optional)
        ),
    ])
