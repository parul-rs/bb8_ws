from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import xacro
import os 

# Launch instructions: 
# In one terminal, in the bb8_ws directory (NOT in src), run:
# source /opt/ros/humble/setup.bash
# source /usr/share/gazebo/setup.sh
# colcon build --symlink-install
# source install/setup.bash
# ros2 launch bb8 spawn_bb8_launch.py

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Define the path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('bb8'),
        'urdf/bb8.xacro'
    )
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_bot_gazebo = get_package_share_directory('bb8')

    description_package_name = "bb8"
    install_dir = get_package_prefix(description_package_name)

    gazebo_models_path = os.path.join(pkg_box_bot_gazebo, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # Get the path to world files
    gazebo_worlds_path = os.path.join(pkg_box_bot_gazebo, 'worlds')

    # Add to GAZEBO_WORLD_PATH if not already set
    if 'GAZEBO_WORLD_PATH' in os.environ:
        os.environ['GAZEBO_WORLD_PATH'] = os.environ['GAZEBO_WORLD_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_worlds_path
    else:
        os.environ['GAZEBO_WORLD_PATH'] = install_dir + '/share' + ':' + gazebo_worlds_path
    
    world = LaunchConfiguration('world', default=os.path.join(get_package_share_directory('bb8'), 'worlds', 'bb8_world.world'))
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={'world': world}.items()  # Pass the world argument to Gazebo launch
    )

    return LaunchDescription([
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
        Node(
            package='bb8',                     
            executable='test_opti_7', # Change to script name you'd like to run
            name='test_command_node', 
            output='screen',          
            parameters=[{'use_sim_time': False}], 
        ),

    ])
