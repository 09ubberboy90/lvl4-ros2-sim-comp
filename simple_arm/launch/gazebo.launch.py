import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess 

from launch_ros.actions import Node

import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    pkg_name = "simple_arm"
    pkg_share = get_package_share_directory(pkg_name)

    config_name = "ur_configs"
    config_share = get_package_share_directory(config_name)

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={
            'world': os.path.join(pkg_share, 'worlds', 'empty.world'),
#            'verbose':"true"
        }.items()
             )


    robot_description_config = load_file(
        'ur_configs', 'urdf/panda/panda_arm_hand.urdf')
    robot_description = {'robot_description': robot_description_config}
    state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'panda'],
                        output='screen')

    lifecycle_pub = Node(
        package='simple_arm',
        executable='gz_srv_disable',
        output='screen',
    )
    return LaunchDescription([
        gazebo,
        state,
        spawn_entity,
        lifecycle_pub
    ])