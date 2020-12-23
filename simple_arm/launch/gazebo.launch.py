import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess 

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg_name = "simple_arm"
    pkg_share = get_package_share_directory(pkg_name)

    config_name = "ur_configs"
    config_share = get_package_share_directory(config_name)

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )


    xacro_file = os.path.join(config_share,
                              'urdf','panda',
                              'panda_arm_hand.urdf.xacro')

    urdf = xacro.process(xacro_file)
    params = {'robot_description': urdf}
    state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
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