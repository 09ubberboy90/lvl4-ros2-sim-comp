import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg_name = "simple_arm"
    pkg_share = get_package_share_directory(pkg_name)

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )


    xacro_file = os.path.join(pkg_share,
                              'urdf',
                              'ur10_robot.urdf.xacro')

    urdf = xacro.process(xacro_file)
    params = {'robot_description': urdf}
    
    state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    joint = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ur10'],
                        output='screen')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', os.path.join(pkg_share, 'rviz', 'dolly_ignition.rviz')],
    )

    return LaunchDescription([
        gazebo,
        state,
        #joint,
        spawn_entity,
        #rviz
    ])