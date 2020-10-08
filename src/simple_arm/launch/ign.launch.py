import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "simple_arm"
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_share = get_package_share_directory(pkg_name)

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
    )

    # Spawn dolly
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'UR10',
                    '-x', '0.0',
                    '-z', '0',
                    '-Y', '0',
                    '-file', os.path.join(pkg_share, 'models', 'ur10',
                                          'ur10.sdf')],
                 output='screen')

    # # Follow node
    # follow = Node(
    #     package='dolly_follow',
    #     executable='dolly_follow',
    #     output='screen',
    #     remappings=[
    #         ('cmd_vel', '/dolly/cmd_vel'),
    #         ('laser_scan', '/dolly/laser_scan')
    #     ]
    # )

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        # arguments=['/dolly/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        #            '/dolly/laser_scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
        #            '/dolly/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', os.path.join(pkg_share, 'rviz', 'dolly_ignition.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'ign_args',
          default_value=[os.path.join(pkg_share, 'worlds', 'empty.sdf') +
                         ' -v 2 --gui-config ' +
                         os.path.join(pkg_share, 'configs', 'ignition.config'), ''],
          description='Ignition Gazebo arguments'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gazebo,
        spawn,
        # follow,
        bridge,
        rviz
    ])