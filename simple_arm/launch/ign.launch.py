import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = "simple_arm"
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_share = get_package_share_directory(pkg_name)
    config_name = "ur_configs"
    config_share = get_package_share_directory(config_name)

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
    )

    # Spawn dolly

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
    xacro_file = os.path.join(config_share,
                              'urdf',
                              'ur10_robot.urdf.xacro')

    urdf = xacro.process(xacro_file)
    params = {'robot_description': urdf}

    ign_pub = Node(
        package=pkg_name,
        executable='ign_publish',
        output='screen',
    )
    # Follow node
    state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]

    )
    # Follow node
    joint = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[params]
    )
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'ur10',
                    '-x', '0.0',
                    '-z', '0',
                    '-Y', '0',
                    '-param', os.path.join(config_share, 'urdf','ur10.sdf')], ## TODO: Need to fix to use topic
                 output='screen')

    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    # args = [f"/model/ur10/joint/{name}/0/cmd_pos@std_msgs/msg/Float32@ignition.msgs.Float" for name in joint_names]
    args = [f"/robot/{name}@std_msgs/msg/Float64@ignition.msgs.Double" for name in joint_names]
    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=args,
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
          default_value=[os.path.join(pkg_share, 'worlds', 'empty.sdf')],
          description='Ignition Gazebo arguments'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gazebo,
        ign_pub,
        spawn,
        joint, 
        state,
        bridge,
        #rviz
    ])