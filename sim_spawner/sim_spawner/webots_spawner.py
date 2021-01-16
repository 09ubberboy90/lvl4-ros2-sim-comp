import rclpy
import os, sys
from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_core.utils import append_webots_python_lib_to_path
from webots_ros2_core.trajectory_follower import TrajectoryFollower
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

try:
    append_webots_python_lib_to_path()
    from controller import Node
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e

class SpawnerNode(WebotsNode):
    def __init__(self, args=None):
        super().__init__("spawner",args)
        package_dir = get_package_share_directory('webots_simple_arm')

        child = self.robot.getRoot().getField("children")
        child.importMFNode(0,os.path.join(package_dir, "worlds/Table.wbo"))
        child.importMFNode(0,os.path.join(package_dir, "worlds/Cube.wbo"))


def main(args=None):
    rclpy.init(args=args)

    spawner = SpawnerNode(args=args)

    rclpy.spin_once(spawner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()