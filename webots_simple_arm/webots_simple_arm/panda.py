import argparse
import sys, os
from math import cos, pi, sin

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from webots_ros2_core.joint_state_publisher import JointStatePublisher
from webots_ros2_core.trajectory_follower import TrajectoryFollower
from webots_ros2_core.webots_node import WebotsNode



class PandaNode(WebotsNode):
    def __init__(self, args=None):
        super().__init__("panda", args)
        os.environ['WEBOTS_ROBOT_NAME'] = "panda"
        self.trajectoryFollower = TrajectoryFollower(self.robot, self,"")


def main(args=None):
    rclpy.init(args=args)
    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    panda_controller = PandaNode(args=args)
    panda_controller.start_device_manager()
    rclpy.spin(panda_controller, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()