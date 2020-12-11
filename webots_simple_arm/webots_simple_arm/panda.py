import argparse
import sys, os
from math import cos, pi, sin

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from webots_ros2_core.joint_state_publisher import JointStatePublisher
from webots_ros2_core.trajectory_follower import TrajectoryFollower
from webots_ros2_core.webots_node import WebotsNode

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory

class PandaNode(WebotsNode):
    def __init__(self, args=None):
        super().__init__("panda", args)
        os.environ['WEBOTS_ROBOT_NAME'] = "panda"
        self.trajectoryFollower = TrajectoryFollower(self.robot, self,"")

        goal = FollowJointTrajectory()
        trajectory = JointTrajectory()
        trajectory.joint_names = ["panda_joint1", "panda_joint2"]
        trajectory.points = []

        goal.Goal.trajectory = 
        self.trajectoryFollower.on_goal()

        self.listener_timer = self.create_timer(0.001 * self.timestep,
                                          self.listener)
    
    def listener(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    panda_controller = PandaNode(args=args)
    panda_controller.start_device_manager()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = rclpy.executors.MultiThreadedExecutor()

    rclpy.spin(panda_controller, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()