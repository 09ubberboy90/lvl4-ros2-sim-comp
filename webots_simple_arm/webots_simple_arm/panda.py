import rclpy
import os, sys
from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_core.utils import append_webots_python_lib_to_path
from webots_ros2_core.trajectory_follower import TrajectoryFollower
from sensor_msgs.msg import JointState
try:
    append_webots_python_lib_to_path()
    from controller import Node
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e

class PandaNode(WebotsNode):
    def __init__(self, args=None):
        super().__init__("panda",args, enableJointState=False)
        os.environ['WEBOTS_ROBOT_NAME'] = "panda"
        self.trajectoryFollower = TrajectoryFollower(self.robot, self, jointPrefix="")
        self.nodes = {}
        for i in range(self.robot.getNumberOfDevices()):
            device = self.robot.getDeviceByIndex(i)
            if device.getNodeType() == Node.POSITION_SENSOR:
                motor = device.getMotor()
                name = motor.getName() if motor is not None else device.getName()
                self.nodes[name] = motor
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)

    def listener_callback(self, msg:JointState):
        for idx, name in enumerate(msg.name):
            self.nodes[name].setPosition(msg.position[idx])

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