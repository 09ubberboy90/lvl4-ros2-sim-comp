import sys

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import rclpy
from rclpy.node import Node


class GZServiceDisable(Node):

    def __init__(self):
        super().__init__('gz_srv_disable')
        self.cli = self.create_client(ChangeState, '/joint_state_controller/change_state')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ChangeState.Request()

    def send_request(self):
        transition = Transition()
        transition.id = 4 # Disable JointStatePublisher
        self.req.transition = transition
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = GZServiceDisable()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            minimal_client.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
