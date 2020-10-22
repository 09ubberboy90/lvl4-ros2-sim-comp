import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class JointSub(Node):

    def __init__(self):
        super().__init__('joint_sub')
        self.subscription = self.create_subscription(
            JointState,                                              # CHANGE
            'joint_states',
            self.listener_callback,
            10)
        self.custom_publishers = {}
        self.pub = self.create_publisher(JointState, f'/ign/ur10/joint_states', 10)

    def listener_callback(self, msg):
        self.pub.publish(msg)
        # for idx, name in enumerate(msg.name):
        #     pub = self.custom_publishers.get(name, None)
        #     if pub is not None:
        #         new_msg = Float32()
        #         new_msg.data = msg.position[idx]
        #         pub.publish(new_msg)
        #     else:
        #         self.custom_publishers[name] = self.create_publisher(Float32, f'/model/ur10/joint/{name}/cmd_vel', 10)

def main(args=None):
    rclpy.init(args=args)

    joint_sub = JointSub()

    rclpy.spin(joint_sub)

    joint_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()