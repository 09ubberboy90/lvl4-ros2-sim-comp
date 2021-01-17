import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_srvs.srv import Empty
from simple_interface.srv import GetPoses

class ObjService(Node):

    def __init__(self):
        super().__init__('obj_info')
        self._cubes = []
        self.srv = self.create_service(GetPoses, 'get_cubes_pose', self.get_cubes_pose)
        print("hello")

    def add_cube(self, request):
        self._cubes.append(request)
        return None

    def get_cubes_pose(self, request, response:GetPoses):
        print(self._cubes)
        poses = PoseArray()
        poses.poses = self._cubes
        response.poses = poses
        return response


def main(args=None):
    rclpy.init(args=args)

    obj_service = ObjService()

    rclpy.spin(obj_service)
    print("Bye")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
