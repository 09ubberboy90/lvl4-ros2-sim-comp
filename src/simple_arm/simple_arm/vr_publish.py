import sys
import time
import openvr
from simple_arm import utils

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion

class VrPublisher(Node):

    def __init__(self, openvr_system):
        super().__init__('vr_publisher')
        self.publisher = self.create_publisher(Pose, 'VrPose', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.devices = {}
        self.system = openvr_system


    def timer_callback(self):
        poses = self.system.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding,0,0)
        self.devices["hmd"] = poses[openvr.k_unTrackedDeviceIndex_Hmd] 
        ## idx 2 and 3 are the lighthouse position so ignored that
        for idx, controller in enumerate(poses[3:]):
            if (controller.bPoseIsValid):
                self.devices["controller_"+str(idx)] = controller
        for key, el in self.devices.items():
            print(f"{key} : {utils.convert_to_quaternion(el.mDeviceToAbsoluteTracking)}")
        
        pose = utils.convert_to_quaternion(self.devices["controller_0"].mDeviceToAbsoluteTracking)

        point = Point()
        point.x = pose[0][0]
        point.y = pose[0][1]
        point.z = pose[0][2]

        rot = Quaternion()
        rot.x = pose[1][0]
        rot.y = pose[1][1]
        rot.z = pose[1][2]
        rot.w = pose[1][3]

        msg = Pose()   
        msg.orientation = rot
        msg.position = point
        self.publisher.publish(msg)
        self.get_logger().info(f"x:{point.x}, y: {point.y}, z:{point.z}")


def main(args=None):
    if not openvr.isRuntimeInstalled:
        raise RuntimeError("OpenVR / SteamVr is not Installed Exit")
    if not openvr.isHmdPresent():
        raise RuntimeError("SteamVr is not running or Headmount is not plugged in")

    rclpy.init(args=args)
    system = openvr.init(openvr.VRApplication_Scene)

    minimal_publisher = VrPublisher(system)

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()
    openvr.shutdown()    


if __name__ == '__main__':
    main()