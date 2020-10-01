import sys
import time
import openvr
from simple_arm import utils

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
import collections

class VrPublisher(Node):

    def __init__(self, openvr_system):
        super().__init__('vr_publisher')
        self.publisher = self.create_publisher(Pose, 'VrPose', 10)     # CHANGE
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.devices = collections.defaultdict(list)
        self.system = openvr_system
        self.poses = []

    def timer_callback(self):
        self.poses = self.system.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding,0,self.poses)
        print("=================")
        ########
        # # ETrackedDeviceClass = ENUM_TYPE
        # # TrackedDeviceClass_Invalid = ENUM_VALUE_TYPE(0)
        # # TrackedDeviceClass_HMD = ENUM_VALUE_TYPE(1)
        # # TrackedDeviceClass_Controller = ENUM_VALUE_TYPE(2)
        # # TrackedDeviceClass_GenericTracker = ENUM_VALUE_TYPE(3)
        # # TrackedDeviceClass_TrackingReference = ENUM_VALUE_TYPE(4)
        # # TrackedDeviceClass_DisplayRedirect = ENUM_VALUE_TYPE(5)
        # # TrackedDeviceClass_Max = ENUM_VALUE_TYPE(6)
        ########
        for idx, controller in enumerate(self.poses):
            ## Needed as the order of the devices may change ( based on which thing got turned on first)
            if self.system.getTrackedDeviceClass(idx) == 1:
                self.devices["hmd"].append(controller)
            elif self.system.getTrackedDeviceClass(idx) == 2:
                self.devices["controller"].append(controller)
        
        pose = utils.convert_to_quaternion(self.devices["controller"][0].mDeviceToAbsoluteTracking)

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