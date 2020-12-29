import sys
import time
import openvr
from simple_arm import utils

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from collections import defaultdict 

class VrPublisher(Node):

    def __init__(self, openvr_system):
        super().__init__('vr_publisher')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.devices = defaultdict(list)
        self.system = openvr_system
        self.poses = []
        self.publishers_dict = {}

    def timer_callback(self):
        self.poses = self.system.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding,0,self.poses)
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
       	# # TrackedControllerRole_LeftHand = 1,					// Tracked device associated with the left hand
        # # TrackedControllerRole_RightHand = 2,				// Tracked device associated with the right hand
        ########
        for idx, controller in enumerate(self.poses):
            ## Needed as the order of the devices may change ( based on which thing got turned on first)
            if not self.system.isTrackedDeviceConnected(idx):
                continue
            if self.system.getTrackedDeviceClass(idx) == 1 and len(self.devices["hmd"]) <= 1:
                self.devices["hmd"].append(("hmd", controller))
            elif self.system.getTrackedDeviceClass(idx) == 2 and len(self.devices["controller"]) <= 2:
                controller_role = self.system.getControllerRoleForTrackedDeviceIndex(idx)
                hand = ""
                if (controller_role ==1):
                    hand = "LeftHand"
                if controller_role == 2:
                    hand = "RightHand"
                self.devices["controller"].append((hand, controller))
        for key, device in self.devices.items():
            for idx, (name, el) in enumerate(device):
                pose = utils.convert_to_quaternion(el.mDeviceToAbsoluteTracking)
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
                name = f"{key}/{name}"
                pub = self.publishers_dict.get(name)
                if pub is None:
                    pub = self.create_publisher(Pose, name, 10)
                    self.publishers_dict[name] = pub
                pub.publish(msg)


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
