import sys
import time
import openvr
from simple_arm import utils
from pyquaternion import Quaternion
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from collections import defaultdict 


## https: // gist.github.com/awesomebytes/75daab3adb62b331f21ecf3a03b3ab46

def from_controller_state_to_dict(pControllerState):
    # docs: https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetControllerState
    d = {}
    d['unPacketNum'] = pControllerState.unPacketNum
    # on trigger .y is always 0.0 says the docs
    d['trigger'] = pControllerState.rAxis[1].x
    # 0.0 on trigger is fully released
    # -1.0 to 1.0 on joystick and trackpads
    d['trackpad_x'] = pControllerState.rAxis[0].x
    d['trackpad_y'] = pControllerState.rAxis[0].y
    # These are published and always 0.0
    # for i in range(2, 5):
    #     d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
    #     d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
    d['ulButtonPressed'] = pControllerState.ulButtonPressed
    d['ulButtonTouched'] = pControllerState.ulButtonTouched
    # To make easier to understand what is going on
    # Second bit marks menu button
    d['menu_button'] = bool(pControllerState.ulButtonPressed >> 1 & 1)
    # 32 bit marks trackpad
    d['trackpad_pressed'] = bool(pControllerState.ulButtonPressed >> 32 & 1)
    d['trackpad_touched'] = bool(pControllerState.ulButtonTouched >> 32 & 1)
    # third bit marks grip button
    d['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1)
    # System button can't be read, if you press it
    # the controllers stop reporting
    return d

class VrPublisher(Node):

    def __init__(self, openvr_system, buttons=False):
        super().__init__('vr_publisher')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.devices = defaultdict(list)
        self.system = openvr_system
        self.poses = []
        self.publishers_dict = {}
        self.prev_time = time.time.now()
        self.point = None
        self.velocity = Point()
        self.ang_velocity = Point()
        self.rot = None
        self.buttons = buttons

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
                if key == "controller":
                    result, pControllerState = self.system.getControllerState(el)
                    d = from_controller_state_to_dict(pControllerState)
                    if self.buttons:
                        print(d)

                pose = utils.convert_to_quaternion(el.mDeviceToAbsoluteTracking)
                point = Point()
                point.x = pose[0][0]
                point.y = pose[0][1]
                point.z = pose[0][2]    
                time = time.time.now()
                dtime = (time-self.time)
                if point is not None:
                    self.velocity.x = (point.x - self.point.x) / dtime
                    self.velocity.y = (point.y - self.point.y)/ dtime
                    self.velocity.z = (point.z - self.point.z)/ dtime
                    print(self.velocity.x, self.velocity.y, self.velocity.z)
                self.point = point



                rot = Quaternion()
                rot.x = pose[1][0]
                rot.y = pose[1][1]
                rot.z = pose[1][2]
                rot.w = pose[1][3]

                q1 = Quaternion(pose[1])
                if rot is not None:
                    diffQuater = q1 - self.rot
                    conjBoxQuater = q1.inverse
                    velQuater = ((diffQuater * 2.0) / dtime) * conjBoxQuater
                    self.ang_velocity = velQuater
                    print(self.ang_velocity)
                self.rot = q1
                
                msg = Pose()   
                msg.orientation = rot
                msg.position = point
                name = f"{key}/{name}"
                pub = self.publishers_dict.get(name)
                if pub is None:
                    pub = self.create_publisher(Pose, name, 10)
                    self.publishers_dict[name] = pub
                pub.publish(msg)


def main(buttons= False , args=None):
    if not openvr.isRuntimeInstalled:
        raise RuntimeError("OpenVR / SteamVr is not Installed Exit")
    if not openvr.isHmdPresent():
        raise RuntimeError("SteamVr is not running or Headmount is not plugged in")

    rclpy.init(args=args)
    system = openvr.init(openvr.VRApplication_Scene)

    minimal_publisher = VrPublisher(system, buttons)

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()
    openvr.shutdown()    


if __name__ == '__main__':
    main()
