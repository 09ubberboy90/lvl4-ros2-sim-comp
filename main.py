import sys
import time
import openvr
import utils

if not openvr.isRuntimeInstalled:
    raise RuntimeError("OpenVR / SteamVr is not Installed Exit")
if not openvr.isHmdPresent():
    raise RuntimeError("SteamVr is not running or Headmount is not plugged in")

system = openvr.init(openvr.VRApplication_Scene)
devices = {}
while(openvr.isHmdPresent()):

    ## Get the pose of everything connected to openvr 
    poses = system.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding,0,0)
    devices["hmd"] = poses[openvr.k_unTrackedDeviceIndex_Hmd] 
    ## idx 2 and 3 are the lighthouse position so ignored that
    for idx, controller in enumerate(poses[3:]):
        if (controller.bPoseIsValid):
            devices["controller_"+str(idx)] = controller

    for key, el in devices.items():
        print(f"{key} : {utils.convert_to_euler(el.mDeviceToAbsoluteTracking)}")
    print("==================================")
    sys.stdout.flush()
    time.sleep(0.2)
openvr.shutdown()    
