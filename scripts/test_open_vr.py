
import sys
import openvr_localizer, localizer_base
import openvr
import time

if __name__ == "__main__":
    vrSys = openvr.init(openvr.VRApplication_Other)
    available_devices = openvr_localizer.OpenVRGlobalFrameEstimator.getAvailableDeviceNameAndIndexes(vrSys)
    
    device_index = None
    for (index,name) in available_devices:
        if 'vr_controller_vive' in name:
            device_index = index
            break
    
    if device_index is None:
        print("No tracker found")
        print(available_devices)
        exit(1)
    
    Tracker = openvr_localizer.OpenVRGlobalFrameEstimator(vrSys, device_index)
    while True:
        sys.stdout.flush()
        Tracker.update()
        print("Location",Tracker.getLocation())
        print("Velocity",Tracker.getVelocity())
        print("Acceleration",Tracker.getAcceleration())
        print("Local Velocity",Tracker.getLocalVelocity())
        print("Local Acceleration",Tracker.getLocalAcceleration())
        print("")
        time.sleep(0.1)
    
    openvr.shutdown()
