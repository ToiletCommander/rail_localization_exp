import time
import typing
import openvr
from localizer_base import GlobalFrameEstimatorImpl, NonBlocking, rotation_angle
import numpy as np

"""
// right-handed system
// +y is up
// +x is to the right
// -z is forward
// Distance unit is  meters
struct HmdMatrix34_t
{
    float m[3][4];
};
"""

"""
But we have (global frame)
+x is to the right
+y is forward
+z is up

therefore world-frame
openVR_X = our_X
openVR_Y = our_Z
openVR_Z = -our_Y
"""

from_openvr_transform_matrix = np.array([
    [1, 0, 0],
    [0, 0, -1],
    [0, 1, 0]
])

to_openvr_transform_matrix = np.array([
    [1, 0, 0],
    [0, 0, 1],
    [0, -1, 0]
])

class OpenVRGlobalFrameEstimator(GlobalFrameEstimatorImpl, NonBlocking):
    def __init__(self, vrSystem: openvr.IVRSystem, device_index : int):
        super().__init__(
            "OpenVRGlobalFrameEstimator", 
            autoUpdateLocation=False, 
            autoUpdateVelocity=False, 
            autoUpdateAcceleration=True,
            autoUpdateLocalVelocity=False,
            autoUpdateLocalAcceleration=False
        )
        self.vrSystem = vrSystem
        self.trackedPoseArray = []
        self.device_index = device_index
        self.__lastLocalVelocityUpdate = 0
    
    @classmethod
    def getAvailableDeviceNameAndIndexes(cls,vrSystem : openvr.IVRSystem) -> typing.List[typing.Tuple[int,str]]:
        lst : typing.List[typing.Tuple[int,str]] = []
        for device_index in range(openvr.k_unMaxTrackedDeviceCount):
            is_connected = vrSystem.isTrackedDeviceConnected(device_index)
            if not(is_connected):
                continue

            device_name = vrSystem.getStringTrackedDeviceProperty(
                device_index, 
                openvr.Prop_RenderModelName_String
            )
            lst.append((device_index, device_name))
            
        return lst
    
    def update(self):
        poses = self.vrSystem.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,0,openvr.k_unMaxTrackedDeviceCount
        )

        tracked_info = poses[self.device_index]
        
        """
        struct TrackedDevicePose_t
        {
            HmdMatrix34_t mDeviceToAbsoluteTracking;
            HmdVector3_t vVelocity;				// velocity in tracker space in m/s
            HmdVector3_t vAngularVelocity;		// angular velocity in radians/s (?)
            ETrackingResult eTrackingResult;
            bool bPoseIsValid;

            // This indicates that there is a device connected for this spot in the pose array.
            // It could go from true to false if the user unplugs the device.
            bool bDeviceIsConnected;
        };
        """
        if not(tracked_info.bPoseIsValid):
            print("Pose Invalid!")
            return

        vr_pose = np.ctypeslib.as_array(tracked_info.mDeviceToAbsoluteTracking[:], shape=(3, 4))
        vr_spaceVelocity = np.ctypeslib.as_array(tracked_info.vVelocity[:], shape=(3,))
        vr_angularVelocity = np.ctypeslib.as_array(tracked_info.vAngularVelocity[:],shape=(3,))

        #print("raw_data:", vr_pose, vr_spaceVelocity, vr_angularVelocity)


        """
        // right-handed system
        // +y is up
        // +x is to the right
        // -z is forward
        // Distance unit is  meters
        struct HmdMatrix34_t
        {
            float m[3][4];
        };
        """



        rotation_matrix_raw = vr_pose[:, :3]
        rotation_matrix_raw_inv = np.linalg.inv(rotation_matrix_raw)
        global_position_raw = vr_pose[:, 3]
        global_rotation_raw = rotation_angle(rotation_matrix_raw)

        spaceVelocity_local_raw = rotation_matrix_raw_inv @  vr_spaceVelocity
        angularVelocity_local_raw = rotation_matrix_raw_inv @ vr_angularVelocity

        print("raw coordinate position",global_position_raw,global_rotation_raw)
        transformed_global_position = from_openvr_transform_matrix @ global_position_raw
        transformed_global_rotation = from_openvr_transform_matrix @ global_rotation_raw
        transformed_spaceVelocity_global = from_openvr_transform_matrix @ vr_spaceVelocity
        transformed_angularVelocity_global = from_openvr_transform_matrix @ vr_angularVelocity
        transformed_spaceVelocity_local = from_openvr_transform_matrix @ spaceVelocity_local_raw
        transformed_angularVelocity_local = from_openvr_transform_matrix @ angularVelocity_local_raw
        self._call_location_update(np.concatenate([transformed_global_position,transformed_global_rotation]).reshape((6,)))
        self._call_velocity_update(np.concatenate([transformed_spaceVelocity_global,transformed_angularVelocity_global]).reshape((6,)))
        
        localVelocity = np.concatenate([transformed_spaceVelocity_local,transformed_angularVelocity_local]).reshape((6,))
        
        #print("transformed coordinate position",transformed_global_position,transformed_global_rotation)

        ctime = time.time()
        if self.__lastLocalVelocityUpdate != 0:
            dt = ctime - self.__lastLocalVelocityUpdate
            self._lastLocalAcceleration = (localVelocity - self._lastLocalVelocity) / dt
            for callback in self._localAccelerationSubscribeList:
                callback(self, self._lastLocalAcceleration)

        self._lastLocalVelocity = localVelocity
        self.__lastLocalVelocityUpdate = ctime
        for callback in self._localVelocitySubscribeList:
            callback(self, localVelocity)

    @classmethod
    def getOpenVRTrackerFromViveTracker() -> typing.Optional[typing.Any]:
        vrSys = openvr.init(openvr.VRApplication_Other)
        available_devices = __class__.getAvailableDeviceNameAndIndexes(vrSys)
        
        device_index = None
        for (index,name) in available_devices:
            if 'vr_tracker_vive' in name:
                device_index = index
                break
        
        if device_index is None:
            return None
            
        Tracker = __class__(vrSys, device_index)
    
    @classmethod
    def terminateOpenVRSystem():
        openvr.shutdown()