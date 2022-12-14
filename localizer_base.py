import typing
import numpy as np
import time
import math

class NonBlocking:
    def update(self) -> None:
        return
    
    def stop(self) -> None:
        return

class Parallel:
    def __init__(self, wait_time : float = 0.01) -> None:
        self._running = True
        self.parallel_wait_time = wait_time
    
    def stop(self) -> None:
        self._running = False
        self.parallel_wait_time = 0.0


"""
Note: The ordering of location/velocity/acceleration is: [x, y, z, degX, degY, degZ]
Or in other words: [x, y, z, roll, pitch, yaw]
autoUpdate decides if any other variables should be used to update the current variable
"""

"""
Returns angX, angY, angZ
"""
def rotation_angle(rot_matrix : np.ndarray) -> typing.Tuple[float, float, float]:
    assert rot_matrix.shape == (3,3)
    return (
        np.arctan2(rot_matrix[2,1], rot_matrix[2,2]), 
        np.arctan2(-rot_matrix[2,0], np.sqrt(rot_matrix[2,1]**2 + rot_matrix[2,2]**2)), 
        np.arctan2(rot_matrix[1,0], rot_matrix[0,0])
    )

# gives WXYZ quaternion
# https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def quaternion_from_rotation(roll, pitch, yaw) -> typing.Tuple[float,float,float,float]:
    # Abbreviations for the various angular functions
    cy = np.cos(yaw * 0.5);
    sy = np.sin(yaw * 0.5);
    cp = np.cos(pitch * 0.5);
    sp = np.sin(pitch * 0.5);
    cr = np.cos(roll * 0.5);
    sr = np.sin(roll * 0.5);

    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;

    return (w,x,y,z);

# https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def rotation_angle_from_quaternion(q) -> typing.Tuple[float,float,float]:
    
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
    cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q[0] * q[2] - q[3] * q[1])
    pitch = 0
    if (np.abs(sinp) >= 1):
        pitch = np.copysign(np.pi / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
    cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)

def rotation_matrix_and_inverse_rotation_matrix(
    roll,
    pitch,
    yaw
) -> np.ndarray:
    c1 = np.cos(roll)
    c2 = np.cos(pitch)
    c3 = np.cos(yaw)
    s1 = np.sin(roll)
    s2 = np.sin(pitch)
    s3 = np.sin(yaw)
    ns1 = -s1
    ns2 = -s2
    ns3 = -s3

    R_Z = np.array([
        [c3, -s3, 0],
        [s3 , c3, 0],
        [0, 0, 1]
    ])

    R_Y = np.array([
        [c2, 0, s2],
        [0 ,1 ,0],
        [-s2, 0, c2]
    ])

    R_X = np.array([
        [1, 0, 0],
        [0, c1, -s1],
        [0, s1, c1]
    ])

    nR_Z = R_Z.copy()
    nR_Z[0,1] = -ns3
    nR_Z[1,0] = ns3
    nR_Y = R_Y.copy()
    nR_Y[0,2] = ns2
    nR_Y[2,0] = -ns2
    nR_X = R_X.copy()
    nR_X[1,2] = -ns1
    nR_X[2,1] = ns1

    rot_mat = (R_Z @ R_Y @ R_X)
    inv_rot_mat = (nR_X @ nR_Y @ nR_Z)
    return rot_mat, inv_rot_mat


def rotation_matrix(
    roll,
    pitch,
    yaw
) -> np.ndarray:
    c1 = np.cos(roll)
    c2 = np.cos(pitch)
    c3 = np.cos(yaw)
    s1 = np.sin(roll)
    s2 = np.sin(pitch)
    s3 = np.sin(yaw)

    R_Z = np.array([
        [c3, -s3, 0],
        [s3 , c3, 0],
        [0, 0, 1]
    ])

    R_Y = np.array([
        [c2, 0, s2],
        [0 ,1 ,0],
        [-s2, 0, c2]
    ])

    R_X = np.array([
        [1, 0, 0],
        [0, c1, -s1],
        [0, s1, c1]
    ])
    
    return (R_Z @ R_Y @ R_X)

    """
    return np.array([
        [c1*c2, c1*s2*s3 - c3*s1, s1*s3 + c1*c3*s2],
        [c2*s1, c1*c3 + s1*s2*s3, c3*s1*s2 - c1*s3],
        [s2, c2*s3, c2*c3]
    ])
    """

def rotation_matrix_inverse(
    roll,
    pitch,
    yaw
) -> np.ndarray:
    #return np.linalg.inv(rotation_matrix(roll, pitch, yaw))
    c1 = np.cos(-roll)
    c2 = np.cos(-pitch)
    c3 = np.cos(-yaw)
    s1 = np.sin(-roll)
    s2 = np.sin(-pitch)
    s3 = np.sin(-yaw)

    R_Z = np.array([
        [c3, -s3, 0],
        [s3 , c3, 0],
        [0, 0, 1]
    ])

    R_Y = np.array([
        [c2, 0, s2],
        [0 ,1 ,0],
        [-s2, 0, c2]
    ])

    R_X = np.array([
        [1, 0, 0],
        [0, c1, -s1],
        [0, s1, c1]
    ])
    
    return (R_X @ R_Y @ R_Z)

def coordinate_transform_to_global(
    to_transform: np.ndarray,
    robot_global_location: np.ndarray
) -> np.ndarray:
    assert to_transform.shape == (6,) and robot_global_location.shape == (6,)

    int_mat = np.hstack([
        to_transform[:3],
        wrap_angle_rad(to_transform[3:])
    ]).reshape(3,2)
    rot_mat = rotation_matrix(*robot_global_location[3:])
    int_rst = rot_mat @ int_mat
    return np.vstack([
        int_rst[:,0] + robot_global_location[:3],
        wrap_angle_rad(int_rst[:,1])
    ]).reshape(6,)

def coordinate_transform_to_local(
    to_transform: np.ndarray,
    robot_global_location: np.ndarray
) -> np.ndarray:
    assert to_transform.shape == (6,) and robot_global_location.shape == (6,)
    
    int_mat = np.hstack([
        to_transform[:3] - robot_global_location[:3],
        wrap_angle_rad(to_transform[3:])
    ]).reshape(3,2)
    rot_mat = rotation_matrix_inverse(*robot_global_location[3:])
    int_rst = rot_mat @ int_mat
    return np.vstack([
        int_rst[:,0],
        wrap_angle_rad(int_rst[:,1])
    ]).reshape(6,)

def wrap_angle_rad(angle : typing.Union[float, np.ndarray]) -> typing.Union[float, np.ndarray]: # Wrap angle to [-pi, pi)
    if isinstance(angle, np.ndarray):
        angle = angle % (2 * np.pi)
        angle[angle >= np.pi] -= 2 * np.pi
        angle[angle < -np.pi] += 2 * np.pi
        return angle
    else:
        angle = angle % (2 * np.pi)
        if angle >= np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
        return angle

class LocalFrameEstimator:
    def reset(self) -> None:
        raise NotImplementedError
    
    def getLocalVelocity(self) -> typing.Optional[np.ndarray]:
        raise NotImplementedError
    
    def getLocalAcceleration(self) -> typing.Optional[np.ndarray]:
        raise NotImplementedError

class LocalFrameEstimatorImpl(LocalFrameEstimator):
    def __init__(self, name: str, autoUpdateVelocity : bool = True, autoUpdateAcceleration : bool = True):
        self.name = name
        self.autoUpdateVelocity = autoUpdateVelocity
        self.autoUpdateAcceleration = autoUpdateAcceleration

        LocalFrameEstimatorImpl.reset(self)
    
    def __str__(self) -> str:
        return self.name + "(LocalFrameEstimatorImpl)"
    
    def reset(self) -> None:
        self._lastLocalVelocity : np.ndarray = np.zeros((6,), dtype=np.float32)
        self._lastLocalVelocityUpdate : float = 0
        self._lastLocalAcceleration : np.ndarray = np.zeros((6,), dtype=np.float32)
        self._lastLocalAccelerationUpdate : float = 0
    
    def _call_local_velocity_update(self, new_local_velocity : np.ndarray) -> None:
        assert new_local_velocity.shape == (6,)
        
        ctime = time.time()
        if self._lastLocalVelocityUpdate != 0:
            dt = ctime - self._lastLocalVelocityUpdate
            self._local_velocity_updated(new_local_velocity, self._lastLocalVelocity, ctime, dt, True)
        else:
            self._local_velocity_updated(new_local_velocity, None, ctime, None, False)
        
    def _call_local_acceleration_update(self, new_local_acceleration : np.ndarray) -> None:
        assert new_local_acceleration.shape == (6,)
        
        ctime = time.time()
        if self._lastLocalAccelerationUpdate != 0:
            dt = ctime - self._lastLocalAccelerationUpdate
            self._local_acceleration_updated(new_local_acceleration, self._lastLocalAcceleration, ctime, dt, True)
        else:
            self._local_acceleration_updated(new_local_acceleration, None, ctime, None, True)
    
    def _local_velocity_updated(self, new_local_velocity : np.ndarray, old_local_velocity : typing.Optional[np.ndarray], ctime : float, dt : float, try_update_local_acceleration : bool = True) -> None:
        self._lastLocalVelocity = new_local_velocity
        self._lastLocalVelocityUpdate = ctime

        if try_update_local_acceleration and self.autoUpdateAcceleration and (old_local_velocity is not None):
            acceleration = (new_local_velocity - old_local_velocity) / dt
            self._local_acceleration_updated(
                acceleration, 
                self._lastLocalAcceleration, 
                ctime, 
                (ctime - self._lastLocalAccelerationUpdate) if self._lastLocalAccelerationUpdate != 0 else None, 
                False
            )

    
    def _local_acceleration_updated(self, new_local_acceleration : np.ndarray, old_local_acceleration : typing.Optional[np.ndarray], ctime : float, dt : float, try_update_local_velocity : bool = True) -> None:
        self._lastLocalAcceleration = new_local_acceleration
        self._lastLocalAccelerationUpdate = ctime

        if try_update_local_velocity and self.autoUpdateVelocity and self._lastLocalVelocityUpdate != 0:
            velocity = np.zeros((6,), dtype=np.float32)
            if old_local_acceleration is not None:
                velocity = self._lastLocalVelocity + (new_local_acceleration + old_local_acceleration) / 2.0 * dt
            else:
                velocity = self._lastLocalVelocity + new_local_acceleration * dt
            self._local_velocity_updated(velocity, self._lastLocalVelocity, ctime, ctime - self._lastLocalVelocityUpdate, False)
        
    def getLocalVelocity(self) -> typing.Optional[np.ndarray]:
        return self._lastLocalVelocity
    
    def getLocalAcceleration(self) -> typing.Optional[np.ndarray]:
        return self._lastLocalAcceleration


class GlobalFrameEstimator:
    def reset(self) -> None:
        raise NotImplementedError
    
    def getLocation(self) -> typing.Optional[np.ndarray]:
        raise NotImplementedError
    
    def getVelocity(self) -> typing.Optional[np.ndarray]:
        raise NotImplementedError
    
    def getAcceleration(self) -> typing.Optional[np.ndarray]:
        raise NotImplementedError

class GlobalFrameEstimatorImpl(LocalFrameEstimator):
    def __init__(self, name : str, autoUpdateLocation : bool = True, autoUpdateVelocity : bool = True, autoUpdateAcceleration : bool = True, autoUpdateLocalVelocity : bool = True, autoUpdateLocalAcceleration : bool = True):
        self.name = name
        self.autoUpdateLocation = autoUpdateLocation
        self.autoUpdateVelocity = autoUpdateVelocity
        self.autoUpdateAcceleration = autoUpdateAcceleration
        self.autoupdateLocalVelocity = autoUpdateLocalVelocity
        self.autoUpdateLocalAcceleration = autoUpdateLocalAcceleration

        GlobalFrameEstimatorImpl.reset(self)

    def __str__(self) -> str:
        return self.name + "(GlobalFrameEstimatorImpl)"
    
    def reset(self) -> None:
        self._lastLocationUpdate : float = 0
        self._lastVelocityUpdate : float = 0
        self._lastAccelerationUpdate : float = 0
        self._lastAcceleration : np.ndarray = np.zeros((6,), dtype=np.float32)
        self._lastVelocity : np.ndarray = np.zeros((6,), dtype=np.float32)
        self._lastLocation : np.ndarray = np.zeros((6,), dtype=np.float32)
        self._lastLocalVelocity : typing.Optional[np.ndarray] = None
        self._lastLocalAcceleration : typing.Optional[np.ndarray] = None
    
    
    def _call_location_update(self, new_location : np.ndarray) -> None:
        assert new_location.shape == (6,)
        
        ctime = time.time()
        if self._lastLocationUpdate != 0:
            dt = ctime - self._lastLocationUpdate
            self._location_updated(new_location, self._lastLocation, ctime, dt, True, True)
        else:
            self._location_updated(new_location, None, ctime, None, False, False)
        
    def _call_velocity_update(self, new_velocity : np.ndarray) -> None:
        assert new_velocity.shape == (6,)
        
        ctime = time.time()
        if self._lastVelocityUpdate != 0:
            dt = ctime - self._lastVelocityUpdate
            self._velocity_updated(new_velocity, self._lastVelocity, ctime, dt, True, True)
        else:
            self._velocity_updated(new_velocity, None, ctime, None, False, False)
    
    def _call_acceleration_update(self, new_acceleration : np.ndarray) -> None:
        assert new_acceleration.shape == (6,)
        
        ctime = time.time()
        if self._lastAccelerationUpdate != 0:
            dt = ctime - self._lastAccelerationUpdate
            self._acceleration_updated(new_acceleration, self._lastAcceleration, ctime, dt, True, True)
        else:
            self._acceleration_updated(new_acceleration, None, ctime, None, False, False)
    
    def _call_local_velocity_update(self, new_local_velocity : np.ndarray) -> None:
        assert new_local_velocity.shape == (6,)
        
        self._lastLocalVelocity = new_local_velocity
        
    
    def _call_local_acceleration_update(self, new_local_acceleration : np.ndarray) -> None:
        assert new_local_acceleration.shape == (6,)
        
        self._lastLocalAcceleration = new_local_acceleration
        

    def _location_updated(self,new_location : np.ndarray, old_location : typing.Optional[np.ndarray], time : float, dt : typing.Optional[float], try_update_velocity: bool = True, try_update_acceleration : bool = True) -> None:
        self._lastLocation = new_location
        self._lastLocationUpdate = time
        
        if try_update_velocity and (old_location is not None) and self.autoUpdateVelocity:
            velocity = (new_location - old_location) / dt
            self._velocity_updated(
                velocity, 
                self._lastVelocity, 
                time, 
                (time - self._lastVelocityUpdate) if self._lastVelocityUpdate != 0 else None, 
                try_update_location = False, 
                try_update_acceleration = try_update_acceleration
            )
    
    def _velocity_updated(self,new_velocity : np.ndarray, old_velocity : typing.Optional[np.ndarray], time : float, dt : typing.Optional[float], try_update_location : bool = True, try_update_acceleration : bool = True) -> None:
        self._lastVelocity = new_velocity
        self._lastVelocityUpdate = time

        if try_update_acceleration and (old_velocity is not None) and self.autoUpdateAcceleration:
            acceleration = (new_velocity - old_velocity) / dt
            self._acceleration_updated(
                acceleration, 
                self._lastAcceleration, 
                time, 
                (time - self._lastAccelerationUpdate) if self._lastAccelerationUpdate != 0 else None, 
                try_update_location = False, 
                try_update_velocity = False
            )
        
        if try_update_location and self.autoUpdateLocation and self._lastLocationUpdate != 0:
            location = np.zeros((6,), dtype=np.float32)
            if old_velocity is not None:
                location = self._lastLocation + (new_velocity + old_velocity) / 2.0 * dt
            else:
                location = self._lastLocation + new_velocity * dt

            self._location_updated(
                location, 
                self._lastLocation, 
                time, 
                (time - self._lastLocationUpdate) if self._lastLocationUpdate != 0  else None, 
                try_update_velocity = False, 
                try_update_acceleration = False
            )
        
        
        if self._lastLocationUpdate != 0 and self.autoupdateLocalVelocity:
            self._lastLocalVelocity = coordinate_transform_to_local(new_velocity, self._lastLocation)
            for callback in self._localVelocitySubscribeList:
                callback(self,self._lastLocalVelocity)


    def _acceleration_updated(self,new_acceleration : np.ndarray, old_acceleration : typing.Optional[np.ndarray], time : float, dt : typing.Optional[float], try_update_location : bool = True, try_update_velocity : bool = True) -> None:
        self._lastAcceleration = new_acceleration
        self._lastAccelerationUpdate = time

        if try_update_velocity and self.autoUpdateVelocity and self._lastVelocityUpdate != 0:
            velocity = np.zeros((6,), dtype=np.float32)
            if old_acceleration is not None:
                velocity = self._lastVelocity + (new_acceleration + old_acceleration) / 2.0 * dt
            else:
                velocity = self._lastVelocity + new_acceleration * dt
            
            self._velocity_updated(
                velocity, 
                self._lastVelocity, 
                time, 
                (time - self._lastVelocityUpdate) if self._lastVelocityUpdate != 0 else None, 
                try_update_location = try_update_location, 
                try_update_acceleration = False
            )

        if self._lastLocationUpdate != 0 and self.autoUpdateLocalAcceleration:
            self._lastLocalAcceleration = coordinate_transform_to_local(new_acceleration, self._lastLocation)
            for callback in self._localAccelerationSubscribeList:
                callback(self,self._lastLocalAcceleration)
        
    def getLocation(self) -> typing.Optional[np.ndarray]:
        return self._lastLocation if self._lastLocationUpdate != 0 else None

    def getVelocity(self) -> typing.Optional[np.ndarray]:
        return self._lastVelocity if self._lastVelocityUpdate != 0 else None
    
    def getAcceleration(self) -> typing.Optional[np.ndarray]:
        return self._lastAcceleration if self._lastAccelerationUpdate != 0 else None
    
    def getLocalVelocity(self) -> typing.Optional[np.ndarray]:
        return self._lastLocalVelocity
    
    def getLocalAcceleration(self) -> typing.Optional[np.ndarray]:
        return self._lastLocalAcceleration

class LocalCoordinateTransformedEstimatorImpl(LocalFrameEstimator):

    @classmethod
    def generateTransformMatrices(cls, newX : np.ndarray, newY : np.ndarray, newZ : np.ndarray) -> typing.Tuple[np.ndarray, np.ndarray]:
        assert newX.shape == (3,) and newY.shape == (3,) and newZ.shape == (3,)
        
        newX_unit = newX / np.linalg.norm(newX)
        newY_unit = newY / np.linalg.norm(newY)
        newZ_unit = newZ / np.linalg.norm(newZ)

        toOriginalFrameMatrix = np.hstack([
            newX.reshape((3,1)),
            newY.reshape((3,1)),
            newZ.reshape((3,1))
        ])

        toNewFrameMatrix = np.linalg.inv(toOriginalFrameMatrix)

        toOriginalFrameMatrixUnit = np.hstack([
            newX_unit.reshape((3,1)),
            newY_unit.reshape((3,1)),
            newZ_unit.reshape((3,1))
        ])

        toNewFrameMatrixUnit = np.linalg.inv(toOriginalFrameMatrixUnit)

        return toOriginalFrameMatrix, toNewFrameMatrix, toOriginalFrameMatrixUnit, toNewFrameMatrixUnit


    def __init__(
        self, 
        base : LocalFrameEstimator, 
        newX : np.ndarray,
        newY : np.ndarray,
        newZ : np.ndarray
    ):
        assert newX.shape == (3,) and newY.shape == (3,) and newZ.shape == (3,)
        self._newX = newX
        self._newY = newY
        self._newZ = newZ
        self.base = base

        self._calcNewTransformMat()
    
    def _calcNewTransformMat(self):
        rst = __class__.generateTransformMatrices(self._newX, self._newY, self._newZ)
        self._toOriginalFrameMat : np.ndarray = rst[0]
        self._toNewFrameMat : np.ndarray = rst[1]
        self._toOriginalFrameMatUnit : np.ndarray = rst[2]
        self._toNewFrameMatUnit : np.ndarray = rst[3]
    

    def getNewX(self):
        return self._newX
    
    def getNewY(self):
        return self._newY
    
    def getNewZ(self):
        return self._newZ
    
    def setNewX(self, newX: np.ndarray):
        assert newX.shape == (3,)

        self._newX = newX
        self._calcNewTransformMat()

    def setNewY(self, newY: np.ndarray):
        assert newY.shape == (3,)

        self._newY = newY
        self._calcNewTransformMat()

    def setNewZ(self, newZ: np.ndarray):
        assert newZ.shape == (3,)

        self._newZ = newZ
        self._calcNewTransformMat()

    def getLocalAcceleration(self) -> typing.Optional[np.ndarray]:
        superAcc = self.base.getLocalAcceleration()
        if superAcc is None:
            return None
        else:
            return np.concatenate([
                self._toNewFrameMat @ superAcc[:3].reshape((3,1)),
                self._toNewFrameMatUnit @ superAcc[3:].reshape((3,1))
            ]).reshape((6,))
    
    def getLocalVelocity(self) -> typing.Optional[np.ndarray]:
        superVel = self.base.getLocalVelocity()
        if superVel is None:
            return None
        else:
            return np.concatenate([
                self._toNewFrameMat @ superVel[:3].reshape((3,1)),
                self._toNewFrameMatUnit @ superVel[3:].reshape((3,1))
            ]).reshape((6,))