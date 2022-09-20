import time
import typing
import openvr
from localizer_base import GlobalFrameEstimatorImpl, LocalFrameEstimatorImpl, NonBlocking, rotation_angle_from_quaternion, rotation_matrix, rotation_matrix_inverse
import numpy as np
from robot_interface import RobotInterface # pytype: disable=import-error
import inspect
import os
import sys
import time

currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)



"""
A1 Local Coordinate Frame
+x => Back
+Y => Right
+Z => Down
"""

"""
But we have
+x is front
+y is left
+z is up

therefore local-frame
a1_X = -our_X
a1_Y = -our_y
a1_Z = -our_Z
"""

from_a1_frame_mat = np.array([
    [-1, 0, 0],
    [0, -1, 0],
    [0, 0, -1]
])

to_a1_frame_mat = from_a1_frame_mat

class A1RobotIMULocalEstimator(GlobalFrameEstimatorImpl, NonBlocking):
    @classmethod
    def getAndInitRobotInterface(cls) -> RobotInterface:
        _robot_interface = RobotInterface() # type: RobotInterface
        _robot_interface.send_command(np.zeros(60, dtype=np.float32))
        return _robot_interface

    def __init__(self, robot_interface : typing.Optional[RobotInterface]):
        GlobalFrameEstimatorImpl.__init__(
            self,
            "A1RobotIMUEstimator",
            autoUpdateLocation=False,
            autoUpdateVelocity=False,
            autoUpdateAcceleration=False,
            autoUpdateLocalVelocity=True,
            autoUpdateLocalAcceleration=True
        )
        self.robot_interface = robot_interface

        ctime = time.time()

        self._lastGlobalAngularLocation : np.ndarray = None
        self._lastGlobalAngularLocationUpdate : float = 0

        self._lastGlobalAngularVelocity : np.ndarray = np.zeros((3,),dtype=np.float32)
        self._lastGlobalAngularVelocityUpdate : float = ctime

        self._lastGlobalLinearVelocity : np.ndarray = np.zeros((3,),dtype=np.float32)
        self._lastGlobalLinearVelocityUpdate : float = ctime

        self._lastGlobalLinearAcceleration : np.ndarray = np.zeros((3,),dtype=np.float32)
        self._lastGlobalLinearAccelerationUpdate : float = ctime

        self.negative_gravity_vector = np.array([0,0,-9.8])

        self.calibrate_for_gravity()
        self.update()

    def reset(self) -> None:
        super().reset()
        ctime = time.time()
        self._lastGlobalAngularLocation : np.ndarray = np.zeros((3,),dtype=np.float32)
        self._lastGlobalAngularLocationUpdate : float = ctime

        self._lastGlobalLinearVelocity : np.ndarray = np.zeros((3,),dtype=np.float32)
        self._lastGlobalLinearVelocityUpdate : float = ctime

        self._lastGlobalLinearAcceleration : np.ndarray = np.zeros((3,),dtype=np.float32)
        self._lastGlobalLinearAccelerationUpdate : float = ctime
    
    def resetAsOrigin(self) -> None:
        if self._lastLocation is not None:
            self._lastLocation[:3] = 0
    
    def getAngularLocation(self) -> np.ndarray:
        return self._lastLocalAngularLocation()
        
    def calibrate_for_gravity(self, duration_to_take_mean) -> None:
        start_time = time.time()
        sum_of_gravity = np.array([0,0,0])
        last_time = start_time
        while True:
            ctime = time.time()
            dt = ctime - last_time
            if ctime - start_time > duration_to_take_mean:
                break


            observation = self.robot_interface.receive_observation()

            q = observation.imu.quaternion
            accelerometer_reading = np.array(observation.imu.accelerometer)
            
            if np.all(accelerometer_reading == 0):
                raise Exception("Accelerometer reading is 0,0,0. Is the IMU connected?")
            
            rot_ang = rotation_angle_from_quaternion(q)
            rot_mat = rotation_matrix(*rot_ang)
            accelerometer_reading_global = rot_mat @ accelerometer_reading
            sum_of_gravity += accelerometer_reading_global * dt

        gravity = sum_of_gravity / duration_to_take_mean
        self.negative_gravity_vector = -gravity

    def update(self) -> None:
        if self.robot_interface is not None:
            self.updateFromObservation(self.robot_interface.receive_observation())

    def updateFromObservation(self, observation : typing.Any) -> None:
        #wxyz quaternion
        q = observation.imu.quaternion

        #xyz acceleration in original coordinates
        raw_accelerometer_reading = np.array(observation.imu.accelerometer)

        raw_rot_ang = rotation_angle_from_quaternion(q)
        raw_inv_rot_mat = rotation_matrix_inverse(*raw_rot_ang)


        calibrated_accelerometer_raw_reading = raw_accelerometer_reading + raw_inv_rot_mat @ self.negative_gravity_vector
        
        if np.all(raw_accelerometer_reading == 0):
            calibrated_accelerometer_raw_reading = np.zeros((3,))
            raw_rot_ang = np.zeros((3,))
            return


        #print(calibrated_accelerometer_reading)

        global_angular_location = from_a1_frame_mat @ raw_rot_ang

        accelerometer_reading_local = from_a1_frame_mat @ calibrated_accelerometer_raw_reading
        rot_mat_to_global = rotation_matrix(*global_angular_location)
        
        global_linear_acceleration = rot_mat_to_global @ accelerometer_reading_local

        global_angular_velocity : typing.Optional[np.ndarray] = None
        global_angular_acceleration : typing.Optional[np.ndarray] = None

        global_linear_velocity : typing.Optional[np.ndarray] = None
        global_linear_location : typing.Optional[np.ndarray] = None

        ctime = time.time()

        if self._lastGlobalAngularLocationUpdate != 0:
            dt = ctime - self._lastGlobalAngularLocationUpdate
            global_angular_velocity = (global_angular_location - self._lastGlobalAngularLocation) / dt
            
            if self._lastGlobalAngularVelocityUpdate != 0:
                dt = ctime - self._lastGlobalAngularVelocityUpdate
                global_angular_acceleration = (global_angular_velocity - self._lastGlobalAngularVelocity) / dt

        if self._lastGlobalLinearAccelerationUpdate != 0:
            dt = ctime - self._lastGlobalLinearAccelerationUpdate
            delta_v = (global_linear_acceleration + self._lastGlobalLinearAcceleration) * dt / 2.0
            
            if self._lastGlobalLinearVelocityUpdate != 0:
                global_linear_velocity = self._lastGlobalLinearVelocity + delta_v
            else:
                global_linear_velocity = delta_v
        
            if self._lastGlobalLinearVelocityUpdate != 0:
                dt = ctime - self._lastGlobalLinearVelocityUpdate
                delta_pos = (global_linear_velocity + self._lastGlobalLinearVelocity) * dt / 2.0
                if self._lastLocationUpdate != 0:
                    global_linear_location = self._lastLocation[:3] + delta_pos
                else:
                    global_linear_location = delta_pos
            
        self._lastGlobalAngularLocation = global_angular_location
        self._lastGlobalAngularLocationUpdate = ctime
        
        if global_angular_velocity is not None:
            self._lastGlobalAngularVelocity = global_angular_velocity
            self._lastGlobalAngularVelocityUpdate = ctime

        self._lastGlobalLinearAcceleration = global_linear_acceleration
        self._lastGlobalLinearAccelerationUpdate = ctime

        if global_linear_velocity is not None:
            self._lastGlobalLinearVelocity = global_linear_velocity
            self._lastGlobalLinearVelocityUpdate = ctime

        
        if self._lastLocationUpdate == 0:
            new_location = np.zeros((6,),dtype=np.float32)
        else:
            new_location = self._lastLocation
        new_location[3:] = global_angular_location
        if global_linear_location is not None:
            new_location[:3] = global_linear_location
        self._call_location_update(new_location)

        if global_angular_velocity is not None or global_linear_velocity is not None:
            if self._lastVelocityUpdate == 0:
                new_velocity = np.zeros((6,),dtype=np.float32)
            else:
                new_velocity = self._lastVelocity
            if global_angular_velocity is not None:
                new_velocity[3:] = global_angular_velocity
            if global_linear_velocity is not None:
                new_velocity[:3] = global_linear_velocity
            self._call_velocity_update(new_velocity)
        
        if global_angular_acceleration is not None or global_linear_acceleration is not None:
            if self._lastAccelerationUpdate == 0:
                new_acceleration = np.zeros((6,),dtype=np.float32)
            else:
                new_acceleration = self._lastAcceleration
            if global_angular_acceleration is not None:
                new_acceleration[3:] = global_angular_acceleration
            if global_linear_acceleration is not None:
                new_acceleration[:3] = global_linear_acceleration
            self._call_acceleration_update(new_acceleration)
        
        
        
