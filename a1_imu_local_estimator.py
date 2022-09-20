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
    def __init__(self, robot_interface : typing.Optional[RobotInterface]):
        GlobalFrameEstimatorImpl.__init__(
            self,
            "A1RobotIMUEstimator",
            autoUpdateLocation=False,
            autoUpdateVelocity=False,
            autoUpdateAcceleration=False,
            autoUpdateLocalVelocity=False,
            autoUpdateLocalAcceleration=False
        )
        self.robot_interface = robot_interface

        self._lastGlobalAngularLocation : np.ndarray = None
        self._lastGlobalAngularLocationUpdate : float = 0

        self._lastGlobalLinearVelocity : np.ndarray = None
        self._lastGlobalLinearVelocityUpdate : float = 0

        self._lastGlobalAcceleration : np.ndarray = None
        self._lastGlobalAccelerationUpdate : float = 0

        self.negative_gravity_vector = np.array([0,0,-9.8])

        self.update()
    
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

        rot_ang_global = from_a1_frame_mat @ raw_rot_ang
        accelerometer_reading_local = from_a1_frame_mat @ calibrated_accelerometer_raw_reading
        rot_mat_to_global = rotation_matrix(*rot_ang_global)
        accelerometer_reading_global = rot_mat_to_global @ accelerometer_reading_local

        
        


