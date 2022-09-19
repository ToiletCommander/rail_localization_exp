import time
import typing
import openvr
from localizer_base import LocalFrameEstimatorImpl, NonBlocking, rotation_angle_from_quaternion, rotation_matrix, rotation_matrix_inverse
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

class A1RobotIMULocalEstimator(LocalFrameEstimatorImpl, NonBlocking):
    def __init__(self, robot_interface : typing.Optional[RobotInterface]):
        LocalFrameEstimatorImpl.__init__(
            self,
            "A1RobotIMULocalEstimator",
            autoUpdateVelocity=False,
            autoUpdateAcceleration=False
        )
        self.robot_interface = robot_interface

        self.__lastLocalAngularLocation : np.ndarray = None
        self.__lastLocalAngularLocationUpdate : float = 0

        self.__lastLocalLinearVelocity : np.ndarray = None
        self.__lastLocalLinearVelocityUpdate : float = 0
    
    def update(self) -> None:
        if self.robot_interface is not None:
            self.updateFromObservation(self.robot_interface.receive_observation())
    
    def updateFromObservation(self, observation : typing.Any) -> None:
        #wxyz quaternion
        q = observation.imu.quaternion

        #xyz acceleration in original coordinates
        accelerometer_reading = np.array(observation.imu.accelerometer)

        """
        _motor_angles = np.array(
            [motor.q for motor in state.motorState[:12]])
        _motor_velocities = np.array(
            [motor.dq for motor in state.motorState[:12]])
        _joint_states = np.array(
            list(zip(_motor_angles, _motor_velocities)))
        _observed_motor_torques = np.array(
            [motor.tauEst for motor in state.motorState[:12]])
        _motor_temperatures = np.array(
            [motor.temperature for motor in state.motorState[:12]])
        """
        rot_ang = rotation_angle_from_quaternion(q)
        inv_rot_mat = rotation_matrix_inverse(*rot_ang)


        calibrated_accelerometer_reading = accelerometer_reading + inv_rot_mat @ np.array([0,0,-9.8])
        
        if np.all(accelerometer_reading == 0):
            calibrated_accelerometer_reading = np.zeros((3,))
            rot_ang = np.zeros((3,))


        #print(calibrated_accelerometer_reading)

        transformed_rot_ang = from_a1_frame_mat @ rot_ang
        transformed_calibrated_accelerometer_reading = from_a1_frame_mat @ calibrated_accelerometer_reading

        ctime = time.time()

        linear_velocity = None

        if self.__lastLocalLinearVelocityUpdate == 0:
            self.__lastLocalLinearVelocityUpdate = ctime
            self.__lastLocalLinearVelocity = np.zeros(3)
        else:
            linear_velocity_dt = ctime - self.__lastLocalLinearVelocityUpdate
            linear_velocity = self.__lastLocalLinearVelocity + transformed_calibrated_accelerometer_reading * linear_velocity_dt
            self.__lastLocalLinearVelocity = linear_velocity
            self.__lastLocalLinearVelocityUpdate = ctime

        angular_velocity = None
        # ehhhh - how to estimate angular velocity from global position reading?
        # TODO: Revisit this
        if self.__lastLocalAngularLocationUpdate == 0:
            self.__lastLocalAngularLocationUpdate = ctime
            self.__lastLocalAngularLocation = np.zeros(3)
        else:
            angular_velocity_dt = ctime - self.__lastLocalAngularLocationUpdate
            angular_velocity = (transformed_rot_ang - self.__lastLocalAngularLocation) / angular_velocity_dt
            self.__lastLocalAngularLocation = transformed_rot_ang
            self.__lastLocalAngularLocationUpdate = ctime

        # update all accelerations
        if self._lastLocalVelocityUpdate != 0:
            last_angular_velocity = self._lastLocalVelocity[3:]
            angular_accel = (angular_velocity - last_angular_velocity) / (ctime - self._lastLocalVelocityUpdate)
            self._call_local_acceleration_update(np.concatenate([transformed_calibrated_accelerometer_reading,angular_accel]).reshape((6,)))
        
        if linear_velocity is not None and angular_velocity is not None:
            self._call_local_velocity_update(np.concatenate([linear_velocity,angular_velocity]).reshape((6,)))



