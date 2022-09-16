"""
Reference https://github.com/ikostrikov/walk_in_the_park/blob/main/real/robots/a1_robot.py
"""

import inspect
import math
import multiprocessing
import os
import re
import time
import pybullet

import numpy as np
from robot_interface import RobotInterface  # pytype: disable=import-error

currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

if __name__ != '__main__':
    print("This script is not meant to be imported")
    exit(1)



# Initiate UDP for robot state and actions
_robot_interface = RobotInterface()
_robot_interface.send_command(np.zeros(60, dtype=np.float32))
pybullet_client = pybullet.connect(pybullet.GUI)

while(True):
    state = _robot_interface.receive_observation()
    _raw_state = state

    print('raw_state', _raw_state)

    # Convert quaternion from wxyz to xyzw, which is default for Pybullet.
    q = state.imu.quaternion
    print('imu',state.imu, 'quaternion', q)

    _base_orientation = np.array([q[1], q[2], q[3], q[0]])

    _accelerometer_reading = np.array(state.imu.accelerometer)
    print('accel_raw',_accelerometer_reading)

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
    
    rot_mat = pybullet_client.getMatrixFromQuaternion(
            _base_orientation)
    
    rot_mat = np.array(rot_mat).reshape((3, 3))
    calibrated_acc = _accelerometer_reading + np.linalg.inv(rot_mat).dot(
            np.array([0., 0., -9.8]))
