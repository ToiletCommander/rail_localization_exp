import numpy as np
import time
import pandas as pd
import os
import inspect
from a1.foot_contact_estimator import A1FootContactLocalVelocityEstimator
import cv2 as cv

currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
parent_parentdir = os.path.dirname(parentdir)
os.sys.path.insert(0, currentdir)
os.sys.path.insert(0, parentdir)
os.sys.path.insert(0, parent_parentdir)

from a1.robot import RobotInterface
from localizer_base import LocalCoordinateTransformedEstimatorImpl
from openvr_localizer import OpenVRGlobalFrameEstimator
from a1.imu_local_estimator import A1RobotIMULocalEstimator
from optical_flow.optical_flow_estimator import OpticalFlowVelocityEstimator

UPDATE_GAP_SECS = 0.1


_robot_interface : RobotInterface = A1RobotIMULocalEstimator.getAndInitRobotInterface()
_openvr_tracker : OpenVRGlobalFrameEstimator = OpenVRGlobalFrameEstimator.getOpenVRTrackerFromViveTracker()
if _openvr_tracker is None:
    print("No tracker found")
    exit(1)

_a1_imu_estimator = A1RobotIMULocalEstimator(_robot_interface)
_a1_leg_estimator = A1FootContactLocalVelocityEstimator(_robot_interface)
_optical_flow_estimator = OpticalFlowVelocityEstimator( #Attach Camera to the left side of the robot
    np.array([0,0,np.pi/2],dtype=np.float32),
    cv.VideoCapture(0)
)

_openvr_local_estimator = LocalCoordinateTransformedEstimatorImpl( 
    _openvr_tracker,
    np.array([0,0,1]),
    np.array([1,0,0]),
    np.array([0,-1,0])
) #Transform OPENVR so that when laying flat the side of the LED is the "X" axis of local frame

_openvr_tracker.update()
a1_obs = _robot_interface.receive_observation()
_a1_imu_estimator.updateFromObservation(a1_obs)
_a1_leg_estimator.updateFromObservation(a1_obs)

start_time = time.time()
last_update_time = time.time()


dts = []
openvr_local_velocities = []
imu_local_velocities = []
foot_local_velocities = []
optical_flow_velocities = []
openvr_local_accels = []
imu_local_accels = []
try:
    while True:
        _openvr_tracker.update()
        a1_obs = _robot_interface.receive_observation()
        _a1_imu_estimator.updateFromObservation(a1_obs)
        _a1_leg_estimator.updateFromObservation(a1_obs)
        _optical_flow_estimator.update()

        ctime = time.time()

        if ctime - last_update_time < UPDATE_GAP_SECS:
            continue

        last_update_time = ctime

        dt = ctime - start_time

        openvr_local = _openvr_local_estimator.getLocalVelocity()
        imu_local = _a1_imu_estimator.getLocalVelocity()
        foot_local = _a1_leg_estimator.getLocalVelocity()
        optical_flow_local = _optical_flow_estimator.getLocalVelocity()
        openvr_local_accel = _openvr_local_estimator.getLocalAcceleration()
        imu_local_accel = _a1_imu_estimator.getLocalAcceleration()

        openvr_local_velocities.append(openvr_local)
        imu_local_velocities.append(imu_local)
        foot_local_velocities.append(foot_local)
        optical_flow_velocities.append(optical_flow_local)
        openvr_local_accels.append(openvr_local_accel)
        imu_local_accels.append(imu_local_accel)


        dts.append(dt)
        print("OpenVR Local Velocity",openvr_local)
        print("IMU Local Velocity",imu_local)
        print("dt",dt)
except KeyboardInterrupt:
    pass


min_length = min(len(dts),len(openvr_local_velocities),len(imu_local_velocities))
dts = dts[:min_length]
openvr_local_velocities = openvr_local_velocities[:min_length]
imu_local_velocities = imu_local_velocities[:min_length]

df = pd.DataFrame({
    'dt':dts,
    'openvr_local_velocity_x': [x[0] for x in openvr_local_velocities],
    'openvr_local_velocity_y': [x[1] for x in openvr_local_velocities],
    'openvr_local_velocity_z': [x[2] for x in openvr_local_velocities],
    'openvr_local_velocity_degx': [x[3] for x in openvr_local_velocities],
    'openvr_local_velocity_degy': [x[4] for x in openvr_local_velocities],
    'openvr_local_velocity_degz': [x[5] for x in openvr_local_velocities],
    'imu_local_velocity_x': [x[0] for x in imu_local_velocities],
    'imu_local_velocity_y': [x[1] for x in imu_local_velocities],
    'imu_local_velocity_z': [x[2] for x in imu_local_velocities],
    'imu_local_velocity_degx': [x[3] for x in imu_local_velocities],
    'imu_local_velocity_degy': [x[4] for x in imu_local_velocities],
    'imu_local_velocity_degz': [x[5] for x in imu_local_velocities],
    'foot_local_velocity_x': [x[0] for x in foot_local_velocities],
    'foot_local_velocity_y': [x[1] for x in foot_local_velocities],
    'foot_local_velocity_z': [x[2] for x in foot_local_velocities],
    'optical_flow_local_velocity_x': [x[0] for x in optical_flow_velocities],
    'optical_flow_local_velocity_y': [x[1] for x in optical_flow_velocities],
    'optical_flow_local_velocity_z': [x[2] for x in optical_flow_velocities],
    'openvr_local_accel_x': [x[0] for x in openvr_local_accels],
    'openvr_local_accel_y': [x[1] for x in openvr_local_accels],
    'openvr_local_accel_z': [x[2] for x in openvr_local_accels],
    'openvr_local_accel_degx': [x[3] for x in openvr_local_accels],
    'openvr_local_accel_degy': [x[4] for x in openvr_local_accels],
    'openvr_local_accel_degz': [x[5] for x in openvr_local_accels],
    'imu_local_accel_x': [x[0] for x in imu_local_accels],
    'imu_local_accel_y': [x[1] for x in imu_local_accels],
    'imu_local_accel_z': [x[2] for x in imu_local_accels],
    'imu_local_accel_degx': [x[3] for x in imu_local_accels],
    'imu_local_accel_degy': [x[4] for x in imu_local_accels],
    'imu_local_accel_degz': [x[5] for x in imu_local_accels],
})

df.to_csv('data.csv')