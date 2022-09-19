from tracemalloc import start
from localizer_base import LocalCoordinateTransformedEstimatorImpl
from openvr_localizer import OpenVRGlobalFrameEstimator
from a1_imu_local_estimator import A1RobotIMULocalEstimator
import numpy as np
from robot_interface import RobotInterface # pytype: disable=import-error
import time
import pandas as pd

UPDATE_GAP_SECS = 0.1

_robot_interface = RobotInterface() # type: RobotInterface
_robot_interface.send_command(np.zeros(60, dtype=np.float32))
_openvr_tracker : OpenVRGlobalFrameEstimator = OpenVRGlobalFrameEstimator.getOpenVRTrackerFromViveTracker()
if _openvr_tracker is None:
    print("No tracker found")
    exit(1)

_a1_imu_estimator = A1RobotIMULocalEstimator(_robot_interface)

_openvr_local_estimator = LocalCoordinateTransformedEstimatorImpl( 
    _openvr_tracker,
    np.array([0,0,1]),
    np.array([1,0,0]),
    np.array([0,-1,0])
) #Transform OPENVR so that when laying flat the side of the LED is the "X" axis of local frame

_openvr_tracker.update()
_a1_imu_estimator.update()

start_time = time.time()
last_update_time = time.time()


dts = []
openvr_local_velocities = []
imu_local_velocities = []

openvr_local_accels = []
imu_local_accels = []
try:
    while True:
        _openvr_tracker.update()
        _a1_imu_estimator.update()

        ctime = time.time()

        if ctime - last_update_time < UPDATE_GAP_SECS:
            continue

        last_update_time = ctime

        dt = ctime - start_time

        openvr_local = _openvr_local_estimator.getLocalVelocity()
        imu_local = _a1_imu_estimator.getLocalVelocity()
        openvr_local_accel = _openvr_local_estimator.getLocalAcceleration()
        imu_local_accel = _a1_imu_estimator.getLocalAcceleration()

        openvr_local_velocities.append(openvr_local)
        imu_local_velocities.append(imu_local)
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