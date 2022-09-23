import time
import typing
import numpy as np
import inspect
import os
import sys
import time


currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, currentdir)
os.sys.path.insert(0, parentdir)

from localizer_base import GlobalFrameEstimatorImpl, LocalFrameEstimatorImpl, NonBlocking, rotation_angle_from_quaternion, rotation_matrix, rotation_matrix_and_inverse_rotation_matrix, rotation_matrix_inverse
from robot_interface import RobotInterface # pytype: disable=import-error


def getFootForces(raw_state) -> np.ndarray:
    return np.array(raw_state.footForce)

def getFootContacts(raw_state, thereshold : float = 20) -> np.ndarray:
    return np.array(raw_state.footForce) >= thereshold

def getMotorVelocities(raw_state) -> np.ndarray:
    return np.array([motor.dq for motor in raw_state.motorState[:12]])

def getMotorAngles(raw_state) -> np.ndarray:
    return np.array([motor.q for motor in raw_state.motorState[:12]])

def getJointStates(raw_state) -> np.ndarray:
    return list(zip(getMotorAngles(raw_state), getMotorVelocities(raw_state)))

def getMotorTemperatures(raw_state) -> np.ndarray:
    return np.array([motor.temperature for motor in raw_state.motorState[:12]])

def getMotorTorques(raw_state) -> np.ndarray:
    return np.array([motor.tauEst for motor in raw_state.motorState[:12]])

# Returns WXYZ quaternion
def getImuQuaternion(raw_state) -> typing.Tuple[float,float,float,float]:
    return np.array(raw_state.imu.quaternion)

def getImuRollPitchYaw(raw_state) -> typing.Tuple[float,float,float]:
    return raw_state.imu.gyroscope

def getAccelerometerReading(raw_state) -> np.ndarray:
    return np.array(raw_state.imu.accelerometer)

def analytical_leg_jacobian(leg_angles, leg_id):
    """
  Computes the analytical Jacobian.
  Args:
  ` leg_angles: a list of 3 numbers for current abduction, hip and knee angle.
    l_hip_sign: whether it's a left (1) or right(-1) leg.
  """
    l_up = 0.2
    l_low = 0.2
    l_hip = 0.08505 * (-1)**(leg_id + 1)

    t1, t2, t3 = leg_angles[0], leg_angles[1], leg_angles[2]
    l_eff = np.sqrt(l_up**2 + l_low**2 + 2 * l_up * l_low * np.cos(t3))
    t_eff = t2 + t3 / 2
    J = np.zeros((3, 3))
    J[0, 0] = 0
    J[0, 1] = -l_eff * np.cos(t_eff)
    J[0, 2] = l_low * l_up * np.sin(t3) * np.sin(
        t_eff) / l_eff - l_eff * np.cos(t_eff) / 2
    J[1, 0] = -l_hip * np.sin(t1) + l_eff * np.cos(t1) * np.cos(t_eff)
    J[1, 1] = -l_eff * np.sin(t1) * np.sin(t_eff)
    J[1, 2] = -l_low * l_up * np.sin(t1) * np.sin(t3) * np.cos(
        t_eff) / l_eff - l_eff * np.sin(t1) * np.sin(t_eff) / 2
    J[2, 0] = l_hip * np.cos(t1) + l_eff * np.sin(t1) * np.cos(t_eff)
    J[2, 1] = l_eff * np.sin(t_eff) * np.cos(t1)
    J[2, 2] = l_low * l_up * np.sin(t3) * np.cos(t1) * np.cos(
        t_eff) / l_eff + l_eff * np.sin(t_eff) * np.cos(t1) / 2
    return J


def getLegJacobian(leg_id : int, motor_angles : np.ndarray):
    return analytical_leg_jacobian(motor_angles[3 * leg_id:3 * leg_id + 3], leg_id)

def from_a1_frame(a1_frame : np.ndarray) -> np.ndarray:
    return -a1_frame

to_a1_frame = from_a1_frame


def getAndInitRobotInterface() -> RobotInterface:
    _robot_interface = RobotInterface() # type: RobotInterface
    _robot_interface.send_command(np.zeros(60, dtype=np.float32))
    return _robot_interface