import time
import typing
import numpy as np
from localizer_base import LocalFrameEstimatorImpl, NonBlocking
from .robot import RobotInterface, from_a1_frame, getImuRollPitchYaw, getFootContacts, getMotorVelocities, getMotorAngles, getLegJacobian, getAndInitRobotInterface as initRobot
from optical_flow.continuous_slidingwindow_filter import ContinuousMovingWindowFilter

class A1FootContactLocalVelocityEstimator(LocalFrameEstimatorImpl, NonBlocking):
    @classmethod
    def getAndInitRobotInterface(cls) -> RobotInterface:
        return initRobot()

    def __init__(
        self, 
        sliding_window_filter : typing.Optional[ContinuousMovingWindowFilter], 
        robot_intf : typing.Optional[RobotInterface]
    ):
        LocalFrameEstimatorImpl.__init__(
            self, 
            name = "A1FootContactLocalVelocityEstimator",
            autoUpdateVelocity = False, 
            autoUpdateAcceleration = False
        )
        NonBlocking.__init__(self)

        self.robot = robot_intf
        self.reset()
        self.filter = sliding_window_filter
        self._lastFootContact = np.array([False, False, False, False])
        self._lastMotorVelocities = np.zeros((12,))
        self._lastMotorAngles = np.zeros((12,))
        self._lastVUpdate = time.time()
        self.update()
    
    def reset(self):
        self._lastFootContact = np.array([False, False, False, False])
        self._lastMotorVelocities = np.zeros((12,))
        self._lastMotorAngles = np.zeros((12,))
        super().reset()
        
    def update(self):
        if self.robot is not None:
            self.updateFromObservation(self.robot.receive_observation())

    def updateFromObservation(self, observation):
        foot_velocities = []
        foot_contact = getFootContacts(observation)
        motor_velocities = getMotorVelocities(observation)
        motor_angles = getMotorAngles(observation)


        if foot_contact is None:
            print("Foot contact is none?")
            foot_contact = self._lastFootContact
        if motor_velocities is None:
            print("Motor velocity is none?")
            motor_velocities = self._lastMotorVelocities
        if motor_angles is None:
            print("Motor angle is none?")
            motor_angles = self._lastMotorAngles
        
        for leg_id in range(4):
            if foot_contact[leg_id]:
                jacobian = getLegJacobian(leg_id, motor_angles)
                # Only pick the jacobian related to joint motors
                joint_velocities = motor_velocities[leg_id *3:(leg_id +1) * 3]
                leg_velocity_in_base_frame = jacobian.dot(joint_velocities)
                base_velocity_in_base_frame = from_a1_frame(leg_velocity_in_base_frame[:3]) #-leg_velocity_in_base_frame[:3]
                foot_velocities.append(base_velocity_in_base_frame)
            else:
                foot_velocities.append(np.zeros(3))

        foot_velocities = np.vstack(foot_velocities)

        if len(foot_velocities) > 0:
            observed_v = np.mean(foot_velocities, axis=0)
            if self.filter is not None:
                ctime = time.time()
                dt = ctime - self._lastVUpdate
                self.filter.add_observation(dt,observed_v)
                transformed_v = self.filter.get_per_size_average()
                self._lastVUpdate = ctime
            else:
                transformed_v = observed_v

            self._call_local_velocity_update(
                np.concatenate([transformed_v, np.zeros((3,))])
            )

        self._lastMotorAngles = motor_angles
        self._lastMotorVelocities = motor_velocities
        self._lastFootContact = foot_contact