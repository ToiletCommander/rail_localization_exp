import typing
import numpy as np
from localizer_base import LocalFrameEstimatorImpl, NonBlocking
from .robot import RobotInterface, from_a1_frame, getImuRollPitchYaw, getFootContacts, getMotorVelocities, getMotorAngles, getLegJacobian, getAndInitRobotInterface as initRobot

class A1FootContactLocalVelocityEstimator(LocalFrameEstimatorImpl, NonBlocking):
    @classmethod
    def getAndInitRobotInterface(cls) -> RobotInterface:
        return initRobot()

    def __init__(self, robot_intf : typing.Optional[RobotInterface]):
        super().__init__(
            __class__.__name__, 
            autoUpdateVelocity = False, 
            autoUpdateAcceleration = True
        )

        self.robot = robot_intf
    
    def update(self):
        if self.robot is not None:
            self.updateFromObservation(self.robot.receive_observation())

    def updateFromObservation(self, observation):
        foot_velocities = []
        foot_contact = getFootContacts(observation)
        motor_velocities = getMotorVelocities(observation)
        
        for leg_id in range(4):
            if foot_contact[leg_id]:
                jacobian = getLegJacobian(leg_id, motor_velocities)
                # Only pick the jacobian related to joint motors
                joint_velocities = motor_velocities[leg_id *3:(leg_id +1) * 3]
                leg_velocity_in_base_frame = jacobian.dot(joint_velocities)
                base_velocity_in_base_frame = -leg_velocity_in_base_frame[:3]
                foot_velocities.append(base_velocity_in_base_frame)
            else:
                foot_velocities.append(np.zeros(3))

        foot_velocities = np.vstack(foot_velocities)

        if len(foot_velocities) > 0:
            observed_v = np.mean(foot_velocities, axis=0)
            transformed_v = from_a1_frame(observed_v)

            self._call_local_velocity_update(
                np.concatenate([transformed_v, np.zeros((3,))])
            )

