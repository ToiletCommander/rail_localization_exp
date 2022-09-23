import typing
from .robot import RobotInterface, getImuQuaternion, getImuRollPitchYaw, getAccelerometerReading, from_a1_frame, to_a1_frame, getAndInitRobotInterface as initRobot
import numpy as np
from localizer_base import GlobalFrameEstimatorImpl, NonBlocking, rotation_matrix_and_inverse_rotation_matrix, rotation_angle_from_quaternion, rotation_matrix, rotation_matrix_inverse
import time

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

class A1RobotIMULocalEstimator(GlobalFrameEstimatorImpl, NonBlocking):
    @classmethod
    def getAndInitRobotInterface(cls) -> RobotInterface:
        return initRobot()

    def __init__(self, robot_intf : typing.Optional[RobotInterface], calibrate_gravity : bool = True) -> None:
        GlobalFrameEstimatorImpl.__init__(
            self,
            "A1RobotIMUEstimator",
            autoUpdateLocation=False,
            autoUpdateVelocity=False,
            autoUpdateAcceleration=False,
            autoUpdateLocalVelocity=False,
            autoUpdateLocalAcceleration=False
        )
        self.robot = robot_intf

        self.reset()

        self.negative_gravity_vector = np.array([0,0,-9.8])

        if calibrate_gravity:
            self.calibrate_for_gravity()
        
        self.update()

    def reset(self) -> None:
        super().reset()
        ctime = time.time()

        self._lastGlobalAngularLocation : np.ndarray = np.zeros((3,),dtype=np.float32)
        self._lastGlobalAngularLocationUpdate : float = 0

        self._lastGlobalAngularVelocity : np.ndarray = np.zeros((3,),dtype=np.float32)
        self._lastGlobalAngularVelocityUpdate : float = ctime

        self._lastGlobalAngularAcceleration : np.ndarray = np.zeros((3,),dtype=np.float32)
        self._lastGlobalAngularAccelerationUpdate : float = ctime

        self._lastGlobalLinearAcceleration : np.ndarray = np.zeros((3,),dtype=np.float32)
        self._lastGlobalLinearAccelerationUpdate : float = ctime

        self._lastGlobalLinearVelocity : np.ndarray = np.zeros((3,),dtype=np.float32)
        self._lastGlobalLinearVelocityUpdate : float = ctime

        self._lastGlobalLinearLocation : np.ndarray = np.zeros((3,),dtype=np.float32)
        self._lastGlobalLinearLocationUpdate : float = ctime
    
    def getLinearVelocityWithNewLinearAcceleration(self, new_linear_acceleration : np.ndarray, ctime : float) -> np.ndarray:
        dt = ctime - self._lastGlobalLinearAccelerationUpdate
        ret = self._lastGlobalLinearVelocity + (new_linear_acceleration + self._lastGlobalLinearAcceleration) / 2.0 * dt
        return ret
    
    def getLinearLocationWithNewLinearVelocity(self, new_linear_velocity : np.ndarray, ctime : float) -> np.ndarray:
        dt = ctime - self._lastGlobalLinearVelocityUpdate
        ret = self._lastGlobalLinearLocation + (new_linear_velocity + self._lastGlobalLinearVelocity) / 2.0 * dt
        return ret
    
    def getAngularVelocityWithNewAngularLocation(self, new_angular_location : np.ndarray, ctime : float) -> typing.Optional[np.ndarray]:
        dt = ctime - self._lastGlobalAngularLocationUpdate

        ret : typing.Optional[np.ndarray] = None

        if self._lastGlobalAngularLocationUpdate != 0:
            ret = (new_angular_location - self._lastGlobalAngularLocation) / dt
        
        return ret
    
    def getAngularAccelerationWithNewAngularVelocity(self, new_angular_velocity : np.ndarray, ctime : float) -> typing.Optional[np.ndarray]:
        dt = ctime - self._lastGlobalAngularVelocityUpdate

        ret = None

        if new_angular_velocity is not None and self._lastGlobalAngularVelocityUpdate != 0:
            ret = (new_angular_velocity - self._lastGlobalAngularVelocity) / dt
        
        return ret
    
    def getAngularLocation(self) -> typing.Optional[np.ndarray]:
        return self._lastGlobalAngularLocation if self._lastGlobalAngularLocationUpdate != 0 else None
        
    def calibrate_for_gravity(self, duration_to_take_mean : float = 2.0) -> None:
        if self.robot is None:
            return
        
        start_time = time.time()
        sum_of_gravity : np.ndarray = np.zeros((3,), dtype=np.float32)
        last_time = start_time
        total_dt : float = 0.0

        while True:
            ctime = time.time()
            dt = ctime - last_time
            if ctime - start_time >= duration_to_take_mean:
                break

            observation = self.robot.receive_observation()

            q = getImuQuaternion(observation)
            accelerometer_reading = getAccelerometerReading(observation)
            
            if np.all(accelerometer_reading == 0):
                raise Exception("Accelerometer reading is 0,0,0. Is the IMU connected?")
                return

            rot_ang = rotation_angle_from_quaternion(q)
            #rot_ang = np.array(getImuRollPitchYaw(observation))
            
            rot_mat = rotation_matrix(*rot_ang)
            accelerometer_reading_global = rot_mat @ accelerometer_reading
            sum_of_gravity += accelerometer_reading_global * dt
            total_dt += dt

            time.sleep(0.05)

        gravity = sum_of_gravity / total_dt
        print("Gravity is", gravity)
        self.negative_gravity_vector = -gravity

    def _renewEveryVariable(
        self,
        global_angular_location : np.ndarray,
        global_angular_velocity : typing.Optional[np.ndarray],
        global_angular_acceleration : typing.Optional[np.ndarray],
        global_linear_location : typing.Optional[np.ndarray],
        global_linear_velocity : typing.Optional[np.ndarray],
        global_linear_acceleration : np.ndarray,
        ctime : float
    ):
        self._lastGlobalAngularLocation = global_angular_location
        self._lastGlobalAngularLocationUpdate = ctime
        
        if global_angular_velocity is not None:
            self._lastGlobalAngularVelocity = global_angular_velocity
            self._lastGlobalAngularVelocityUpdate = ctime
        
        if global_angular_acceleration is not None:
            self._lastGlobalAngularAcceleration = global_angular_acceleration
            self._lastGlobalAngularAccelerationUpdate = ctime
        
        if global_linear_location is not None:
            self._lastGlobalLinearLocation = global_linear_location
            self._lastGlobalLinearLocationUpdate = ctime
        
        if global_linear_velocity is not None:
            self._lastGlobalLinearVelocity = global_linear_velocity
            self._lastGlobalLinearVelocityUpdate = ctime
        
        self._lastGlobalLinearAcceleration = global_linear_acceleration
        self._lastGlobalLinearAccelerationUpdate = ctime


    def update(self) -> None:
        if self.robot is not None:
            self.updateFromObservation(self.robot.receive_observation())

    def updateFromObservation(self, observation : typing.Any) -> None:
        #wxyz quaternion
        #q = getImuQuaternion(observation)

        #xyz acceleration in original coordinates
        raw_accelerometer_reading = getAccelerometerReading(observation)

        if np.all(raw_accelerometer_reading == 0):
            calibrated_accelerometer_raw_reading = np.zeros((3,))
            raw_rot_ang = np.zeros((3,))
            print("Accelerometer reading is 0,0,0. Is the IMU connected?")
            return

        #raw_rot_ang = np.array(rotation_angle_from_quaternion(q))
        raw_rot_ang = np.array(getImuRollPitchYaw(observation))
        
        raw_rot_mat, raw_inv_rot_mat = rotation_matrix_and_inverse_rotation_matrix(*raw_rot_ang)

        calibrated_accelerometer_raw_reading = raw_accelerometer_reading + raw_inv_rot_mat @ self.negative_gravity_vector

        global_angular_location =  from_a1_frame(raw_rot_ang)
        #local_linear_acceleration = from_a1_frame(calibrated_accelerometer_raw_reading)
        global_linear_acceleration = from_a1_frame(raw_rot_mat @ calibrated_accelerometer_raw_reading)

        ctime = time.time()

        global_angular_velocity = self.getAngularVelocityWithNewAngularLocation(global_angular_location, ctime)
        global_angular_acceleration = self.getAngularAccelerationWithNewAngularVelocity(global_angular_velocity, ctime)
        global_linear_velocity = self.getLinearVelocityWithNewLinearAcceleration(global_linear_acceleration, ctime)
        global_linear_location = self.getLinearLocationWithNewLinearVelocity(global_linear_velocity, ctime)

        self._renewEveryVariable(
            global_angular_location,
            global_angular_velocity,
            global_angular_acceleration,
            global_linear_location,
            global_linear_velocity,
            global_linear_acceleration,
            ctime
        )
        
        if global_linear_location is not None:
            self._call_location_update(np.concatenate((global_linear_location, global_angular_location)))

        if global_linear_velocity is not None and global_angular_velocity is not None:
            global_velocity = np.concatenate((global_linear_velocity, global_angular_velocity))
            self._call_velocity_update(global_velocity)

            a1_frame_global_v = to_a1_frame(global_velocity)
            a1_frame_global_v = np.hstack([a1_frame_global_v[:3].reshape((3,1)),a1_frame_global_v[3:].reshape((3,1))])

            local_velocity = from_a1_frame(raw_inv_rot_mat @ a1_frame_global_v)
            local_velocity = np.concatenate([local_velocity[:,0],local_velocity[:,1]])

            self._call_local_velocity_update(local_velocity)
        
        if global_linear_acceleration is not None and global_angular_acceleration is not None:
            global_acceleration = np.concatenate((global_linear_acceleration, global_angular_acceleration))
            self._call_acceleration_update(global_acceleration)
            local_linear_acceleration = from_a1_frame(calibrated_accelerometer_raw_reading)
            local_acceleration = np.concatenate((
                from_a1_frame(raw_inv_rot_mat @ to_a1_frame(global_angular_acceleration)),
                local_linear_acceleration
            ))
            self._call_local_acceleration_update(local_acceleration)

            