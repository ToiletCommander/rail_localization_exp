import pyrealsense2 as rs
from localizer_base import LocalFrameEstimatorImpl, NonBlocking, rotation_angle_from_quaternion, rotation_matrix_inverse
import numpy as np
import time

# https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
class IntelT265Estimator(NonBlocking, LocalFrameEstimatorImpl):
    def __init__(self):
        LocalFrameEstimatorImpl.__init__(self, "IntelT265Estimator", False, False)
        NonBlocking.__init__(self)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.pose)
        self.pipeline.start(self.config)
        
        self.last_pose = None
        self.last_pose_t = 0
    
    def update(self):
        frames = self.pipeline.wait_for_frames()

        # Fetch pose frame
        
        pose = frames.get_pose_frame()
        if pose:
            # Print some of the pose data to the terminal
            data = pose.get_pose_data()
            new_pose = np.array([data.translation.x, data.translation.y, data.translation.z])
            if self.last_pose is not None:
                q = [data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z]
                roll, pitch, yaw = rotation_angle_from_quaternion(q)
                inv_mat = rotation_matrix_inverse(roll, pitch, yaw)
                last_global_velocity_raw = new_pose - self.last_pose
                last_local_velocity_raw = inv_mat @ last_global_velocity_raw
                
                last_local_velocity = np.array([
                    -last_local_velocity_raw[2],
                    -last_local_velocity_raw[0],
                    last_local_velocity_raw[1],
                    0,
                    0,
                    0
                ])

                self._call_local_velocity_update(last_local_velocity)
            self.last_pose = new_pose
            self.last_pose_t = time.time()
               
            #print("Frame #{}".format(pose.frame_number))
            #print("Position: {}".format(data.translation))

            #print("Velocity: {}".format(data.velocity))
            #print("Acceleration: {}\n".format(data.acceleration))
            #print("data", data)
    
    def stop(self):
        self.pipeline.stop()
    
    def reset(self):
        self.last_pose = None
        self.last_pose_t = 0
        LocalFrameEstimatorImpl.reset(self)