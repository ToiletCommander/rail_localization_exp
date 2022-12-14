"""
Optical Flow Coordinate Definition
Let camera facing down, the top pixel captured by the camera pointing in front of the robot,
be the pose
xRot: 0,
yRot: 0,
zRot: 0
"""
import time
import typing
from localizer_base import LocalFrameEstimator, LocalFrameEstimatorImpl, NonBlocking, rotation_matrix, rotation_matrix_inverse
import cv2 as cv
import numpy as np
from .slidingwindow_filter import MovingWindowFilter
import threading

def from_camera_coordinate(camera_coord: np.ndarray) -> np.ndarray:
    assert camera_coord.shape == (2,)
    return np.array([-camera_coord[1], -camera_coord[0],0])

class OpticalFlowVelocityEstimator(LocalFrameEstimatorImpl, NonBlocking):
    def __init__(
        self, 
        camera_rotation = np.array([0,0,0],dtype=np.float32),
        x_y_multiplier : typing.Union[float,np.ndarray] = 1.0,
        moving_window_size : int = 3,
        vid_cap : typing.Optional[cv.VideoCapture] = None,
        init_seperate_thread : bool = False,
        preview_window : bool = True
    ):
        super().__init__(
            "OpticalFlowVelocityEstimator", 
            autoUpdateVelocity=False, 
            autoUpdateAcceleration=True
        )

        self.x_y_multiplier = x_y_multiplier
        self.video = vid_cap
        self.preview_window = preview_window
        self.prevGray : typing.Optional[np.ndarray] = None
        self.prevGray_time : float = 0
        self.setCameraRotation(camera_rotation)
        self.slidingWindow = MovingWindowFilter(moving_window_size, shape=(3,), weights=np.array([0.3,0.3,0.4]))
        self.update()
        if init_seperate_thread and self.video is not None:
            t = threading.Thread(target=self.seperateThreadUpdate)
            t.daemon = True
            t.start()
    
    def seperateThreadUpdate(self):
        while True:
            self.update()
            time.sleep(0.05)

    def getCamerRotation(self) -> np.ndarray:
        return self.__camera_rot
        
    def setCameraRotation(self, rot: np.ndarray) -> None:
        assert rot.shape == (3,)
        self.__camera_rot = rot
        self.__camera_rot_mat = rotation_matrix(*rot)
    
    def reset(self) -> None:
        self.slidingWindow.reset()
        self.prevGray = None
        self.prevGray_time = 0


    def update(self):
        if self.video is not None:
            ret, frame = self.video.read()
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            flow = self.updateWithFrame(gray)

            if self.preview_window and flow is not None:
                magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1])
                preview_mask = np.zeros_like(frame)
                # Sets mask saturation to maximum
                preview_mask[..., 1] = 255
                preview_mask[..., 0] = angle * 180 / np.pi / 2
      
                # Sets image value according to the optical flow
                # magnitude (normalized)
                preview_mask[..., 2] = cv.normalize(magnitude, None, 0, 255, cv.NORM_MINMAX)
                preview_mask = cv.cvtColor(preview_mask, cv.COLOR_HSV2BGR)

                cv.imshow("Optical Flow Estimator Gray",gray)
                cv.imshow("Dense Optical Flow", preview_mask)

                if cv.waitKey(1) & 0xFF == ord('q'):
                    self.preview_window = False
                    cv.destroyAllWindows()
            
    
    def updateWithFrame(self, grayFrame: np.ndarray):
        if self.prevGray is None:
            self.prevGray = grayFrame
            self.prevGray_time = time.time()
            return

        ctime = time.time()
        dt = ctime - self.prevGray_time

        flow = cv.calcOpticalFlowFarneback(self.prevGray, grayFrame, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        avgFlow = np.mean(flow,axis=(0,1))
        cameraDeltaPos = -avgFlow

        robotLocalDeltaPos = self.x_y_multiplier * (self.__camera_rot_mat @ from_camera_coordinate(cameraDeltaPos))

        self.prevGray = grayFrame
        self.prevGray_time = ctime

        robotLocalDeltaSpeed = robotLocalDeltaPos / dt

        self.slidingWindow.add_observation(robotLocalDeltaSpeed)

        estimated_velocity = self.slidingWindow.get_average()

        self._call_local_velocity_update(np.concatenate([estimated_velocity.reshape((3,)), np.zeros((3,))]))
        return flow