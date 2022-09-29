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
from localizer_base import LocalFrameEstimator, LocalFrameEstimatorImpl, NonBlocking, rotation_matrix_inverse
import cv2 as cv
import numpy as np

def from_camera_coordinate(camera_coord: np.ndarray) -> np.ndarray:
    assert camera_coord.shape == (2,)
    return np.array([-camera_coord[1], -camera_coord[0],0])

class OpticalFlowVelocityEstimator(LocalFrameEstimatorImpl, NonBlocking):
    def __init__(
        self, 
        camera_rotation = np.array([0,0,0],dtype=np.float32),
        VideoCapture : typing.Optional[cv.VideoCapture] = None
    ):
        super().__init__(
            "OpticalFlowVelocityEstimator", 
            autoUpdateVelocity=False, 
            autoUpdateAcceleration=True
        )
        self.video = VideoCapture
        self.prevGray : typing.Optional[np.ndarray] = None
        self.prevGray_time : float = 0
        self.setCameraRotation(camera_rotation)
        self.update()

    def getCamerRotation(self) -> np.ndarray:
        return self.__camera_rot
        
    def setCameraRotation(self, rot: np.ndarray) -> None:
        assert rot.shape == (3,)
        self.__camera_rot = rot
        self.__camera_rot_inv = rotation_matrix_inverse(*rot)
    
    def update(self):
        if self.video is not None:
            ret, frame = self.video.read()
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            self.updateWithFrame(gray)
    
    def updateWithFrame(self, grayFrame: np.ndarray):
        if self.prevGray is None:
            self.prevGray = grayFrame
            self.prevGray_time = time.time()
            return

        flow = cv.calcOpticalFlowFarneback(self.prevGray, grayFrame, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        avgFlow = np.mean(flow,axis=(0,1))
        cameraDeltaPos = -avgFlow

        robotLocalDeltaPos = self.__camera_rot_inv @ from_camera_coordinate(cameraDeltaPos)

        self.prevGray = grayFrame
        self.prevGray_time = time.time()

        self._call_local_velocity_update(np.concatenate([robotLocalDeltaPos.reshape((3,)), np.zeros((3,))]))


    
