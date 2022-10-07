import time
import cv2 as cv
from optical_flow.optical_flow_estimator import OpticalFlowVelocityEstimator
import numpy as np

_videoCap = cv.VideoCapture(0)
#_videoCap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
#_videoCap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

_cameraTransform = np.array([0,0,np.pi/2.0],dtype= np.float32)
_estimator = OpticalFlowVelocityEstimator(_cameraTransform,vid_cap=_videoCap,init_seperate_thread=True,preview_window=True)
while True:
    print(_estimator.getLocalVelocity())