import time
import cv2 as cv
from optical_flow.optical_flow_estimator import OpticalFlowVelocityEstimator
import numpy as np

_videoCap = cv.VideoCapture(1)
#_videoCap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
#_videoCap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

_cameraTransform = np.array([0,0,np.pi/2.0],dtype= np.float32)
_estimator = OpticalFlowVelocityEstimator(_cameraTransform)
while True:
    ret, frame = _videoCap.read()
    
    grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    _estimator.updateWithFrame(grayFrame)
    print(_estimator.getLocalVelocity())
    cv.imshow("input", frame)
    cv.waitKey(100)