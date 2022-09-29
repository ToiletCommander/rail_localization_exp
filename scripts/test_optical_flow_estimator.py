import cv2 as cv
from optical_flow.optical_flow_estimator import OpticalFlowVelocityEstimator
import numpy as np

_videoCap = cv.VideoCapture(0)
_videoCap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
_videoCap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

_cameraTransform = np.array([0,0,0],dtype= np.float32)
_estimator = OpticalFlowVelocityEstimator(_cameraTransform)
while True:
    ret, frame = _videoCap.read()
    cv.imshow("input", frame)
    _estimator.updateWithFrame(frame)
    print(_estimator.getLocalVelocity())
