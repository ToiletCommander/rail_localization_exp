import time
from optical_flow.pmw_3901_board.pwm_3901 import PWM3901HardwareEstimator
from optical_flow.continuous_slidingwindow_filter import ContinuousMovingWindowFilter
from localizer_base import LocalCoordinateTransformedEstimatorImpl
import numpy as np

X_MULTIPLIER = 0.002667847599121442 #Tuned on 2022-11-21
Y_MULTIPLIER = 0.002667847599121442
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
SEPERATE_THREAD = True
FILTER = ContinuousMovingWindowFilter(
    window_size=0.1,
    recalculation_period=0.0, #recalculate mean every time
    shape=(2,)
)
DIST_FILTER = ContinuousMovingWindowFilter(
    window_size=0.2,
    recalculation_period=0.0,
    shape=()
)

pwm3901_estimator = PWM3901HardwareEstimator(
    x_multiplier=X_MULTIPLIER,
    y_multiplier=Y_MULTIPLIER,
    serial_port=SERIAL_PORT,
    serial_baudrate=BAUD_RATE,
    parallel=SEPERATE_THREAD,
    sliding_window_filter=FILTER,
    dist_cm_sliding_window=DIST_FILTER
)

transformed_est = LocalCoordinateTransformedEstimatorImpl(
    pwm3901_estimator,
    np.array([0, -1, 0],dtype=np.float),
    np.array([-1, 0, 0],dtype=np.float),
    np.array([0,0,1], dtype=np.float)
)

try:
    while True:
        if not(SEPERATE_THREAD):
            pwm3901_estimator.update()
        
        print(pwm3901_estimator.last_hardware_dt, transformed_est.getLocalVelocity())
        time.sleep(0.2)
except KeyboardInterrupt:
    pwm3901_estimator.stop()