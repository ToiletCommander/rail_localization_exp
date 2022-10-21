import time
from optical_flow.pmw_3901_board.pwm_3901 import PWM3901HardwareEstimator, constant_distance
from optical_flow.continuous_slidingwindow_filter import ContinuousMovingWindowFilter

X_MULTIPLIER = 0.002
Y_MULTIPLIER = 0.002
SERIAL_PORT = "COM5"
BAUD_RATE = 115200
SEPERATE_THREAD = True
DIST_EST = constant_distance(0.2)
FILTER = ContinuousMovingWindowFilter(
    window_size=0.1,
    recalculation_period=0.0, #recalculate mean every time
    shape=(2,)
)

estimator = PWM3901HardwareEstimator(
    dist_estimator=DIST_EST,
    x_multiplier=X_MULTIPLIER,
    y_multiplier=Y_MULTIPLIER,
    serial_port=SERIAL_PORT,
    serial_baudrate=BAUD_RATE,
    parallel=SEPERATE_THREAD,
    sliding_window_filter=FILTER
)

try:
    while True:
        if not(SEPERATE_THREAD):
            estimator.update()
        
        print(estimator.last_hardware_dt, estimator.getLocalVelocity())
        time.sleep(0.2)
except KeyboardInterrupt:
    estimator.stop()