import time
from optical_flow.pmw_3901_board.pwm_3901 import PWM3901HardwareEstimator
from optical_flow.continuous_slidingwindow_filter import ContinuousMovingWindowFilter

X_MULTIPLIER = 0.01
Y_MULTIPLIER = 0.01
SERIAL_PORT = "COM5"
BAUD_RATE = 115200
SEPERATE_THREAD = True
FILTER = ContinuousMovingWindowFilter(
    window_size=0.3,
    recalculation_period=0.0,
    shape=(2,)
)

estimator = PWM3901HardwareEstimator(
    x_multiplier=X_MULTIPLIER,
    y_multiplier=Y_MULTIPLIER,
    serial_port=SERIAL_PORT,
    serial_baudrate=BAUD_RATE,
    seperate_thread=SEPERATE_THREAD,
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