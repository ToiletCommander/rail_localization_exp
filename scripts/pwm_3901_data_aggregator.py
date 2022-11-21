import time
from openvr_localizer import OpenVRGlobalFrameEstimator
from optical_flow.pmw_3901_board.pwm_3901 import PWM3901HardwareEstimator
from optical_flow.continuous_slidingwindow_filter import ContinuousMovingWindowFilter
import pandas

time_exp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
FILENAME = "aggregate_data_" + time_exp + ".csv"

PWM3901_X_MULTIPLIER = 1
PWM3901_Y_MULTIPLIER = 1
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
    x_multiplier=PWM3901_X_MULTIPLIER,
    y_multiplier=PWM3901_Y_MULTIPLIER,
    serial_port=SERIAL_PORT,
    serial_baudrate=BAUD_RATE,
    parallel=SEPERATE_THREAD,
    sliding_window_filter=FILTER,
    dist_cm_sliding_window=DIST_FILTER
)

openvr_estimator : OpenVRGlobalFrameEstimator = OpenVRGlobalFrameEstimator.getOpenVRTrackerFromViveTracker()

start_time = time.time()
dts = []
openvr_velocities = []
pwm3901_velocities = []

try:
    while True:
        if not(SEPERATE_THREAD):
            pwm3901_estimator.update()
        openvr_estimator.update()
        

        ctime = time.time()
        dt = ctime - start_time
        dts.append(dt)
        openvr_velocities.append(openvr_estimator.getLocalVelocity())
        pwm3901_velocities.append(pwm3901_estimator.getLocalVelocity())

        time.sleep(0.2)
        
except KeyboardInterrupt:
    pwm3901_estimator.stop()

df = pandas.DataFrame({
    'dts': dts,
    'pwm3901_vx': [v[0] for v in pwm3901_velocities],
    'pwm3901_vy': [v[1] for v in pwm3901_velocities],
    'openvr_vx': [v[0] for v in openvr_velocities],
    'openvr_vy': [v[1] for v in openvr_velocities],
    'openvr_vz': [v[2] for v in openvr_velocities],
    'openvr_v_angx': [v[3] for v in openvr_velocities],
    'openvr_v_angy': [v[4] for v in openvr_velocities],
    'openvr_v_angz': [v[5] for v in openvr_velocities]
})
df.to_csv(FILENAME)