import time
import typing
from localizer_base import LocalFrameEstimatorImpl
import serial
import json
import threading
from optical_flow.continuous_slidingwindow_filter import ContinuousMovingWindowFilter
import numpy as np

class BitcrazeHardwareEstimator(LocalFrameEstimatorImpl):
    def __init__(
        self, 
        x_multiplier : float = 1.0,
        y_multiplier : float = 1.0,
        serial_port : str = "COM3", 
        serial_baudrate : int = 115200, 
        seperate_thread : bool = False, 
        sliding_window_filter : typing.Optional[ContinuousMovingWindowFilter] = None
    ) -> None:
        super().__init__(name="BitcrazeHardwareEstimator", autoUpdateVelocity=False, autoUpdateAcceleration=True)
        
        self.ser = serial.Serial(serial_port, serial_baudrate, timeout=1)
        
        self.lastHardwareDataReceivedTime = time.time()
        self.update()
        if seperate_thread:
            t = threading.Thread(target=self.seperateThreadUpdate)
            t.start()
        self.sliding_window_filter = sliding_window_filter
        self.x_multiplier = x_multiplier
        self.y_multiplier = y_multiplier

    def update(self) -> None:
        if self.ser.isOpen():
            retDat = self.ser.readline()
            try:
                json_retDat = json.loads(retDat)
            except:
                print("Hardware communicated invalid JSON")
                return
            if 'error' in json_retDat and json_retDat['error'] is not None:
                print("Hardware Error:", json_retDat['error'])
                return
            elif 'dist' not in json_retDat or 'dx' not in json_retDat or 'dy' not in json_retDat:
                print("Hardware communicated invalid JSON")
                return
            try:
                dist_mm = float(json_retDat['dist'])
                dx_px = float(json_retDat['dx'])
                dy_px = float(json_retDat['dy'])
            except:
                print("Hardware communicated invalid JSON")
                return
            
            
            ctime = time.time()
            obs_dt = ctime - self.lastHardwareDataReceivedTime
            dx_scaled = dist_mm * dx_px * self.x_multiplier
            dy_scaled = dist_mm * dy_px * self.y_multiplier
            
            if dist_mm <= 80: #according to specs, the flow camera only works up to 80mm
                obs_v = np.zeros(2)
                print("Flow camera out of range (<= 80mm)")
            else:
                obs_v = np.array([dx_scaled, dy_scaled]) / obs_dt

            if self.sliding_window_filter is not None:
                self.sliding_window_filter.add_observation(obs_dt, obs_v)
                estimated_velocity = self.sliding_window_filter.get_per_size_average()
            else:
                estimated_velocity = obs_v
            self._call_local_velocity_update(np.append(estimated_velocity, np.zeros(4)))
            self.lastHardwareDataReceivedTime = ctime
        else:
            print("Serial port is not opened, reopening...")
            self.ser.open()

    def seperateThreadUpdate(self):
        while True:
            self.update()
            time.sleep(0.05)