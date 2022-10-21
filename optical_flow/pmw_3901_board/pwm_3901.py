import time
import typing
from localizer_base import LocalFrameEstimatorImpl, NonBlocking, Parallel
import serial
import json
import threading
from optical_flow.continuous_slidingwindow_filter import ContinuousMovingWindowFilter
import numpy as np


class PWM3901HardwareEstimator(LocalFrameEstimatorImpl, NonBlocking, Parallel):
    def __init__(
        self, 
        x_multiplier : float = 1.0,
        y_multiplier : float = 1.0,
        serial_port : str = "COM3", 
        serial_baudrate : int = 115200, 
        seperate_thread : bool = False, 
        sliding_window_filter : typing.Optional[ContinuousMovingWindowFilter] = None
    ) -> None:
        LocalFrameEstimatorImpl.__init__(self,name="PWM3901HardwareEstimator", autoUpdateVelocity=False, autoUpdateAcceleration=True)
        Parallel.__init__(self)

        self.ser = serial.Serial(serial_port, serial_baudrate, timeout=1.0)
        self.sliding_window_filter = sliding_window_filter
        self.x_multiplier = x_multiplier
        self.y_multiplier = y_multiplier
        self.last_hardware_dt = 0.0

        self.update()
        
        if seperate_thread:
            t = threading.Thread(target=self.seperateThreadUpdate)
            t.start()
    
    
    def processLine(self, line : str, update_v : bool = False) -> None:
        if line == "":
            return
        
        if "ERROR" in line:
            print("Optical Flow Error")
            return
        
        try:
            spLi = line.split(",")
            dx_px = float(spLi[0])
            dy_px = float(spLi[1])
            obs_dt_ms = float(spLi[2])
            obs_dt = obs_dt_ms / 1000.0
        except:
            print("Optical Flow reported unreadable data")
            return
        
        dx_scaled = dx_px * self.x_multiplier
        dy_scaled = dy_px * self.y_multiplier
        
        obs_d = np.array([dx_scaled, dy_scaled])
        obs_v = obs_d / obs_dt

        self.last_hardware_dt = obs_dt

        if self.sliding_window_filter is not None:
            self.sliding_window_filter.add_observation(obs_dt, obs_v, obs_d)

        if update_v:
            if self.sliding_window_filter is not None:
                estimated_velocity = self.sliding_window_filter.get_per_size_average()
            else:
                estimated_velocity = obs_v
            
            self._call_local_velocity_update(np.append(estimated_velocity, np.zeros(4)))

    def update(self) -> None:
        if self.ser.isOpen():
            self.ser.write(b'\x02') #write arbitrary byte
            self.ser.flush()

            lines = self.ser.read_all().decode("ascii").split("\n") #self.ser.readline()

            while len(lines) > 0 and lines[-1].strip() == "":
                lines = lines[:-1]
            
            for idx, line in enumerate(lines):
                self.processLine(line, update_v=(idx == len(lines) - 1))
        else:
            print("Serial port is not opened, reopening...")
            self.ser.open()

    def seperateThreadUpdate(self):
        while self._running:
            self.update()
            time.sleep(0.05)