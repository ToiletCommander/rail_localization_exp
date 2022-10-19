"""Moving window filter to smooth out sensor readings."""
import collections
import typing
import numpy as np


class ContinuousMovingWindowFilter:
    def __init__(self, window_size: float, recalculation_period : float = -1, shape = (3,)):
        self.values : typing.Deque[typing.Tuple[float,np.ndarray]] = collections.deque()
        
        self.aggregated : float = np.zeros(shape)
        self.aggregated_size : float = 0.0
        
        self.window_size : float = window_size
        
        self.recalculation_period : float = recalculation_period
        self.size_since_last_recalculation : float = 0.0
    
    def __remove_front_values(self, size : float) -> None:
        removed_size = 0.0
        while removed_size < size and len(self.values) > 0:
            front_size, front_value = self.values[0]
            c_remove_size = min(size - removed_size, front_size)

            if c_remove_size < front_size:
                self.values[0] = (front_size - c_remove_size, front_value)
            else:
                self.values.popleft()
            self.aggregated_size -= c_remove_size
            self.aggregated -= front_value * c_remove_size
            removed_size += c_remove_size
    
    def __recalculate(self) -> None:
        self.aggregated = np.zeros_like(self.aggregated)
        self.aggregated_size = 0.0
        for size, value in self.values:
            self.aggregated += value * size
            self.aggregated_size += size
        self.size_since_last_recalculation = 0.0
            

    def add_observation(self, new_value_size : float, new_value: np.ndarray) -> None:
        self.values.append((new_value_size, new_value))
        self.aggregated += new_value * new_value_size
        self.aggregated_size += new_value_size
        self.size_since_last_recalculation += new_value_size
        
        if self.aggregated_size > self.window_size:
            self.__remove_front_values(self.aggregated_size - self.window_size)
        
        if self.recalculation_period > 0 and self.size_since_last_recalculation > self.recalculation_period:
            self.__recalculate()

    
    def get_per_size_average(self) -> np.ndarray:
        return self.aggregated / self.aggregated_size if self.aggregated_size > 0 else np.zeros_like(self.aggregated)
    
    def reset(self) -> None:
        self.values.clear()
        self.aggregated = np.zeros_like(self.aggregated)
        self.aggregated_size = 0.0
        self.size_since_last_recalculation = 0.0