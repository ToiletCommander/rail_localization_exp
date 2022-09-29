"""Moving window filter to smooth out sensor readings."""
# See https://github.com/ikostrikov/walk_in_the_park/blob/main/real/utilities/moving_window_filter.py#L25

import collections
import typing
import numpy as np


class MovingWindowFilter:
    def __init__(self, window_size: int, shape = (3,), weights : typing.Optional[np.ndarray] = None):
        self.values : typing.Deque[np.ndarray] = collections.deque(maxlen=window_size)
        self.avg = np.zeros(shape)
        self.weights = weights

    def add_observation(self, new_value: np.ndarray) -> None:
        self.values.append(new_value)
        weights = self.weights

        if weights is not None:
            weights = weights[:len(self.values)]
        
        self.avg = np.average(self.values, axis=0, weights=weights)

    
    def get_average(self) -> np.ndarray:
        return self.avg
    
    def reset(self) -> None:
        self.values.clear()
        self.avg = np.zeros_like(self.avg)