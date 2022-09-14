import typing
import numpy as np
import time


class NonBlocking:
    def update(self) -> None:
        return


"""
Note: The ordering of location/velocity/acceleration is: [x, y, z, degX, degY, degZ]
autoUpdate decides if any other variables should be used to update the current variable
"""

class PoseProvider:
    def __init__(self, name : str, autoUpdateLocation : bool = True, autoUpdateVelocity : bool = True, autoUpdateAcceleration : bool = True):
        self.name = name
        self.locationSubscribeList : typing.List[typing.Callable[[],np.ndarray]] = []
        self.velocitySubscribeList : typing.List[typing.Callable[[],np.ndarray]] = []
        self.accelerationSubscribeList : typing.List[typing.Callable[[],np.ndarray]] = []
        
        self.autoUpdateLocation = autoUpdateLocation
        self.autoUpdateVelocity = autoUpdateVelocity
        self.autoUpdateAcceleration = autoUpdateAcceleration

        self.lastLocationUpdate : float = 0
        self.lastVelocityUpdate : float = 0
        self.lastAccelerationUpdate : float = 0
        self.lastAcceleration : np.ndarray = np.zeros(6)
        self.lastVelocity : np.ndarray = np.zeros(6)
        self.lastLocation : np.ndarray = np.zeros(6)

    def __str__(self) -> str:
        return self.name
    
    def subscribe_location(self, callback : typing.Callable[[],np.ndarray]):
        self.locationSubscribeList.append(callback)
    
    def unsubscribe_location(self, callback : typing.Callable[[],np.ndarray]):
        self.locationSubscribeList.remove(callback)
    
    def subscribe_velocity(self, callback : typing.Callable[[],np.ndarray]):
        self.velocitySubscribeList.append(callback)
    
    def unsubscribe_velocity(self, callback : typing.Callable[[],np.ndarray]):
        self.velocitySubscribeList.remove(callback)

    def subscribe_acceleration(self, callback : typing.Callable[[],np.ndarray]):
        self.accelerationSubscribeList.append(callback)
    
    def unsubscribe_acceleration(self, callback : typing.Callable[[],np.ndarray]):
        self.accelerationSubscribeList.remove(callback)

    def __location_update(self, new_location : np.ndarray) -> None:
        assert new_location.shape == (6,)
        
        ctime = time.time()
        if self.lastLocationUpdate != 0:
            dt = ctime - self.lastLocationUpdate
            self.__location_updated(new_location, self.lastLocation, dt, True)

        for callback in self.locationSubscribeList:
            callback(new_location)
    
    def __location_updated(self,new_location, old_location, time, dt, try_update_velocity: True) -> None:
        self.lastLocation = new_location
        self.lastLocationUpdate = time
        
        if self.autoUpdateVelocity and self.lastVelocityUpdate != 0:
            velocity = (new_location - old_location) / dt
            self.__velocity_update(velocity, self.lastVelocity, time, time - self.lastVelocityUpdate)
    
    def __velocity_updated(self,new_velocity, old_velocity, time, dt, try_update_) -> None:
        self.lastVelocity = new_velocity
        self.lastVelocityUpdate = time

        if self.autoUpdateAcceleration and self.lastAccelerationUpdate != 0:
            acceleration = (new_velocity - old_velocity) / dt
            self.__acceleration_update(acceleration, self.lastAcceleration, time, time - self.lastAccelerationUpdate)

    def __acceleration_updated(self,new_acceleration, old_acceleration, time, dt) -> None:
        pass