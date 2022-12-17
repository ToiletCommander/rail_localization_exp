
from intel_t265.intel_t265_estimator import IntelT265Estimator
import time

if __name__ == "__main__":
    t265_estimator = IntelT265Estimator()
    try:
        while True:
            
            t265_estimator.update()
            #print("Location",Tracker.getLocation())
            #print("Velocity",Tracker.getVelocity())
            #print("Acceleration",Tracker.getAcceleration())
            print("Local Velocity",t265_estimator.getLocalVelocity())
            #print("Local Acceleration",Tracker.getLocalAcceleration())
            print("")
            time.sleep(0.1)

    finally:
        t265_estimator.stop()
