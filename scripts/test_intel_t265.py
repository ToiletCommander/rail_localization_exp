import pyrealsense2 as rs
import time

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)

try:
    while True:
        # Wait for the next set of frames from the camera
        t = time.time()
        frames = pipe.wait_for_frames()

        # Fetch pose frame
        
        pose = frames.get_pose_frame()
        print("time", time.time() - t)
        if pose:
            # Print some of the pose data to the terminal
            data = pose.get_pose_data()
            print("Frame #{}".format(pose.frame_number))
            print("Position: {}".format(data.translation))

            print("Velocity: {}".format(data.velocity))
            print("Acceleration: {}\n".format(data.acceleration))
            #print("data", data)
        
        time.sleep(0.5)

finally:
    pipe.stop()