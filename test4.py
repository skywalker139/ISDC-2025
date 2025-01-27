import os
import time
import json
import numpy as np
import pyrealsense2 as rs
from pymavlink import mavutil
from PIL import Image

# Create folder for depth images
depth_images_folder = "depth_images"
os.makedirs(depth_images_folder, exist_ok=True)

connection = mavutil.mavlink_connection('/dev/ttyACM0')
connection.wait_heartbeat()
print("Drone connection established.")

# Initialize the RealSense depth camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)
print("pipeline started")
# Connect to the drone using pymavlink

# Initialize the data storage
data = []

try:
    while True:
        # Capture a depth frame from the RealSense camera
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            print("Error: Unable to capture depth frame.")
            continue

        # Get timestamp for filenames
        timestamp = time.strftime("%Y%m%d_%H%M%S%f")
        depth_image_name = f"{timestamp}.png"
        depth_image_path = os.path.join(depth_images_folder, depth_image_name)

        # Convert depth frame to numpy array (no normalization)
        depth_image = np.asanyarray(depth_frame.get_data())

        # Save depth image as PNG using PIL (no normalization)
        pil_image = Image.fromarray(depth_image)
        pil_image.save(depth_image_path)

        # Fetch GPS and attitude data from the drone
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            gps_data = {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "alt": msg.alt / 1000
            }
        else:
            gps_data = None

        attitude_msg = connection.recv_match(type='ATTITUDE', blocking=True)
        if attitude_msg:
            yaw_data = {
                "yaw": attitude_msg.yaw
            }
        else:
            yaw_data = None

        if gps_data and yaw_data:
            # Append data to the list
            data.append({
                "timestamp": timestamp,
                "depth_image_path": depth_image_path,
                "gps_data": gps_data,
                "yaw_data": yaw_data
            })

        # Sleep for 0.5 seconds
        time.sleep(0.5)
        print(gps_data)

except KeyboardInterrupt:
    print("Process interrupted by user.")

finally:
    # Save the GPS and yaw data to a JSON file
    json_file_path = "data.json"
    with open(json_file_path, "w") as json_file:
        json.dump(data, json_file, indent=4)

    # Stop the RealSense pipeline
    pipeline.stop()
    print(f"Data saved to {json_file_path}. Exiting.")
