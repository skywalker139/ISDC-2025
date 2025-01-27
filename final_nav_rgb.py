import os
import time
import cv2
import json
import sys

if sys.version_info.major == 3 and sys.version_info.minor >= 10:
    import collections
    setattr(collections, "MutableMapping", collections.abc.MutableMapping)

from dronekit import connect, VehicleMode
from collections.abc import MutableMapping
# Create the "images" folder if it doesn't exist
images_folder = "images"
os.makedirs(images_folder, exist_ok=True)
print("start")
# Initialize the camera
cap = cv2.VideoCapture(4)  # Use the first camera (change index if needed)
if not cap.isOpened():
    print("Error: Unable to access the camera.")
    exit()
print("fuck")
# Connect to the drone using DroneKit
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
print("Drone connection established.")

# Initialize the data storage
data = []

try:
    while True:
        # Capture an image from the camera
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to capture image.")
            break

        # Get timestamp for the image filename
        timestamp = time.strftime("%Y%m%d_%H%M%S%f")
        image_name = f"{timestamp}.jpg"
        image_path = os.path.join(images_folder, image_name)

        # Save the image to the "images" folder
        cv2.imwrite(image_path, frame)

        # Fetch GPS and yaw data from the drone
        gps_data = None
        yaw_data = None

        if vehicle.location.global_frame:
            gps_data = {
                "lat": vehicle.location.global_frame.lat,
                "lon": vehicle.location.global_frame.lon,
                "alt": vehicle.location.global_frame.alt
            }

        if vehicle.attitude:
            yaw_data = {
                "yaw": vehicle.attitude.yaw
            }

        if gps_data and yaw_data:
            # Append data to the list
            data.append({
                "timestamp": timestamp,
                "image_path": image_path,
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

    # Release the camera and close the connection
    cap.release()
    vehicle.close()
    print(f"Data saved to {json_file_path}. Exiting.")
