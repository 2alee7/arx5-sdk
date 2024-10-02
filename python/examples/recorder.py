import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
from datetime import datetime
import shutil

"""
Note: All text prompts require the OpenCV window to be in focus. Sorry.
"""

# Function to save frames to a specified path
def save_frame(frame, path, index):
    if not os.path.exists(path):
        os.makedirs(path)
    filename = os.path.join(path, f"frame_{index}.png")
    cv2.imwrite(filename, frame)
    print(f"Frame {index} saved to {filename}")

ctx = rs.context()
devices = ctx.query_devices()
for dev in devices:
    dev.hardware_reset()
print("Devices reset")

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgra8, 30)

try:
    pipeline.start(config)
except RuntimeError as e:
    print(f"Failed to start RealSense pipeline: {e}")
    exit(1)

# Create OpenCV window
cv2.namedWindow('Recording', cv2.WINDOW_AUTOSIZE)

try:
    while True:
        print("Press SPACE to start recording, or ESC to exit")
        while True:
            key = cv2.waitKey(1000)
            if key == 32:  # SPACE key
                break
            elif key == 27:  # ESC key
                exit(0)

        print("Recording started...")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = f"/home/abraham/RealSense/recordings/recording_{timestamp}"
        frame_index = 0

        # Retrieve the first frame to account for initial lag
        while True:
            try:
                frameset = pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frameset.get_color_frame()
                if not color_frame:
                    continue
                color_image = np.asanyarray(color_frame.get_data())
                save_frame(color_image, save_path, frame_index)
                frame_index += 1
                cv2.imshow('Recording', color_image)
                break  # Exit the loop after the first frame is retrieved
            except RuntimeError as e:
                print(f"Error retrieving frame: {e}")
                continue

        # Start the timer after the first frame is retrieved
        start_time = time.time()

        while time.time() - start_time < 5:
            try:
                frameset = pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frameset.get_color_frame()
                if not color_frame:
                    continue
                color_image = np.asanyarray(color_frame.get_data())
                save_frame(color_image, save_path, frame_index)
                frame_index += 1
                cv2.imshow('Recording', color_image)
                
                key = cv2.waitKey(1)
                if key == 32:  # SPACE key to stop recording
                    break
            except RuntimeError as e:
                print(f"Error retrieving frame: {e}")
                continue
                
        print("Recording stopped.")

        print("Save data [y/n]?")
        while True:
            key = cv2.waitKey(100)
            if key == ord('y') or key == ord('Y'):
                print(f"Data saved in {save_path}")
                break
            elif key == ord('n') or key == ord('N'):
                shutil.rmtree(save_path)
                print(f"Data deleted from {save_path}")
                break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()