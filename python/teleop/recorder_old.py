import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
import sys
import json
import pickle
from datetime import datetime
import shutil
import threading
import asyncio
import queue

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from arx5_interface import Arx5CartesianController, Gain, LogLevel

"""
Note: All text prompts require the OpenCV window to be in focus. Sorry.
"""


def load_robot_config(config_path):
    with open(config_path, 'r') as file:
        config = json.load(file)
    return config

def initialize_controllers(config, side=None):
    controllers = []
    urdf_path = "../models/arx5.urdf"

    def create_controller_pair(pair):
        leader = Arx5CartesianController(
            pair['leader']['model'],
            pair['leader']['interface_name'],
            urdf_path,
        )
        follower = Arx5CartesianController(
            pair['follower']['model'],
            pair['follower']['interface_name'],
            urdf_path,
        )
        gain = Gain(
            leader.get_controller_config().default_kp / 10000,
            leader.get_controller_config().default_kd / 1000,
            0.0,
            0.0
        )
        
        leader.set_log_level(LogLevel.WARNING)
        follower.set_log_level(LogLevel.WARNING)
        leader.reset_to_home()
        follower.reset_to_home()
        leader.set_gain(gain)  # Set reduced damping coeffs. to leader only
        controllers.append((leader, follower))

    if side:
        # Find the robot pair matching the specified side
        pair = next((p for p in config['robot_pairs'] if p['side'] == side), None)
        if pair:
            create_controller_pair(pair)
        else:
            print(f"No robot pair found for side: {side}")
    else:
        # Initialize all robot pairs
        for pair in config['robot_pairs']:
            create_controller_pair(pair)

    return controllers

def initialize_cameras(config):
    pipelines = []

    def init_camera_pipeline(serial_no):
        # Create a RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial_no)
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgra8, 30)
        pipeline.start(config)
        pipelines.append(pipeline)

    for id, serial in config['cameras']:
        init_camera_pipeline(serial)

    return pipelines

def poll_joint_states(controller, joint_states_queue, stop_event, recording_event, rate = 0.01):
    """Poll joint states at default 100 Hz and store them in joint_states_queue."""
    while not stop_event.is_set():
        if recording_event.is_set():
            timestamp = controller.get_timestamp()
            state = controller.get_joint_state()
            joint_states_queue.put((timestamp, state))
        time.sleep(rate)

def frame_capture_loop(pipelines, stop_event, recording_event, frames_queue):
    """
    Continuously capture frames from each camera, show them asynchronously,
    and add (frame index, timestamp, camera index, frame data) to the frames_queue
    only when recording_event is set.
    """
    frame_index = 0
    while not stop_event.is_set():
        for cam_idx, pipeline in enumerate(pipelines):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Get timestamp and convert frame to numpy array.
            timestamp = color_frame.get_timestamp()
            color_image = np.asanyarray(color_frame.get_data())

            # Display the frame (asynchronous visualization)
            cv2.imshow("Recording", color_image)
            key = cv2.waitKey(1) & 0xFF

            # Control key events:
            # SPACE key starts recording (if not already recording).
            if key == 32:
                if not recording_event.is_set():
                    print("Recording started...")
                    # Clear the frames_queue for a fresh recording session.
                    while not frames_queue.empty():
                        frames_queue.get()
                    recording_event.set()
            # ESC key stops recording if active, or stops the loop if not recording.
            elif key == 27:
                if recording_event.is_set():
                    print("Recording stopped.")
                    recording_event.clear()
                else:
                    print("Exiting frame capture loop.")
                    stop_event.set()
                    break

            # If recording is active, add the frame and metadata to the queue.
            if recording_event.is_set():
                frames_queue.put((frame_index, timestamp, cam_idx, color_image))
                frame_index += 1

def save_frames_and_metadata(frames_queue, traj_no):
    """
    Saves frames and corresponding metadata for each camera view.
    Data is saved to:
      - observations/traj_{traj_no}/color_frames/{camera_view}
      - observations/traj_{traj_no}/metadata/{camera_view}
    """
    base_obs = "observations"
    # Drain the frames_queue into a dictionary keyed by camera index.
    frames_dict = {}
    while not frames_queue.empty():
        frame_index, timestamp, cam_idx, color_image = frames_queue.get()
        frames_dict.setdefault(cam_idx, []).append((frame_index, timestamp, color_image))

    for cam_idx, frames in frames_dict.items():
        # Create folders for frames and metadata for this camera view.
        frames_dir = os.path.join(base_obs, f"traj_{traj_no}", "color_frames", f"cam_{cam_idx}")
        metadata_dir = os.path.join(base_obs, f"traj_{traj_no}", "metadata", f"cam_{cam_idx}")
        os.makedirs(frames_dir, exist_ok=True)
        os.makedirs(metadata_dir, exist_ok=True)

        metadata_list = []
        frames.sort(key=lambda x: x[0])
        for (frame_index, timestamp, color_image) in frames:
            frame_path = os.path.join(frames_dir, f"frame_{frame_index}.png")
            cv2.imwrite(frame_path, color_image)
            metadata_list.append({
                'frame_idx': frame_index,
                'timestamp': timestamp,
                'camera_idx': cam_idx,
                'frame_path': frame_path
            })

        metadata_path = os.path.join(metadata_dir, "metadata.pkl")
        with open(metadata_path, 'wb') as f:
            pickle.dump(metadata_list, f)
        print(f"Saved {len(frames)} frames and metadata for camera {cam_idx}")


def save_joint_states(joint_states_queue, save_dir):
    joint_states = []
    while not joint_states_queue.empty():
        joint_states.append(joint_states_queue.get())
    os.makedirs(save_dir, exist_ok=True)
    file_path = os.path.join(save_dir, "joint_states.pkl")
    with open(file_path, 'wb') as f:
        pickle.dump(joint_states, f)
    # print(f"Saved {len(joint_states)} joint states to {file_path}")

def get_next_traj_folder(base_path):
    """Get the next available trajectory folder in the specified base path."""
    index = 0
    while os.path.exists(os.path.join(base_path, f"traj{index}")):
        index += 1
    return os.path.join(base_path, f"traj_{index}")

# Control loop function
def control_loop_open(leader_controller, follower_controller, stop_event):
    while not stop_event.is_set():
        try:
            # Collect EEFState from leader controller
            leader_eef_state = leader_controller.get_eef_state()
            follower_cmd = leader_eef_state
            follower_cmd.gripper_pos *= 4.8
            follower_cmd.timestamp = 0.0

            # Set EEFState to follower controller
            follower_controller.set_eef_cmd(follower_cmd)
        except Exception as e:
            print(f"Error in control loop: {e}")

        # Sleep to achieve 100Hz loop
        time.sleep(0.005)

# Function to record a single trajectory
def record_traj(pipelines, controllers):
    """
    Continuously record frames from each camera pipeline into frames_queue.
    Press ESC to stop recording.
    """
    frames_queue.clear()
    joint_states_queue.clear()

    polling_threads = []
    stop_event = threading.Event()

    # Start joint state polling threads for each controller
    for leader_controller, follower_controller in controllers:
        leader_joint_thread = threading.Thread(
            target=poll_joint_states,
            args=(leader_controller, joint_states_queue, stop_event)
        )
        follower_joint_thread = threading.Thread(
            target=poll_joint_states,
            args=(follower_controller, joint_states_queue, stop_event)
        )
        leader_joint_thread.start()
        follower_joint_thread.start()
        polling_threads.extend([leader_joint_thread, follower_joint_thread])

    print("Recording started. Press ESC to stop.")

    while True:
        color_image = None

        for idx, pipeline in enumerate(pipelines):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert device timestamp (ms) to seconds
            # timestamp = color_frame.get_timestamp() / 1000.0
            timestamp = color_frame.get_timestamp()
            color_image = np.asanyarray(color_frame.get_data())

            # Store (timestamp, camera_idx, color_image) in frames_queue
            frames_queue.append((timestamp, idx, color_image))

        # Display the last captured frame from any camera
        if color_image is not None:
            cv2.imshow("Recording", color_image)

        key = cv2.waitKey(5)
        if key == 27:  # ESC
            print("Recording stopped.")
            stop_event.set()
            break

        time.sleep(0.005)

def main():
    
    config = load_robot_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'teleop_config.json'))
    controllers = initialize_controllers(config)
    # pipelines = initialize_cameras(config)

    frames_queue = queue.Queue()
    leader_joint_states_queue = queue.Queue()
    follower_joint_states_queue = queue.Queue()

    stop_event = threading.Event()
    recording_event = threading.Event()
    polling_threads = []

    # Create OpenCV window
    # cv2.namedWindow('Recording', cv2.WINDOW_AUTOSIZE)

    # frame_thread = threading.Thread(
    #     target=frame_capture_loop,
    #     args=(pipelines, stop_event, recording_event, frames_queue)
    # )
    # frame_thread.start()

    joint_state_threads = []
    for leader_controller, follower_controller in controllers:
        t_leader = threading.Thread(
            target=poll_joint_states,
            args=(leader_controller, leader_joint_states_queue, stop_event, recording_event, 0.01)
        )
        t_follower = threading.Thread(
            target=poll_joint_states,
            args=(follower_controller, follower_joint_states_queue, stop_event, recording_event, 0.01)
        )
        t_leader.start()
        t_follower.start()
        joint_state_threads.extend([t_leader, t_follower])
    
    control_stop_events = []
    control_threads = []
    for leader_controller, follower_controller in controllers:
        ctrl_stop_event = threading.Event()
        control_stop_events.append(ctrl_stop_event)
        t = threading.Thread(
            target=control_loop_open,
            args=(leader_controller, follower_controller, ctrl_stop_event)
        )
        t.start()
        control_threads.append(t)

    try:
        while not stop_event.is_set():
            # When recording stops and data is in any queue, prompt the user.
            if (not recording_event.is_set() and 
                (not frames_queue.empty() or not leader_joint_states_queue.empty() or not follower_joint_states_queue.empty())):
                print("Recording session ended.")
                print("Press 's' to save the recording, or any other key to discard.")
                key = cv2.waitKey(0) & 0xFF
                traj_no = get_next_traj_folder()
                if key == ord('s'):
                    # Save frames and metadata.
                    save_frames_and_metadata(frames_queue, traj_no)
                    # Save leader joint states.
                    leader_dir = os.path.join("observations", f"traj_{traj_no}", "leader_joint_states")
                    save_joint_states(leader_joint_states_queue, leader_dir)
                    # Save follower joint states.
                    follower_dir = os.path.join("actions", f"traj_{traj_no}", "follower_joint_states")
                    save_joint_states(follower_joint_states_queue, follower_dir)
                    print(f"Recording saved as trajectory {traj_no}.")
                else:
                    # Clear all queues.
                    while not frames_queue.empty():
                        frames_queue.get()
                    while not leader_joint_states_queue.empty():
                        leader_joint_states_queue.get()
                    while not follower_joint_states_queue.empty():
                        follower_joint_states_queue.get()
                    print("Recording discarded.")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Interrupted by user, stopping...")
    finally:
        stop_event.set()
        recording_event.clear()
        # frame_thread.join()
        for t in joint_state_threads:
            t.join()
        for ev in control_stop_events:
            ev.set()
        for t in control_threads:
            t.join()

        for leader, follower in controllers:
            leader.set_to_damping()
            follower.set_to_damping()
            leader.reset_to_home()
            follower.reset_to_home()

        # cv2.destroyAllWindows()

if __name__ == "__main__":
    main()