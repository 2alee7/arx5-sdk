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
import signal
import argparse

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from arx5_interface import Arx5CartesianController, Gain, LogLevel

parser = argparse.ArgumentParser(description= "Specify single side, robot or controller")
parser.add_argument("--right", action="store_true", help="Specify right side robot")
parser.add_argument("--left", action="store_true", help="Specify left side robot")
parser.add_argument("--robot", type=int, help="Identical robot teleoperation")
parser.add_argument("--controller", type=int, help="Lightweight controller teleoperation")
args = parser.parse_args()

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

    for pair in controllers:
        for controller in pair:
            controller.set_log_level(LogLevel.WARNING)
            controller.reset_to_home()
        
        gain = Gain(
            pair[0].get_controller_config().default_kp * 0,
            pair[0].get_controller_config().default_kd * 0,
            0.0,
            0.0
        )
        pair[0].set_gain(gain)  # Set reduced damping coeffs. to leader only

    return controllers

def initialize_cameras(config):
    pipelines = []

    def init_camera_pipeline(serial_no):
        # Create a RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        print("Enabling camera with serial number:", serial_no)
        config.enable_device(serial_no)
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgra8, 30)
        pipeline.start(config)
        pipelines.append(pipeline)

    for cam in config['cameras']:
        print("Initializing camera", cam['name'], "with serial number:", cam['serial'])
        init_camera_pipeline(cam['serial'])

    return pipelines

def poll_joint_states(controller, joint_states_queue, stop_event, recording_event, rate = 0.01):
    """Poll joint states at default 100 Hz and store them in joint_states_queue."""
    while not stop_event.is_set():
        if recording_event.is_set():
            timestamp = controller.get_timestamp()
            state = controller.get_joint_state()
            joint_states_queue.put((timestamp, state))
        asyncio.sleep(rate)

def frame_capture_loop(pipelines, stop_event, recording_event, frames_queue, latest_frames):
    """
    Continuously capture frames from each camera.
    Always update latest_frames for display.
    Enqueue frames for saving only if recording_event is set.
    """
    frame_index = 0
    while not stop_event.is_set():
        for cam_idx, pipeline in enumerate(pipelines):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Get the frame and timestamp
            timestamp = color_frame.get_timestamp() / 1000.0  # convert to seconds
            color_image = np.asanyarray(color_frame.get_data())
            
            # Update the latest frame for this camera (make a copy to be safe)
            latest_frames[cam_idx] = color_image.copy()
            
            # If recording, add the frame to the recording queue
            if recording_event.is_set():
                frames_queue.put((frame_index, timestamp, cam_idx, color_image))
                frame_index += 1
        # Adjust sleep to your desired frame rate
        asyncio.sleep(0.01)

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
def control_loop_open(leader_controller, follower_controller, stop_event, robot = True):
    """ 'IK Failed' error can be ignored safely."""
    while not stop_event.is_set():
        if robot:
            try:
                # Collect EEFState from leader controller
                leader_eef_state = leader_controller.get_eef_state()
                follower_cmd = leader_eef_state
                follower_cmd.gripper_pos *= 4.8
                follower_cmd.timestamp = 0.0

            except Exception as e:
                print(f"Error in control loop: {e}")

        else:
            try:
                # Collect EEFState from leader controller #TODO: Implement lightweight controller
                leader_eef_state = leader_controller.get_eef_state()
                follower_cmd = leader_eef_state
                follower_cmd.gripper_pos *= 4.8
                follower_cmd.timestamp = 0.0

            except Exception as e:
                print(f"Error in control loop: {e}")

        # Set EEFState to follower controller
        follower_controller.set_eef_cmd(follower_cmd)

        # Sleep to achieve 100Hz loop
        time.sleep(0.005)

def main():
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'teleop_config.json')
    config = load_robot_config(config_path)
    controllers = initialize_controllers(config)
    pipelines = initialize_cameras(config)

    def handle_sigint(sig, frame):
        stop_event.set()
        print("Interrupted by user, stopping...")

    signal.signal(signal.SIGINT, handle_sigint)

    # Queues and shared dict for frames and joint states
    frames_queue = queue.Queue()
    latest_frames = {}  # Shared dict to hold the latest frame per camera

    # Define events for thread stopping and recording control
    stop_event = threading.Event()
    recording_event = threading.Event()

    # Start frame capture thread (always running)
    frame_thread = threading.Thread(
        target=frame_capture_loop,
        args=(pipelines, stop_event, recording_event, frames_queue, latest_frames)
    )
    frame_thread.start()

    #Start joint state polling and control threads here
    joint_state_threads = []
    control_threads = []
    control_stop_events = []
    for leader, follower in controllers:
        joint_states_queue = queue.Queue()
        leader_state_thread = threading.Thread(
            target=poll_joint_states,
            args=(leader, joint_states_queue, stop_event, recording_event)
        )
        leader_state_thread.start()
        joint_state_threads.append(leader_state_thread)

        follower_state_thread = threading.Thread(
            target=poll_joint_states,
            args=(follower, joint_states_queue, stop_event, recording_event)
        )
        follower_state_thread.start()
        joint_state_threads.append(follower_state_thread)

        control_stop_event = threading.Event()
        control_stop_events.append(control_stop_event)
        control_thread = threading.Thread(
            target=control_loop_open,
            args=(leader, follower, control_stop_event)
        )
        control_thread.start()
        control_threads.append(control_thread)

    try:
        cv2.namedWindow("Camera Views", cv2.WINDOW_NORMAL)
        while not stop_event.is_set():
            # Continuously build a composite display from the latest frames.
            frames = []
            # For four cameras, use a default blank image if no frame is available.
            for i in range(4):
                if i in latest_frames:
                    frame = latest_frames[i]
                    # Convert from BGRA to BGR if necessary
                    if frame.shape[2] == 4:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                else:
                    frame = np.zeros((480, 848, 3), dtype=np.uint8)
                frames.append(frame)

            # Arrange the 4 frames into a 2x2 grid
            row1 = cv2.hconcat([frames[0], frames[1]])
            row2 = cv2.hconcat([frames[3], frames[2]])
            composite = cv2.vconcat([row1, row2])
            cv2.putText(composite, "top_vew", (0, 480), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
            cv2.putText(composite, "45_deg_view", (848, 480), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
            cv2.putText(composite, "wrist_left", (0, 960), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
            cv2.putText(composite, "wrist_right", (848, 960), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)

            cv2.imshow("Camera Views", composite)
            key = cv2.waitKey(1) & 0xFF
            if key == 32:  # SPACE: toggle recording
                if recording_event.is_set():
                    print("Recording stopped...")
                    recording_event.clear()
                else:
                    print("Recording started...")
                    recording_event.set()
            elif key == 27:  # ESC: exit
                stop_event.set()
                break

            # After recording stops, process the queued frames.
            # For example, if recording just ended (recording_event cleared) and there is data:
            if (not recording_event.is_set() and not frames_queue.empty()):
                print("Recording session ended. Press 's' to save or any other key to discard.")
                key = cv2.waitKey(0) & 0xFF
                traj_no = get_next_traj_folder("observations")
                if key == ord('s'):
                    save_frames_and_metadata(frames_queue, traj_no)
                    # Similarly, save joint state data if applicable.
                    print(f"Recording saved as trajectory {traj_no}.")
                else:
                    # Clear frames_queue if discarded
                    while not frames_queue.empty():
                        frames_queue.get()
                    print("Recording discarded.")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Interrupted by user, stopping...")
    finally:
        for pipeline in pipelines:
            pipeline.stop()
        stop_event.set()
        # display_thread.join()
        cv2.destroyAllWindows()
        recording_event.clear()
        frame_thread.join()

        for leader, follower in controllers:
            leader.set_to_damping()
            follower.set_to_damping()
            leader.reset_to_home()
            follower.reset_to_home()

        for t in joint_state_threads:
            t.join()
        for ev in control_stop_events:
            ev.set()
        for t in control_threads:
            t.join()


if __name__ == "__main__":
    main()