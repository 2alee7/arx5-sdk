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

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from arx5_interface import Arx5CartesianController, Gain

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

# Function to save frames to a specified path
def save_frame(frame, path, index):
    if not os.path.exists(path):
        os.makedirs(path)
    filename = os.path.join(path, f"frame_{index}.png")
    cv2.imwrite(filename, frame)
    print(f"Frame {index} saved to {filename}")

# Function to get the next available trajectory folder
def get_next_traj_folder(base_path):
    index = 0
    while os.path.exists(os.path.join(base_path, f"traj{index}")):
        index += 1
    return os.path.join(base_path, f"traj{index}")

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
def record_traj(pipeline, follower_controller, traj_folder, frame_ct=40):
    joint_states = []
    frame_index = 0

    while frame_index < frame_ct:
        start_time = time.time()

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        # Save frame directly
        save_frame(color_image, traj_folder, frame_index)

        # Collect JointState
        joint_state = follower_controller.get_joint_state()
        joint_state_str = joint_state.to_string()
        joint_states.append(joint_state_str)

        cv2.imshow('Recording', color_image)
        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            print("Recording stopped...")
            break

        frame_index += 1

        # Sleep to achieve 5Hz loop
        elapsed_time = time.time() - start_time
        sleep_time = max(0, 0.2 - elapsed_time)
        time.sleep(sleep_time)

    # Save all joint states to a single .pkl file
    joint_states_folder = "joint_states"
    os.makedirs(joint_states_folder, exist_ok=True)
    joint_states_file = os.path.join(joint_states_folder, os.path.basename(traj_folder) + ".pkl")
    with open(joint_states_file, "wb") as f:
        pickle.dump(joint_states, f)

    # Load and print the .pkl file for sanity check
    with open(joint_states_file, "rb") as f:
        loaded_joint_states = pickle.load(f)
        print("Loaded joint states from .pkl file:")
        for state in loaded_joint_states:
            print(state)

def main():
    
    config = load_robot_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'teleop_config.json'))
    controllers = initialize_controllers(config)
    stop_events = []
    threads = []

    # Create OpenCV window
    cv2.namedWindow('Recording', cv2.WINDOW_AUTOSIZE)

    # Initialize RealSense pipeline
    # pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.color, 848, 480, rs.format.bgra8, 30)

    # try:
    #     pipeline.start(config)
    # except RuntimeError as e:
    #     print(f"Failed to start RealSense pipeline: {e}")
    #     exit(1)

    try:
        # Start control loop threads for each controller pair
        for leader_controller, follower_controller in controllers:
            stop_event = threading.Event()
            stop_events.append(stop_event)

            control_thread = threading.Thread(
                target=control_loop_open,
                args=(leader_controller, follower_controller, stop_event)
            )
            
            control_thread.start()
            threads.append(control_thread)

        while True:
            print("Press SPACE to start recording, or ESC to exit")
            while True:
                key = cv2.waitKey(1000)
                if key == 27:  # ESC key
                    print("Exiting...")
                    raise KeyboardInterrupt
                elif key == 32:  # SPACE key
                    print("Recording started...")
                    break

            # Create new trajectory folder
            traj_folder = get_next_traj_folder("recorded_frames")
            os.makedirs(traj_folder, exist_ok=True)

            # Record a single trajectory
            # record_traj(leader_controller, traj_folder)

            # Print the number of trajectories currently saved
            traj_count = len([
                name for name in os.listdir("recorded_frames")
                if os.path.isdir(os.path.join("recorded_frames", name))
            ])
            print(f"Number of trajectories currently saved: {traj_count}")

    except KeyboardInterrupt:
        print("Interrupted by user, stopping...")
    finally:
        # Signal all threads to stop
        for event in stop_events:
            event.set()

        # Wait for all threads to finish
        for thread in threads:
            thread.join()

        # Reset all controllers to home position
        for leader_controller, follower_controller in controllers:
            leader_controller.reset_to_home()
            follower_controller.reset_to_home()

        # Clean up OpenCV windows
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()