import os
import sys
import json
import pickle
import threading
import asyncio
import numpy as np
import pyrealsense2 as rs
import cv2
import time

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from arx5_interface import Arx5CartesianController, Gain, LogLevel

def load_robot_config(config_path):
    with open(config_path, 'r') as file:
        return json.load(file)

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
        pair = next((p for p in config['robot_pairs'] if p['side'] == side), None)
        if pair:
            create_controller_pair(pair)
        else:
            print(f"No robot pair found for side: {side}")
    else:
        for pair in config['robot_pairs']:
            create_controller_pair(pair)

    for leader, follower in controllers:
        for controller in (leader, follower):
            controller.set_log_level(LogLevel.WARNING)
            controller.reset_to_home()

        gain = Gain(
            leader.get_controller_config().default_kp * 0,
            leader.get_controller_config().default_kd * 0,
            0.0, 0.0
        )
        leader.set_gain(gain)

    return controllers

def poll_joint_states(controller, joint_states_queue, stop_event, recording_event, rate=0.001):
    while not stop_event.is_set():
        if recording_event.is_set():
            timestamp = controller.get_timestamp()
            state = controller.get_joint_state()
            joint_states_queue.put((timestamp, state))
        time.sleep(rate)

def frame_capture_loop(pipelines, stop_event, recording_event, frames_queue, latest_frames):
    frame_index = 0
    while not stop_event.is_set():
        for cam_idx, pipeline in enumerate(pipelines):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            timestamp = color_frame.get_timestamp() / 1000.0
            color_image = np.asanyarray(color_frame.get_data())
            latest_frames[cam_idx] = color_image.copy()

            if recording_event.is_set():
                frames_queue.put((frame_index, timestamp, cam_idx, color_image))
                frame_index += 1
        time.sleep(0.01)

def save_frames_and_metadata(frames_queue, traj_path):
    base_obs = "observations"
    frames_dict = {}
    while not frames_queue.empty():
        frame_index, timestamp, cam_idx, color_image = frames_queue.get()
        frames_dict.setdefault(cam_idx, []).append((frame_index, timestamp, color_image))

    for cam_idx, frames in frames_dict.items():
        frames_dir = os.path.join(base_obs, f"{traj_path}", "color_frames", f"cam_{cam_idx}")
        metadata_dir = os.path.join(base_obs, f"{traj_path}", "metadata", f"cam_{cam_idx}")
        os.makedirs(frames_dir, exist_ok=True)
        os.makedirs(metadata_dir, exist_ok=True)

        metadata_list = []
        frames.sort(key=lambda x: x[0])
        for frame_index, timestamp, color_image in frames:
            frame_path = os.path.join(frames_dir, f"frame_{frame_index}.png")
            cv2.imwrite(frame_path, color_image)
            metadata_list.append({
                'frame_idx': frame_index,
                'timestamp': timestamp,
                'camera_idx': cam_idx,
                'frame_path': frame_path
            })

        with open(os.path.join(metadata_dir, "metadata.pkl"), 'wb') as f:
            pickle.dump(metadata_list, f)
        print(f"Saved {len(frames)} frames and metadata for camera {cam_idx}")

def save_joint_states(joint_states_queue, save_dir):
    joint_states = []
    while not joint_states_queue.empty():
        joint_states.append(joint_states_queue.get())
    os.makedirs(save_dir, exist_ok=True)
    with open(os.path.join(save_dir, "joint_states.pkl"), 'wb') as f:
        pickle.dump(joint_states, f)

def get_next_traj_folder(base_path):
    index = 0
    while os.path.exists(os.path.join(base_path, f"traj_{index}")):
        index += 1
    return f"traj_{index}"
