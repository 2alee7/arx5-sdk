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
from tqdm import tqdm
from PIL import Image
import argparse

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from arx5_interface import Arx5CartesianController, Arx5JointController, Gain, LogLevel, JointState

"""
Note: All text prompts require the OpenCV window to be in focus. Sorry.
"""


def load_robot_config(config_path):
    with open(config_path, 'r') as file:
        config = json.load(file)
    return config

def initialize_controllers(config, side=None, follower_action_space="cartesian", leader_action_space="cartesian"):
    controllers = []
    controller_names = []
    urdf_path = "../models/arx5.urdf"

    def create_controller_pair(pair):
        if follower_action_space == "joint":
            follower = Arx5JointController(
                pair['follower']['model'],
                pair['follower']['interface_name'],
            )
            follower.enable_background_send_recv()
            # Do NOT turn on gravity compensation for follower when using joint control.
            follower.enable_gravity_compensation(urdf_path)
        elif follower_action_space == "cartesian":
            follower = Arx5CartesianController(
                pair['follower']['model'],
                pair['follower']['interface_name'],
                urdf_path,
            )
        if leader_action_space == "joint":
            # leader = Arx5JointController(
            #     pair['leader']['model'],
            #     pair['leader']['interface_name'],
            # )
            # leader.enable_gravity_compensation(urdf_path)
            # gain = Gain(
            #     leader.get_controller_config().default_kp / 10000,
            #     leader.get_controller_config().default_kd / 1000,
            #     0.0,
            #     0.0
            # )
            # leader.set_gain(gain)  # Set reduced damping coeffs. to leader only
            raise NotImplementedError("Joint leader action space not implemented yet.")
        elif leader_action_space == "cartesian":
            leader = Arx5CartesianController(
                pair['leader']['model'],
                pair['leader']['interface_name'],
                urdf_path,
            )
            gain = Gain(
                leader.get_controller_config().default_kp / 10000,
                leader.get_controller_config().default_kd / 1000,
                0.0,
                0.0
            )
            leader.set_gain(gain)  # Set reduced damping coeffs. to leader only
        
        leader.set_log_level(LogLevel.WARNING)
        follower.set_log_level(LogLevel.WARNING)
        leader.reset_to_home()
        follower.reset_to_home()
        controllers.append((leader, follower))
        controller_names.append((pair['leader']['name'], pair['follower']['name']))

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

    return controllers, controller_names

def initialize_cameras(config):
    pipelines = {}

    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        print(dev)
        dev.hardware_reset()

    def init_camera_pipeline(serial_no):
        # Create a RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial_no)
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgra8, 30)
        pipeline.start(config)
        # pipelines.append(pipeline)
        return pipeline

    for camera_dict in config['cameras']:
        camera_name = camera_dict["name"]
        camera_serial = camera_dict["serial"]
        pipeline = init_camera_pipeline(camera_serial)
        pipelines[camera_name] = pipeline

    return pipelines

def poll_joint_states(controller, joint_states_queue, stop_event, queue_event, rate = 0.01):
    """Poll joint states at default 100 Hz and store them in joint_states_queue."""
    while not stop_event.is_set():
        if queue_event.is_set():
            timestamp = controller.get_timestamp()
            if type(controller) is Arx5JointController:
                state = controller.get_state()
            elif type(controller) is Arx5CartesianController:
                state = controller.get_joint_state()
            else:
                raise TypeError("Controller must be either Arx5JointController or Arx5CartesianController.")
            joint_states_queue.put((timestamp, state))
        time.sleep(rate)

def separate_frame_capture_loop(pipeline, stop_event, queue_event, frames_queue, track_frame_index=True):
    """
    Capture frames from a _single_ camera pipeline and add them to the frames_queue.
    """
    frame_index = 0
    while not stop_event.is_set():
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Get timestamp and convert frame to numpy array.
        timestamp = color_frame.get_timestamp()
        color_image = np.asanyarray(color_frame.get_data())

        # If recording is active, add the frame and metadata to the queue.
        if queue_event.is_set():
            frames_queue.put((frame_index, timestamp, color_image))
            if track_frame_index:
                frame_index += 1

# TODO: Make these configurable.
# Maximum size of the frames and joint states queues.
FRAME_RATE = 120 # TODO: Set to maximum framerate of the cameras.
JOINT_RATE = 100 # TODO: Set to maximum joint state polling rate.
FRAME_MAX_SIZE = FRAME_RATE * 5
JOINT_MAX_SIZE = JOINT_RATE * 5

CONTROL_FREQ = 50 # Control loop frequency in Hz.
MAX_SECONDS = 20 # Maximum recording duration in seconds.
MAX_STEPS = MAX_SECONDS * CONTROL_FREQ  # Maximum number of steps to record.

MAX_EPISODES = 100 # Maximum number of episodes to record.

class ArxGym:
    def __init__(
            self, follower_controllers, pipelines, stop_recording_event, queue_event, 
            action_space="joint", ctrl_freq=50., track_frame_index=False,
        ):
        assert action_space in ["joint", "cartesian"], "Action space must be either 'joint' or 'cartesian'."
        for follower_name, follower_controller in follower_controllers.items():
            if action_space == "joint":
                controller_type = Arx5JointController
            elif action_space == "cartesian":
                controller_type = Arx5CartesianController
            assert isinstance(follower_controller, controller_type), \
                f"Follower controller {follower_name} must be an instance of Arx5CartesianController."

        self.action_space = action_space
        self.ctrl_freq = ctrl_freq
        self.track_frame_index = track_frame_index
        self.stop_recording_event = stop_recording_event

        self.follower_joint_states_queues = {}
        self.follower_controllers = follower_controllers
        
        # queue_event signals whether _all_ queues are being updated (frames _and_ states). Can be paused / resumed.
        self.queue_event = queue_event

        self.pipelines = pipelines
        if len(self.pipelines) != 0:
            self.frame_queues = {}
            for camera_name, pipeline in self.pipelines.items():
                # Create a queue for each camera's frames
                self.frame_queues[camera_name] = queue.Queue(maxsize=FRAME_MAX_SIZE)
                # Start a thread to capture frames from each camera pipeline
                t_camera = threading.Thread(
                    target=separate_frame_capture_loop,
                    args=(
                        pipeline, 
                        self.stop_recording_event, 
                        self.queue_event, 
                        self.frame_queues[camera_name], 
                        self.track_frame_index
                    )
                )
                t_camera.start()

        # Start polling joint states for each follower controller
        self.joint_state_threads = {}
        self.ctrl_stop_events = {}
        for follower_name, follower_controller in self.follower_controllers.items():
            # Create a stop event for each follower controller's control loop
            ctrl_stop_event = threading.Event()
            self.follower_joint_states_queues[follower_name] = queue.Queue(maxsize=JOINT_MAX_SIZE)
            t_follower = threading.Thread(
                target=poll_joint_states,
                args=(follower_controller, self.follower_joint_states_queues[follower_name], ctrl_stop_event, self.queue_event, 0.01)
            )
            t_follower.start()
            self.joint_state_threads[follower_name] = t_follower
            self.ctrl_stop_events[follower_name] = ctrl_stop_event

    def reset(self):
        for follower_name, follower_controller in self.follower_controllers.items():
            print(f"Resetting follower controller: {follower_name}")
            if self.action_space == "cartesian":
                # Only set to damping for cartesian control, as this causes joint control to flop :(
                follower_controller.set_to_damping()
            follower_controller.reset_to_home()

        return self.get_obs()  # Return the initial observation after reset

    def get_obs(self):
        # TODO: Properly format observation to be in-line with ALOHA / Moo Jin's HDF5 format.
        # assert self.queue_event.is_set(), "Frames are not being recorded. Call start_recording() first."
        if len(self.pipelines) != 0:
            assert not self.stop_recording_event.is_set(), "Recording has already ended."
            # raise NotImplementedError
            visual_obs = {}
            for camera_name, frames_queue in self.frame_queues.items():
                # Get the latest frame from the queue
                frame_index, timestamp, color_image = frames_queue.get()
                visual_obs[camera_name] = {
                    'frame_index': frame_index,
                    'timestamp': timestamp,
                    'color_image': cv2.cvtColor(color_image, cv2.COLOR_BGRA2RGB)
                }
        else:
            visual_obs = {}

        state_obs = {}
        for follower_name in self.follower_controllers.keys():
            follower_joint_states_queue = self.follower_joint_states_queues[follower_name]
            if not follower_joint_states_queue.empty():
                # Get the latest joint state from the queue
                timestamp, joint_state = follower_joint_states_queue.get()
                state_obs[follower_name] = {
                    'timestamp': timestamp,
                    'joint_state': joint_state
                }

        return {
            "images": visual_obs,
            "states": state_obs,
        }
    
    
    def start_recording(self):
        """Start recording frames and joint states."""
        if not self.queue_event.is_set():
            print("Starting recording...")
            self.queue_event.set()
        else:
            print("Recording is already active.")

    def stop_recording(self):
        """Stop recording frames and joint states."""
        if self.queue_event.is_set():
            print("Stopping recording...")
            self.queue_event.clear()
        else:
            print("Recording is not active.")

    def step(self, action):
        starting_time = time.time()

        if self.action_space == "joint":
            # raise NotImplementedError("Joint action space not implemented yet.")
            for follower_name, follower_controller in self.follower_controllers.items():
                # Convert action to joint state and set it to the follower controller
                follower_cmd = action[follower_name]
                # print(f"Setting joint command for follower {follower_name}: {follower_cmd.pos()}")
                follower_controller.set_joint_cmd(follower_cmd)
        elif self.action_space == "cartesian":
            for follower_name, follower_controller in self.follower_controllers.items():
                # Convert action to eef state and set it to the follower controller
                follower_cmd = action[follower_name]
                follower_controller.set_eef_cmd(follower_cmd)
        else:
            raise ValueError("Invalid action space. Must be either 'joint' or 'cartesian'.")
            
        elapsed_time = time.time() - starting_time
        if self.ctrl_freq > 0 and elapsed_time < (1 / self.ctrl_freq):
            time.sleep((1 / self.ctrl_freq) - elapsed_time)
        
        return self.get_obs(), None, None, {} # TODO: Implement proper return values for step method.


def main(args):
    follower_action_space = "joint"
    leader_action_space = "cartesian"
    no_record = True

    config_path = os.path.join(ROOT_DIR, "teleop", "arx5_config.json")
    config = load_robot_config(config_path)
    controllers, controller_names = initialize_controllers(
        config,
        follower_action_space=follower_action_space,
        leader_action_space=leader_action_space,
    )
    if no_record:
        print("Running without recording frames.")
        pipelines = {}
    else:
        pipelines = initialize_cameras(config)
        for camera_name, pipeline in pipelines.items():
            image = pipeline.wait_for_frames().get_color_frame()
            if image:
                image = np.asanyarray(image.get_data())
                print(f"Initial image from camera {camera_name} has shape: {image.shape}")


    stop_event = threading.Event()
    queue_event = threading.Event()

    leader_joint_state_threads = {}
    leader_joint_state_queues = {}
    follower_controllers = {}
    leader_controllers = {}
    for (leader_controller, follower_controller), (leader_name, follower_name) in zip(controllers, controller_names):
        leader_joint_state_queue = queue.Queue(maxsize=JOINT_MAX_SIZE)
        t_leader = threading.Thread(
            target=poll_joint_states,
            args=(leader_controller, leader_joint_state_queue, stop_event, queue_event, 0.01)
        )
        t_leader.start()

        leader_joint_state_threads[leader_name] = t_leader
        leader_joint_state_queues[leader_name] = leader_joint_state_queue

        leader_controllers[leader_name] = leader_controller
        follower_controllers[follower_name] = follower_controller

    # Initialize the ArxGym environment
    env = ArxGym(
        follower_controllers=follower_controllers,
        pipelines=pipelines,  # TODO: Pass initialized pipelines if cameras are used
        stop_recording_event=stop_event,
        queue_event=queue_event,
        action_space=follower_action_space,
        ctrl_freq=CONTROL_FREQ,
        track_frame_index=False,  # Set to True if frame index tracking is needed
    )
    env.start_recording()

    obs = env.reset()
    all_actions = np.load('../../open_loop_actions.npy', allow_pickle=True)[::2]
    try: 
        print("Starting the teleoperation loop. Press Ctrl+C to stop.")
        # while True:
        for t in tqdm(range(MAX_STEPS)):
            # if t % 5 == 0:
            #     # Save images
            #     images = []
            #     for camera_name, image_dict in obs["images"].items():
            #         image = image_dict["color_image"]
            #         images.append(image)
            #     # Combine images into a single image
            #     if len(images) > 0:
            #         image = np.concatenate(images, axis=1)
            #         all_images.append(Image.fromarray(image))

            action = {}
            for (leader_name, follower_name) in controller_names:
                # Get the leader controller's eef state

                follower_cmd = JointState(
                    all_actions[t][follower_name][:6].reshape(-1, 1), 
                    np.zeros(6).reshape(-1, 1),
                    np.zeros(6).reshape(-1, 1),
                    all_actions[t][follower_name][6].reshape(-1, 1),
                )

                follower_cmd.timestamp = 0.0  # Reset timestamp

                action[follower_name] = follower_cmd
            obs, reward, done, info = env.step(action)

    except KeyboardInterrupt:
        print("Interrupted by user, stopping...")
    
    finally:
        env.reset()
        stop_event.set()
        queue_event.clear()

        for leader, follower in controllers:
            leader.set_to_damping()
            leader.reset_to_home()
            # follower.set_to_damping()
            # follower.reset_to_home()


    # Stop all threads
    stop_event.set()
    queue_event.clear()

if __name__ == "__main__":
    # Define args
    parser = argparse.ArgumentParser(description="Run the ARX5 teleoperation recorder.")
    parser.add_argument("--no-record", action="store_true", help="Run without recording frames.")
    # TODO: Add side argument to specify which robot pair to control.
    # parser.add_argument("--side", type=str, choices=["left", "right", "both"], help="Specify the side of the robot to control.")
    parser.add_argument("--follower-action-space", type=str, choices=["joint", "cartesian"], default="joint",
                        help="Action space for the follower controller (default: joint).")
    parser.add_argument("--leader-action-space", type=str, choices=["joint", "cartesian"], default="cartesian",
                        help="Action space for the leader controller (default: cartesian).")
    args = parser.parse_args()
    main(args)