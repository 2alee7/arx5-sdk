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
import signal
from contextlib import contextmanager

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
print("ROOT_DIR", ROOT_DIR)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# OFFLINE flag for testing without hardware
OFFLINE = False

if not OFFLINE:
    import pyrealsense2 as rs
    from arx5_interface import Arx5CartesianController, Arx5JointController, Gain, LogLevel
else:
    from offline_mocks import MockController, MockJointState, MockEefState, MockCamera, MockRs
    rs = MockRs

# Import openpi client for policy server communication
try:
    from openpi_client import websocket_client_policy
    from openpi_client import image_tools
except ImportError:
    print("Warning: openpi_client not found. Please install it to use policy inference.")
    print("You can install it with: pip install -e openpi/packages/openpi-client")
    websocket_client_policy = None
    image_tools = None

@contextmanager
def prevent_keyboard_interrupt():
    """Context manager to prevent KeyboardInterrupt during critical operations."""
    def signal_handler(signum, frame):
        pass  # Do nothing, just prevent the interrupt
    
    # Store the original signal handler
    original_handler = signal.signal(signal.SIGINT, signal_handler)
    
    try:
        yield
    finally:
        # Restore the original signal handler
        signal.signal(signal.SIGINT, original_handler)

"""
Note: All text prompts require the OpenCV window to be in focus. Sorry.
"""


def load_robot_config(config_path):
    with open(config_path, 'r') as file:
        config = json.load(file)
    return config

def initialize_controllers(config, follower_action_space="cartesian"):
    controllers = []
    controller_names = []
    urdf_path = "../models/arx5.urdf"

    def create_controller(arm):
        if OFFLINE:
            follower = MockController(arm['name'])
        else:
            if follower_action_space == "joint":
                follower = Arx5JointController(
                    arm['model'],
                    arm['interface_name'],
                )
                follower.enable_background_send_recv()
                # Do NOT turn on gravity compensation for follower when using joint control.
                follower.enable_gravity_compensation(urdf_path)
            elif follower_action_space == "cartesian":
                follower = Arx5CartesianController(
                    arm['model'],
                    arm['interface_name'],
                    urdf_path,
                )

            follower.set_log_level(LogLevel.WARNING)
        follower.reset_to_home()
        controllers.append(follower)
        controller_names.append(arm['name'])

    for arm in config['arms']:
        create_controller(arm)

    return controllers, controller_names

def initialize_cameras(config):
    pipelines = {}

    if not OFFLINE:
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
            return pipeline

        for camera_dict in config['cameras']:
            camera_name = camera_dict["name"]
            camera_serial = camera_dict["serial"]
            pipeline = init_camera_pipeline(camera_serial)
            pipelines[camera_name] = pipeline
    else:
        # Use mock cameras for offline testing
        print("Using mock cameras for offline testing")
        for camera_dict in config['cameras']:
            camera_name = camera_dict["name"]
            pipeline = MockCamera(camera_name)
            pipelines[camera_name] = pipeline

    return pipelines

def poll_joint_states(controller, joint_states_queue, stop_event, queue_event, rate = 0.01):
    """Poll joint states at default 100 Hz and store them in joint_states_queue."""
    while not stop_event.is_set():
        if queue_event.is_set():
            timestamp = controller.get_timestamp()
            if OFFLINE:
                state = controller.get_joint_state()
            elif type(controller) is Arx5JointController:
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
DROID_CONTROL_FREQUENCY = 10  # DROID control frequency in Hz for consistent timing
MAX_SECONDS = 20 # Maximum recording duration in seconds.
MAX_STEPS = MAX_SECONDS * CONTROL_FREQ  # Maximum number of steps to record.

MAX_EPISODES = 100 # Maximum number of episodes to record.

class ArxGym:
    def __init__(
            self, follower_controllers, pipelines, stop_recording_event, queue_event, 
            action_space="joint", ctrl_freq=50., track_frame_index=False, instruction=None,
        ):
        # assert action_space in ["joint", "cartesian"], "Action space must be either 'joint' or 'cartesian'."
        # for follower_name, follower_controller in follower_controllers.items():
        #     if action_space == "joint":
        #         controller_type = Arx5JointController
        #     elif action_space == "cartesian":
        #         controller_type = Arx5CartesianController
        #     assert isinstance(follower_controller, controller_type), \
        #         f"Follower controller {follower_name} must be an instance of Arx5CartesianController."

        self.action_space = action_space
        self.ctrl_freq = ctrl_freq
        self.track_frame_index = track_frame_index
        self.stop_recording_event = stop_recording_event
        self.instruction = instruction
        self.follower_joint_states_queues = {}
        self.follower_controllers = follower_controllers
        
        # queue_event signals whether _all_ queues are being updated (frames _and_ states). Can be paused / resumed.
        self.queue_event = queue_event

        self.pipelines = pipelines
        self.camera_threads = []
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
                self.camera_threads.append(t_camera)

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
            follower_controller.set_to_damping()
            follower_controller.reset_to_home()

        return self.get_obs()  # Return the initial observation after reset

    def get_obs(self):
        if self.stop_recording_event.is_set():
                return None
        
        if len(self.pipelines) != 0:
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
    
    def format_observation_for_policy(self, obs):
        """Format observation for policy server input."""
        return {
            "images": obs["images"],
            "states": obs["states"],
            "prompt": self.instruction,
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
            for follower_name, follower_controller in self.follower_controllers.items():
                # Get joint command for this follower
                joint_cmd = action[follower_name]
                # Set joint velocity command to the follower controller
                follower_controller.set_joint_cmd(joint_cmd)
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
    follower_action_space = args.follower_action_space
    no_record = args.no_record

    config_path = os.path.join(ROOT_DIR, "python", "policy_inference", "arx5_config.json")
    config = load_robot_config(config_path)
    controllers, controller_names = initialize_controllers(
        config,
        follower_action_space=follower_action_space,
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

    if websocket_client_policy is None:
        print("Error: openpi_client not available. Please install it first.")
        return

    # Connect to policy server
    print(f"Connecting to policy server at {args.policy_host}:{args.policy_port}")
    policy_client = websocket_client_policy.WebsocketClientPolicy(args.policy_host, args.policy_port)
    print(f"Connected to policy server. Metadata: {policy_client.get_server_metadata()}")

    stop_event = threading.Event()
    queue_event = threading.Event()

    follower_controllers = {}
    for follower_controller, follower_name in zip(controllers, controller_names):
        follower_controllers[follower_name] = follower_controller

    # Initialize the ArxGym environment
    env = ArxGym(
        follower_controllers=follower_controllers,
        pipelines=pipelines,
        stop_recording_event=stop_event,
        queue_event=queue_event,
        action_space=follower_action_space,
        ctrl_freq=CONTROL_FREQ,
        track_frame_index=False,
    )

    # Create results dataframe
    import pandas as pd
    df = pd.DataFrame(columns=["success", "duration", "video_filename"])

    while True:
        # Get instruction from user
        instruction = input("Enter instruction: ")
        env.instruction = instruction

        # Rollout parameters
        actions_from_chunk_completed = 0
        pred_action_chunk = None

        # Prepare to save video of rollout
        timestamp = datetime.now().strftime("%Y_%m_%d_%H:%M:%S")
        video = []
        
        # Start recording
        env.start_recording()
        obs = env.reset()
        
        print("Running rollout... press Ctrl+C to stop early.")
        bar = tqdm(range(args.max_timesteps))
        
        for t_step in bar:
            start_time = time.time()
            try:
                # Save video frame
                if len(obs["images"]) > 0:
                    # Get first camera image for video
                    first_camera = list(obs["images"].keys())[0]
                    video.append(obs["images"][first_camera]["color_image"])

                # Send websocket request to policy server if it's time to predict a new chunk
                if actions_from_chunk_completed == 0 or actions_from_chunk_completed >= args.open_loop_horizon:
                    actions_from_chunk_completed = 0

                    # Format observation for policy
                    request_data = env.format_observation_for_policy(obs)
                    
                    # Wrap the server call in a context manager to prevent Ctrl+C from interrupting it
                    # Ctrl+C will be handled after the server call is complete
                    with prevent_keyboard_interrupt():
                        # Get action chunk from policy server
                        try:
                            pred_action_chunk = policy_client.infer(request_data)["actions"]
                            print(f"Received action chunk of shape: {pred_action_chunk.shape}")
                        except Exception as e:
                            print(f"Error getting action from policy server: {e}")
                            break

                # Select current action to execute from chunk
                if pred_action_chunk is not None:
                    action = pred_action_chunk[actions_from_chunk_completed]
                    actions_from_chunk_completed += 1

                    # Convert action to robot commands for each follower arm
                    robot_action = {}
                    for follower_name in controller_names:
                        # DROID outputs 8D actions, but ARX5 needs 6 DOF + 1 gripper
                        # action shape: [joint_vel_1, joint_vel_2, ..., joint_vel_6, gripper_pos, unused]
                        joint_velocities = action[:6]  # First 6 dimensions are joint velocities (6 DOF)
                        gripper_position = action[6]   # 7th dimension is gripper position
                        
                        if OFFLINE:
                            # Create mock joint command for offline testing
                            from offline_mocks import MockJointCommand
                            joint_cmd = MockJointCommand()
                            joint_cmd.vel = joint_velocities
                            joint_cmd.gripper_pos = gripper_position
                        else:
                            # Create joint command with velocities
                            from arx5_interface import JointCommand
                            joint_cmd = JointCommand()
                            joint_cmd.vel = joint_velocities  # Set joint velocities (6 DOF)
                            joint_cmd.pos = None  # Don't set position (velocity control)
                        
                        # Binarize gripper action like DROID does
                        if gripper_position > 0.5:
                            gripper_position = 1.0
                        else:
                            gripper_position = 0.0
                        
                        # Set gripper position in the command
                        joint_cmd.gripper_pos = gripper_position
                        
                        # Clip all dimensions of action to [-1, 1] like DROID
                        joint_cmd.vel = np.clip(joint_cmd.vel, -1, 1)
                        
                        robot_action[follower_name] = joint_cmd
                    
                    obs, reward, done, info = env.step(robot_action)

                # Sleep to match DROID data collection frequency
                elapsed_time = time.time() - start_time
                if elapsed_time < 1 / DROID_CONTROL_FREQUENCY:
                    time.sleep(1 / DROID_CONTROL_FREQUENCY - elapsed_time)
                    
            except KeyboardInterrupt:
                break

        # Save video
        if len(video) > 0:
            try:
                from moviepy.editor import ImageSequenceClip
                video = np.stack(video)
                save_filename = "video_" + timestamp
                ImageSequenceClip(list(video), fps=10).write_videofile(save_filename + ".mp4", codec="libx264")
                print(f"Video saved as {save_filename}.mp4")
            except ImportError:
                print("moviepy not available, saving individual frames instead")
                os.makedirs("frames", exist_ok=True)
                for i, frame in enumerate(video):
                    Image.fromarray(frame).save(f"frames/frame_{i:04d}.png")
                save_filename = f"frames_{timestamp}"
        else:
            save_filename = "no_video_" + timestamp

        # Get success evaluation from user
        success: str | float | None = None
        while not isinstance(success, float):
            success = input(
                "Did the rollout succeed? (enter y for 100%, n for 0%), or a numeric value 0-100 based on the evaluation spec: "
            )
            if success == "y":
                success = 1.0
            elif success == "n":
                success = 0.0
            else:
                try:
                    success = float(success) / 100
                    if not (0 <= success <= 1):
                        print(f"Success must be a number in [0, 100] but got: {success * 100}")
                        success = None
                except ValueError:
                    print("Please enter 'y', 'n', or a number between 0-100")
                    success = None

        # Add result to dataframe
        df = pd.concat([df, pd.DataFrame([{
            "success": success,
            "duration": t_step,
            "video_filename": save_filename,
        }])], ignore_index=True)

        # Ask if user wants to do another evaluation
        if input("Do one more eval? (enter y or n): ").lower() != "y":
            break
        
        # Reset environment for next episode
        env.reset()

    # Save results
    os.makedirs("results", exist_ok=True)
    timestamp = datetime.now().strftime("%I:%M%p_%B_%d_%Y")
    csv_filename = os.path.join("results", f"eval_{timestamp}.csv")
    df.to_csv(csv_filename)
    print(f"Results saved to {csv_filename}")

    # Cleanup
    print("Cleaning up...")
    stop_event.set()
    queue_event.clear()
    env.reset()

    # Kill all polling and camera threads
    for t in env.joint_state_threads.values():
        t.join(timeout=0.5)
    for t in env.camera_threads:
        t.join(timeout=0.5)

    print("Done.")


if __name__ == "__main__":
    # Define args
    parser = argparse.ArgumentParser(description="Run the ARX5 teleoperation recorder.")
    parser.add_argument("--no-record", action="store_true", help="Run without recording frames.")
    # TODO: Add side argument to specify which robot pair to control.
    # parser.add_argument("--side", type=str, choices=["left", "right", "both"], help="Specify the side of the robot to control.")
    parser.add_argument("--follower-action-space", type=str, choices=["joint", "cartesian"], default="joint",
                        help="Action space for the follower controller (default: joint).")
    parser.add_argument("--policy-host", type=str, default="localhost", help="Policy server host (default: localhost).")
    parser.add_argument("--policy-port", type=int, default=8000, help="Policy server port (default: 8000).")
    parser.add_argument("--max-timesteps", type=int, default=600, help="Maximum number of timesteps to record (default: 600).")
    parser.add_argument("--open-loop-horizon", type=int, default=8, help="Open loop horizon for policy (default: 8).")
    parser.add_argument("--save-video", action="store_true", help="Save video to disk.")
    parser.add_argument("--video-output-dir", type=str, default="./policy_recordings", help="Output directory for video (default: ./policy_recordings).")
    args = parser.parse_args()
    main(args)