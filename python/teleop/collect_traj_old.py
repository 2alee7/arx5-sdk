import os
import sys
import threading
import queue
import time
import signal
import argparse
import numpy as np
import cv2
from teleop.utils.realsense_utils import init_synced_cameras
from teleop.utils.teleop_utils import (load_robot_config, initialize_controllers, poll_joint_states, save_frames_and_metadata, get_next_traj_folder)
from teleop.utils.control_loops import control_loop_open

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

parser = argparse.ArgumentParser(description="Specify single side, robot or controller")
parser.add_argument("--right", action="store_true")
parser.add_argument("--left", action="store_true")
parser.add_argument("--robot", type=int)
parser.add_argument("--controller", type=int)
args = parser.parse_args()

def main():
    config = load_robot_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'arx5_config.json'))
    controllers = initialize_controllers(config, side='right' if args.right else 'left' if args.left else None)
    # pipelines = initialize_cameras(config)

    stop_event = threading.Event()
    recording_event = threading.Event()
    frames_queue = queue.Queue()
    latest_frames = {}

    def handle_sigint(sig, frame):
        stop_event.set()
        print("Interrupted by user, stopping...")

    signal.signal(signal.SIGINT, handle_sigint)

    joint_state_threads = []
    control_threads = []
    control_stop_events = []

    for leader, follower in controllers:
        joint_states_queue = queue.Queue()

        for ctrl in (leader, follower):
            t = threading.Thread(
                target=poll_joint_states,
                args=(ctrl, joint_states_queue, stop_event, recording_event)
            )
            t.start()
            joint_state_threads.append(t)

        control_stop_event = threading.Event()
        control_stop_events.append(control_stop_event)
        t = threading.Thread(
            target=control_loop_open,
            args=(leader, follower, control_stop_event)
        )
        t.start()
        control_threads.append(t)

    try:
        cv2.namedWindow("Camera Views", cv2.WINDOW_NORMAL)
        while not stop_event.is_set():
            frames = []
            for i in range(4):
                frame = latest_frames.get(i, np.zeros((480, 848, 3), dtype=np.uint8))
                if frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                frames.append(frame)

            row1 = cv2.hconcat([frames[0], frames[1]])
            row2 = cv2.hconcat([frames[3], frames[2]])
            composite = cv2.vconcat([row1, row2])

            labels = ["top_vew", "front_view", "wrist_left", "wrist_right"]
            positions = [(0, 480), (848, 480), (0, 960), (848, 960)]
            for label, pos in zip(labels, positions):
                cv2.putText(composite, label, pos, cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)

            cv2.imshow("Camera Views", composite)
            key = cv2.waitKey(1) & 0xFF
            if key == 32:
                if recording_event.is_set():
                    print("Recording stopped...")
                    recording_event.clear()
                else:
                    print("Recording started...")
                    recording_event.set()
            elif key == 27:
                stop_event.set()
                break

            if not recording_event.is_set() and not frames_queue.empty():
                print("Recording session ended. Press 's' to save or any other key to discard.")
                key = cv2.waitKey(0) & 0xFF
                traj_path = get_next_traj_folder("observations")
                if key == ord('s'):
                    save_frames_and_metadata(frames_queue, traj_path)
                    print(f"Recording saved as {traj_path}.")
                else:
                    while not frames_queue.empty():
                        frames_queue.get()
                    print("Recording discarded.")

            time.sleep(0.1)

    finally:

        stop_event.set()
        cv2.destroyAllWindows()
        recording_event.clear()

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
