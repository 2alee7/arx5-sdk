import threading
import signal
import argparse
import os
import shutil
import cv2
import numpy as np
import sys
from pynput import keyboard
import time

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from arx5_interface import Arx5CartesianController

from teleop.utils.realsense_utils import init_cameras, pop_latest_frames
from teleop.utils.teleop_utils import (load_robot_config, initialize_controllers, poll_joint_states)
from teleop.utils.control_loops import control_loop_open

def control_loop_poll(leader_controller: Arx5CartesianController, 
                     follower_controller: Arx5CartesianController, stop_event, recording_event, states=None, 
                     data_path=None, pause_event=None):
    
    # Loop timing control - 50Hz
    target_loop_time = 0.02  # 50Hz
    
    while not (pause_event.is_set() or stop_event.is_set()):
        loop_start = time.time()
        
        try:
            # Get leader state
            leader_state = ((leader_controller.get_eef_state(), leader_controller.get_joint_state()))
            follower_state = ((follower_controller.get_eef_state(), follower_controller.get_joint_state()))
            
            # If recording, capture frameset and save state
            if recording_event.is_set() and leader_state is not None:
                timestamp = leader_controller.get_timestamp()
                
                frameset = pop_latest_frames()
                
                if frameset is not None:
                    states.append({
                        'timestamp': timestamp,
                        'leader_state': leader_state,
                        'follower_state': follower_state,
                        'frames': frameset
                    })
            
            # Send leader state to follower
            follower_cmd = leader_state  # Creates a copy
            follower_cmd.gripper_pos *= 4.8
            follower_cmd.timestamp = 0.0
            follower_controller.set_eef_cmd(follower_cmd)
            
        except Exception as e:
            print(f"Error in control loop: {e}")
        
        elapsed = time.time() - loop_start
        sleep_time = max(0, target_loop_time - elapsed)
        time.sleep(sleep_time)
        
        if elapsed > target_loop_time:
            print(f"Warning: Control loop taking longer than target: {elapsed:.4f}s")
    
    if stop_event.is_set():
        print("Control loop stopped.")
    else:
        print("Control loop paused.")

def main():
    parser = argparse.ArgumentParser("collect_traj")
    parser.add_argument("--config",   type=str, required=True)
    parser.add_argument("--task",     type=str, required=True)
    parser.add_argument("--timesteps",type=int, required=True)
    parser.add_argument("--path",     type=str, required=True)
    parser.add_argument("--side",     type=str, choices=["left","right"], required=True)
    args = parser.parse_args()

    cfg = load_robot_config(os.path.abspath(args.config))

    states = []

    # camera init
    RES = (848, 480)
    FPS = 60
    cam_workers, syncer = init_cameras(cfg, RES, FPS)

    # setup robot + joint‐state polling
    controllers = initialize_controllers(cfg, side=args.side)
    stop_event      = threading.Event()
    recording_event = threading.Event()

    # # start joint‐state polling threads (they will only log when recording_event is set)
    # for ctrl in controllers:
    #     t = threading.Thread(
    #         target=poll_joint_states,
    #         args=(ctrl, args.path, stop_event, recording_event),
    #         daemon=True
    #     )
    #     t.start()

    for ctrl in controllers:
        t = threading.Thread(
            target=control_loop_poll,
            args=(ctrl[0], ctrl[1], stop_event, recording_event, states),
            daemon=True
        )
        t.start()

    # keys: ENTER to start/stop, SPACE to pause/resume, CTRL+C to exit
    def on_press(key):
        if key == keyboard.Key.space:
            if recording_event.is_set():
                recording_event.clear()
                print("  [PAUSED]")
            else:
                recording_event.set()
                print("  [RESUMED]")
    listener = keyboard.Listener(on_press=on_press)
    listener.daemon = True
    listener.start()

    signal.signal(signal.SIGINT, lambda *_: stop_event.set())

    print("Ready to record. Press ENTER to begin an episode.")
    episode = 0
    try:
        while not stop_event.is_set():
            _ = input()  # ENTER
            ep_dir = os.path.join(args.path, args.task, args.side, f"episode_{episode:03d}")
            # prepare folders
            for _, name in cam_workers:
                os.makedirs(os.path.join(ep_dir, name), exist_ok=True)
            print(f"  Recording episode {episode:03d} … [SPACE=pause/resume]")
            recording_event.set()

            count = 0
            while recording_event.is_set() and count < args.timesteps and not stop_event.is_set():
                synced = syncer.next_synced_frames()
                # synced is list of (ts,img)
                for (ts, img), (_, name) in zip(synced, cam_workers):
                    fname = os.path.join(ep_dir, name, f"{count:06d}.png")
                    cv2.imwrite(fname, img)
                count += 1

            recording_event.clear()
            print(f"  Episode ended ({count} frames). Save? (Y/N)")
            ans = input().strip().lower()
            if ans == "y":
                episode += 1
                print(f"  Saved as episode_{episode-1:03d}\nPress ENTER to start next…")
            else:
                shutil.rmtree(ep_dir)
                print("  Discarded. Press ENTER to retry…")

    finally:
        stop_event.set()
        listener.stop()
        for w, _ in cam_workers:
            w.stop()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()