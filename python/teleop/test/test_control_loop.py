import os
import sys
import signal
import threading
import argparse
from pynput import keyboard


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from utils.teleop_utils import load_robot_config, initialize_controllers, poll_joint_states
from utils.control_loops import control_loop_open


def control_loop_runner(leader, follower, stop_event, pause_event):
    """
    Wraps control_loop_open to allow pausing/resuming via pause_event.
    """
    while not stop_event.is_set():
        if pause_event.is_set():
            pause_event.wait(timeout=0.1)
            continue
        control_loop_open(leader, follower, pause_event)

def main():
    parser = argparse.ArgumentParser("Minimal test for robot control loop with pause/resume")
    parser.add_argument(
        "--config", type=str,
        default="teleop/test/arx5_config_test.json",
        help="Path to robot config JSON"
    )
    parser.add_argument(
        "--side", type=str,
        choices=["left", "right", ],
        default=None,
        help="Which side to test"
    )
    args = parser.parse_args()

    # Load configuration and initialize controllers
    config = load_robot_config(os.path.abspath(args.config))
    controllers = initialize_controllers(config, side=args.side)

    # Control flags
    stop_event = threading.Event()
    pause_event = threading.Event()
    recording_event = threading.Event()  # for poll_joint_states

    # Ctrl-C handler
    signal.signal(signal.SIGINT, lambda sig, frame: stop_event.set())

    # Toggle pause via keyboard
    def on_press(key):
        if key == keyboard.Key.space:
            if pause_event.is_set():
                pause_event.clear()
                print("Resuming control loops")
            else:
                pause_event.set()
                print("Pausing control loops")

    listener = keyboard.Listener(on_press=on_press)
    listener.daemon = True
    listener.start()

    # Start threads
    threads = []
    for leader, follower in controllers:
        for ctrl in (leader, follower):
            t = threading.Thread(
                target=poll_joint_states,
                args=(ctrl, None, stop_event, recording_event),
            )
            t.daemon = True
            t.start()
            threads.append(t)

        ctl_thread = threading.Thread(
            target=control_loop_runner,
            args=(leader, follower, stop_event, pause_event),
        )
        ctl_thread.daemon = True
        ctl_thread.start()
        threads.append(ctl_thread)

    print("Control loops running. Press SPACE to pause/resume, Ctrl-C to quit.")
    stop_event.wait()

    # Cleanup
    listener.stop()
    print("Stopping all threads...")
    stop_event.set()
    for t in threads:
        t.join()

    # Return robots to damping/home
    for leader, follower in controllers:
        leader.set_to_damping()
        follower.set_to_damping()
        leader.reset_to_home()
        follower.reset_to_home()

    print("Shutdown complete.")

if __name__ == "__main__":
    main()
