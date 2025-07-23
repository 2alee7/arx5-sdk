import time
import threading
import argparse
import os
from collections import deque
import pyrealsense2 as rs
import sys
import numpy as np
import cv2
from multiprocessing.managers import SharedMemoryManager
from pynput import keyboard
import signal

# Add python directory to path
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from teleop.utils.teleop_utils import load_robot_config
from shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from shared_memory.shared_memory_util import ArraySpec

# Frame sync parameters
FRAME_PERIOD = 1.0 / 60.0
SYNC_TOLERANCE = FRAME_PERIOD / 2 # +/- 8.33ms tolerance @ 60FPS

buffers = {}
locks = {}
recording_event = threading.Event()
stop_event = threading.Event()

class CameraWorker(threading.Thread):
    def __init__(self, serial, cfg, buffer, lock):
        super().__init__(daemon=True)
        self.serial = serial
        self.cfg = cfg
        self.buffer = buffer
        self.lock = lock
        self.pipeline = rs.pipeline()
        self.running = True
        self.init_ts = None

    def run(self):
        self.pipeline.start(self.cfg)
        while self.running and not stop_event.is_set():
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            color = frames.get_color_frame()
            if not color:
                continue
            ts = color.get_timestamp() / 1000.0  # seconds
            if self.init_ts is None:
                self.init_ts = ts
            rel_ts = ts - self.init_ts
            with self.lock:
                self.buffer.append((rel_ts, color))

    def stop(self):
        self.running = False
        self.pipeline.stop()


def sample_within(cam_id, t_n, delta):
    # Sample a frame from the buffer for the given camera ID about target time t_n
    with locks[cam_id]:
        buf = list(buffers[cam_id])
    best = min(buf, key=lambda x: abs(x[0] - t_n), default=None)
    if best and abs(best[0] - t_n) <= delta:
        return best[1]
    return None

def sync_loop(cam_ids, t0, sync_buffer):
    n = 0
    while not stop_event.is_set():
        t_n = t0 + n * FRAME_PERIOD
        sleep = t_n - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            t_n = time.monotonic()

        frameset = []
        for cam in cam_ids:
            frame = sample_within(cam, t_n, SYNC_TOLERANCE)
            if frame is None:
                break
            frame_data = frame.get_data()
            img = np.asanyarray(frame_data)
            frameset.append(img)
        
        if len(frameset) == len(cam_ids):
            # Convert frames to dictionary for SharedMemoryRingBuffer
            data = {
                'timestamp': t_n,
                'frames': np.stack(frameset)  # Stack all frames into one array
            }
            try:
                sync_buffer.put(data)
                n += 1
            except TimeoutError as e:
                print(f"Put too fast: {e}")
        else:
            time.sleep(0.001)

def recorder(episode_path, cam_ids, target_len, sync_buffer):
    count = 0
    while not stop_event.is_set():
        if not recording_event.is_set():
            time.sleep(0.1)
            continue
            
        try:
            # Get latest data from the buffer
            data = sync_buffer.get_last_k(k=1)
            if data is None:
                time.sleep(0.01)
                continue
                
            t_n = data['timestamp']
            frames = data['frames']
            
            for i, frame in enumerate(frames):
                cam_path = os.path.join(episode_path, f"view_{cam_ids[i]}")
                os.makedirs(cam_path, exist_ok=True)
                filepath = os.path.join(cam_path, f"{count:06d}.png")
                cv2.imwrite(filepath, frame)
            
            count += 1
            if count >= target_len:
                recording_event.clear()
                count = 0
                
        except Exception as e:
            print(f"Error in recorder: {e}")
            time.sleep(0.01)


def on_press(key):
    if key == keyboard.Key.space:
        if recording_event.is_set():
            recording_event.clear()
            print("Recording paused")
        else:
            recording_event.set()
            print("Recording started")


def main():
    parser = argparse.ArgumentParser("Master-sync timed robot control test")
    parser.add_argument("--config", type=str, default="teleop/test/arx5_config_test.json")
    args = parser.parse_args()

    config = load_robot_config(os.path.abspath(args.config))

    serials = [cam['serial'] for cam in config['cameras']]   
    width, height, fps = 640, 480, 60
    cam_ids = list(range(len(serials)))

    # Initialize shared memory manager
    shm_mgr = SharedMemoryManager()
    shm_mgr.start()

    # Create example data for the shared buffer
    example_frame = np.zeros((len(serials), height, width, 3), dtype=np.uint8)
    example_data = {
        'timestamp': 0.0,
        'frames': example_frame
    }

    # Create SharedMemoryRingBuffer
    sync_buffer = SharedMemoryRingBuffer.create_from_examples(
        shm_manager=shm_mgr,
        examples=example_data,
        get_max_k=10,           # Maximum number of frames to retrieve at once
        get_time_budget=0.01,   # Maximum time for retrieval operations
        put_desired_frequency=fps  # Target update frequency
    )

    workers = []
    for i, s in enumerate(serials):
        cfg = rs.config()
        cfg.enable_device(s)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        buf = deque(maxlen=10)
        lk = threading.Lock()
        buffers[i] = buf
        locks[i] = lk
        worker = CameraWorker(s, cfg, buf, lk)
        worker.start()
        workers.append(worker)

    # Wait for all workers to emit their first frame
    first_ts = {}
    while len(first_ts) < len(workers):
        for i in range(len(workers)):
            with locks[i]:
                if i not in first_ts and buffers[i]:
                    first_ts[i] = buffers[i][0][0]
        time.sleep(0.01)

    t0 = max(first_ts.values())

    # Start sync and recorder threads
    sync_thread = threading.Thread(
        target=sync_loop, 
        args=(cam_ids, t0, sync_buffer), 
        daemon=True
    )
    sync_thread.start()
    
    recorder_thread = threading.Thread(
        target=recorder, 
        args=("episode_data", cam_ids, 100, sync_buffer), 
        daemon=True
    )
    recorder_thread.start()

    # Start keyboard listener
    listener = keyboard.Listener(on_press=on_press)
    listener.daemon = True
    listener.start()

    # Setup signal handler
    signal.signal(signal.SIGINT, lambda sig, frame: stop_event.set())
    
    print("Running. Press SPACE to start/stop recording, Ctrl-C to exit.")
    try:
        stop_event.wait()
    except KeyboardInterrupt:
        stop_event.set()
    
    print("Shutting down...")
    for w in workers:
        w.stop()
    
    # Clean up shared memory
    shm_mgr.shutdown()


if __name__ == "__main__":
    main()