import time
import threading
import argparse
import os
import statistics
from collections import deque
import bisect
import pyrealsense2 as rs

# Import robot config loader
from teleop_utils import load_robot_config

# --- Camera Worker & Synchronizer ---
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
        while self.running:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            color = frames.get_color_frame()
            if not color:
                continue
            ts = color.get_timestamp() / 1000.0  # seconds
            if self.init_ts is None:
                self.init_ts = ts
            rel_ts = ts - self.init_ts
            with self.lock:
                # maintain sorted buffer for bisect searches
                self.buffer.append(rel_ts)

    def stop(self):
        self.running = False
        self.pipeline.stop()

class FrameSynchronizer:
    def __init__(self, buffers, locks, tol):
        self.buffers = buffers
        self.locks = locks
        self.tol = tol

    def next_synced(self):
        while True:
            ts_list = []
            for buf, lk in zip(self.buffers, self.locks):
                with lk:
                    if not buf:
                        break
                    ts_list.append(buf[0])
            else:
                min_ts, max_ts = min(ts_list), max(ts_list)
                if max_ts - min_ts <= self.tol:
                    synced = []
                    for buf, lk in zip(self.buffers, self.locks):
                        with lk:
                            synced.append(buf.popleft())
                    return synced
                # drop oldest frame(s)
                for i, t in enumerate(ts_list):
                    if t == min_ts:
                        with self.locks[i]:
                            self.buffers[i].popleft()
            time.sleep(0.001)

# --- Test Procedures ---

def init_workers(serials, width, height, fps):
    buffers, locks, workers = [], [], []
    for s in serials:
        cfg = rs.config()
        cfg.enable_device(s)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        buf = deque()
        lk = threading.Lock()
        w = CameraWorker(s, cfg, buf, lk)
        workers.append(w)
        buffers.append(buf)
        locks.append(lk)
        w.start()
    return workers, buffers, locks


def test_option1(buffers, locks, tol):
    # Use buffer 0 as reference
    with locks[0]:
        ref_ts = list(buffers[0])
    success_count = 0
    total = len(ref_ts)
    # prepare sorted lists for others
    other_ts = []
    for buf, lk in zip(buffers[1:], locks[1:]):
        with lk:
            other_ts.append(sorted(buf))
    # for each reference timestamp, check if each other camera has a timestamp within tol
    for t in ref_ts:
        found = True
        for ts_list in other_ts:
            # binary search
            idx = bisect.bisect_left(ts_list, t)
            match = False
            for j in (idx-1, idx, idx+1):
                if 0 <= j < len(ts_list) and abs(ts_list[j] - t) <= tol:
                    match = True
                    break
            if not match:
                found = False
                break
        if found:
            success_count += 1
    rate = success_count / total if total else 0
    return total, success_count, rate


def test_option2(sync, duration):
    start = time.time()
    count = 0
    while time.time() - start < duration:
        sync.next_synced()
        count += 1
    return count


def main():
    parser = argparse.ArgumentParser(description="Realsense Sync Test")
    parser.add_argument('--option', choices=['1','2'], required=True)
    parser.add_argument('--duration', type=float, default=20.0, help='Test duration in seconds')
    parser.add_argument('--tol', type=float, default=0.05, help='Sync tolerance in seconds')
    parser.add_argument('--width', type=int, default=848)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=30)
    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(base_dir, 'arx5_config.json')
    config = load_robot_config(config_path)
    serials = [cam['serial'] for cam in config['cameras']]
    print(f"Using camera serials from config: {serials}")

    workers, buffers, locks = init_workers(serials, args.width, args.height, args.fps)

    print("Warming up for 5 seconds...")
    time.sleep(5)

    if args.option == '1':
        print("Running Option 1: reference-based matching...")
        time.sleep(args.duration)
        total, success, rate = test_option1(buffers, locks, args.tol)
        print(f"Total ref frames: {total}")
        print(f"Fully aligned sets within {args.tol}s: {success}")
        print(f"Alignment success rate: {rate*100:.2f}%")
    else:
        sync = FrameSynchronizer(buffers, locks, args.tol)
        print("Running Option 2: synchronized capture on-the-fly...")
        count = test_option2(sync, args.duration)
        print(f"Aligned frame sets captured: {count}")
        print(f"Average rate: {count/args.duration:.2f} sets/sec")

    print("Stopping workers...")
    for w in workers:
        w.stop()

if __name__ == '__main__':
    main()
