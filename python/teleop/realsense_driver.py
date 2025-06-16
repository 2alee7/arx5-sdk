import threading
from collections import deque
import pyrealsense2 as rs
import numpy as np
import time

class CamWorker(threading.Thread):
    # Worker thread to capture frames from a single RealSense D405
    def __init__(self, serial, cfg, buffer, lock):
        super().__init__(daemon=True)
        self.serial = serial
        self.cfg = cfg
        self.buffer = buffer
        self.lock = lock
        self.pipeline = rs.pipeline()
        self.running = True

    def run(self):
        self.pipeline.start(self.cfg)
        while self.running:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            timestamp = color_frame.get_timestamp() / 1000.0  # seconds
            color_image = np.asanyarray(color_frame.get_data())
            with self.lock:
                self.buffer.append((timestamp, color_image))

    def stop(self):
        self.running = False
        self.pipeline.stop()

class FrameSynchronizer:
    def __init__(self, buffers, locks, tolerance=0.005):
        self.buffers = buffers
        self.locks = locks
        self.tolerance = tolerance  # in seconds

    def next_synced_frames(self):
        while True:
            timestamps = []
            for buf, lock in zip(self.buffers, self.locks):
                with lock:
                    if not buf:
                        break
                    timestamps.append(buf[0][0])
            else:
                min_ts = min(timestamps)
                max_ts = max(timestamps)
                if max_ts - min_ts <= self.tolerance:
                    synced = []
                    for buf, lock in zip(self.buffers, self.locks):
                        with lock:
                            ts, frame = buf.popleft()
                            synced.append((ts, frame))
                    return synced
                # Drop oldest frames from those ahead of sync window
                for i, ts in enumerate(timestamps):
                    if ts == min_ts:
                        with self.locks[i]:
                            self.buffers[i].popleft()
            time.sleep(0.001)

def init_synced_cameras(config, resolution=(848, 480), fps=30):
    buffers = []
    locks = []
    workers = []

    for cam in config['cameras']:
        serial = cam['serial']
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgra8, fps)
        buffer = deque()
        lock = threading.Lock()
        worker = CamWorker(serial, cfg, buffer, lock)
        buffers.append(buffer)
        locks.append(lock)
        workers.append(worker)
        worker.start()

    sync = FrameSynchronizer(buffers, locks)
    return workers, sync
