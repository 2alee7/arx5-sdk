import threading
from collections import deque
import pyrealsense2 as rs
import numpy as np
import time
from shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer

class CamWorker(threading.Thread):
    def __init__(self, serial, cfg, buffer: SharedMemoryRingBuffer, lock: threading.Lock, recording_event: threading.Event):
        super().__init__(daemon=True)
        self.serial = serial
        self.cfg    = cfg
        self.buffer = buffer
        self.lock   = lock
        self.rec    = recording_event
        self.pipeline = rs.pipeline()
        self.running  = True
        # self.init_rec_ts = None # timestamp when recording starts, resets to None when recording stops

    def run(self):
        # start once, run pipeline until shutdown
        self.pipeline.start(self.cfg)
        while self.running:
            frames = self.pipeline.wait_for_frames()
            color  = frames.get_color_frame()
            if not color:
                continue

            ts  = color.get_timestamp() / 1000.0
            img = np.asanyarray(color.get_data())

            with self.lock:
                if self.rec.is_set():   # recording: record initial timestamp, enqueue frames into buffer
                    
                    # if self.init_rec_ts is None: TODO: necessary?
                    #     self.init_rec_ts = ts

                    self.buffer.append((ts, img))
                
                else:
                    self.buffer.clear() # not recording: discard any old frames
        
        self.pipeline.stop()    # stop pipeline only when thread is torn down

    def stop(self):
        self.running = False

class FrameSynchronizer:
    def __init__(self, buffers, locks, tolerance=0.00833):  # 8.33 ms for 60 fps, 16.67 ms for 30 fps
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

def init_synced_cameras(config, resolution=(848, 480), fps=60):
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

def pop_latest_frames(buffers, locks):
    # Call pop_latest_frames on all buffers and return a list of frames
    frames = []
    for buffer, lock in zip(buffers, locks):
        with lock:
            if buffer:
                frames.append(buffer.popleft()[1])
            else:
                frames.append(np.zeros((480, 848, 3), dtype=np.uint8))
    return frames
    
