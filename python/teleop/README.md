# ARX5 Teleoperation & Trajectory Recording Guide

How to teleoperate and record trajectories using ARX5 with the ARX5 follower arms.

## Test Scripts

- `test/test_control_loop.py`  
  Start/test control threads.

- `test/test_cameras.py`  
  Start/test camera feed and visualization threads.

## Utilities

- `control_loops.py`  
  Robot control loop logic.

- `post_utils.py`  
  Converts recorded frames and joint data to `.hdf5`. Method available for conversion to `tfrecords`.

- `realsense_utils.py`  
  Camera initialization and frameset synchronization.

- `viz_utils.py`  
  OpenCV-based visualization of camera views.

- `teleop_utils.py`  
  *Deprecated or unused.*


## Setup Instructions

### 1. Initialize Conda Environment & SLCAN Adapters

```bash
conda activate arx5-py310
init_arx5
```

> Refer to SLCAN documentation for adapter setup.

### 2. Navigate to Teleoperation Directory

```bash
cd arx5-sdk/python/teleop
```

### 3. Run Teleoperation & Record Demonstrations

```bash
python collect_traj.py --task <string> --timesteps <int> --path <string> --side <"left" | "right">
```

#### Controls

- Wait for **"ready to record"** message.
- Press **ENTER** to begin/end recording an episode.
- Press **SPACE** to pause/resume recording.
- Enter **Y** or **N** to save/discard the episode when prompted.
- Press **CTRL + C** to exit the program.