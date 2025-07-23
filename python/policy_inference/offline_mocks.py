import numpy as np
import time

class MockJointState:
    def __init__(self):
        self.joint_positions = np.random.uniform(-np.pi, np.pi, 6)  # 6 DOF for ARX5
        self.gripper_position = np.random.uniform(0, 1)  # Separate gripper position

    def get_state(self):
        state = list(self.joint_positions)
        state.append(self.gripper_position)
        return state

class MockJointCommand:
    def __init__(self):
        self.vel = np.zeros(6)  # 6 DOF joint velocities for ARX5
        self.pos = None  # Position (not used in velocity control)
        self.gripper_pos = 0.0  # Gripper position

class MockEefState:
    def __init__(self):
        self.position = np.random.uniform(-0.5, 0.5, 3)
        self.orientation = np.array([1, 0, 0, 0])  # Quaternion
        self.gripper_pos = np.random.uniform(0, 1)
        self.timestamp = time.time()

class MockController:
    def __init__(self, name):
        self.name = name
        self._timestamp = 0
        self._joint_state = MockJointState()
    def get_timestamp(self):
        self._timestamp += 0.01
        return self._timestamp
    def get_joint_state(self):
        return self._joint_state.get_state()
    def get_eef_state(self):
        return MockEefState()
    def set_eef_cmd(self, cmd):
        pass
    def set_joint_cmd(self, cmd):
        # Update joint state based on velocity command (simplified simulation)
        if hasattr(cmd, 'vel') and cmd.vel is not None:
            # Simple integration: position += velocity * dt
            dt = 0.01  # 10Hz control frequency
            self._joint_state.joint_positions += cmd.vel * dt
        if hasattr(cmd, 'gripper_pos'):
            self._joint_state.gripper_position = cmd.gripper_pos
    def set_to_damping(self):
        pass
    def reset_to_home(self):
        # Reset to random home position
        self._joint_state = MockJointState()
    def set_log_level(self, level):
        pass
    def set_gain(self, gain):
        pass
    def get_controller_config(self):
        class MockConfig:
            default_kp = 1000.0
            default_kd = 100.0
        return MockConfig()

class MockCamera:
    def __init__(self, name, width=848, height=480):
        self.name = name
        self.width = width
        self.height = height
        self._frame_count = 0
    def wait_for_frames(self):
        return MockFrames(self)

class MockFrames:
    def __init__(self, camera):
        self.camera = camera
    def get_color_frame(self):
        return MockColorFrame(self.camera)

class MockColorFrame:
    def __init__(self, camera):
        self.camera = camera
        self._timestamp = time.time()
    def get_timestamp(self):
        return self._timestamp
    def get_data(self):
        # Generate random RGB image
        image = np.random.randint(0, 255, (self.camera.height, self.camera.width, 3), dtype=np.uint8)
        # Convert to BGRA format to match RealSense
        bgra = np.zeros((self.camera.height, self.camera.width, 4), dtype=np.uint8)
        bgra[:, :, :3] = image
        bgra[:, :, 3] = 255  # Alpha channel
        return bgra

class MockRsContext:
    def query_devices(self):
        return [MockDevice()]

class MockDevice:
    def __init__(self):
        self.name = "Mock RealSense Camera"
        self.serial = "123456789012"
    def hardware_reset(self):
        pass

# Mock pyrealsense2 replacement
class MockRs:
    context = MockRsContext
    pipeline = MockCamera
    config = type('MockConfig', (), {
        'enable_device': lambda self, serial: None,
        'enable_stream': lambda self, stream, width, height, format, fps: None,
        'start': lambda self, config: None
    })
    stream = type('MockStream', (), {'color': 'color'})
    format = type('MockFormat', (), {'bgra8': 'bgra8'}) 