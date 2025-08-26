import os
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple
import numpy as np
import sys

# Add ARX5 python directory to path
# Current file is in: /python/teleop/utils/arx5_dynamixel.py
# Need to go up to: /python/
PYTHON_DIR = os.path.dirname(os.path.dirname(os.path.dirname(   os.path.abspath(__file__))))
sys.path.insert(0, PYTHON_DIR)

import arx5_interface as arx5
from dynamixel_driver import DynamixelRobot

@dataclass
class ARX5DynamixelConfig:
    """Configuration for ARX5 teleoperation with Dynamixel leader arm"""
    
    # Dynamixel leader arm config
    joint_ids: Sequence[int]
    joint_offsets: Sequence[float] 
    joint_signs: Sequence[int]
    gripper_config: Tuple[int, int, int]  # (id, open_pos, closed_pos)
    
    # ARX5 follower config
    arx5_model: str  # "X5" or "L5"
    arx5_interface: str  # "can0", "can1", etc.
    
    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)
        assert len(self.joint_ids) == 6  # ARX5 has 6 DOF


class ARX5TeleopDriver:
    """Streamlined teleoperation driver for ARX5 with Dynamixel leader"""
    
    def __init__(
        self, 
        config: ARX5DynamixelConfig,
        dynamixel_port: str = "/dev/ttyUSB0",
        joint_scaling: float = 1.0,
        gripper_scaling: float = 1.0
    ):
        self.config = config
        self.joint_scaling = joint_scaling
        self.gripper_scaling = gripper_scaling
        
        # Initialize Dynamixel leader arm
        self.leader = DynamixelRobot(
            joint_ids=config.joint_ids,
            joint_offsets=list(config.joint_offsets),
            joint_signs=list(config.joint_signs),
            port=dynamixel_port,
            gripper_config=config.gripper_config,
            real=True
        )
        
        # Initialize ARX5 follower
        self.follower = arx5.Arx5JointController(
            config.arx5_model, 
            config.arx5_interface
        )
        
        # Set up ARX5 for teleoperation
        self._setup_arx5_follower()
        
    def _setup_arx5_follower(self):
        """Configure ARX5 for teleoperation"""
        # Reset to home position
        self.follower.reset_to_home()
        
        # Set appropriate gains for teleoperation
        robot_config = self.follower.get_robot_config()
        gain = arx5.Gain(robot_config.joint_dof)
        
        # Reduce gains for smoother teleoperation
        controller_config = self.follower.get_controller_config()
        gain.kp()[:] = controller_config.default_kp * 0.5  # Reduce stiffness
        gain.kd()[:] = controller_config.default_kd * 0.8  # Maintain damping
        
        # IMPORTANT: Set gripper gain for gripper control to work
        gain.gripper_kp = controller_config.default_gripper_kp * 0.8  # Enable gripper control
        gain.gripper_kd = controller_config.default_gripper_kd * 0.8  # Set gripper damping
        
        print(f"Setting gripper gains: kp={gain.gripper_kp:.2f}, kd={gain.gripper_kd:.2f}")
        
        self.follower.set_gain(gain)
        self.follower.set_log_level(arx5.LogLevel.INFO)  # More verbose for debugging
        
    def get_leader_state(self) -> np.ndarray:
        """Get current state from Dynamixel leader arm"""
        return self.leader.get_joint_state()
        
    def get_follower_state(self) -> arx5.JointState:
        """Get current state from ARX5 follower"""
        return self.follower.get_joint_state()
        
    def teleop_step(self) -> bool:
        """
        Execute one teleoperation step
            
        Returns:
            bool: True if teleoperation should continue, False to stop
        """
        try:
            # Get leader joint state
            leader_joints = self.get_leader_state()
            
            # Extract joint positions and gripper
            leader_positions = leader_joints[:-1] * self.joint_scaling
            leader_gripper = leader_joints[-1]
            
            # Convert gripper from normalized [0,1] to ARX5 units [0, gripper_width]
            follower_config = self.follower.get_robot_config()
            gripper_pos = leader_gripper * follower_config.gripper_width * self.gripper_scaling
            
            # Debug gripper values
            print(f"Leader gripper: {leader_gripper:.3f}, Follower gripper_pos: {gripper_pos:.4f}m, Max: {follower_config.gripper_width:.4f}m")
            
            # Validate gripper bounds
            if gripper_pos < 0:
                gripper_pos = 0.0
            elif gripper_pos > follower_config.gripper_width:
                gripper_pos = follower_config.gripper_width
            
            # Create ARX5 command
            follower_cmd = arx5.JointState(follower_config.joint_dof)
            follower_cmd.pos()[:] = leader_positions
            follower_cmd.gripper_pos = gripper_pos
            
            # Check current gripper state for comparison
            current_state = self.follower.get_joint_state()
            print(f"Current gripper pos: {current_state.gripper_pos:.4f}m, Command: {gripper_pos:.4f}m")
            
            # Send command to ARX5
            self.follower.set_joint_cmd(follower_cmd)
            
            return True
            
        except Exception as e:
            print(f"Teleoperation error: {e}")
            self.follower.set_to_damping()
            return False
    
    def shutdown(self):
        """Safely shutdown teleoperation"""
        try:
            self.follower.reset_to_home()
        except:
            pass


# Example configuration for ARX5
ARX5_CONFIG_RIGHT = ARX5DynamixelConfig(
    # Dynamixel leader configuration (adjust based on your leader arm)
    joint_ids=(0, 1, 2, 3, 4, 5),
    joint_offsets=(
        2 * np.pi/2,
        3 * np.pi/2,
        3 * np.pi/2,
        2 * np.pi/2,
        2 * np.pi/2,
        2 * np.pi/2
    ),
    joint_signs=(1, -1, -1, -1, 1, 1),
    gripper_config=(6, 186, 145),  # (joint_id, open_position, closed_position)
    
    # ARX5 follower configuration
    arx5_model="L5",
    arx5_interface="can1"  # Change to your CAN interface
)

ARX5_CONFIG_LEFT = ARX5DynamixelConfig(
    # Dynamixel leader configuration (adjust based on your leader arm)
    joint_ids=(0, 1, 2, 3, 4, 5),
    joint_offsets=(
        2 * np.pi/2,
        3 * np.pi/2,
        3 * np.pi/2,
        2 * np.pi/2,
        3 * np.pi/2,
        2 * np.pi/2
    ),
    joint_signs=(1, -1, -1, -1, 1, 1),
    gripper_config=(6, 168, 214),  # (joint_id, open_position, closed_position)

    # ARX5 follower configuration
    arx5_model="L5",
    arx5_interface="can3"  # Change to your CAN interface
)


def main():
    """Bimanual teleoperation loop for left and right ARX5 arms"""
    import time
    import threading
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ARX5 Bimanual Teleoperation')
    parser.add_argument('--left-port', default='/dev/ttyUSB1', 
                       help='Dynamixel port for left arm leader (default: /dev/ttyUSB1)')
    parser.add_argument('--right-port', default='/dev/ttyUSB2', 
                       help='Dynamixel port for right arm leader (default: /dev/ttyUSB2)')
    parser.add_argument('--single-arm', choices=['left', 'right'], 
                       help='Run single arm only (left or right)')
    parser.add_argument('--joint-scaling', type=float, default=1.0,
                       help='Joint scaling factor (default: 1.0)')
    parser.add_argument('--gripper-scaling', type=float, default=1.0,
                       help='Gripper scaling factor (default: 1.0)')
    
    args = parser.parse_args()
    
    teleop_drivers = {}
    threads = {}
    stop_event = threading.Event()
    
    def run_arm_teleop(arm_name, driver):
        """Run teleoperation for a single arm"""
        print(f"Starting {arm_name} arm teleoperation")
        try:
            while not stop_event.is_set():
                driver.teleop_step()
                time.sleep(0.02)  # 50 Hz
        except Exception as e:
            print(f"{arm_name} arm error: {e}")
            stop_event.set()
    
    try:
        # Initialize drivers based on arguments
        if args.single_arm != 'right':  # Include left arm
            print(f"Initializing LEFT arm with port {args.left_port}")
            teleop_drivers['left'] = ARX5TeleopDriver(
                config=ARX5_CONFIG_LEFT,
                dynamixel_port=args.left_port,
                joint_scaling=args.joint_scaling,
                gripper_scaling=args.gripper_scaling
            )
        
        if args.single_arm != 'left':  # Include right arm
            print(f"Initializing RIGHT arm with port {args.right_port}")
            teleop_drivers['right'] = ARX5TeleopDriver(
                config=ARX5_CONFIG_RIGHT,
                dynamixel_port=args.right_port,
                joint_scaling=args.joint_scaling,
                gripper_scaling=args.gripper_scaling
            )
        
        print(f"Starting ARX5 {'bimanual' if len(teleop_drivers) == 2 else args.single_arm} teleoperation. Press Ctrl+C to stop.")
        
        # Start threads for each arm
        for arm_name, driver in teleop_drivers.items():
            thread = threading.Thread(
                target=run_arm_teleop,
                args=(arm_name, driver),
                daemon=True
            )
            threads[arm_name] = thread
            thread.start()
        
        # Wait for interruption
        while not stop_event.is_set():
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nTeleoperation interrupted by user")
        stop_event.set()
    except Exception as e:
        print(f"Error during initialization: {e}")
        stop_event.set()
    finally:
        # Shutdown all drivers
        print("Shutting down...")
        stop_event.set()
        
        # Wait for threads to finish
        for thread in threads.values():
            thread.join(timeout=1.0)
        
        # Shutdown drivers
        for arm_name, driver in teleop_drivers.items():
            try:
                print(f"Shutting down {arm_name} arm")
                driver.shutdown()
            except Exception as e:
                print(f"Error shutting down {arm_name} arm: {e}")
        
        print("Teleoperation ended safely")


if __name__ == "__main__":
    main()