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
        
        self.follower.set_gain(gain)
        self.follower.set_log_level(arx5.LogLevel.WARNING)
        
    def get_leader_state(self) -> np.ndarray:
        """Get current state from Dynamixel leader arm"""
        return self.leader.get_joint_state()
        
    def get_follower_state(self) -> arx5.JointState:
        """Get current state from ARX5 follower"""
        return self.follower.get_joint_state()
        
    def teleop_step(self, gripper_threshold: float = 0.2) -> bool:
        """
        Execute one teleoperation step
        
        Args:
            gripper_threshold: Threshold for gripper activation (0-1)
            
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
            
            # Create ARX5 command
            follower_cmd = arx5.JointState(follower_config.joint_dof)
            follower_cmd.pos()[:] = leader_positions
            follower_cmd.gripper_pos = gripper_pos
            
            # Check for safety stop (gripper threshold)
            if leader_gripper < gripper_threshold:
                # Emergency stop - set to damping mode
                self.follower.set_to_damping()
                return False
            
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
        3 * np.pi/2,
        2 * np.pi/2
    ),
    joint_signs=(1, 1, 1, 1, 1, 1),
    gripper_config=(6, 280, 230),  # (joint_id, open_position, closed_position)
    
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
    joint_signs=(1, 1, 1, 1, 1, 1),
    gripper_config=(6, 147, 186),  # (joint_id, open_position, closed_position)

    # ARX5 follower configuration
    arx5_model="L5",
    arx5_interface="can3"  # Change to your CAN interface
)


def main():
    """Example teleoperation loop"""
    import time
    
    # Initialize teleoperation driver
    teleop = ARX5TeleopDriver(
        config=ARX5_CONFIG_RIGHT,
        dynamixel_port="/dev/ttyUSB2",  # Adjust to your Dynamixel port
        joint_scaling=1.0,
        gripper_scaling=1.0
    )
    
    print("Starting ARX5 teleoperation. Close gripper fully to stop.")
    
    try:
        while True:
            # Execute teleoperation step
            if not teleop.teleop_step(gripper_threshold=0.2):
                print("Teleoperation stopped (gripper threshold reached)")
                break
                
            # Control loop frequency (50 Hz)
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        print("Teleoperation interrupted by user")
    finally:
        teleop.shutdown()
        print("Teleoperation ended safely")


if __name__ == "__main__":
    main()