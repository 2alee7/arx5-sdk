import time
from arx5_interface import Arx5CartesianController

def control_loop_open(leader_controller: Arx5CartesianController, follower_controller: Arx5CartesianController, stop_event, robot=True):
    while not stop_event.is_set():
        try:
            leader_eef_state = leader_controller.get_eef_state()
            follower_cmd = leader_eef_state
            follower_cmd.gripper_pos *= 4.8
            follower_cmd.timestamp = 0.0
            follower_controller.set_eef_cmd(follower_cmd)
        except Exception as e:
            print(f"Error in control loop: {e}") 
        time.sleep(0.02)