import os
import sys
import json


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from arx5_interface import Arx5CartesianController, Gain, LogLevel

"""
Note: All text prompts require the OpenCV window to be in focus. Sorry.
"""

def load_robot_config(config_path):
    with open(config_path, 'r') as file:
        config = json.load(file)
    return config

def initialize_controllers(config, side=None):
    controllers = []
    controller_names = []
    urdf_path = "../models/arx5.urdf"

    def create_controller_pair(pair):
        leader = Arx5CartesianController(
            pair['leader']['model'],
            pair['leader']['interface_name'],
            urdf_path,
        )
        follower = Arx5CartesianController(
            pair['follower']['model'],
            pair['follower']['interface_name'],
            urdf_path,
        )
        gain = Gain(
            leader.get_controller_config().default_kp / 10000,
            leader.get_controller_config().default_kd / 1000,
            0.0,
            0.0
        )
        
        leader.set_log_level(LogLevel.WARNING)
        follower.set_log_level(LogLevel.WARNING)
        leader.reset_to_home()
        follower.reset_to_home()
        leader.set_gain(gain)  # Set reduced damping coeffs. to leader only
        controllers.append((leader, follower))
        controller_names.append((pair['leader']['name'], pair['follower']['name']))

    if side:
        # Find the robot pair matching the specified side
        pair = next((p for p in config['robot_pairs'] if p['side'] == side), None)
        if pair:
            create_controller_pair(pair)
        else:
            print(f"No robot pair found for side: {side}")
    else:
        # Initialize all robot pairs
        for pair in config['robot_pairs']:
            create_controller_pair(pair)

    return controllers, controller_names

def main():
    config = load_robot_config("/home/verityw/arx5-sdk/python/teleop/arx5_config.json")
    controllers, controller_names = initialize_controllers(config)
    return

if __name__ == "__main__":
    main()