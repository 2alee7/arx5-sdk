import os
import sys
import time

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5

def print_joint_state(can_id, value="pos"):
    joint_controller = arx5.Arx5JointController("L5", can_id)
    joint_controller.enable_background_send_recv()
    
    while True: 
        state = joint_controller.get_state()
        if value == "pos":
            pos = state.pos()
            print(", ".join([f"{x:.3f}" for x in pos]))
        elif value == "vel":
            vel = state.vel()
            print(", ".join([f"{x:.3f}" for x in vel]))
        elif value == "torque":
            torque = state.torque()
            print(", ".join([f"{x:.3f}" for x in torque]))
        print(", ".join([f"{x:.3f}" for x in pos]))
        time.sleep(0.1)

def main():
    print_joint_state("can2", "pos")

if __name__ == "__main__":
    main()