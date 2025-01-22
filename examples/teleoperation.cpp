#include "app/cartesian_controller.h"
#include "app/common.h"
#include "app/joint_controller.h"
#include "app/solver.h"
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

// initialize 2 controllers
using namespace arx;

Arx5JointController *arx5_leader_controller = new Arx5JointController("L5", "can1");
Arx5JointController *arx5_follower_controller = new Arx5JointController("L5", "can0");

// signal handling
void signal_handler(int signal)
{
    std::cout << "SIGINT received" << std::endl;
    delete arx5_leader_controller;
    delete arx5_follower_controller;
    exit(signal);
}

int main()
{
    // EEFState home_cmd;
    int dof = arx5_leader_controller->get_robot_config().joint_dof;
    Gain gain{dof};
    // Arx5Solver solver("../models/arx5.urdf", dof);

    double gripper_width = arx5_leader_controller->get_robot_config().gripper_width;

    arx5_leader_controller->reset_to_home();
    arx5_follower_controller->reset_to_home();

    gain.kd = (arx5_leader_controller->get_controller_config()).default_kd / 1000;
    arx5_leader_controller->set_gain(gain);
    arx5_follower_controller->set_gain(gain);

    std::signal(SIGINT, signal_handler);

    arx5_leader_controller->reset_to_home();
    arx5_follower_controller->reset_to_home();

    while (true)
    {
        JointState leader_joint_state = arx5_leader_controller->get_state();
        JointState follower_cmd = JointState(dof);
        follower_cmd.pos = leader_joint_state.pos;
        follower_cmd.gripper_pos *= 4.8;
        follower_cmd.timestamp = 0.0f;

        arx5_follower_controller->set_joint_cmd(follower_cmd);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
