#include "app/cartesian_controller.h"
#include "app/common.h"
#include "app/solver.h"
#include <chrono>
#include <csignal>

// P-P (PD) architecture for bilateral teleoperation of one leader/follower pair of ARX5-L5.

/*
Requires direct controller - server - controller communication on multiple synchronized threads for lowest latency.
*/

// initialize 2 controllers
using namespace arx;

Arx5CartesianController *arx5_leader_controller = new Arx5CartesianController("L5", "can0", "../models/arx5.urdf");
Arx5CartesianController *arx5_follower_controller = new Arx5CartesianController("L5", "can1", "../models/arx5.urdf");

// signal handling
void signal_handler(int signal)
{
    std::cout << "SIGINT received" << std::endl;
    delete arx5_leader_controller;
    delete arx5_follower_controller;
    exit(signal);
}

static double kp = 0.1;
static double kd = 0.0;

VecDoF pd_callback(float kp, float kd, JointState joint_state_1, JointState joint_state_2)
{
    // calculate PD control: tau = kp * (q_2 - q_1) + kd * (dq_2 - dq_1)
    VecDoF q1 = joint_state_1.pos;
    VecDoF q2 = joint_state_2.pos;
    VecDoF dq1 = joint_state_1.vel;
    VecDoF dq2 = joint_state_2.vel;

    // calculate tau by applying PD law to each joint in VecDoF
    VecDoF tau = kp * (q2 - q1) + kd * (dq2 - dq1);
    return tau;
}

// TODO: initialize connected robots (enable inertia, gravity compensation, friction(?) compensation)
int main()
{
    EEFState cmd;
    // int loop_cnt = 0;
    int dof = arx5_leader_controller->get_robot_config().joint_dof;
    Gain gain{dof};
    Arx5Solver solver("../models/arx5.urdf", dof);

    arx5_leader_controller->reset_to_home();
    arx5_follower_controller->reset_to_home();

    gain.kd = (arx5_leader_controller->get_controller_config()).default_kd / 10;
    std::signal(SIGINT, signal_handler);

    arx5_leader_controller->set_gain(gain);
    arx5_follower_controller->set_gain(gain);

    cmd.pose_6d = arx5_leader_controller->get_home_pose();

    arx5_leader_controller->set_eef_cmd(cmd);
    arx5_follower_controller->set_eef_cmd(cmd);

    while (true)
    {
        EEFState leader_eef_state = arx5_leader_controller->get_eef_state();
        JointState leader_joint_state = arx5_leader_controller->get_joint_state();

        EEFState follower_eef_state = arx5_follower_controller->get_eef_state();
        JointState follower_joint_state = arx5_follower_controller->get_joint_state();

        VecDoF leader_cmd = pd_callback(kp, kd, leader_joint_state, follower_joint_state);
        VecDoF follower_cmd = pd_callback(kp, kd, follower_joint_state, leader_joint_state);

        arx5_leader_controller->set_joint_cmd(leader_cmd);
        arx5_follower_controller->set_joint_cmd(follower_cmd);

        // TODO: publish leader joint state to follower, and vice versa.
        // Need joint torque cmd interface in cartesian_controller
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}

// TODO: define gains for PD controller

// TODO: use performant PD functions (implemented elsewhere if needed) --> FF current + PD current
