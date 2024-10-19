#include "cartesian_controller.h"
#include "common.h"
#include <sstream>
#include <thread>
#include <zmq.hpp>

using namespace arx;

class ArxServer
{
  public:
    ArxServer(const std::string &address, int port) : context(1), req_socket(context, ZMQ_REQ)
    {
        std::string full_address = "tcp://" + address + ":" + std::to_string(port);
        req_socket.connect(full_address);
        std::cout << "Connected to " << full_address << std::endl;
    }

    // Method to send JointState and receive EEFState
    // TODO: Send EEFState to the robot and receive JointState
    EEFState send_joint_state(const JointState &joint_state)
    {
        // Serialize JointState
        std::ostringstream oss;
        oss << joint_state.timestamp << " ";
        for (const auto &pos : joint_state.pos)
            oss << pos << " ";
        for (const auto &vel : joint_state.vel)
            oss << vel << " ";
        for (const auto &torque : joint_state.torque)
            oss << torque << " ";
        oss << joint_state.gripper_pos << " " << joint_state.gripper_vel << " " << joint_state.gripper_torque;

        std::string serialized_joint_state = oss.str();

        // Send the serialized JointState to the robot
        zmq::message_t command(serialized_joint_state.size());
        memcpy(command.data(), serialized_joint_state.c_str(), serialized_joint_state.size());
        req_socket.send(command, zmq::send_flags::none);

        // Receive feedback from the robot
        zmq::message_t feedback;
        req_socket.recv(feedback);
        std::string serialized_eef_state(static_cast<char *>(feedback.data()), feedback.size());

        // Deserialize EEFState
        std::istringstream iss(serialized_eef_state);
        // EEFState eef_state;
        // iss >> eef_state.pose_6d.x >> eef_state.pose_6d.y >> eef_state.pose_6d.z >> eef_state.pose_6d.roll >>
        //     eef_state.pose_6d.pitch >> eef_state.pose_6d.yaw >> eef_state.gripper_pos >> eef_state.gripper_vel >>
        //     eef_state.gripper_torque >> eef_state.timestamp;

        // return eef_state;
    }

  private:
    zmq::context_t context;
    zmq::socket_t req_socket; // REQ socket for communication
};