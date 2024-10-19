#include "app/cartesian_controller.h"
#include "app/common.h"
#include "app/joint_controller.h"
#include <zmq.hpp>

using namespace arx;

class ArxClient
{
  public:
    ArxClient(const std::string &model, const std::string &interface_name, const std::string &control_address,
              const std::string &state_address)
        : control_address(control_address), state_address(state_address)
    {
        context = zmq::context_t(1);
        control_socket = zmq::socket_t(context, ZMQ_PAIR); // Control commands
        control_socket.connect(control_address);

        state_socket = zmq::socket_t(context, ZMQ_PAIR); // State updates from the other robot
        state_socket.connect(state_address);

        Arx5JointController *arx5_joint_controller = new Arx5JointController(model, interface_name);
    }

    ~ArxClient()
    {
        delete arx5_joint_controller; // Properly delete the allocated memory
    }

    void run()
    {
        while (true)
        {
            // Receive JointState commands from other robot
        }
    }

  private:
    // TODO: Allow pairing of robots (leader-follower pair) by interface_name
    zmq::context_t context;
    zmq::socket_t control_socket;
    zmq::socket_t state_socket;
    std::string control_address;
    std::string state_address;
    Arx5JointController *arx5_joint_controller;

    std::string receive_command()
    {
        zmq::message_t message;
        control_socket.recv(&message);
        return std::string(static_cast<char *>(message.data()), message.size());
    }

    void execute_command(const std::string &command)
    {
        // Command execution logic
    }

    JointState collect_state_info()
    {
        return arx5_joint_controller->get_state();
    }

    void send_state_to_other_robot(const JointState &state)
    {
        // zmq::message_t message(state.serialize());
        // state_socket.send(message, zmq::send_flags::dontwait);
    }

    JointState receive_state_from_other_robot()
    {
        zmq::message_t message;
        state_socket.recv(&message);
        // return JointState::deserialize(message.data(), message.size());
    }
};
