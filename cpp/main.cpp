#include <zmqpp/zmqpp.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <iostream>

#include "robot_control_model.hpp"
#include "tracker_model.hpp"

using namespace std;

namespace rcm = robotControlModel;
namespace trm = trackerModel;

int main(int argc, char *argv[])
{
    const string endpoint = "ipc:///tmp/transnet.tracker";

    zmqpp::context context;

    zmqpp::socket s_tracker(context, zmqpp::socket_type::sub);

    cout << "Opening connection to " << endpoint << "..." << endl;
    s_tracker.connect(endpoint);
    s_tracker.set(zmqpp::socket_option::subscribe, "");

    zmqpp::socket s_control(context, zmqpp::socket_type::pub);
    s_control.connect("ipc:///tmp/ether.signals.xsub");

    while (true)
    {
        // receive a message
        zmqpp::message message;
        s_tracker.receive(message);
        string s_data;
        message >> s_data;
        nlohmann::json data;
        data = nlohmann::json::parse(s_data);
        // cout << "Received message: " << data.dump(2) << endl;
        trm::TrackerWrapperPacket trackerPacket = data;

        cout << "Received message uuid: " << trackerPacket.uuid << endl;
        cout << "Received message frame number: " << trackerPacket.tracked_frame->frame_number << endl;
        cout << "Received message balls: " << endl;
        for (auto& ball : trackerPacket.tracked_frame->balls) {
            cout << "Ball pos: " << ball.pos.x << ", " << ball.pos.y << ", " << ball.pos.z << endl;
            cout << "Ball vel: " << ball.vel->x << ", " << ball.vel->y << ", " << ball.vel->z << endl;
        }


        rcm::RobotCommand robotCommand(
            0,
            rcm::RobotMoveCommand(
                rcm::MoveLocalVelocity(1, 1, 1)
            )
        );

        rcm::RobotControlExt robotControl(
            true,
            {
                robotCommand
            }
        );

        nlohmann::json j = robotControl;

        std::string control_message = "{\"transnet\": \"actuate_robot\", \"data\": " + j.dump() + "}";

        cout << "Sending message: " << control_message << endl;

        s_control.send(control_message);
    }


    // // send a message
    // cout << "Sending text and a number..." << endl;
    // zmqpp::message message;
    // // compose a message from a string and a number
    // message << "Hello World!" << 42;
    // socket.send(message);

    // cout << "Sent message." << endl;
    // cout << "Finished." << endl;
}