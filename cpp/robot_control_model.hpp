#include <cstdint>
#include <optional>
#include <variant>
#include <vector>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace robotControlModel {

// Move Velocity Structures
struct MoveWheelVelocity {
    float front_right;
    float back_right;
    float back_left;
    float front_left;

    MoveWheelVelocity(float fr = 0, float br = 0, float bl = 0, float fl = 0)
        : front_right(fr), back_right(br), back_left(bl), front_left(fl) {}
};

struct MoveLocalVelocity {
    float forward;
    float left;
    float angular;

    MoveLocalVelocity(float fwd = 0, float lft = 0, float ang = 0)
        : forward(fwd), left(lft), angular(ang) {}
};

struct MoveGlobalVelocity {
    float x;
    float y;
    float angular;

    MoveGlobalVelocity(float x_val = 0, float y_val = 0, float ang = 0)
        : x(x_val), y(y_val), angular(ang) {}
};

// JSON Serialization/Deserialization for Move Velocity types
void to_json(json& j, const MoveWheelVelocity& m) {
    j = json{
        {"front_right", m.front_right},
        {"back_right", m.back_right},
        {"back_left", m.back_left},
        {"front_left", m.front_left}
    };
}

void from_json(const json& j, MoveWheelVelocity& m) {
    j.at("front_right").get_to(m.front_right);
    j.at("back_right").get_to(m.back_right);
    j.at("back_left").get_to(m.back_left);
    j.at("front_left").get_to(m.front_left);
}

void to_json(json& j, const MoveLocalVelocity& m) {
    j = json{
        {"forward", m.forward},
        {"left", m.left},
        {"angular", m.angular}
    };
}

void from_json(const json& j, MoveLocalVelocity& m) {
    j.at("forward").get_to(m.forward);
    j.at("left").get_to(m.left);
    j.at("angular").get_to(m.angular);
}

void to_json(json& j, const MoveGlobalVelocity& m) {
    j = json{
        {"x", m.x},
        {"y", m.y},
        {"angular", m.angular}
    };
}

void from_json(const json& j, MoveGlobalVelocity& m) {
    j.at("x").get_to(m.x);
    j.at("y").get_to(m.y);
    j.at("angular").get_to(m.angular);
}

// RobotMoveCommand (std::variant wrapper)
struct RobotMoveCommand {
    std::variant<MoveWheelVelocity, MoveLocalVelocity, MoveGlobalVelocity> command;

    RobotMoveCommand(MoveWheelVelocity cmd)
        : command(cmd) {}
    RobotMoveCommand(MoveLocalVelocity cmd = MoveLocalVelocity())
        : command(cmd) {}
    RobotMoveCommand(MoveGlobalVelocity cmd)
        : command(cmd) {}
};

void to_json(json& j, const RobotMoveCommand& cmd) {
    // Initialize all fields to null
    j = {
        {"wheel_velocity", nullptr},
        {"local_velocity", nullptr},
        {"global_velocity", nullptr}
    };

    // Overwrite the active command
    std::visit([&j](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, MoveWheelVelocity>) {
            j["wheel_velocity"] = arg;
        } else if constexpr (std::is_same_v<T, MoveLocalVelocity>) {
            j["local_velocity"] = arg;
        } else if constexpr (std::is_same_v<T, MoveGlobalVelocity>) {
            j["global_velocity"] = arg;
        }
    }, cmd.command);
}

void from_json(const json& j, RobotMoveCommand& cmd) {
    if (j.contains("wheel_velocity")) {
        cmd.command = j.at("wheel_velocity").get<MoveWheelVelocity>();
    } else if (j.contains("local_velocity")) {
        cmd.command = j.at("local_velocity").get<MoveLocalVelocity>();
    } else if (j.contains("global_velocity")) {
        cmd.command = j.at("global_velocity").get<MoveGlobalVelocity>();
    }
}

// RobotCommand Structure
struct RobotCommand {
    uint32_t id;
    std::optional<RobotMoveCommand> move_command;
    std::optional<float> kick_speed;
    float kick_angle;
    std::optional<float> dribbler_speed;

    RobotCommand(uint32_t id = 0,
                 std::optional<RobotMoveCommand> move_cmd = std::nullopt,
                 std::optional<float> kick_spd = std::nullopt,
                 float kick_ang = 0.0f,
                 std::optional<float> dribbler_spd = std::nullopt)
        : id(id),
          move_command(std::move(move_cmd)),
          kick_speed(kick_spd),
          kick_angle(kick_ang),
          dribbler_speed(dribbler_spd) {}
};

void to_json(json& j, const RobotCommand& rc) {
    j = json{{"id", rc.id}};
    if (rc.move_command.has_value()) {
        j["move_command"] = *rc.move_command;
    }
    if (rc.kick_speed.has_value()) {
        j["kick_speed"] = *rc.kick_speed;
    }
    if (rc.kick_angle != 0.0f) {
        j["kick_angle"] = rc.kick_angle;
    }
    if (rc.dribbler_speed.has_value()) {
        j["dribbler_speed"] = *rc.dribbler_speed;
    }
}

void from_json(const json& j, RobotCommand& rc) {
    j.at("id").get_to(rc.id);
    if (j.contains("move_command")) {
        rc.move_command = j.at("move_command").template get<RobotMoveCommand>();
    }
    if (j.contains("kick_speed")) {
        rc.kick_speed = j.at("kick_speed").get<float>();
    }
    rc.kick_angle = j.value("kick_angle", 0.0f);
    if (j.contains("dribbler_speed")) {
        rc.dribbler_speed = j.at("dribbler_speed").get<float>();
    }
}

// RobotControl Structure
struct RobotControl {
    std::vector<RobotCommand> robot_commands;

    RobotControl(std::vector<RobotCommand> robot_commands)
        : robot_commands(std::move(robot_commands)) {}
};

void to_json(json& j, const RobotControl& ctrl) {
    j = json{{"robot_commands", ctrl.robot_commands}};
}

void from_json(const json& j, RobotControl& ctrl) {
    j.at("robot_commands").get_to(ctrl.robot_commands);
}

struct RobotCommandExt : public RobotCommand {
    bool isteamyellow;

    RobotCommandExt(bool isteamyellow = false,
                    uint32_t id = 0,
                    std::optional<RobotMoveCommand> move_cmd = std::nullopt,
                    std::optional<float> kick_spd = std::nullopt,
                    float kick_ang = 0.0f,
                    std::optional<float> dribbler_spd = std::nullopt)
        : RobotCommand(id, move_cmd, kick_spd, kick_ang, dribbler_spd),
          isteamyellow(isteamyellow) {}
};

void to_json(json& j, const RobotCommandExt& rc) {
    to_json(j, static_cast<const RobotCommand&>(rc));
    j["isteamyellow"] = rc.isteamyellow;
}

void from_json(const json& j, RobotCommandExt& rc) {
    j.at("isteamyellow").get_to(rc.isteamyellow);
    from_json(j, static_cast<RobotCommand&>(rc));
}

struct RobotControlExt : public RobotControl {
    bool isteamyellow;

    RobotControlExt(bool isteamyellow = false, std::vector<RobotCommand> robot_commands = {})
        : RobotControl(robot_commands), isteamyellow(isteamyellow) {}
};

void to_json(json& j, const RobotControlExt& ctrl) {
    to_json(j, static_cast<const RobotControl&>(ctrl));
    j["isteamyellow"] = ctrl.isteamyellow;
}

void from_json(const json& j, RobotControlExt& ctrl) {
    j.at("isteamyellow").get_to(ctrl.isteamyellow);
    from_json(j, static_cast<RobotControl&>(ctrl));
}

} // namespace robotControlModel
