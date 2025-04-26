#include <nlohmann/json.hpp>
#include <optional>
#include <vector>
#include <string>
#include <stdexcept>

using json = nlohmann::json;

namespace trackerModel {

enum class Team {
    UNKNOWN = 0,
    YELLOW = 1,
    BLUE = 2
};

void to_json(json& j, const Team& tc) {
    j = static_cast<int>(tc);
}

void from_json(const json& j, Team& tc) {
    tc = static_cast<Team>(j.get<int>());
}

enum class Capability {
    CAPABILITY_UNKNOWN = 0,
    CAPABILITY_DETECT_FLYING_BALLS = 1,
    CAPABILITY_DETECT_MULTIPLE_BALLS = 2,
    CAPABILITY_DETECT_KICKED_BALLS = 3
};

void to_json(json& j, const Capability& cap) {
    j = static_cast<int>(cap);
}

void from_json(const json& j, Capability& cap) {
    cap = static_cast<Capability>(j.get<int>());
}

struct Vector2 {
    float x;
    float y;
};

void to_json(json& j, const Vector2& v) {
    j = json{{"x", v.x}, {"y", v.y}};
}

void from_json(const json& j, Vector2& v) {
    j.at("x").get_to(v.x);
    j.at("y").get_to(v.y);
}

struct Vector3 {
    float x;
    float y;
    float z;
};

void to_json(json& j, const Vector3& v) {
    j = json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
}

void from_json(const json& j, Vector3& v) {
    j.at("x").get_to(v.x);
    j.at("y").get_to(v.y);
    j.at("z").get_to(v.z);
}

struct RobotId {
    uint32_t id;
    Team team;
};

void to_json(json& j, const RobotId& r) {
    j = json{{"id", r.id}, {"team", r.team}};
}

void from_json(const json& j, RobotId& r) {
    j.at("id").get_to(r.id);
    j.at("team").get_to(r.team);
}

struct TrackedBall {
    Vector3 pos;
    std::optional<Vector3> vel;
    std::optional<float> visibility;
};

void to_json(json& j, const TrackedBall& tb) {
    j = json{{"pos", tb.pos}};
    if (tb.vel) {
        j["vel"] = *tb.vel;
    }
    if (tb.visibility) {
        j["visibility"] = *tb.visibility;
    }
}

void from_json(const json& j, TrackedBall& tb) {
    j.at("pos").get_to(tb.pos);
    if (j.contains("vel")) {
        tb.vel = j["vel"].get<Vector3>();
    }
    if (j.contains("visibility") && j["visibility"].is_number()) {
        tb.visibility = j["visibility"].get<float>();
    }
}

struct KickedBall {
    Vector2 pos;
    Vector3 vel;
    double start_timestamp;
    std::optional<double> stop_timestamp;
    std::optional<Vector2> stop_pos;
    std::optional<RobotId> robot_id;
};

void to_json(json& j, const KickedBall& kb) {
    j = json{
        {"pos", kb.pos},
        {"vel", kb.vel},
        {"start_timestamp", kb.start_timestamp}
    };
    if (kb.stop_timestamp) {
        j["stop_timestamp"] = *kb.stop_timestamp;
    }
    if (kb.stop_pos) {
        j["stop_pos"] = *kb.stop_pos;
    }
    if (kb.robot_id) {
        j["robot_id"] = *kb.robot_id;
    }
}

void from_json(const json& j, KickedBall& kb) {
    j.at("pos").get_to(kb.pos);
    j.at("vel").get_to(kb.vel);
    j.at("start_timestamp").get_to(kb.start_timestamp);
    if (j.contains("stop_timestamp")) {
        kb.stop_timestamp = j["stop_timestamp"].get<double>();
    }
    if (j.contains("stop_pos")) {
        kb.stop_pos = j["stop_pos"].get<Vector2>();
    }
    if (j.contains("robot_id")) {
        kb.robot_id = j["robot_id"].get<RobotId>();
    }
}

struct TrackedRobot {
    RobotId robot_id;
    Vector2 pos;
    float orientation;
    std::optional<Vector2> vel;
    std::optional<float> vel_angular;
    std::optional<float> visibility;
};

void to_json(json& j, const TrackedRobot& tr) {
    j = json{
        {"robot_id", tr.robot_id},
        {"pos", tr.pos},
        {"orientation", tr.orientation}
    };
    if (tr.vel) {
        j["vel"] = *tr.vel;
    }
    if (tr.vel_angular) {
        j["vel_angular"] = *tr.vel_angular;
    }
    if (tr.visibility) {
        j["visibility"] = *tr.visibility;
    }
}

void from_json(const json& j, TrackedRobot& tr) {
    j.at("robot_id").get_to(tr.robot_id);
    j.at("pos").get_to(tr.pos);
    j.at("orientation").get_to(tr.orientation);
    if (j.contains("vel")) {
        tr.vel = j["vel"].get<Vector2>();
    }
    if (j.contains("vel_angular")) {
        tr.vel_angular = j["vel_angular"].get<float>();
    }
    if (j.contains("visibility") && j["visibility"].is_number()) {
        tr.visibility = j["visibility"].get<float>();
    }
}

struct TrackedFrame {
    uint32_t frame_number;
    double timestamp;
    std::vector<TrackedBall> balls;
    std::vector<TrackedRobot> robots;
    std::optional<KickedBall> kicked_ball;
    std::vector<Capability> capabilities;
};

void to_json(json& j, const TrackedFrame& tf) {
    j = json{
        {"frame_number", tf.frame_number},
        {"timestamp", tf.timestamp},
        {"balls", tf.balls},
        {"robots", tf.robots},
        {"capabilities", tf.capabilities}
    };
    if (tf.kicked_ball) {
        j["kicked_ball"] = *tf.kicked_ball;
    }
}

void from_json(const json& j, TrackedFrame& tf) {
    j.at("frame_number").get_to(tf.frame_number);
    j.at("timestamp").get_to(tf.timestamp);
    j.at("balls").get_to(tf.balls);
    j.at("robots").get_to(tf.robots);
    if (j.contains("kicked_ball") && j["kicked_ball"].is_object()) {
        tf.kicked_ball = j["kicked_ball"].get<KickedBall>();
    }
    j.at("capabilities").get_to(tf.capabilities);
}

struct TrackerWrapperPacket {
    std::string uuid;
    std::optional<std::string> source_name;
    std::optional<TrackedFrame> tracked_frame;
};

void to_json(json& j, const TrackerWrapperPacket& twp) {
    j = json{{"uuid", twp.uuid}};
    if (twp.source_name) {
        j["source_name"] = *twp.source_name;
    }
    if (twp.tracked_frame) {
        j["tracked_frame"] = *twp.tracked_frame;
    }
}

void from_json(const json& j, TrackerWrapperPacket& twp) {
    j.at("uuid").get_to(twp.uuid);
    if (j.contains("source_name")) {
        twp.source_name = j["source_name"].get<std::string>();
    }
    if (j.contains("tracked_frame")) {
        twp.tracked_frame = j["tracked_frame"].get<TrackedFrame>();
    }
}

} // namespace trackerModel
