// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#include "autonomy/DecisionEngine.hpp"
#include "security/DroneSecurity.hpp"
#include "swarm/V2XMeshNetwork.hpp"

#include <cstring>
#include <optional>
#include <string>
#include <string_view>

namespace drone::security {

enum class RemoteCommandAction : uint8_t {
    NONE = 0,
    HOLD_POSITION,
    RETURN_HOME,
    EMERGENCY_LAND,
    FORMATION_HOLD,
};

inline std::string_view to_string(RemoteCommandAction action) {
    switch (action) {
    case RemoteCommandAction::NONE: return "NONE";
    case RemoteCommandAction::HOLD_POSITION: return "HOLD_POSITION";
    case RemoteCommandAction::RETURN_HOME: return "RETURN_HOME";
    case RemoteCommandAction::EMERGENCY_LAND: return "EMERGENCY_LAND";
    case RemoteCommandAction::FORMATION_HOLD: return "FORMATION_HOLD";
    }
    return "UNKNOWN";
}

struct RemoteCommandEnvelope {
    RemoteCommandAction action{RemoteCommandAction::NONE};
    uint32_t src_id{0};
    uint32_t seq_num{0};
    double issued_at_s{0.0};
    bool critical{false};
    std::string summary;
};

struct CommandPolicyDecision {
    bool accepted{false};
    bool critical{false};
    std::string reason;
};

inline std::optional<RemoteCommandEnvelope> command_from_swarm_message(const swarm::SwarmMessage& msg) {
    RemoteCommandEnvelope out;
    out.src_id = msg.src_id;
    out.seq_num = msg.seq_num;
    out.issued_at_s = msg.timestamp;

    switch (msg.type) {
    case swarm::SwarmMessage::Type::EMERGENCY_STOP:
        out.action = RemoteCommandAction::EMERGENCY_LAND;
        out.critical = true;
        out.summary = "Emergency stop received from secure swarm peer";
        return out;
    case swarm::SwarmMessage::Type::FORMATION_CMD:
        out.action = RemoteCommandAction::FORMATION_HOLD;
        out.summary = "Formation command received from swarm leader";
        if (!msg.payload.empty()) {
            const auto shape = static_cast<swarm::FormationCommand::Formation>(msg.payload[0]);
            if (shape == swarm::FormationCommand::Formation::FREE) {
                out.action = RemoteCommandAction::HOLD_POSITION;
                out.summary = "Free-formation command mapped to local hold position";
            }
        }
        return out;
    case swarm::SwarmMessage::Type::MISSION_SYNC:
        out.action = RemoteCommandAction::RETURN_HOME;
        out.summary = "Mission sync requested safe return posture";
        return out;
    default:
        break;
    }
    return std::nullopt;
}

inline CommandPolicyDecision evaluate_remote_command(const DroneSecurityAssessment& security,
                                                     const RemoteCommandEnvelope& command) {
    CommandPolicyDecision out;
    out.critical = command.critical;

    if (command.action == RemoteCommandAction::EMERGENCY_LAND) {
        out.accepted = true;
        out.reason = "Critical remote emergency command accepted";
        return out;
    }

    if (!security.remote_command_allowed) {
        out.accepted = false;
        out.reason = "Remote command rejected by onboard security state";
        return out;
    }

    switch (security.state) {
    case DroneSecurityState::TRUSTED:
    case DroneSecurityState::DEGRADED_LINK:
        out.accepted = true;
        out.reason = "Remote command accepted by onboard policy";
        return out;
    case DroneSecurityState::AUTH_SUSPECT:
    case DroneSecurityState::PEER_SPOOF_SUSPECT:
    case DroneSecurityState::COMMAND_REPLAY_SUSPECT:
    case DroneSecurityState::CONTROL_PLANE_UNTRUSTED:
    case DroneSecurityState::ISOLATED_AUTONOMY:
    case DroneSecurityState::SAFE_RETURN:
    case DroneSecurityState::LAND_IMMEDIATELY:
        out.accepted = false;
        out.reason = "Remote command blocked under hardened security posture";
        return out;
    }

    out.accepted = false;
    out.reason = "Remote command blocked by default deny policy";
    return out;
}

inline void apply_remote_command(const RemoteCommandEnvelope& command,
                                 autonomy::DecisionCommand& decision,
                                 const vio::PoseEstimate& pose) {
    using autonomy::BehaviorMode;

    switch (command.action) {
    case RemoteCommandAction::HOLD_POSITION:
    case RemoteCommandAction::FORMATION_HOLD:
        decision.mode = BehaviorMode::HOLD_POSITION;
        decision.requires_operator_attention = false;
        decision.desired_velocity = -pose.velocity * 0.30;
        decision.desired_yaw_rate_rads = 0.0;
        decision.summary = command.summary;
        return;
    case RemoteCommandAction::RETURN_HOME:
        decision.mode = BehaviorMode::RETURN_HOME;
        decision.requires_operator_attention = true;
        decision.desired_velocity = (-pose.velocity * 0.35) + (pose.R_wb() * Eigen::Vector3d{0.0, 1.0, 0.0} * 0.12);
        decision.desired_yaw_rate_rads = 0.0;
        decision.summary = command.summary;
        return;
    case RemoteCommandAction::EMERGENCY_LAND:
        decision.mode = BehaviorMode::EMERGENCY_LAND;
        decision.requires_operator_attention = true;
        decision.desired_velocity = -(pose.R_wb() * Eigen::Vector3d{0.0, 1.0, 0.0}) * 0.85;
        decision.desired_yaw_rate_rads = 0.0;
        decision.summary = command.summary;
        return;
    case RemoteCommandAction::NONE:
        return;
    }
}

} // namespace drone::security
