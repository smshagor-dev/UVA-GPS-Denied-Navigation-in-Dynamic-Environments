// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace drone::security {

enum class DroneSecurityState : uint8_t {
    TRUSTED = 0,
    DEGRADED_LINK,
    AUTH_SUSPECT,
    PEER_SPOOF_SUSPECT,
    COMMAND_REPLAY_SUSPECT,
    CONTROL_PLANE_UNTRUSTED,
    ISOLATED_AUTONOMY,
    SAFE_RETURN,
    LAND_IMMEDIATELY,
};

inline std::string_view to_string(DroneSecurityState state) {
    switch (state) {
    case DroneSecurityState::TRUSTED: return "TRUSTED";
    case DroneSecurityState::DEGRADED_LINK: return "DEGRADED_LINK";
    case DroneSecurityState::AUTH_SUSPECT: return "AUTH_SUSPECT";
    case DroneSecurityState::PEER_SPOOF_SUSPECT: return "PEER_SPOOF_SUSPECT";
    case DroneSecurityState::COMMAND_REPLAY_SUSPECT: return "COMMAND_REPLAY_SUSPECT";
    case DroneSecurityState::CONTROL_PLANE_UNTRUSTED: return "CONTROL_PLANE_UNTRUSTED";
    case DroneSecurityState::ISOLATED_AUTONOMY: return "ISOLATED_AUTONOMY";
    case DroneSecurityState::SAFE_RETURN: return "SAFE_RETURN";
    case DroneSecurityState::LAND_IMMEDIATELY: return "LAND_IMMEDIATELY";
    }
    return "UNKNOWN";
}

struct DroneSecurityInputs {
    std::string security_profile{"lab"};
    bool swarm_security_enabled{false};
    bool hardened_profile{false};
    bool placeholder_secret{false};
    bool localization_lost{false};
    bool emergency_fault{false};
    double battery_pct{100.0};
    double link_quality{1.0};
    double sync_confidence{1.0};
    double peer_clock_offset_ms{0.0};
    size_t peer_count{0};
};

struct DroneSecurityAssessment {
    DroneSecurityState state{DroneSecurityState::TRUSTED};
    bool remote_command_allowed{true};
    bool telemetry_uplink_allowed{true};
    double link_integrity_score{1.0};
    std::string summary{"All trust signals nominal"};
    std::vector<std::string> health_flags{};
};

inline DroneSecurityAssessment assess_security(const DroneSecurityInputs& in) {
    DroneSecurityAssessment out;
    out.link_integrity_score = std::clamp(
        (in.link_quality * 0.45) +
        (in.sync_confidence * 0.35) +
        ((in.peer_count > 0 ? 1.0 : 0.75) * 0.20),
        0.0,
        1.0);

    if (in.emergency_fault || in.battery_pct <= 7.0 || in.link_quality < 0.05) {
        out.state = DroneSecurityState::LAND_IMMEDIATELY;
        out.remote_command_allowed = false;
        out.telemetry_uplink_allowed = false;
        out.summary = "Critical trust or survivability margin lost, landing immediately";
        out.health_flags = {"security-critical", "landing-now"};
        return out;
    }

    if (in.hardened_profile && (!in.swarm_security_enabled || in.placeholder_secret)) {
        out.state = DroneSecurityState::ISOLATED_AUTONOMY;
        out.remote_command_allowed = false;
        out.telemetry_uplink_allowed = true;
        out.summary = "Hardened profile lost secure command trust, isolating into onboard autonomy";
        out.health_flags = {"security-isolated", "remote-command-blocked"};
        return out;
    }

    if (in.localization_lost && (in.sync_confidence < 0.35 || in.link_quality < 0.18)) {
        out.state = DroneSecurityState::SAFE_RETURN;
        out.remote_command_allowed = false;
        out.telemetry_uplink_allowed = true;
        out.summary = "Navigation trust degraded with unstable link, returning via safe autonomy";
        out.health_flags = {"security-safe-return", "remote-command-blocked"};
        return out;
    }

    if (in.hardened_profile && in.sync_confidence < 0.20) {
        out.state = DroneSecurityState::CONTROL_PLANE_UNTRUSTED;
        out.remote_command_allowed = false;
        out.telemetry_uplink_allowed = true;
        out.summary = "Secure timing and control-plane trust degraded, external control suspended";
        out.health_flags = {"control-plane-untrusted", "remote-command-blocked"};
        return out;
    }

    if (in.peer_clock_offset_ms > 120.0) {
        out.state = DroneSecurityState::PEER_SPOOF_SUSPECT;
        out.remote_command_allowed = false;
        out.telemetry_uplink_allowed = true;
        out.summary = "Peer clock offset exceeded trust window, spoof suspicion raised";
        out.health_flags = {"peer-spoof-suspect", "remote-command-blocked"};
        return out;
    }

    if (in.hardened_profile && in.link_quality < 0.12) {
        out.state = DroneSecurityState::COMMAND_REPLAY_SUSPECT;
        out.remote_command_allowed = false;
        out.telemetry_uplink_allowed = true;
        out.summary = "Command channel quality collapsed below hardened threshold, replay defense active";
        out.health_flags = {"replay-suspect", "remote-command-blocked"};
        return out;
    }

    if (in.link_quality < 0.18 || in.sync_confidence < 0.45) {
        out.state = DroneSecurityState::DEGRADED_LINK;
        out.remote_command_allowed = !in.hardened_profile;
        out.telemetry_uplink_allowed = true;
        out.summary = "Link integrity degraded, operating cautiously with reduced trust";
        out.health_flags = {"degraded-link"};
        return out;
    }

    out.state = DroneSecurityState::TRUSTED;
    out.remote_command_allowed = true;
    out.telemetry_uplink_allowed = true;
    out.summary = "All trust signals nominal";
    return out;
}

} // namespace drone::security
