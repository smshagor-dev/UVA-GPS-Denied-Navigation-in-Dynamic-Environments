// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <optional>
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
    bool geofence_clear{true};
    bool no_fly_lock{false};
    bool swarm_consistency_ok{true};
    bool backend_trust_ok{true};
    bool command_channel_fresh{true};
    double battery_pct{100.0};
    double link_quality{1.0};
    double sync_confidence{1.0};
    double peer_clock_offset_ms{0.0};
    double localization_confidence{1.0};
    double issuer_trust_score{1.0};
    double tamper_score{0.0};
    size_t peer_count{0};
};

struct DroneSecurityAssessment {
    DroneSecurityState state{DroneSecurityState::TRUSTED};
    bool remote_command_allowed{true};
    bool telemetry_uplink_allowed{true};
    double link_integrity_score{1.0};
    uint64_t trust_epoch{1};
    double last_auth_failure_at_s{0.0};
    double tamper_score{0.0};
    std::string summary{"All trust signals nominal"};
    std::string transition_reason{"initial-trust"};
    std::string firmware_measurement{"lab-local-build"};
    std::vector<std::string> health_flags{};
};

class SecurityRuntimeMonitor {
public:
    explicit SecurityRuntimeMonitor(std::string firmware_measurement = "lab-local-build")
        : firmware_measurement_(std::move(firmware_measurement)) {}

    void note_mesh_security_error(std::string_view error, double now_s) {
        const std::string normalized = normalize_text(error);
        if (normalized.empty()) {
            return;
        }
        if (normalized == last_mesh_security_error_) {
            return;
        }
        last_mesh_security_error_ = normalized;
        if (contains(normalized, "replay")) {
            ++replay_failures_;
            last_replay_failure_at_s_ = now_s;
            return;
        }
        if (contains(normalized, "ledger chain mismatch") || contains(normalized, "spoof")) {
            ++peer_spoof_events_;
            last_peer_spoof_at_s_ = now_s;
            return;
        }
        ++auth_failures_;
        last_auth_failure_at_s_ = now_s;
    }

    void note_remote_command_rejection(std::string_view reason, bool critical, double now_s) {
        const std::string normalized = normalize_text(reason);
        if (normalized.empty()) {
            return;
        }
        if (contains(normalized, "replay") || contains(normalized, "freshness")) {
            ++replay_failures_;
            last_replay_failure_at_s_ = now_s;
            return;
        }
        if (contains(normalized, "issuer trust") || contains(normalized, "authorization") || contains(normalized, "blocked")) {
            auth_failures_ += critical ? 2u : 1u;
            last_auth_failure_at_s_ = now_s;
        }
    }

    void note_control_plane_status(std::string_view status, double now_s) {
        const std::string normalized = normalize_text(status);
        if (normalized.empty() || normalized == last_control_plane_status_) {
            return;
        }
        last_control_plane_status_ = normalized;
        if (!contains(normalized, "error:")) {
            return;
        }
        ++control_plane_failures_;
        last_control_plane_failure_at_s_ = now_s;
        if (contains(normalized, "certificate") || contains(normalized, "policy rejected backend")) {
            backend_identity_failed_ = true;
        }
    }

    DroneSecurityAssessment evaluate(const DroneSecurityInputs& in, double now_s) {
        decay_failures(now_s);

        DroneSecurityAssessment out;
        out.tamper_score = std::clamp(
            in.tamper_score +
                (replay_failures_ > 0 ? 0.20 : 0.0) +
                (peer_spoof_events_ > 0 ? 0.25 : 0.0) +
                (control_plane_failures_ > 0 ? 0.12 : 0.0) +
                (auth_failures_ > 0 ? 0.10 : 0.0),
            0.0,
            1.0);
        out.link_integrity_score = std::clamp(
            (in.link_quality * 0.35) +
                (in.sync_confidence * 0.25) +
                (in.localization_confidence * 0.15) +
                (in.issuer_trust_score * 0.15) +
                ((in.peer_count > 0 ? 1.0 : 0.65) * 0.10),
            0.0,
            1.0);
        out.firmware_measurement = firmware_measurement_;
        out.last_auth_failure_at_s = last_auth_failure_at_s_;

        std::string reason = "nominal";
        if (in.emergency_fault || in.battery_pct <= 7.0 || in.link_quality < 0.05) {
            out.state = DroneSecurityState::LAND_IMMEDIATELY;
            out.remote_command_allowed = false;
            out.telemetry_uplink_allowed = false;
            out.summary = "Critical trust or survivability margin lost, landing immediately";
            out.health_flags = {"security-critical", "landing-now"};
            reason = "critical-margin-lost";
        } else if (in.hardened_profile && (!in.swarm_security_enabled || in.placeholder_secret)) {
            out.state = DroneSecurityState::ISOLATED_AUTONOMY;
            out.remote_command_allowed = false;
            out.telemetry_uplink_allowed = true;
            out.summary = "Hardened profile lost secure command trust, isolating into onboard autonomy";
            out.health_flags = {"security-isolated", "remote-command-blocked"};
            reason = "secure-command-trust-lost";
        } else if (in.no_fly_lock || !in.geofence_clear || !in.swarm_consistency_ok ||
                   (in.localization_lost && (in.sync_confidence < 0.35 || in.link_quality < 0.18))) {
            out.state = DroneSecurityState::SAFE_RETURN;
            out.remote_command_allowed = false;
            out.telemetry_uplink_allowed = true;
            out.summary = in.no_fly_lock
                ? "Mission lock engaged, safe return autonomy enforced"
                : (!in.geofence_clear
                    ? "Geofence trust violated, safe return autonomy enforced"
                    : (!in.swarm_consistency_ok
                        ? "Swarm consistency degraded, holding safe autonomous return"
                        : "Navigation trust degraded with unstable link, returning via safe autonomy"));
            out.health_flags = {"security-safe-return", "remote-command-blocked"};
            if (in.no_fly_lock) {
                out.health_flags.push_back("mission-lock");
            }
            if (!in.geofence_clear) {
                out.health_flags.push_back("geofence-blocked");
            }
            if (!in.swarm_consistency_ok) {
                out.health_flags.push_back("swarm-inconsistent");
            }
            reason = in.no_fly_lock ? "mission-lock" : (!in.geofence_clear ? "geofence-breach" : "safety-consistency-failed");
        } else if (replay_failures_ >= 2u || (in.hardened_profile && !in.command_channel_fresh)) {
            out.state = DroneSecurityState::COMMAND_REPLAY_SUSPECT;
            out.remote_command_allowed = false;
            out.telemetry_uplink_allowed = true;
            out.summary = "Command channel freshness degraded, replay defense active";
            out.health_flags = {"replay-suspect", "remote-command-blocked"};
            reason = "replay-detected";
        } else if (peer_spoof_events_ > 0u || in.peer_clock_offset_ms > 120.0) {
            out.state = DroneSecurityState::PEER_SPOOF_SUSPECT;
            out.remote_command_allowed = false;
            out.telemetry_uplink_allowed = true;
            out.summary = "Peer identity or timing mismatch exceeded trust window";
            out.health_flags = {"peer-spoof-suspect", "remote-command-blocked"};
            reason = "peer-identity-mismatch";
        } else if ((in.hardened_profile && (!in.backend_trust_ok || backend_identity_failed_)) ||
                   control_plane_failures_ > 0u ||
                   (in.hardened_profile && in.sync_confidence < 0.20)) {
            out.state = DroneSecurityState::CONTROL_PLANE_UNTRUSTED;
            out.remote_command_allowed = false;
            out.telemetry_uplink_allowed = true;
            out.summary = "Backend or timing trust degraded, external control suspended";
            out.health_flags = {"control-plane-untrusted", "remote-command-blocked"};
            reason = "backend-trust-failed";
        } else if (auth_failures_ >= 3u || (in.hardened_profile && in.issuer_trust_score < 0.45)) {
            out.state = DroneSecurityState::AUTH_SUSPECT;
            out.remote_command_allowed = false;
            out.telemetry_uplink_allowed = true;
            out.summary = "Repeated authorization failures raised onboard auth suspicion";
            out.health_flags = {"auth-suspect", "remote-command-blocked"};
            reason = "authorization-failures";
        } else if (in.link_quality < 0.18 || in.sync_confidence < 0.45 || in.localization_confidence < 0.35) {
            out.state = DroneSecurityState::DEGRADED_LINK;
            out.remote_command_allowed = !in.hardened_profile;
            out.telemetry_uplink_allowed = true;
            out.summary = "Link integrity degraded, operating cautiously with reduced trust";
            out.health_flags = {"degraded-link"};
            reason = "degraded-link";
        } else {
            out.state = DroneSecurityState::TRUSTED;
            out.remote_command_allowed = true;
            out.telemetry_uplink_allowed = true;
            out.summary = "All trust signals nominal";
            reason = "nominal";
        }

        if (out.tamper_score >= 0.60) {
            out.health_flags.push_back("tamper-suspect");
        }
        if (!out.remote_command_allowed && !flag_present(out.health_flags, "remote-command-blocked")) {
            out.health_flags.push_back("remote-command-blocked");
        }

        if (out.state != current_state_) {
            ++trust_epoch_;
            current_state_ = out.state;
            out.transition_reason = reason;
        } else {
            out.transition_reason = last_transition_reason_.empty() ? reason : last_transition_reason_;
        }
        last_transition_reason_ = reason;
        out.trust_epoch = trust_epoch_;
        latest_ = out;
        return out;
    }

    [[nodiscard]] const DroneSecurityAssessment& latest() const {
        return latest_;
    }

private:
    static bool contains(std::string_view haystack, std::string_view needle) {
        return haystack.find(needle) != std::string_view::npos;
    }

    static std::string normalize_text(std::string_view value) {
        std::string out(value);
        std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        return out;
    }

    static bool flag_present(const std::vector<std::string>& flags, std::string_view needle) {
        return std::any_of(flags.begin(), flags.end(), [&](const std::string& item) {
            return item == needle;
        });
    }

    void decay_failures(double now_s) {
        const auto expire_if_old = [now_s](double last_seen_s, double window_s) {
            return last_seen_s > 0.0 && (now_s - last_seen_s) > window_s;
        };
        if (expire_if_old(last_auth_failure_at_s_, 90.0)) {
            auth_failures_ = 0u;
        }
        if (expire_if_old(last_replay_failure_at_s_, 120.0)) {
            replay_failures_ = 0u;
        }
        if (expire_if_old(last_peer_spoof_at_s_, 120.0)) {
            peer_spoof_events_ = 0u;
        }
        if (expire_if_old(last_control_plane_failure_at_s_, 120.0)) {
            control_plane_failures_ = 0u;
            backend_identity_failed_ = false;
        }
    }

    DroneSecurityState current_state_{DroneSecurityState::TRUSTED};
    uint64_t trust_epoch_{1};
    uint32_t auth_failures_{0};
    uint32_t replay_failures_{0};
    uint32_t peer_spoof_events_{0};
    uint32_t control_plane_failures_{0};
    bool backend_identity_failed_{false};
    double last_auth_failure_at_s_{0.0};
    double last_replay_failure_at_s_{0.0};
    double last_peer_spoof_at_s_{0.0};
    double last_control_plane_failure_at_s_{0.0};
    std::string last_mesh_security_error_{};
    std::string last_control_plane_status_{};
    std::string last_transition_reason_{"initial-trust"};
    std::string firmware_measurement_{};
    DroneSecurityAssessment latest_{};
};

inline DroneSecurityAssessment assess_security(const DroneSecurityInputs& in) {
    SecurityRuntimeMonitor monitor;
    return monitor.evaluate(in, 0.0);
}

} // namespace drone::security
