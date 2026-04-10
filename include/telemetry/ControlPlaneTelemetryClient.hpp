// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#include <Eigen/Core>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace drone::telemetry {

struct TelemetrySnapshot {
    uint32_t drone_id{0};
    std::string cluster_id{"cluster-01"};
    std::string role{"FOLLOWER"};
    std::string connectivity{"Mesh"};
    bool reachable{true};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d attitude_rpy{Eigen::Vector3d::Zero()};
    Eigen::Vector3d thrust_vector{Eigen::Vector3d{0.0, 0.0, 9.81}};
    double commanded_altitude_m{0.0};
    double commanded_speed_mps{0.0};
    double drift_m{0.0};
    double battery_pct{0.0};
    double rssi_dbm{-100.0};
    double cpu_temp_c{0.0};
    double gpu_load_pct{0.0};
    std::string mission_state{"standby"};
    std::string localization_source{"vision-inertial"};
    std::string localization_state{"nominal"};
    double localization_confidence{1.0};
    double tdoa_confidence{0.0};
    double confidence_trend{0.0};
    int relocalization_count{0};
    int visible_anchor_count{0};
    double occupancy_ratio{0.0};
    double sync_confidence{1.0};
    double imu_camera_offset_ms{0.0};
    std::string security_state{"TRUSTED"};
    std::string security_summary{"All trust signals nominal"};
    bool remote_command_allowed{true};
    bool telemetry_uplink_allowed{true};
    double link_integrity_score{1.0};
    std::vector<std::string> health_flags{};
};

class ControlPlaneTelemetryClient {
public:
    explicit ControlPlaneTelemetryClient(std::string backend_url, int interval_ms = 1000);

    [[nodiscard]] bool enabled() const;
    [[nodiscard]] bool should_publish(std::chrono::steady_clock::time_point now) const;
    bool publish(const TelemetrySnapshot& snapshot, std::chrono::steady_clock::time_point now);
    [[nodiscard]] std::string last_status() const { return last_status_; }

private:
    struct ParsedEndpoint {
        bool https{false};
        std::string host{"127.0.0.1"};
        uint16_t port{8080};
        std::string path{"/api/v1/telemetry"};
    };

    [[nodiscard]] static ParsedEndpoint parse_backend_url(const std::string& backend_url);
    [[nodiscard]] static std::string build_payload(const TelemetrySnapshot& snapshot);
    void mark_result(bool ok, std::string status, std::chrono::steady_clock::time_point now);

    ParsedEndpoint endpoint_{};
    std::chrono::milliseconds interval_{1000};
    std::chrono::steady_clock::time_point last_attempt_{};
    std::string last_status_{"disabled"};
    bool enabled_{false};
};

} // namespace drone::telemetry
