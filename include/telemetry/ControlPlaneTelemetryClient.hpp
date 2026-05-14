// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#include <Eigen/Core>

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string_view>
#include <string>
#include <utility>
#include <vector>

namespace drone::telemetry {

struct SensorVector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct CameraTelemetryPayload {
    std::string status{"unavailable"};
    double fps{0.0};
    double frame_age_ms{0.0};
    std::string resolution{"N/A"};
    int dropped_frames{0};
    std::string source{"unavailable"};
    std::string preview_url{};
    std::string latest_frame_ref{};
};

struct IMUTelemetryPayload {
    std::string status{"unavailable"};
    double sample_rate_hz{0.0};
    double last_sample_age_ms{0.0};
    SensorVector3 accel{};
    SensorVector3 gyro{};
    std::string health{"unavailable"};
    std::string source{"unavailable"};
};

struct LidarPoint2DPayload {
    double x{0.0};
    double y{0.0};
    double intensity{0.0};
};

struct LidarTelemetryPayload {
    std::string status{"unavailable"};
    double packet_rate_hz{0.0};
    double scan_age_ms{0.0};
    int point_count{0};
    std::vector<LidarPoint2DPayload> points_2d{};
    double min_range_m{0.0};
    double max_range_m{0.0};
    std::string source{"unavailable"};
};

struct TDOAAnchorTelemetryPayload {
    std::string id{};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    bool visible{false};
    double last_seen_ms{0.0};
};

struct TDOATelemetryPayload {
    std::string status{"unavailable"};
    std::string source{"unavailable"};
    int visible_anchor_count{0};
    std::vector<TDOAAnchorTelemetryPayload> anchors{};
    SensorVector3 estimated_position{};
    std::string calibration_warning{};
};

struct ReplayTelemetryPayload {
    std::string status{"unavailable"};
    bool active{false};
    std::string file_name{};
    double progress{0.0};
    double current_time{0.0};
    std::vector<double> confidence_series{};
    std::string source{"unavailable"};
};

struct TelemetrySnapshot {
    uint32_t drone_id{0};
    std::string source{"unavailable"};
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
    std::string localization_data_source{"unavailable"};
    std::string localization_state{"nominal"};
    double localization_confidence{1.0};
    double tdoa_confidence{0.0};
    double confidence_trend{0.0};
    int relocalization_count{0};
    int visible_anchor_count{0};
    double occupancy_ratio{0.0};
    double sync_confidence{1.0};
    double imu_camera_offset_ms{0.0};
    int peer_count{0};
    int stale_peer_count{0};
    std::string mesh_topology_mode{"adaptive_mesh"};
    std::string local_consensus_state{"single_node"};
    uint64_t local_consensus_epoch{0};
    double peer_latency_ms{0.0};
    double mesh_bandwidth_kbps{0.0};
    std::string edge_serialization_mode{"json"};
    double edge_average_packet_size_bytes{0.0};
    double edge_bandwidth_savings_estimate_pct{0.0};
    double edge_packet_encode_latency_us{0.0};
    bool disconnected_operation{false};
    std::string edge_health_status{"nominal"};
    std::string edge_autonomy_state{"backend_assisted"};
    std::string edge_inference_status{"idle"};
    double edge_inference_fps{0.0};
    double edge_inference_confidence{0.0};
    int local_obstacle_count{0};
    int shared_obstacle_count{0};
    std::string security_state{"TRUSTED"};
    std::string security_summary{"All trust signals nominal"};
    std::string security_transition_reason{"initial-trust"};
    bool remote_command_allowed{true};
    bool telemetry_uplink_allowed{true};
    double link_integrity_score{1.0};
    uint64_t trust_epoch{1};
    double last_auth_failure_at_s{0.0};
    double tamper_score{0.0};
    std::string firmware_measurement{"lab-local-build"};
    std::string firmware_version{"0.0.0"};
    std::string secure_boot_state{"LAB_BOOT"};
    std::string boot_trust_summary{"Lab boot trust bypassed"};
    uint64_t rollback_counter{0};
    bool maintenance_mode{false};
    std::string update_channel_state{"idle"};
    std::string safety_state{"NORMAL"};
    std::string safety_summary{"Nominal safety envelope"};
    std::vector<std::string> health_flags{};
    CameraTelemetryPayload camera{};
    IMUTelemetryPayload imu{};
    LidarTelemetryPayload lidar{};
    TDOATelemetryPayload tdoa{};
    ReplayTelemetryPayload replay{};
};

class ControlPlaneTelemetryClient {
public:
    struct TLSRuntimeConfig {
        bool skip_verify{false};
        std::string ca_file{};
        std::string client_pfx_file{};
        std::string client_pfx_password{};
    };
    struct ParsedEndpoint {
        bool https{false};
        std::string host{"127.0.0.1"};
        uint16_t port{8080};
        std::string path{"/api/v1/telemetry"};
    };

    [[nodiscard]] static TLSRuntimeConfig load_tls_runtime_config();
    struct HttpResponse {
        bool transport_ok{false};
        int status_code{0};
        std::string status_text{};
    };

    using HeaderList = std::vector<std::pair<std::string, std::string>>;
    using TransportFn = std::function<HttpResponse(
        const ParsedEndpoint&,
        std::string_view,
        const HeaderList&,
        int)>;

    explicit ControlPlaneTelemetryClient(std::string backend_url,
                                         std::string auth_token = {},
                                         int interval_ms = 1000,
                                         int timeout_ms = 1500,
                                         TransportFn transport = {});

    [[nodiscard]] bool enabled() const;
    [[nodiscard]] bool should_publish(std::chrono::steady_clock::time_point now) const;
    bool publish(const TelemetrySnapshot& snapshot, std::chrono::steady_clock::time_point now);
    [[nodiscard]] std::string last_status() const;
    [[nodiscard]] int consecutive_failures() const;
    [[nodiscard]] static std::string serialize_payload(const TelemetrySnapshot& snapshot);

private:
    [[nodiscard]] static ParsedEndpoint parse_backend_url(const std::string& backend_url);
    [[nodiscard]] static HeaderList build_headers(std::string_view auth_token, uint32_t drone_id);
    [[nodiscard]] static HttpResponse default_transport(const ParsedEndpoint& endpoint,
                                                        std::string_view body,
                                                        const HeaderList& headers,
                                                        int timeout_ms);
    [[nodiscard]] std::chrono::milliseconds current_backoff() const;
    void mark_result(bool ok, std::string status, std::chrono::steady_clock::time_point now);

    ParsedEndpoint endpoint_{};
    TLSRuntimeConfig tls_{};
    std::chrono::milliseconds interval_{1000};
    std::chrono::milliseconds timeout_{1500};
    std::chrono::steady_clock::time_point last_attempt_{};
    std::chrono::steady_clock::time_point next_retry_not_before_{};
    std::string last_status_{"disabled"};
    std::string auth_token_{};
    bool enabled_{false};
    int consecutive_failures_{0};
    TransportFn transport_{};
    mutable std::mutex state_mutex_{};
};

} // namespace drone::telemetry
