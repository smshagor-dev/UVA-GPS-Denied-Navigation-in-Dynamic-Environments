#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace drone::runtime {

enum class RuntimeMode : uint8_t {
    SIMULATION = 0,
    BENCH,
    PRODUCTION,
    EDGE_SWARM,
};

struct RuntimeFileConfig {
    bool loaded{false};
    std::optional<RuntimeMode> runtime_mode;
    std::string anchor_config_path;
    std::string lidar_config_path;
    std::string detector_labels_path;
    bool estimator_config_present{false};
    bool estimator_config_valid{true};
    std::vector<std::string> estimator_errors{};
    std::string estimator_mode{"minimal"};
    bool estimator_enable_experimental_hybrid{false};
    bool estimator_enable_fej{false};
    bool estimator_enable_msckf{false};
    bool estimator_enable_loop_closure_correction{false};
    bool estimator_enable_automatic_zupt{false};
    bool estimator_shadow_requested{false};
    bool estimator_enable_shadow_estimator{false};
    bool estimator_shadow_effective{false};
#ifdef DRONE_ENABLE_EXPERIMENTAL_ESTIMATOR_SHADOW
    bool estimator_shadow_compile_time_supported{true};
#else
    bool estimator_shadow_compile_time_supported{false};
#endif
    bool estimator_shadow_comparison_enabled{true};
    std::string estimator_shadow_implementation{"minimal_eskf_clone"};
    size_t estimator_shadow_max_queue_depth{512};
    double estimator_shadow_max_lag_ms{100.0};
    double estimator_shadow_position_divergence_m{0.25};
    double estimator_shadow_velocity_divergence_mps{0.20};
    double estimator_shadow_orientation_divergence_deg{5.0};
    uint32_t estimator_shadow_required_consecutive_divergent_samples{10};
    bool estimator_reject_non_finite_measurements{true};
    bool estimator_require_monotonic_timestamps{true};
    double estimator_max_imu_dt_s{0.1};
    bool estimator_diagnostics_enabled{true};
};

struct RuntimeValidationInputs {
    RuntimeMode runtime_mode{RuntimeMode::SIMULATION};
    std::string anchor_config_path;
    bool has_csv_source{false};
    bool has_udp_source{false};
    bool has_serial_source{false};
};

struct RuntimeValidationResult {
    bool ok{true};
    std::vector<std::string> errors{};
};

struct AnchorDefinition {
    std::string source_id;
    uint32_t id{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

struct AnchorConfigLoadResult {
    bool ok{false};
    std::string coordinate_frame{"unknown"};
    std::string units{"unknown"};
    std::vector<AnchorDefinition> anchors{};
    std::vector<std::string> warnings{};
    std::vector<std::string> errors{};
};

struct LidarConfigLoadResult {
    bool ok{false};
    std::string host{"0.0.0.0"};
    uint16_t port{2368};
    std::string model{"generic_udp_cartesian_v1"};
    std::string frame_id{"lidar"};
    float min_range_m{0.3f};
    float max_range_m{80.0f};
    bool required{false};
    std::vector<std::string> errors{};
};

struct LidarRuntimeValidationInputs {
    RuntimeMode runtime_mode{RuntimeMode::SIMULATION};
    bool lidar_enabled{false};
    bool lidar_required{false};
    bool lidar_initialized{false};
};

[[nodiscard]] std::string_view to_string(RuntimeMode mode);
[[nodiscard]] RuntimeMode parse_runtime_mode(std::string_view value);
[[nodiscard]] RuntimeFileConfig load_runtime_file(const std::string& path);
[[nodiscard]] RuntimeValidationResult
validate_runtime_configuration(const RuntimeValidationInputs& input);
[[nodiscard]] AnchorConfigLoadResult load_anchor_config_json(const std::string& path);
[[nodiscard]] LidarConfigLoadResult load_lidar_config_json(const std::string& path);
[[nodiscard]] RuntimeValidationResult
validate_lidar_runtime_configuration(const LidarRuntimeValidationInputs& input);
[[nodiscard]] std::string determine_localization_data_source(RuntimeMode mode, bool used_synthetic,
                                                             bool used_csv_playback,
                                                             bool used_live_external,
                                                             bool has_measurements);

} // namespace drone::runtime
