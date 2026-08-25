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
    bool estimator_config_valid{true};
    std::string estimator_mode{"baseline"};
    bool estimator_enable_experimental_hybrid{false};
    bool estimator_enable_fej{false};
    bool estimator_fej_enabled{false};
    bool estimator_fej_validation_checks{true};
    bool estimator_fej_diagnostics_enabled{true};
    bool estimator_enable_msckf{false};
    bool estimator_msckf_enabled{false};
    uint32_t estimator_msckf_max_camera_states{8};
    std::string estimator_msckf_eviction_policy{"oldest_first"};
    bool estimator_msckf_diagnostics_enabled{true};
    bool estimator_msckf_triangulation_enabled{false};
    uint32_t estimator_msckf_triangulation_minimum_observations{2};
    double estimator_msckf_triangulation_minimum_baseline{0.05};
    double estimator_msckf_triangulation_maximum_reprojection_error{2.5};
    double estimator_msckf_triangulation_minimum_depth{0.1};
    double estimator_msckf_triangulation_maximum_depth{50.0};
    bool estimator_msckf_update_enabled{false};
    double estimator_msckf_update_chi_square_probability{0.95};
    uint32_t estimator_msckf_update_minimum_track_length{3};
    uint32_t estimator_msckf_update_maximum_track_length{8};
    double estimator_msckf_update_maximum_residual{2.5};
    bool estimator_msckf_update_validation_checks{true};
    bool estimator_msckf_update_diagnostics_enabled{true};
    bool estimator_enable_loop_closure_correction{false};
    bool estimator_enable_automatic_zupt{false};
    bool estimator_enable_shadow_estimator{false};
    bool estimator_shadow_comparison_enabled{false};
    uint32_t estimator_shadow_max_queue_depth{128};
    double estimator_shadow_max_lag_ms{250.0};
    double estimator_shadow_position_divergence_m{2.0};
    double estimator_shadow_velocity_divergence_mps{1.5};
    double estimator_shadow_orientation_divergence_deg{15.0};
    uint32_t estimator_shadow_required_consecutive_divergent_samples{3};
    bool estimator_reject_non_finite_measurements{true};
    bool estimator_require_monotonic_timestamps{true};
    double estimator_max_imu_dt_s{0.1};
    double estimator_min_imu_dt_s{1.0e-6};
    double estimator_covariance_symmetry_tolerance{1.0e-9};
    double estimator_variance_negativity_tolerance{1.0e-10};
    double estimator_quaternion_min_norm{1.0e-6};
    double estimator_zupt_sigma_velocity_mps{0.01};
    bool estimator_stationary_detector_enabled{false};
    double estimator_stationary_detector_accel_threshold{0.15};
    double estimator_stationary_detector_gyro_threshold{0.02};
    uint32_t estimator_stationary_detector_window_size{20};
    uint32_t estimator_stationary_detector_enter_count{16};
    uint32_t estimator_stationary_detector_exit_count{10};
    double estimator_stationary_detector_minimum_stationary_time{0.15};
    double estimator_stationary_detector_accel_exit_threshold{0.25};
    double estimator_stationary_detector_gyro_exit_threshold{0.04};
    bool estimator_zupt_enabled{false};
    double estimator_zupt_velocity_noise_mps{0.01};
    double estimator_zupt_max_update_rate_hz{10.0};
    bool estimator_lidar_depth_correction_enabled{false};
    bool estimator_diagnostics_enabled{true};
    std::vector<std::string> estimator_errors{};
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
