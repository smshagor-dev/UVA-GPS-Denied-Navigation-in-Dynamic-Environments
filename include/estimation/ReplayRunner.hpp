#pragma once

#include "estimation/EstimatorCoordinator.hpp"
#include "estimation/MeasurementAdapters.hpp"
#include "estimation/MinimalEskfAdapter.hpp"

#include <filesystem>
#include <optional>
#include <string>

namespace drone::estimation {

enum class ReplayMode : uint8_t {
    ACTIVE_ONLY = 0,
    ACTIVE_WITH_IDENTICAL_SHADOW,
};

struct ReplayLimits {
    size_t max_file_bytes{512 * 1024};
    size_t max_record_count{100000};
    size_t max_string_length{128};
};

struct ReplayRunConfig {
    ReplayMode mode{ReplayMode::ACTIVE_ONLY};
    ReplayLimits limits{};
    drone::vio::EKFConfig ekf_config{};
    drone::vio::EstimatorValidationConfig validation{};
};

struct ReplayReport {
    int report_schema_version{1};
    int replay_input_schema_version{0};
    std::string replay_mode{"active_only"};
    std::string active_estimator_name{"minimal_eskf"};
    std::string shadow_estimator_name{"disabled"};
    std::string coordinate_frame{"unknown"};
    size_t input_record_count{0};
    size_t processed_record_count{0};
    uint64_t accepted_update_count{0};
    uint64_t rejected_update_count{0};
    uint64_t invalid_record_count{0};
    uint64_t timestamp_violation_count{0};
    uint64_t unsupported_measurement_count{0};
    drone::vio::PoseEstimate final_pose{};
    double final_position_uncertainty_m{0.0};
    std::string active_estimator_health{"initializing"};
    std::string shadow_estimator_health{"disabled"};
    double active_shadow_position_delta_m{0.0};
    double active_shadow_velocity_delta_mps{0.0};
    double active_shadow_orientation_delta_deg{0.0};
    uint64_t comparable_snapshot_count{0};
    uint64_t skipped_comparison_count{0};
    uint64_t shadow_dropped_event_count{0};
    size_t shadow_queue_high_water_mark{0};
    std::string shadow_synchronization_state{"disabled"};
    std::string deterministic_result_hash{};
    double average_propagation_latency_us{0.0};
    double maximum_propagation_latency_us{0.0};
    double average_measurement_update_latency_us{0.0};
    double maximum_measurement_update_latency_us{0.0};
    bool success{false};
    std::string failure_reason{"none"};
};

struct ReplayRunResult {
    ReplayReport report{};
    int exit_code{1};
};

[[nodiscard]] ReplayRunResult run_replay_file(const std::filesystem::path& input_path,
                                              const ReplayRunConfig& config);
[[nodiscard]] std::string replay_report_json(const ReplayReport& report);
[[nodiscard]] std::string to_string(ReplayMode mode);

} // namespace drone::estimation
