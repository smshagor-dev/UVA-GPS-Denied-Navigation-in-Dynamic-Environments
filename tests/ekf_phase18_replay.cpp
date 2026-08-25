#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"
#include "vio/Phase17ESKFEstimator.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace drone::vio;

namespace {

struct ReplaySegment {
    int imu_steps{0};
    Eigen::Vector3d accel{0.0, 0.0, 9.81};
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
};

struct ReplayScenario {
    std::string name;
    double dt_s{0.01};
    std::vector<ReplaySegment> segments{};
    uint64_t expected_min_stationary_intervals{0};
    bool expect_zupt{false};
};

struct ReplayResult {
    std::string name;
    bool active_equivalent{false};
    bool shadow_success{false};
    bool deterministic{false};
    bool covariance_finite{false};
    bool zupt_only_when_stationary{false};
    uint64_t shadow_processed_count{0};
    uint64_t shadow_failure_count{0};
    uint64_t stationary_interval_count{0};
    uint64_t zupt_count{0};
    uint64_t rejected_zupt_count{0};
    double position_delta_m{0.0};
    double velocity_delta_mps{0.0};
    double orientation_delta_deg{0.0};
    double covariance_trace{0.0};
    std::string active_checksum;
    std::string shadow_checksum;
    std::vector<StationaryIntervalRecord> stationary_intervals{};
};

EstimatorValidationConfig make_cfg(bool shadow_enabled) {
    EstimatorValidationConfig cfg;
    cfg.enable_shadow_estimator = shadow_enabled;
    cfg.shadow_comparison_enabled = shadow_enabled;
    cfg.shadow_max_queue_depth = 16384;
    cfg.shadow_max_lag_ms = 50.0;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    cfg.stationary_detector.enabled = true;
    cfg.stationary_detector.accel_threshold_mps2 = 0.12;
    cfg.stationary_detector.gyro_threshold_rads = 0.02;
    cfg.stationary_detector.window_size = 12;
    cfg.stationary_detector.enter_count = 10;
    cfg.stationary_detector.exit_count = 6;
    cfg.stationary_detector.minimum_stationary_time_s = 0.05;
    cfg.stationary_detector.accel_exit_threshold_mps2 = 0.22;
    cfg.stationary_detector.gyro_exit_threshold_rads = 0.04;
    cfg.zupt.enabled = true;
    cfg.zupt.velocity_noise_mps = 0.01;
    cfg.zupt.max_update_rate_hz = 20.0;
    return cfg;
}

std::string checksum_for(const EstimatorStateSnapshot& snapshot) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(12) << snapshot.position_m.transpose() << '|'
        << snapshot.velocity_mps.transpose() << '|' << snapshot.orientation.coeffs().transpose()
        << '|' << snapshot.accel_bias.transpose() << '|' << snapshot.gyro_bias.transpose() << '|'
        << snapshot.covariance.trace << '|' << snapshot.stationary_detected << '|'
        << snapshot.stationary_duration_s << '|' << snapshot.zupt_updates_applied << '|'
        << snapshot.detector_state_changes;
    return oss.str();
}

ReplayResult run_once(const ReplayScenario& scenario) {
    auto active_only = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "phase17"));
    active_only.configure_validation(make_cfg(false));
    active_only.initialize();

    auto with_shadow = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "phase17"));
    with_shadow.configure_validation(make_cfg(true));
    with_shadow.initialize();
    (void)with_shadow.start();

    uint64_t sequence_id = 0;
    double timestamp_s = 0.0;
    for (const auto& segment : scenario.segments) {
        for (int i = 0; i < segment.imu_steps; ++i) {
            MeasurementEnvelope env;
            env.type = MeasurementType::Imu;
            env.source_id = "imu";
            env.timestamp_s = timestamp_s;
            env.sequence_id = sequence_id++;
            env.frame = MeasurementFrame::Body;
            env.payload = ImuMeasurementPayload{segment.accel, segment.gyro};
            (void)active_only.process_measurement(env);
            (void)with_shadow.process_measurement(env);
            timestamp_s += scenario.dt_s;
            if ((sequence_id % 64u) == 0u) {
                std::this_thread::sleep_for(std::chrono::milliseconds{1});
            }
        }
    }

    with_shadow.flush_shadow();
    with_shadow.stop();
    const auto active_a = active_only.active_snapshot();
    const auto active_b = with_shadow.active_snapshot();
    const auto shadow = with_shadow.shadow_snapshot();
    const auto diagnostics = with_shadow.diagnostics();

    ReplayResult result;
    result.name = scenario.name;
    result.active_equivalent =
        (active_a.position_m - active_b.position_m).norm() < 1.0e-12 &&
        (active_a.velocity_mps - active_b.velocity_mps).norm() < 1.0e-12 &&
        std::abs(std::abs(active_a.orientation.dot(active_b.orientation)) - 1.0) < 1.0e-12;
    result.shadow_success = diagnostics.shadow_failure_count == 0 && shadow.has_value();
    result.shadow_processed_count = diagnostics.shadow_processed_count;
    result.shadow_failure_count = diagnostics.shadow_failure_count;
    result.position_delta_m = diagnostics.last_comparison.position_delta_norm_m;
    result.velocity_delta_mps = diagnostics.last_comparison.velocity_delta_norm_mps;
    result.orientation_delta_deg = diagnostics.last_comparison.orientation_delta_deg;
    result.active_checksum = checksum_for(active_b);
    result.shadow_checksum = shadow.has_value() ? checksum_for(*shadow) : "none";
    if (shadow.has_value()) {
        result.stationary_interval_count =
            static_cast<uint64_t>(shadow->stationary_intervals.size());
        result.zupt_count = shadow->zupt_updates_applied;
        result.rejected_zupt_count = shadow->zupt_updates_rejected;
        result.stationary_intervals = shadow->stationary_intervals;
        result.covariance_trace = shadow->covariance.trace;
        result.covariance_finite = std::isfinite(shadow->covariance.trace) &&
                                   shadow->covariance.position_std_m.array().isFinite().all();
        uint64_t interval_zupt_total = 0;
        for (const auto& interval : shadow->stationary_intervals) {
            interval_zupt_total += interval.zupt_updates;
        }
        result.zupt_only_when_stationary =
            result.zupt_count == interval_zupt_total &&
            (result.zupt_count == 0 || !shadow->stationary_intervals.empty());
    }

    if (result.stationary_interval_count < scenario.expected_min_stationary_intervals) {
        result.shadow_success = false;
    }
    if (scenario.expect_zupt && result.zupt_count == 0) {
        result.shadow_success = false;
    }
    return result;
}

void write_report(const std::vector<ReplayResult>& results) {
    const std::filesystem::path out_dir = std::filesystem::path("artifacts") / "phase18";
    std::filesystem::create_directories(out_dir);
    const std::filesystem::path out_path = out_dir / "ekf_phase18_replay_report.json";

    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"schema_version\": 1,\n";
    oss << "  \"date\": \"2026-07-18\",\n";
    oss << "  \"scenarios\": [\n";
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        oss << "    {\n";
        oss << "      \"name\": \"" << r.name << "\",\n";
        oss << "      \"active_equivalent\": " << (r.active_equivalent ? "true" : "false") << ",\n";
        oss << "      \"shadow_success\": " << (r.shadow_success ? "true" : "false") << ",\n";
        oss << "      \"deterministic\": " << (r.deterministic ? "true" : "false") << ",\n";
        oss << "      \"covariance_finite\": " << (r.covariance_finite ? "true" : "false") << ",\n";
        oss << "      \"zupt_only_when_stationary\": "
            << (r.zupt_only_when_stationary ? "true" : "false") << ",\n";
        oss << "      \"shadow_processed_count\": " << r.shadow_processed_count << ",\n";
        oss << "      \"shadow_failure_count\": " << r.shadow_failure_count << ",\n";
        oss << "      \"stationary_interval_count\": " << r.stationary_interval_count << ",\n";
        oss << "      \"zupt_count\": " << r.zupt_count << ",\n";
        oss << "      \"rejected_zupt_count\": " << r.rejected_zupt_count << ",\n";
        oss << "      \"position_delta_m\": " << r.position_delta_m << ",\n";
        oss << "      \"velocity_delta_mps\": " << r.velocity_delta_mps << ",\n";
        oss << "      \"orientation_delta_deg\": " << r.orientation_delta_deg << ",\n";
        oss << "      \"covariance_trace\": " << r.covariance_trace << ",\n";
        oss << "      \"active_checksum\": \"" << r.active_checksum << "\",\n";
        oss << "      \"shadow_checksum\": \"" << r.shadow_checksum << "\",\n";
        oss << "      \"stationary_intervals\": [\n";
        for (size_t j = 0; j < r.stationary_intervals.size(); ++j) {
            const auto& interval = r.stationary_intervals[j];
            oss << "        {\"start_timestamp_s\": " << interval.start_timestamp_s
                << ", \"end_timestamp_s\": " << interval.end_timestamp_s
                << ", \"duration_s\": " << interval.duration_s
                << ", \"zupt_updates\": " << interval.zupt_updates << "}"
                << (j + 1 < r.stationary_intervals.size() ? "," : "") << "\n";
        }
        oss << "      ]\n";
        oss << "    }" << (i + 1 < results.size() ? "," : "") << "\n";
    }
    oss << "  ]\n";
    oss << "}\n";

    std::ofstream out(out_path, std::ios::binary);
    out << oss.str();
}

} // namespace

int main() {
    std::vector<ReplayScenario> scenarios{
        {"stationary_long",
         0.01,
         {{240, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()}},
         1,
         true},
        {"stop_and_go",
         0.01,
         {{60, Eigen::Vector3d{0.8, 0.0, 9.81}, Eigen::Vector3d::Zero()},
          {160, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()},
          {60, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d{0.0, 0.0, 0.08}}},
         1,
         true},
        {"intermittent_motion",
         0.01,
         {{80, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()},
          {40, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d{0.0, 0.0, 0.09}},
          {80, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()},
          {40, Eigen::Vector3d{0.25, 0.0, 9.81}, Eigen::Vector3d::Zero()},
          {80, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()}},
         2,
         true},
    };

    std::vector<ReplayResult> results;
    results.reserve(scenarios.size());
    bool ok = true;
    for (const auto& scenario : scenarios) {
        auto first = run_once(scenario);
        auto second = run_once(scenario);
        first.deterministic = first.active_checksum == second.active_checksum &&
                              first.shadow_checksum == second.shadow_checksum &&
                              first.stationary_interval_count == second.stationary_interval_count &&
                              first.zupt_count == second.zupt_count;
        ok = ok && first.active_equivalent && first.shadow_success && first.deterministic &&
             first.covariance_finite && first.zupt_only_when_stationary;
        results.push_back(std::move(first));
    }

    write_report(results);
    return ok ? 0 : 1;
}
