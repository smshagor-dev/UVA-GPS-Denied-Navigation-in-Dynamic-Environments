#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"
#include "vio/Phase17ESKFEstimator.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace drone::vio;

namespace {

struct ReplayScenario {
    std::string name;
    int imu_steps{0};
    double dt_s{0.0025};
    Eigen::Vector3d accel{0.0, 0.0, 9.81};
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
    bool apply_periodic_zupt{false};
    bool inject_invalid_sample{false};
};

struct ReplayResult {
    std::string name;
    bool active_equivalent{false};
    bool shadow_success{false};
    bool deterministic{false};
    uint64_t shadow_processed_count{0};
    uint64_t shadow_failure_count{0};
    double position_delta_m{0.0};
    double velocity_delta_mps{0.0};
    double orientation_delta_deg{0.0};
    std::string active_checksum;
    std::string shadow_checksum;
};

EstimatorValidationConfig make_cfg(bool shadow_enabled) {
    EstimatorValidationConfig cfg;
    cfg.enable_shadow_estimator = shadow_enabled;
    cfg.shadow_comparison_enabled = shadow_enabled;
    cfg.shadow_max_queue_depth = 16384;
    cfg.shadow_max_lag_ms = 50.0;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    return cfg;
}

std::string checksum_for(const EstimatorStateSnapshot& snapshot) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(12) << snapshot.position_m.transpose() << '|'
        << snapshot.velocity_mps.transpose() << '|' << snapshot.orientation.coeffs().transpose()
        << '|' << snapshot.accel_bias.transpose() << '|' << snapshot.gyro_bias.transpose() << '|'
        << snapshot.covariance.trace;
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

    for (int i = 0; i < scenario.imu_steps; ++i) {
        const double timestamp = i * scenario.dt_s;
        MeasurementEnvelope env;
        env.type = MeasurementType::Imu;
        env.source_id = "imu";
        env.timestamp_s = timestamp;
        env.sequence_id = static_cast<uint64_t>(i);
        env.frame = MeasurementFrame::Body;
        env.payload = ImuMeasurementPayload{scenario.accel, scenario.gyro};
        (void)active_only.process_measurement(env);
        (void)with_shadow.process_measurement(env);
        if ((i % 64) == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }

        if (scenario.apply_periodic_zupt && (i % 250) == 0 && i > 0) {
            const auto zupt = make_manual_zupt_envelope(
                MeasurementStamp{timestamp, 100000u + static_cast<uint64_t>(i)});
            (void)active_only.process_measurement(zupt);
            (void)with_shadow.process_measurement(zupt);
        }
        if (scenario.inject_invalid_sample && i == (scenario.imu_steps / 2)) {
            auto invalid = env;
            invalid.timestamp_s += scenario.dt_s;
            invalid.sequence_id += 1;
            invalid.payload = ImuMeasurementPayload{
                Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 9.81},
                scenario.gyro};
            (void)active_only.process_measurement(invalid);
            (void)with_shadow.process_measurement(invalid);
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
    return result;
}

void write_report(const std::vector<ReplayResult>& results) {
    const std::filesystem::path out_dir = std::filesystem::path("artifacts") / "phase17";
    std::filesystem::create_directories(out_dir);
    const std::filesystem::path out_path = out_dir / "ekf_phase17_replay_report.json";

    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"schema_version\": 1,\n";
    oss << "  \"date\": \"2026-07-17\",\n";
    oss << "  \"scenarios\": [\n";
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        oss << "    {\n";
        oss << "      \"name\": \"" << r.name << "\",\n";
        oss << "      \"active_equivalent\": " << (r.active_equivalent ? "true" : "false") << ",\n";
        oss << "      \"shadow_success\": " << (r.shadow_success ? "true" : "false") << ",\n";
        oss << "      \"deterministic\": " << (r.deterministic ? "true" : "false") << ",\n";
        oss << "      \"shadow_processed_count\": " << r.shadow_processed_count << ",\n";
        oss << "      \"shadow_failure_count\": " << r.shadow_failure_count << ",\n";
        oss << "      \"position_delta_m\": " << r.position_delta_m << ",\n";
        oss << "      \"velocity_delta_mps\": " << r.velocity_delta_mps << ",\n";
        oss << "      \"orientation_delta_deg\": " << r.orientation_delta_deg << ",\n";
        oss << "      \"active_checksum\": \"" << r.active_checksum << "\",\n";
        oss << "      \"shadow_checksum\": \"" << r.shadow_checksum << "\"\n";
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
        {"stationary", 1200, 0.0025, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero(), true,
         false},
        {"constant_yaw", 1200, 0.0025, Eigen::Vector3d{0.0, 0.0, 9.81},
         Eigen::Vector3d{0.0, 0.0, 0.25}, true, false},
        {"long_duration_shadow", 8000, 0.0025, Eigen::Vector3d{0.05, 0.0, 9.81},
         Eigen::Vector3d{0.0, 0.0, 0.01}, true, false},
    };

    std::vector<ReplayResult> results;
    results.reserve(scenarios.size());
    bool ok = true;
    for (const auto& scenario : scenarios) {
        auto first = run_once(scenario);
        auto second = run_once(scenario);
        first.deterministic = first.active_checksum == second.active_checksum &&
                              first.shadow_checksum == second.shadow_checksum;
        ok = ok && first.active_equivalent && first.shadow_success && first.deterministic;
        results.push_back(std::move(first));
    }

    write_report(results);
    return ok ? 0 : 1;
}
