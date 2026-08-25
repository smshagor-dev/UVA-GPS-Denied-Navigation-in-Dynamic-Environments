#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"
#include "vio/Phase17ESKFEstimator.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace drone::vio;

namespace {

struct ReplayScenario {
    std::string name;
    int camera_updates{0};
    uint32_t max_camera_states{4};
    bool reset_mid_run{false};
};

struct ReplayResult {
    std::string name;
    bool active_equivalent{false};
    bool deterministic{false};
    bool covariance_finite{false};
    bool shadow_only_msckf{true};
    uint64_t maximum_window_size{0};
    uint64_t states_created{0};
    uint64_t states_removed{0};
    uint64_t final_window_size{0};
    std::string checksum;
};

EstimatorValidationConfig make_cfg() {
    EstimatorValidationConfig cfg;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    cfg.enable_shadow_estimator = true;
    cfg.shadow_comparison_enabled = true;
    cfg.shadow_max_queue_depth = 1024;
    cfg.shadow_max_lag_ms = 50.0;
    return cfg;
}

MsckfConfig make_msckf_cfg(uint32_t max_camera_states, bool diagnostics_enabled = true) {
    MsckfConfig cfg;
    cfg.enabled = true;
    cfg.max_camera_states = max_camera_states;
    cfg.eviction_policy = "oldest_first";
    cfg.diagnostics_enabled = diagnostics_enabled;
    return cfg;
}

Eigen::Matrix3d camera_intrinsics() {
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    K(0, 0) = 320.0;
    K(1, 1) = 320.0;
    K(0, 2) = 320.0;
    K(1, 2) = 240.0;
    K(2, 2) = 1.0;
    return K;
}

Eigen::Vector2d project_feature(const Phase17ESKFEstimator& ekf, const Eigen::Vector3d& feature) {
    const auto state = ekf.state();
    const Eigen::Matrix3d R = state.orientation.toRotationMatrix();
    const Eigen::Vector3d p_c = R.transpose() * (feature - state.position);
    const auto K = camera_intrinsics();
    return {
        (K(0, 0) * p_c.x() / p_c.z()) + K(0, 2),
        (K(1, 1) * p_c.y() / p_c.z()) + K(1, 2),
    };
}

std::vector<Eigen::Vector2d> project_features(const Phase17ESKFEstimator& ekf,
                                              const std::vector<Eigen::Vector3d>& features) {
    std::vector<Eigen::Vector2d> pixels;
    pixels.reserve(features.size());
    for (const auto& feature : features) {
        pixels.push_back(project_feature(ekf, feature));
    }
    return pixels;
}

void propagate(Phase17ESKFEstimator& ekf, double& timestamp_s, uint64_t steps) {
    for (uint64_t i = 0; i < steps; ++i) {
        const auto result = ekf.process_imu_measurement(
            Eigen::Vector3d{0.04, 0.0, 9.81}, Eigen::Vector3d{0.0, 0.0, 0.01}, timestamp_s);
        if (result != EstimatorOperationResult::Accepted) {
            break;
        }
        timestamp_s += 0.01;
    }
}

bool active_equivalence_control() {
    auto active_only = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "phase17"));
    EstimatorValidationConfig off = make_cfg();
    off.enable_shadow_estimator = false;
    off.shadow_comparison_enabled = false;
    active_only.configure_validation(off);
    active_only.initialize();

    auto with_shadow = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "phase17"));
    with_shadow.configure_validation(make_cfg());
    with_shadow.configure_shadow_msckf(make_msckf_cfg(4));
    with_shadow.initialize();
    (void)with_shadow.start();

    for (int i = 0; i < 240; ++i) {
        MeasurementEnvelope env;
        env.type = MeasurementType::Imu;
        env.source_id = "imu";
        env.timestamp_s = i * 0.01;
        env.sequence_id = static_cast<uint64_t>(i);
        env.frame = MeasurementFrame::Body;
        env.payload = ImuMeasurementPayload{Eigen::Vector3d{0.04, 0.0, 9.81},
                                            Eigen::Vector3d{0.0, 0.0, 0.01}};
        (void)active_only.process_measurement(env);
        (void)with_shadow.process_measurement(env);
    }
    with_shadow.flush_shadow();
    with_shadow.stop();

    const auto a = active_only.active_snapshot();
    const auto b = with_shadow.active_snapshot();
    return (a.position_m - b.position_m).norm() < 1.0e-12 &&
           (a.velocity_mps - b.velocity_mps).norm() < 1.0e-12 &&
           std::abs(std::abs(a.orientation.dot(b.orientation)) - 1.0) < 1.0e-12;
}

ReplayResult run_once(const ReplayScenario& scenario) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_cfg());
    ekf.configure_msckf(make_msckf_cfg(scenario.max_camera_states));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const std::vector<Eigen::Vector3d> features{
        {0.2, 0.1, 4.5},
        {-0.1, 0.15, 5.2},
    };

    double timestamp_s = 0.0;
    uint64_t observed_max_window_size = 0;
    for (int i = 0; i < scenario.camera_updates; ++i) {
        propagate(ekf, timestamp_s, 4);
        ekf.update_vision(project_features(ekf, features), features, camera_intrinsics());
        observed_max_window_size =
            std::max<uint64_t>(observed_max_window_size, ekf.diagnostics().msckf_window_size);
        if (scenario.reset_mid_run && i == (scenario.camera_updates / 2)) {
            ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                      Eigen::Vector3d::Zero());
            timestamp_s = 0.0;
        }
    }

    const auto cov = ekf.covariance();
    const auto diag = ekf.diagnostics();
    const auto state = ekf.state();
    const auto ids = ekf.msckf_state_ids_for_test();
    const auto timestamps = ekf.msckf_state_timestamps_for_test();

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(12) << state.position.transpose() << '|'
        << state.velocity.transpose() << '|' << state.orientation.coeffs().transpose() << '|'
        << cov.trace() << '|' << diag.msckf_window_size << '|' << diag.msckf_states_created << '|'
        << diag.msckf_states_removed << '|' << diag.msckf_deterministic_evictions;
    for (const auto id : ids) {
        oss << '|' << id;
    }
    for (const auto ts : timestamps) {
        oss << '|' << ts;
    }

    ReplayResult out;
    out.name = scenario.name;
    out.active_equivalent = active_equivalence_control();
    out.covariance_finite = cov.array().isFinite().all() && std::isfinite(cov.trace()) &&
                            (cov - cov.transpose()).cwiseAbs().maxCoeff() < 1.0e-8;
    out.shadow_only_msckf = out.active_equivalent;
    out.maximum_window_size = observed_max_window_size;
    out.states_created = diag.msckf_states_created;
    out.states_removed = diag.msckf_states_removed;
    out.final_window_size = diag.msckf_window_size;
    out.checksum = oss.str();
    return out;
}

void write_report(const std::vector<ReplayResult>& results) {
    const std::filesystem::path out_dir = std::filesystem::path("artifacts") / "phase20";
    std::filesystem::create_directories(out_dir);
    const std::filesystem::path out_path = out_dir / "ekf_phase20_replay_report.json";

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
        oss << "      \"deterministic\": " << (r.deterministic ? "true" : "false") << ",\n";
        oss << "      \"covariance_finite\": " << (r.covariance_finite ? "true" : "false") << ",\n";
        oss << "      \"shadow_only_msckf\": " << (r.shadow_only_msckf ? "true" : "false") << ",\n";
        oss << "      \"maximum_window_size\": " << r.maximum_window_size << ",\n";
        oss << "      \"states_created\": " << r.states_created << ",\n";
        oss << "      \"states_removed\": " << r.states_removed << ",\n";
        oss << "      \"final_window_size\": " << r.final_window_size << ",\n";
        oss << "      \"checksum\": \"" << r.checksum << "\"\n";
        oss << "    }" << (i + 1 < results.size() ? "," : "") << "\n";
    }
    oss << "  ]\n";
    oss << "}\n";

    std::ofstream out(out_path, std::ios::binary);
    out << oss.str();
}

} // namespace

int main() {
    const std::vector<ReplayScenario> scenarios{
        {"continuous_camera_stream", 12, 4, false},
        {"long_window", 18, 8, false},
        {"repeated_reset", 12, 5, true},
        {"eviction_stress", 20, 3, false},
    };

    std::vector<ReplayResult> results;
    results.reserve(scenarios.size());
    bool ok = true;
    for (const auto& scenario : scenarios) {
        auto first = run_once(scenario);
        auto second = run_once(scenario);
        first.deterministic = first.checksum == second.checksum &&
                              first.maximum_window_size == second.maximum_window_size &&
                              first.states_created == second.states_created &&
                              first.states_removed == second.states_removed &&
                              first.final_window_size == second.final_window_size;
        ok = ok && first.active_equivalent && first.deterministic && first.covariance_finite &&
             first.shadow_only_msckf;
        results.push_back(std::move(first));
    }

    write_report(results);
    return ok ? 0 : 1;
}
