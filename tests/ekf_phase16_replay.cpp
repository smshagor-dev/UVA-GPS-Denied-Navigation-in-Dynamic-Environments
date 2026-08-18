#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace drone::vio;

namespace {

enum class ReplayOp : uint8_t { Imu = 0, Zupt, Depth };

struct ReplayRecord {
    ReplayOp op{ReplayOp::Imu};
    double timestamp_s{0.0};
    double depth_or_sigma{0.0};
    double sigma{0.05};
    Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
};

struct ReplayScenario {
    std::string name;
    std::vector<ReplayRecord> records;
};

struct ReplaySummary {
    std::string name;
    bool active_equivalent{false};
    bool shadow_success{false};
    bool deterministic_replay{false};
    bool overall_success{false};
    uint64_t active_processed_count{0};
    uint64_t shadow_processed_count{0};
    uint64_t queue_drop_count{0};
    uint64_t stale_count{0};
    uint64_t shadow_failure_count{0};
    size_t queue_capacity{0};
    size_t queue_high_water_mark{0};
    EstimatorStateSnapshot active_final{};
    std::optional<EstimatorStateSnapshot> shadow_final{};
    EstimatorComparison comparison{};
    std::string active_checksum{};
    std::string shadow_checksum{};
    std::string lifecycle_result{"completed"};
};

std::string format_double(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9) << value;
    return oss.str();
}

std::string checksum_for(const EstimatorStateSnapshot& snapshot) {
    std::ostringstream oss;
    oss << format_double(snapshot.position_m.x()) << '|' << format_double(snapshot.position_m.y())
        << '|' << format_double(snapshot.position_m.z()) << '|'
        << format_double(snapshot.velocity_mps.x()) << '|'
        << format_double(snapshot.velocity_mps.y()) << '|'
        << format_double(snapshot.velocity_mps.z()) << '|'
        << format_double(snapshot.orientation.w()) << '|' << format_double(snapshot.orientation.x())
        << '|' << format_double(snapshot.orientation.y()) << '|'
        << format_double(snapshot.orientation.z()) << '|'
        << format_double(snapshot.covariance.trace);
    return oss.str();
}

std::vector<ReplayScenario> make_scenarios() {
    std::vector<ReplayScenario> scenarios;
    const double dt = 0.0025;

    ReplayScenario stationary{"stationary_imu", {}};
    for (int i = 0; i < 400; ++i) {
        stationary.records.push_back({ReplayOp::Imu, i * dt, 0.0, 0.05,
                                      Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()});
    }
    scenarios.push_back(stationary);

    ReplayScenario constant_yaw{"constant_yaw", {}};
    for (int i = 0; i < 400; ++i) {
        constant_yaw.records.push_back({ReplayOp::Imu, i * dt, 0.0, 0.05,
                                        Eigen::Vector3d{0.0, 0.0, 9.81},
                                        Eigen::Vector3d{0.0, 0.0, 0.2}});
    }
    scenarios.push_back(constant_yaw);

    ReplayScenario repeated_zupt{"repeated_manual_zupt", {}};
    for (int i = 0; i < 200; ++i) {
        repeated_zupt.records.push_back({ReplayOp::Imu, i * dt, 0.0, 0.05,
                                         Eigen::Vector3d{0.1, 0.0, 9.81}, Eigen::Vector3d::Zero()});
    }
    for (int i = 0; i < 20; ++i) {
        repeated_zupt.records.push_back({ReplayOp::Zupt, 1.0 + i * dt});
    }
    scenarios.push_back(repeated_zupt);

    ReplayScenario invalid_timestamp{"invalid_timestamp", {}};
    invalid_timestamp.records.push_back(
        {ReplayOp::Imu, 0.0, 0.0, 0.05, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()});
    invalid_timestamp.records.push_back(
        {ReplayOp::Imu, 0.01, 0.0, 0.05, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()});
    invalid_timestamp.records.push_back({ReplayOp::Imu, 0.005, 0.0, 0.05,
                                         Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()});
    scenarios.push_back(invalid_timestamp);

    ReplayScenario disabled_lidar{"disabled_lidar", {}};
    disabled_lidar.records.push_back(
        {ReplayOp::Imu, 0.0, 0.0, 0.05, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()});
    disabled_lidar.records.push_back({ReplayOp::Depth, 0.01, 3.0, 0.05});
    scenarios.push_back(disabled_lidar);

    ReplayScenario long_duration{"long_duration_shadow", {}};
    for (int i = 0; i < 4000; ++i) {
        long_duration.records.push_back({ReplayOp::Imu, i * dt, 0.0, 0.05,
                                         Eigen::Vector3d{0.0, 0.0, 9.81},
                                         Eigen::Vector3d{0.0, 0.0, 0.001}});
    }
    for (int i = 0; i < 10; ++i) {
        long_duration.records.push_back({ReplayOp::Zupt, 10.1 + (i * 0.05)});
    }
    long_duration.records.push_back(
        {ReplayOp::Imu, 10.7, 0.0, 0.05,
         Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 9.81},
         Eigen::Vector3d::Zero()});
    long_duration.records.push_back({ReplayOp::Depth, 10.8, 2.5, 0.05});
    scenarios.push_back(long_duration);

    return scenarios;
}

ReplaySummary run_scenario(const ReplayScenario& scenario) {
    EstimatorCoordinator active_only(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "active_a"),
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "shadow_a"));
    EstimatorCoordinator with_shadow(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "active_b"),
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "shadow_b"));

    EstimatorValidationConfig base_cfg;
    const size_t requested_queue_depth =
        std::max<size_t>(8, scenario.records.size() + static_cast<size_t>(16));
    base_cfg.shadow_max_queue_depth = static_cast<uint32_t>(std::min<size_t>(
        requested_queue_depth, static_cast<size_t>(std::numeric_limits<uint32_t>::max())));
    active_only.configure_validation(base_cfg);
    active_only.initialize();

    auto shadow_cfg = base_cfg;
    shadow_cfg.enable_shadow_estimator = true;
    shadow_cfg.shadow_comparison_enabled = true;
    with_shadow.configure_validation(shadow_cfg);
    with_shadow.initialize();
    (void)with_shadow.start();

    uint64_t sequence_id = 0;
    for (const auto& record : scenario.records) {
        switch (record.op) {
        case ReplayOp::Imu: {
            MeasurementEnvelope env;
            env.type = MeasurementType::Imu;
            env.source_id = "imu";
            env.timestamp_s = record.timestamp_s;
            env.sequence_id = sequence_id++;
            env.frame = MeasurementFrame::Body;
            env.payload = ImuMeasurementPayload{record.accel, record.gyro};
            (void)active_only.process_measurement(env);
            (void)with_shadow.process_measurement(env);
            break;
        }
        case ReplayOp::Zupt: {
            auto env =
                make_manual_zupt_envelope(MeasurementStamp{record.timestamp_s, sequence_id++});
            (void)active_only.process_measurement(env);
            (void)with_shadow.process_measurement(env);
            break;
        }
        case ReplayOp::Depth: {
            auto env =
                make_lidar_depth_envelope(MeasurementStamp{record.timestamp_s, sequence_id++},
                                          record.depth_or_sigma, record.sigma, false);
            (void)active_only.process_measurement(env);
            (void)with_shadow.process_measurement(env);
            break;
        }
        }
    }
    with_shadow.flush_shadow();

    ReplaySummary summary;
    summary.name = scenario.name;
    summary.active_final = with_shadow.active_snapshot();
    summary.shadow_final = with_shadow.shadow_snapshot();
    summary.comparison = with_shadow.diagnostics().last_comparison;

    const auto active_a = active_only.active_snapshot();
    const auto active_b = with_shadow.active_snapshot();
    summary.active_equivalent = checksum_for(active_a) == checksum_for(active_b);
    const auto diagnostics = with_shadow.diagnostics();
    summary.shadow_success = diagnostics.shadow_failure_count == 0;
    summary.shadow_failure_count = diagnostics.shadow_failure_count;
    summary.active_processed_count = diagnostics.active_processed_count;
    summary.shadow_processed_count = diagnostics.shadow_processed_count;
    summary.queue_drop_count = diagnostics.queue.dropped_count;
    summary.stale_count = diagnostics.queue.stale_count;
    summary.queue_capacity = diagnostics.queue.capacity;
    summary.queue_high_water_mark = diagnostics.queue.peak_depth;
    summary.active_checksum = checksum_for(summary.active_final);
    summary.shadow_checksum =
        summary.shadow_final.has_value() ? checksum_for(*summary.shadow_final) : "none";
    summary.overall_success = summary.active_equivalent && summary.shadow_success;
    return summary;
}

std::string snapshot_to_json(const EstimatorStateSnapshot& snapshot) {
    std::ostringstream oss;
    oss << "{"
        << "\"timestamp_s\":" << format_double(snapshot.timestamp_s) << ',' << "\"position_m\":["
        << format_double(snapshot.position_m.x()) << ',' << format_double(snapshot.position_m.y())
        << ',' << format_double(snapshot.position_m.z()) << "],"
        << "\"velocity_mps\":[" << format_double(snapshot.velocity_mps.x()) << ','
        << format_double(snapshot.velocity_mps.y()) << ','
        << format_double(snapshot.velocity_mps.z()) << "],"
        << "\"orientation_xyzw\":[" << format_double(snapshot.orientation.x()) << ','
        << format_double(snapshot.orientation.y()) << ',' << format_double(snapshot.orientation.z())
        << ',' << format_double(snapshot.orientation.w()) << "],"
        << "\"covariance_trace\":" << format_double(snapshot.covariance.trace) << "}";
    return oss.str();
}

std::string to_json(const std::vector<ReplaySummary>& summaries) {
    std::ostringstream oss;
    oss << "{\n  \"schema_version\": 3,\n";
    oss << "  \"build_type\": \""
#if defined(NDEBUG)
        << "Release";
#else
        << "Debug";
#endif
    oss << "\",\n";
    oss << "  \"scenarios\": [\n";
    for (size_t i = 0; i < summaries.size(); ++i) {
        const auto& s = summaries[i];
        oss << "    {\n";
        oss << "      \"name\": \"" << s.name << "\",\n";
        oss << "      \"active_equivalent\": " << (s.active_equivalent ? "true" : "false") << ",\n";
        oss << "      \"shadow_success\": " << (s.shadow_success ? "true" : "false") << ",\n";
        oss << "      \"deterministic_replay\": " << (s.deterministic_replay ? "true" : "false")
            << ",\n";
        oss << "      \"overall_success\": " << (s.overall_success ? "true" : "false") << ",\n";
        oss << "      \"lifecycle_result\": \"" << s.lifecycle_result << "\",\n";
        oss << "      \"active_processed_count\": " << s.active_processed_count << ",\n";
        oss << "      \"shadow_processed_count\": " << s.shadow_processed_count << ",\n";
        oss << "      \"queue_drop_count\": " << s.queue_drop_count << ",\n";
        oss << "      \"stale_count\": " << s.stale_count << ",\n";
        oss << "      \"shadow_failure_count\": " << s.shadow_failure_count << ",\n";
        oss << "      \"queue_capacity\": " << s.queue_capacity << ",\n";
        oss << "      \"queue_high_water_mark\": " << s.queue_high_water_mark << ",\n";
        oss << "      \"comparison_valid\": " << (s.comparison.valid ? "true" : "false") << ",\n";
        oss << "      \"position_delta_m\": " << format_double(s.comparison.position_delta_norm_m)
            << ",\n";
        oss << "      \"orientation_delta_deg\": "
            << format_double(s.comparison.orientation_delta_deg) << ",\n";
        oss << "      \"active_final_snapshot\": " << snapshot_to_json(s.active_final) << ",\n";
        oss << "      \"shadow_final_snapshot\": "
            << (s.shadow_final.has_value() ? snapshot_to_json(*s.shadow_final)
                                           : std::string("null"))
            << ",\n";
        oss << "      \"active_checksum\": \"" << s.active_checksum << "\",\n";
        oss << "      \"shadow_checksum\": \"" << s.shadow_checksum << "\"\n";
        oss << "    }" << (i + 1 < summaries.size() ? "," : "") << "\n";
    }
    oss << "  ]\n}\n";
    return oss.str();
}

} // namespace

int main(int argc, char** argv) {
    const std::filesystem::path output_path =
        (argc > 1) ? std::filesystem::path(argv[1])
                   : std::filesystem::path("artifacts/phase16/ekf_phase16_replay_report.json");
    std::filesystem::create_directories(output_path.parent_path());

    const auto scenarios = make_scenarios();
    std::vector<ReplaySummary> summaries;
    summaries.reserve(scenarios.size());
    bool ok = true;
    for (const auto& scenario : scenarios) {
        const auto first = run_scenario(scenario);
        const auto second = run_scenario(scenario);
        ReplaySummary summary = first;
        summary.deterministic_replay = first.active_checksum == second.active_checksum &&
                                       first.shadow_checksum == second.shadow_checksum;
        summary.overall_success =
            summary.active_equivalent && summary.shadow_success && summary.deterministic_replay;
        ok = ok && summary.overall_success;
        summaries.push_back(std::move(summary));
    }

    std::ofstream out(output_path);
    out << to_json(summaries);
    out.close();

    if (!ok) {
        std::cerr << "Phase 16 replay validation failed\n";
        return 1;
    }
    std::cout << output_path.string() << '\n';
    return 0;
}
