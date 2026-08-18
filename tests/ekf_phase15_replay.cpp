#include "vio/EKFEstimator.hpp"

#include <Eigen/Core>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

using namespace drone::vio;

namespace {

enum class ReplayOp : uint8_t { Imu = 0, Zupt, Depth };

struct ReplayRecord {
    ReplayOp op{ReplayOp::Imu};
    double timestamp_s{0.0};
    Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
    double scalar{0.0};
    double sigma{0.05};
};

struct ReplayScenario {
    std::string name;
    std::vector<ReplayRecord> records;
};

struct ReplayScenarioResult {
    std::string scenario_name;
    bool success{false};
    size_t processed_record_count{0};
    size_t accepted_count{0};
    size_t rejected_count{0};
    PoseEstimate final_state{};
    CovMat final_covariance{CovMat::Identity()};
    EKFDiagnostics diagnostics{};
    bool finite_state{false};
    bool covariance_symmetric{false};
    std::string checksum;
};

std::string format_double(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9) << value;
    return oss.str();
}

std::string checksum_for(const PoseEstimate& pose, const CovMat& cov, const EKFDiagnostics& diag) {
    std::ostringstream oss;
    oss << format_double(pose.position.x()) << '|' << format_double(pose.position.y()) << '|'
        << format_double(pose.position.z()) << '|' << format_double(pose.velocity.x()) << '|'
        << format_double(pose.velocity.y()) << '|' << format_double(pose.velocity.z()) << '|'
        << format_double(pose.orientation.w()) << '|' << format_double(pose.orientation.x()) << '|'
        << format_double(pose.orientation.y()) << '|' << format_double(pose.orientation.z()) << '|'
        << format_double(cov.trace()) << '|' << diag.accepted_propagation_count << '|'
        << diag.rejected_propagation_count << '|' << diag.accepted_update_count << '|'
        << diag.rejected_update_count;
    return oss.str();
}

std::vector<ReplayScenario> make_scenarios() {
    const double dt = 0.0025;
    std::vector<ReplayScenario> scenarios;

    ReplayScenario stationary{"stationary_imu", {}};
    for (int i = 0; i < 400; ++i) {
        stationary.records.push_back(
            {ReplayOp::Imu, i * dt, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()});
    }
    scenarios.push_back(stationary);

    ReplayScenario constant_yaw{"constant_yaw_rate", {}};
    for (int i = 0; i < 400; ++i) {
        constant_yaw.records.push_back({ReplayOp::Imu, i * dt, Eigen::Vector3d{0.0, 0.0, 9.81},
                                        Eigen::Vector3d{0.0, 0.0, 0.25}});
    }
    scenarios.push_back(constant_yaw);

    ReplayScenario repeated_zupt{"repeated_manual_zupt", {}};
    for (int i = 0; i < 200; ++i) {
        repeated_zupt.records.push_back(
            {ReplayOp::Imu, i * dt, Eigen::Vector3d{0.1, 0.0, 9.81}, Eigen::Vector3d::Zero()});
    }
    for (int i = 0; i < 20; ++i) {
        repeated_zupt.records.push_back({ReplayOp::Zupt, 1.0 + (i * dt)});
    }
    scenarios.push_back(repeated_zupt);

    ReplayScenario invalid_timestamp{"invalid_timestamp_sequence", {}};
    invalid_timestamp.records = {
        {ReplayOp::Imu, 0.0, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()},
        {ReplayOp::Imu, 0.01, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()},
        {ReplayOp::Imu, 0.005, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()},
    };
    scenarios.push_back(invalid_timestamp);

    ReplayScenario non_finite{"non_finite_measurement", {}};
    non_finite.records = {
        {ReplayOp::Imu, 0.0, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()},
        {ReplayOp::Imu, 0.01, Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 9.81},
         Eigen::Vector3d::Zero()},
    };
    scenarios.push_back(non_finite);

    ReplayScenario oversized_dt{"oversized_time_step", {}};
    oversized_dt.records = {
        {ReplayOp::Imu, 0.0, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()},
        {ReplayOp::Imu, 0.5, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()},
    };
    scenarios.push_back(oversized_dt);

    ReplayScenario disabled_lidar{"disabled_unsafe_lidar_correction", {}};
    disabled_lidar.records = {
        {ReplayOp::Imu, 0.0, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()},
        {ReplayOp::Depth, 0.01, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 3.0, 0.05},
    };
    scenarios.push_back(disabled_lidar);

    ReplayScenario long_duration{"long_duration_stationary", {}};
    for (int i = 0; i < 4000; ++i) {
        long_duration.records.push_back({ReplayOp::Imu, i * dt, Eigen::Vector3d{0.0, 0.0, 9.81},
                                         Eigen::Vector3d{0.0, 0.0, 0.001}});
    }
    for (int i = 0; i < 10; ++i) {
        long_duration.records.push_back({ReplayOp::Zupt, 10.1 + (i * 0.05)});
    }
    scenarios.push_back(long_duration);

    return scenarios;
}

ReplayScenarioResult run_scenario(const ReplayScenario& scenario) {
    EKFEstimator ekf;
    EstimatorValidationConfig validation;
    validation.lidar_depth_correction_enabled = false;
    ekf.configure_validation(validation);
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    ReplayScenarioResult result;
    result.scenario_name = scenario.name;

    for (const auto& record : scenario.records) {
        ++result.processed_record_count;
        EstimatorOperationResult op_result = EstimatorOperationResult::Accepted;
        switch (record.op) {
        case ReplayOp::Imu:
            op_result = ekf.process_imu_measurement(record.accel, record.gyro, record.timestamp_s);
            break;
        case ReplayOp::Zupt:
            ekf.update_zupt();
            op_result = ekf.diagnostics().last_operation_result;
            break;
        case ReplayOp::Depth:
            ekf.update_depth(record.scalar, record.sigma);
            op_result = ekf.diagnostics().last_operation_result;
            break;
        }
        if (op_result == EstimatorOperationResult::Accepted) {
            ++result.accepted_count;
        } else {
            ++result.rejected_count;
        }
    }

    result.final_state = ekf.state();
    result.final_covariance = ekf.covariance();
    result.diagnostics = ekf.diagnostics();
    result.finite_state = result.final_state.position.array().isFinite().all() &&
                          result.final_state.velocity.array().isFinite().all() &&
                          result.final_state.orientation.coeffs().array().isFinite().all();
    result.covariance_symmetric =
        (result.final_covariance - result.final_covariance.transpose()).cwiseAbs().maxCoeff() <
        1.0e-8;
    result.checksum = checksum_for(result.final_state, result.final_covariance, result.diagnostics);
    result.success = result.finite_state && result.covariance_symmetric;
    return result;
}

std::string to_json(const std::vector<ReplayScenarioResult>& results) {
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"schema_version\": 1,\n";
    oss << "  \"scenario_count\": " << results.size() << ",\n";
    oss << "  \"scenarios\": [\n";
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& result = results[i];
        oss << "    {\n";
        oss << "      \"scenario_name\": \"" << result.scenario_name << "\",\n";
        oss << "      \"success\": " << (result.success ? "true" : "false") << ",\n";
        oss << "      \"processed_record_count\": " << result.processed_record_count << ",\n";
        oss << "      \"accepted_count\": " << result.accepted_count << ",\n";
        oss << "      \"rejected_count\": " << result.rejected_count << ",\n";
        oss << "      \"final_position\": [" << format_double(result.final_state.position.x())
            << ", " << format_double(result.final_state.position.y()) << ", "
            << format_double(result.final_state.position.z()) << "],\n";
        oss << "      \"final_velocity\": [" << format_double(result.final_state.velocity.x())
            << ", " << format_double(result.final_state.velocity.y()) << ", "
            << format_double(result.final_state.velocity.z()) << "],\n";
        oss << "      \"final_orientation\": [" << format_double(result.final_state.orientation.w())
            << ", " << format_double(result.final_state.orientation.x()) << ", "
            << format_double(result.final_state.orientation.y()) << ", "
            << format_double(result.final_state.orientation.z()) << "],\n";
        oss << "      \"covariance_trace\": " << format_double(result.final_covariance.trace())
            << ",\n";
        oss << "      \"finite_state\": " << (result.finite_state ? "true" : "false") << ",\n";
        oss << "      \"covariance_symmetric\": "
            << (result.covariance_symmetric ? "true" : "false") << ",\n";
        oss << "      \"diagnostics\": {\n";
        oss << "        \"accepted_propagation_count\": "
            << result.diagnostics.accepted_propagation_count << ",\n";
        oss << "        \"rejected_propagation_count\": "
            << result.diagnostics.rejected_propagation_count << ",\n";
        oss << "        \"accepted_update_count\": " << result.diagnostics.accepted_update_count
            << ",\n";
        oss << "        \"rejected_update_count\": " << result.diagnostics.rejected_update_count
            << ",\n";
        oss << "        \"disabled_lidar_correction_count\": "
            << result.diagnostics.disabled_lidar_correction_count << ",\n";
        oss << "        \"last_operation_result\": \""
            << to_string(result.diagnostics.last_operation_result) << "\"\n";
        oss << "      },\n";
        oss << "      \"checksum\": \"" << result.checksum << "\"\n";
        oss << "    }" << (i + 1 < results.size() ? "," : "") << "\n";
    }
    oss << "  ]\n";
    oss << "}\n";
    return oss.str();
}

} // namespace

int main(int argc, char** argv) {
    const std::filesystem::path output_path =
        (argc > 1) ? std::filesystem::path(argv[1])
                   : std::filesystem::path("artifacts/phase15/ekf_replay_report.json");
    std::filesystem::create_directories(output_path.parent_path());

    const auto scenarios = make_scenarios();
    std::vector<ReplayScenarioResult> results;
    results.reserve(scenarios.size());

    bool ok = true;
    for (const auto& scenario : scenarios) {
        const auto first = run_scenario(scenario);
        const auto second = run_scenario(scenario);
        if (first.checksum != second.checksum) {
            ok = false;
        }
        results.push_back(first);
        ok = ok && first.success;
    }

    std::ofstream out(output_path);
    out << to_json(results);
    out.close();

    if (!ok) {
        std::cerr << "Phase 15 replay validation failed\n";
        return 1;
    }
    std::cout << output_path.string() << '\n';
    return 0;
}
