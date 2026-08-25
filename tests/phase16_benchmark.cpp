#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace drone::vio;

namespace {

using Clock = std::chrono::steady_clock;

struct BenchmarkStats {
    double min_us{0.0};
    double median_us{0.0};
    double max_us{0.0};
    double p95_us{0.0};
};

struct BenchmarkCaseResult {
    std::string name;
    BenchmarkStats latency{};
    uint64_t total_records{0};
    uint64_t queue_drops{0};
    uint64_t shadow_processed{0};
    uint64_t active_processed{0};
};

EstimatorValidationConfig make_cfg(bool shadow_enabled, uint32_t queue_depth,
                                   double max_lag_ms = 25.0) {
    EstimatorValidationConfig cfg;
    cfg.enable_shadow_estimator = shadow_enabled;
    cfg.shadow_comparison_enabled = shadow_enabled;
    cfg.shadow_max_queue_depth = queue_depth;
    cfg.shadow_max_lag_ms = max_lag_ms;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    return cfg;
}

MeasurementEnvelope imu_env(double timestamp_s, uint64_t sequence_id) {
    MeasurementEnvelope envelope;
    envelope.type = MeasurementType::Imu;
    envelope.source_id = "imu";
    envelope.timestamp_s = timestamp_s;
    envelope.sequence_id = sequence_id;
    envelope.frame = MeasurementFrame::Body;
    envelope.payload =
        ImuMeasurementPayload{Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()};
    return envelope;
}

class DelayedShadowEstimator final : public StateEstimator {
public:
    explicit DelayedShadowEstimator(std::chrono::milliseconds delay)
        : inner_(EKFConfig{}, "benchmark_shadow", "phase16-bench"), delay_(delay) {}

    void configure_validation(const EstimatorValidationConfig& cfg) override {
        inner_.configure_validation(cfg);
    }
    void reset(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
               const Eigen::Vector3d& v0) override {
        inner_.reset(p0, q0, v0);
    }
    EstimatorOperationResult process_measurement(const MeasurementEnvelope& envelope) override {
        if (delay_.count() > 0) {
            std::this_thread::sleep_for(delay_);
        }
        return inner_.process_measurement(envelope);
    }
    EstimatorStateSnapshot snapshot() const override {
        return inner_.snapshot();
    }
    EKFDiagnostics diagnostics() const override {
        return inner_.diagnostics();
    }
    bool is_initialized() const override {
        return inner_.is_initialized();
    }
    std::string estimator_name() const override {
        return inner_.estimator_name();
    }
    std::string estimator_version() const override {
        return inner_.estimator_version();
    }

private:
    EKFStateEstimatorAdapter inner_;
    std::chrono::milliseconds delay_;
};

BenchmarkStats compute_stats(std::vector<double> samples_us) {
    BenchmarkStats stats;
    if (samples_us.empty()) {
        return stats;
    }
    std::sort(samples_us.begin(), samples_us.end());
    stats.min_us = samples_us.front();
    stats.max_us = samples_us.back();
    stats.median_us = samples_us[samples_us.size() / 2];
    stats.p95_us = samples_us[std::min(samples_us.size() - 1,
                                       static_cast<size_t>(samples_us.size() * 95 / 100))];
    return stats;
}

BenchmarkCaseResult run_processing_case(const std::string& name, bool shadow_enabled,
                                        std::chrono::milliseconds shadow_delay,
                                        uint32_t queue_depth, uint64_t warmup_count,
                                        uint64_t measured_count) {
    std::unique_ptr<StateEstimator> shadow_estimator;
    if (shadow_delay.count() > 0) {
        shadow_estimator = std::make_unique<DelayedShadowEstimator>(shadow_delay);
    } else {
        shadow_estimator = std::make_unique<EKFStateEstimatorAdapter>(
            EKFConfig{}, "benchmark_shadow", "phase16-bench");
    }

    EstimatorCoordinator coordinator(std::make_unique<EKFStateEstimatorAdapter>(
                                         EKFConfig{}, "benchmark_active", "phase16-bench"),
                                     std::move(shadow_estimator));
    coordinator.configure_validation(make_cfg(shadow_enabled, queue_depth));
    coordinator.initialize();
    if (shadow_enabled) {
        (void)coordinator.start();
    }

    std::vector<double> samples_us;
    samples_us.reserve(static_cast<size_t>(measured_count));
    uint64_t sequence_id = 0;
    for (uint64_t i = 0; i < warmup_count + measured_count; ++i) {
        const auto start = Clock::now();
        (void)coordinator.process_measurement(
            imu_env(static_cast<double>(i) * 0.0025, sequence_id++));
        const auto elapsed_us =
            std::chrono::duration<double, std::micro>(Clock::now() - start).count();
        if (i >= warmup_count) {
            samples_us.push_back(elapsed_us);
        }
    }
    coordinator.flush_shadow();

    const auto diagnostics = coordinator.diagnostics();
    BenchmarkCaseResult result;
    result.name = name;
    result.latency = compute_stats(std::move(samples_us));
    result.total_records = measured_count;
    result.queue_drops = diagnostics.queue.dropped_count;
    result.shadow_processed = diagnostics.shadow_processed_count;
    result.active_processed = diagnostics.active_processed_count;
    return result;
}

BenchmarkCaseResult run_snapshot_case(uint64_t warmup_count, uint64_t measured_count) {
    EstimatorCoordinator coordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "snapshot_active", "phase16-bench"),
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "snapshot_shadow",
                                                   "phase16-bench"));
    coordinator.configure_validation(make_cfg(false, 8));
    coordinator.initialize();
    for (uint64_t i = 0; i < 32; ++i) {
        (void)coordinator.process_measurement(imu_env(static_cast<double>(i) * 0.0025, i));
    }

    std::vector<double> samples_us;
    samples_us.reserve(static_cast<size_t>(measured_count));
    for (uint64_t i = 0; i < warmup_count + measured_count; ++i) {
        const auto start = Clock::now();
        (void)coordinator.active_snapshot();
        const auto elapsed_us =
            std::chrono::duration<double, std::micro>(Clock::now() - start).count();
        if (i >= warmup_count) {
            samples_us.push_back(elapsed_us);
        }
    }

    BenchmarkCaseResult result;
    result.name = "snapshot_publication";
    result.latency = compute_stats(std::move(samples_us));
    result.total_records = measured_count;
    return result;
}

std::string format_double(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << value;
    return oss.str();
}

std::string compiler_id() {
#if defined(_MSC_FULL_VER)
    return "MSVC " + std::to_string(_MSC_FULL_VER);
#elif defined(__clang__)
    return "Clang";
#elif defined(__GNUC__)
    return "GCC";
#else
    return "unknown";
#endif
}

std::string build_type() {
#if defined(NDEBUG)
    return "Release";
#else
    return "Debug";
#endif
}

std::string to_json(const std::vector<BenchmarkCaseResult>& cases, uint64_t warmup_count,
                    uint64_t measured_count) {
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"schema_version\": 1,\n";
    oss << "  \"compiler\": \"" << compiler_id() << "\",\n";
    oss << "  \"build_type\": \"" << build_type() << "\",\n";
    oss << "  \"warmup_count\": " << warmup_count << ",\n";
    oss << "  \"measured_iterations\": " << measured_count << ",\n";
    oss << "  \"cases\": [\n";
    for (size_t i = 0; i < cases.size(); ++i) {
        const auto& c = cases[i];
        oss << "    {\n";
        oss << "      \"name\": \"" << c.name << "\",\n";
        oss << "      \"total_records\": " << c.total_records << ",\n";
        oss << "      \"active_processed\": " << c.active_processed << ",\n";
        oss << "      \"shadow_processed\": " << c.shadow_processed << ",\n";
        oss << "      \"queue_drops\": " << c.queue_drops << ",\n";
        oss << "      \"latency_us\": {\n";
        oss << "        \"min\": " << format_double(c.latency.min_us) << ",\n";
        oss << "        \"median\": " << format_double(c.latency.median_us) << ",\n";
        oss << "        \"max\": " << format_double(c.latency.max_us) << ",\n";
        oss << "        \"p95\": " << format_double(c.latency.p95_us) << "\n";
        oss << "      }\n";
        oss << "    }" << (i + 1 < cases.size() ? "," : "") << "\n";
    }
    oss << "  ]\n";
    oss << "}\n";
    return oss.str();
}

} // namespace

int main(int argc, char** argv) {
    const std::filesystem::path output_path =
        (argc > 1) ? std::filesystem::path(argv[1])
                   : std::filesystem::path("artifacts/phase16/phase16_benchmark_report.json");
    std::filesystem::create_directories(output_path.parent_path());

    constexpr uint64_t kWarmup = 64;
    constexpr uint64_t kMeasured = 512;

    std::vector<BenchmarkCaseResult> cases;
    cases.push_back(run_processing_case("active_only", false, std::chrono::milliseconds{0}, 8,
                                        kWarmup, kMeasured));
    cases.push_back(run_processing_case("active_plus_shadow", true, std::chrono::milliseconds{0},
                                        64, kWarmup, kMeasured));
    cases.push_back(run_processing_case("active_plus_slow_shadow", true,
                                        std::chrono::milliseconds{2}, 64, kWarmup, kMeasured));
    cases.push_back(run_processing_case("overload_path", true, std::chrono::milliseconds{10}, 8,
                                        kWarmup, kMeasured));
    cases.push_back(run_snapshot_case(kWarmup, kMeasured));

    std::ofstream out(output_path);
    out << to_json(cases, kWarmup, kMeasured);
    out.close();

    std::cout << output_path.string() << '\n';
    return 0;
}
