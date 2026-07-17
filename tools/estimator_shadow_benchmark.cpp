#include "estimation/EstimatorCoordinator.hpp"
#include "estimation/MinimalEskfAdapter.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

namespace drone::estimation {
namespace {

using json = nlohmann::json;

struct LatencyStats {
    double mean_us{0.0};
    double median_us{0.0};
    double p95_us{0.0};
    double p99_us{0.0};
    double min_us{0.0};
    double max_us{0.0};
    double stddev_us{0.0};
    size_t sample_count{0};
};

struct BenchmarkScenarioConfig {
    std::string mode;
    size_t warmup_iteration_count{250};
    size_t measured_iteration_count{2000};
    std::chrono::milliseconds shadow_delay{0};
    size_t queue_depth{512};
    double max_lag_ms{100.0};
    size_t throttle_queue_depth{0};
    bool wait_for_shadow_idle_each_iteration{false};
};

struct BenchmarkScenarioResult {
    BenchmarkScenarioConfig config{};
    bool success{false};
    std::string failure_reason{"none"};
    LatencyStats active_process_latency{};
    LatencyStats shadow_processing_latency{};
    double active_path_regression_percent{0.0};
    size_t shadow_queue_high_water_mark{0};
    uint64_t shadow_dropped_event_count{0};
    double shadow_lag_ms{0.0};
    std::string shadow_health{"disabled"};
    double shutdown_time_ms{0.0};
    bool active_output_matches_baseline{true};
    double warmup_duration_ms{0.0};
    double measured_duration_ms{0.0};
    double total_duration_ms{0.0};
    std::vector<double> active_process_samples_us{};
    std::vector<double> shadow_processing_samples_us{};
};

struct BenchmarkCliOptions {
    std::optional<std::filesystem::path> output_path{};
    std::string scenario{"all"};
    size_t warmup_iteration_count{250};
    size_t measured_iteration_count{2000};
    bool emit_samples{false};
};

class DelayedShadowEstimator final : public StateEstimator {
public:
    DelayedShadowEstimator(vio::EKFConfig config, std::chrono::milliseconds delay)
        : inner_(config), delay_(delay) {}

    void set_validation_config(const vio::EstimatorValidationConfig& config) {
        validation_ = config;
        inner_.set_validation_config(validation_);
    }

    void reset(const EstimatorInitialState& initial) override {
        inner_.reset(initial);
    }

    EstimatorUpdateResult process(const EstimatorMeasurement& measurement) override {
        if (delay_.count() > 0) {
            std::this_thread::sleep_for(delay_);
        }
        const auto started_at = std::chrono::steady_clock::now();
        auto result = inner_.process(measurement);
        last_processing_latency_us_.store(
            std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - started_at)
                .count(),
            std::memory_order_relaxed);
        return result;
    }

    [[nodiscard]] EstimatorSnapshot snapshot() const override {
        return inner_.snapshot();
    }

    [[nodiscard]] vio::EstimatorDiagnostics diagnostics() const override {
        return inner_.diagnostics();
    }

    [[nodiscard]] EstimatorCapabilities capabilities() const override {
        return inner_.capabilities();
    }

    [[nodiscard]] std::string_view name() const noexcept override {
        return "delayed_minimal_eskf_shadow";
    }

    [[nodiscard]] double last_processing_latency_us() const {
        return last_processing_latency_us_.load(std::memory_order_relaxed);
    }

private:
    MinimalEskfAdapter inner_;
    vio::EstimatorValidationConfig validation_{};
    std::chrono::milliseconds delay_{0};
    std::atomic<double> last_processing_latency_us_{0.0};
};

EstimatorMeasurement make_imu_measurement(uint64_t sequence_id, double timestamp_s) {
    EstimatorMeasurement measurement;
    measurement.header.timestamp_s = timestamp_s;
    measurement.header.sequence_id = sequence_id;
    measurement.header.source = SensorSource::IMU;
    measurement.header.frame = CoordinateFrame::BODY;
    measurement.header.type = MeasurementType::IMU;
    measurement.data = ImuMeasurementData{Eigen::Vector3d{0.02, 0.0, 9.81},
                                          Eigen::Vector3d{0.0, 0.0, 0.01}};
    return measurement;
}

EstimatorMeasurement make_visual_measurement(uint64_t sequence_id, double timestamp_s) {
    EstimatorMeasurement measurement;
    measurement.header.timestamp_s = timestamp_s;
    measurement.header.sequence_id = sequence_id;
    measurement.header.source = SensorSource::VISUAL_FRONTEND;
    measurement.header.frame = CoordinateFrame::WORLD;
    measurement.header.type = MeasurementType::VISUAL_POSE;

    VisualPoseMeasurementData visual;
    visual.position = Eigen::Vector3d{0.1, 0.0, 0.04};
    visual.velocity = Eigen::Vector3d{0.01, 0.0, 0.0};
    visual.quality = 0.9;
    visual.covariance.dimension = 6;
    visual.covariance.matrix.setZero();
    visual.covariance.matrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.04;
    visual.covariance.matrix.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.04;
    measurement.data = visual;
    return measurement;
}

EstimatorMeasurement make_depth_measurement(uint64_t sequence_id, double timestamp_s) {
    EstimatorMeasurement measurement;
    measurement.header.timestamp_s = timestamp_s;
    measurement.header.sequence_id = sequence_id;
    measurement.header.source = SensorSource::DEPTH_SENSOR;
    measurement.header.frame = CoordinateFrame::WORLD;
    measurement.header.type = MeasurementType::ALTITUDE;
    measurement.data = AltitudeMeasurementData{0.03, 0.05};
    return measurement;
}

double percentile_from_sorted(const std::vector<double>& values, double percentile) {
    if (values.empty()) {
        return 0.0;
    }
    const auto scaled = percentile * static_cast<double>(values.size() - 1);
    return values[static_cast<size_t>(std::round(scaled))];
}

LatencyStats compute_stats(std::vector<double> values) {
    LatencyStats stats;
    if (values.empty()) {
        return stats;
    }
    std::sort(values.begin(), values.end());
    stats.sample_count = values.size();
    stats.mean_us =
        std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
    stats.median_us = percentile_from_sorted(values, 0.50);
    stats.p95_us = percentile_from_sorted(values, 0.95);
    stats.p99_us = percentile_from_sorted(values, 0.99);
    stats.min_us = values.front();
    stats.max_us = values.back();

    double sum_sq = 0.0;
    for (const double value : values) {
        const double delta = value - stats.mean_us;
        sum_sq += delta * delta;
    }
    stats.stddev_us = std::sqrt(sum_sq / static_cast<double>(values.size()));
    return stats;
}

std::string shadow_health_to_string(ShadowHealthState state) {
    switch (state) {
    case ShadowHealthState::DISABLED:
        return "disabled";
    case ShadowHealthState::STARTING:
        return "starting";
    case ShadowHealthState::SYNCHRONIZED:
        return "synchronized";
    case ShadowHealthState::LAGGING:
        return "lagging";
    case ShadowHealthState::STALE:
        return "stale";
    case ShadowHealthState::DIVERGED:
        return "diverged";
    case ShadowHealthState::FAILED:
        return "failed";
    case ShadowHealthState::STOPPED:
        return "stopped";
    }
    return "unknown";
}

bool scenario_uses_shadow(std::string_view mode) {
    return mode == "active_with_shadow" || mode == "shadow_overload";
}

void process_iteration_batch(EstimatorCoordinator& coordinator, MinimalEskfAdapter& baseline,
                             std::shared_ptr<DelayedShadowEstimator> delayed_shadow,
                             uint64_t& next_sequence_id, size_t iteration_count,
                             std::vector<double>* active_latencies,
                             std::vector<double>* shadow_latencies,
                             const BenchmarkScenarioConfig& config) {
    for (size_t i = 0; i < iteration_count; ++i) {
        const uint64_t sequence_id = ++next_sequence_id;
        const double timestamp_s = 0.0025 * static_cast<double>(sequence_id);

        const auto started_at = std::chrono::steady_clock::now();
        static_cast<void>(coordinator.process(make_imu_measurement(sequence_id, timestamp_s)));
        const double active_latency_us =
            std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - started_at)
                .count();
        if (active_latencies) {
            active_latencies->push_back(active_latency_us);
        }
        static_cast<void>(baseline.process(make_imu_measurement(sequence_id, timestamp_s)));

        if ((sequence_id % 20u) == 0u) {
            static_cast<void>(coordinator.process(
                make_visual_measurement(sequence_id + 100000u, timestamp_s + 0.0005)));
            static_cast<void>(baseline.process(
                make_visual_measurement(sequence_id + 100000u, timestamp_s + 0.0005)));
        }
        if ((sequence_id % 50u) == 0u) {
            static_cast<void>(coordinator.process(
                make_depth_measurement(sequence_id + 200000u, timestamp_s + 0.0010)));
            static_cast<void>(baseline.process(
                make_depth_measurement(sequence_id + 200000u, timestamp_s + 0.0010)));
        }

        if (shadow_latencies && delayed_shadow) {
            shadow_latencies->push_back(delayed_shadow->last_processing_latency_us());
        }

        if (config.throttle_queue_depth > 0) {
            while (coordinator.telemetry().queue_depth > config.throttle_queue_depth) {
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
        }

        if (config.wait_for_shadow_idle_each_iteration && delayed_shadow &&
            !coordinator.wait_for_shadow_idle(std::chrono::milliseconds(5000))) {
            throw std::runtime_error("shadow benchmark pacing wait timed out");
        }
    }
}

BenchmarkScenarioResult run_scenario(const BenchmarkScenarioConfig& config, bool emit_samples) {
    BenchmarkScenarioResult result;
    result.config = config;
    const auto scenario_started_at = std::chrono::steady_clock::now();

    try {
        auto active = std::make_shared<MinimalEskfAdapter>();
        EstimatorCoordinator coordinator(std::static_pointer_cast<StateEstimator>(active));
        coordinator.reset({});

        std::shared_ptr<DelayedShadowEstimator> delayed_shadow;
        if (scenario_uses_shadow(config.mode)) {
            ShadowEstimatorConfig shadow_config;
            shadow_config.enabled = true;
            shadow_config.max_queue_depth = config.queue_depth;
            shadow_config.max_lag_ms = config.max_lag_ms;

            auto owned_shadow = std::make_unique<DelayedShadowEstimator>(vio::EKFConfig{},
                                                                         config.shadow_delay);
            owned_shadow->set_validation_config(vio::EstimatorValidationConfig{});
            delayed_shadow = std::shared_ptr<DelayedShadowEstimator>(owned_shadow.get(),
                                                                     [](DelayedShadowEstimator*) {});
            coordinator.configure_shadow(std::move(shadow_config), std::move(owned_shadow));
        }

        MinimalEskfAdapter baseline;
        baseline.reset({});

        uint64_t next_sequence_id = 0;

        const auto warmup_started_at = std::chrono::steady_clock::now();
        process_iteration_batch(coordinator, baseline, delayed_shadow, next_sequence_id,
                                config.warmup_iteration_count, nullptr, nullptr, config);
        result.warmup_duration_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                      warmup_started_at)
                .count();

        std::vector<double> active_latencies;
        std::vector<double> shadow_processing_latencies;
        active_latencies.reserve(config.measured_iteration_count);
        shadow_processing_latencies.reserve(config.measured_iteration_count);

        const auto measured_started_at = std::chrono::steady_clock::now();
        process_iteration_batch(coordinator, baseline, delayed_shadow, next_sequence_id,
                                config.measured_iteration_count, &active_latencies,
                                scenario_uses_shadow(config.mode) ? &shadow_processing_latencies
                                                                  : nullptr,
                                config);
        result.measured_duration_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                      measured_started_at)
                .count();

        if (scenario_uses_shadow(config.mode) &&
            !coordinator.wait_for_shadow_idle(std::chrono::milliseconds(10000))) {
            throw std::runtime_error("shadow benchmark did not drain");
        }

        result.active_process_latency = compute_stats(active_latencies);
        result.shadow_processing_latency = compute_stats(shadow_processing_latencies);
        result.active_output_matches_baseline = coordinator.active_snapshot().pose.position.isApprox(
            baseline.snapshot().pose.position, 1e-9);

        const auto telemetry = coordinator.telemetry();
        result.shadow_queue_high_water_mark = telemetry.queue_high_water_mark;
        result.shadow_dropped_event_count = telemetry.dropped_events;
        result.shadow_lag_ms = telemetry.lag_ms;
        result.shadow_health = shadow_health_to_string(telemetry.shadow_health);

        const auto shutdown_started_at = std::chrono::steady_clock::now();
        coordinator.stop_shadow();
        result.shutdown_time_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                      shutdown_started_at)
                .count();
        result.total_duration_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                      scenario_started_at)
                .count();

        if (emit_samples) {
            result.active_process_samples_us = std::move(active_latencies);
            result.shadow_processing_samples_us = std::move(shadow_processing_latencies);
        }
        result.success = true;
    } catch (const std::exception& ex) {
        result.failure_reason = ex.what();
        result.total_duration_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                      scenario_started_at)
                .count();
    }

    return result;
}

json stats_to_json(const LatencyStats& stats) {
    return json{{"mean", stats.mean_us},
                {"median", stats.median_us},
                {"p95", stats.p95_us},
                {"p99", stats.p99_us},
                {"min", stats.min_us},
                {"max", stats.max_us},
                {"stddev", stats.stddev_us},
                {"sample_count", stats.sample_count}};
}

json scenario_config_to_json(const BenchmarkScenarioConfig& config) {
    return json{{"mode", config.mode},
                {"warmup_iteration_count", config.warmup_iteration_count},
                {"measured_iteration_count", config.measured_iteration_count},
                {"shadow_delay_ms", config.shadow_delay.count()},
                {"queue_depth", config.queue_depth},
                {"max_lag_ms", config.max_lag_ms},
                {"throttle_queue_depth", config.throttle_queue_depth},
                {"wait_for_shadow_idle_each_iteration",
                 config.wait_for_shadow_idle_each_iteration}};
}

json scenario_to_json(const BenchmarkScenarioResult& scenario, bool emit_samples) {
    json output{{"config", scenario_config_to_json(scenario.config)},
                {"success", scenario.success},
                {"failure_reason", scenario.failure_reason},
                {"active_process_latency_us", stats_to_json(scenario.active_process_latency)},
                {"shadow_processing_latency_us", stats_to_json(scenario.shadow_processing_latency)},
                {"queue_push_latency_us", nullptr},
                {"active_path_regression_percent", scenario.active_path_regression_percent},
                {"shadow_queue_high_water_mark", scenario.shadow_queue_high_water_mark},
                {"shadow_dropped_event_count", scenario.shadow_dropped_event_count},
                {"shadow_lag_ms", scenario.shadow_lag_ms},
                {"shadow_health", scenario.shadow_health},
                {"shutdown_time_ms", scenario.shutdown_time_ms},
                {"active_output_matches_baseline", scenario.active_output_matches_baseline},
                {"warmup_duration_ms", scenario.warmup_duration_ms},
                {"measured_duration_ms", scenario.measured_duration_ms},
                {"total_duration_ms", scenario.total_duration_ms}};
    if (emit_samples) {
        output["active_process_samples_us"] = scenario.active_process_samples_us;
        output["shadow_processing_samples_us"] = scenario.shadow_processing_samples_us;
    }
    return output;
}

BenchmarkCliOptions parse_cli_options(int argc, char** argv) {
    BenchmarkCliOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--output" && i + 1 < argc) {
            options.output_path = std::filesystem::path(argv[++i]);
        } else if (arg == "--scenario" && i + 1 < argc) {
            options.scenario = argv[++i];
        } else if (arg == "--iterations" && i + 1 < argc) {
            options.measured_iteration_count = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--warmup-iterations" && i + 1 < argc) {
            options.warmup_iteration_count = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--emit-samples") {
            options.emit_samples = true;
        } else {
            throw std::runtime_error("unsupported benchmark argument");
        }
    }
    return options;
}

std::vector<BenchmarkScenarioConfig> scenario_configs_from_options(
    const BenchmarkCliOptions& options) {
    const auto make_active_only = [&options]() {
        return BenchmarkScenarioConfig{"active_only", options.warmup_iteration_count,
                                       options.measured_iteration_count, std::chrono::milliseconds(0),
                                       512, 100.0, 0, false};
    };
    const auto make_active_with_shadow = [&options]() {
        return BenchmarkScenarioConfig{"active_with_shadow", options.warmup_iteration_count,
                                       options.measured_iteration_count, std::chrono::milliseconds(0),
                                       512, 100.0, 0, true};
    };
    const auto make_shadow_overload = [&options]() {
        return BenchmarkScenarioConfig{"shadow_overload", options.warmup_iteration_count,
                                       options.measured_iteration_count, std::chrono::milliseconds(2), 8,
                                       0.5, 0, false};
    };

    if (options.scenario == "all") {
        return {make_active_only(), make_active_with_shadow(), make_shadow_overload()};
    }
    if (options.scenario == "active_only") {
        return {make_active_only()};
    }
    if (options.scenario == "active_with_shadow") {
        return {make_active_with_shadow()};
    }
    if (options.scenario == "shadow_overload") {
        return {make_shadow_overload()};
    }
    throw std::runtime_error("unsupported benchmark scenario");
}

} // namespace
} // namespace drone::estimation

int main(int argc, char** argv) {
    using namespace drone::estimation;

    try {
        const auto options = parse_cli_options(argc, argv);
        auto configs = scenario_configs_from_options(options);

        std::vector<BenchmarkScenarioResult> scenarios;
        scenarios.reserve(configs.size());
        for (const auto& config : configs) {
            scenarios.push_back(run_scenario(config, options.emit_samples));
        }

        std::optional<double> active_only_mean_us;
        for (const auto& scenario : scenarios) {
            if (scenario.config.mode == "active_only" && scenario.success &&
                scenario.active_process_latency.mean_us > 0.0) {
                active_only_mean_us = scenario.active_process_latency.mean_us;
                break;
            }
        }
        if (active_only_mean_us.has_value()) {
            for (auto& scenario : scenarios) {
                if (scenario.config.mode != "active_only" && scenario.success) {
                    scenario.active_path_regression_percent =
                        ((scenario.active_process_latency.mean_us - *active_only_mean_us) /
                         *active_only_mean_us) *
                        100.0;
                }
            }
        }

        json output{{"report_schema_version", 2},
                    {"date_utc", "2026-07-17"},
#if defined(_WIN32)
                    {"operating_system", "windows"},
#elif defined(__linux__)
                    {"operating_system", "linux"},
#else
                    {"operating_system", "unknown"},
#endif
#if defined(_MSC_VER)
                    {"compiler", std::string("MSVC ") + std::to_string(_MSC_VER)},
#elif defined(__clang__)
                    {"compiler", std::string("Clang ") + __clang_version__},
#elif defined(__GNUC__)
                    {"compiler", std::string("GCC ") + __VERSION__},
#else
                    {"compiler", "unknown"},
#endif
#ifdef NDEBUG
                    {"build_type", "Release"},
#else
                    {"build_type", "Debug"},
#endif
                    {"cpu_model", "not_captured_in_tool"},
                    {"timer_source", "std::chrono::steady_clock"},
                    {"limitations",
                     json::array({"software benchmark only",
                                  "does not imply hard real-time guarantees",
                                  "does not imply flight readiness",
                                  "queue push latency is not measured separately"})}};

        output["scenarios"] = json::array();
        bool overall_success = true;
        for (const auto& scenario : scenarios) {
            output["scenarios"].push_back(scenario_to_json(scenario, options.emit_samples));
            overall_success = overall_success && scenario.success;
        }
        output["success"] = overall_success;

        if (options.output_path.has_value()) {
            std::ofstream out(*options.output_path, std::ios::trunc);
            if (!out.is_open()) {
                std::cerr << "failed to open benchmark output path" << std::endl;
                return 1;
            }
            out << output.dump(2) << std::endl;
        } else {
            std::cout << output.dump(2) << std::endl;
        }

        return overall_success ? 0 : 1;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
}
