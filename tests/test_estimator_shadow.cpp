#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stop_token>
#include <thread>

#include "estimation/EstimatorCoordinator.hpp"
#include "estimation/MeasurementAdapters.hpp"
#include "estimation/MinimalEskfAdapter.hpp"
#include "runtime/RuntimeMode.hpp"

using namespace drone;

namespace {

estimation::EstimatorMeasurement make_imu_measurement(uint64_t sequence_id, double timestamp_s,
                                                      const Eigen::Vector3d& accel,
                                                      const Eigen::Vector3d& gyro) {
    estimation::EstimatorMeasurement measurement;
    measurement.header.timestamp_s = timestamp_s;
    measurement.header.sequence_id = sequence_id;
    measurement.header.source = estimation::SensorSource::IMU;
    measurement.header.frame = estimation::CoordinateFrame::BODY;
    measurement.header.type = estimation::MeasurementType::IMU;
    measurement.data = estimation::ImuMeasurementData{accel, gyro};
    return measurement;
}

estimation::EstimatorMeasurement make_visual_measurement(uint64_t sequence_id, double timestamp_s) {
    estimation::EstimatorMeasurement measurement;
    measurement.header.timestamp_s = timestamp_s;
    measurement.header.sequence_id = sequence_id;
    measurement.header.source = estimation::SensorSource::VISUAL_FRONTEND;
    measurement.header.frame = estimation::CoordinateFrame::WORLD;
    measurement.header.type = estimation::MeasurementType::VISUAL_POSE;

    estimation::VisualPoseMeasurementData visual;
    visual.position = Eigen::Vector3d{0.2, -0.1, 0.05};
    visual.velocity = Eigen::Vector3d{0.05, 0.0, 0.0};
    visual.orientation = Eigen::Quaterniond::Identity();
    visual.quality = 0.85;
    visual.covariance.dimension = 6;
    visual.covariance.matrix.setZero();
    visual.covariance.matrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.04;
    visual.covariance.matrix.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.04;
    measurement.data = visual;
    return measurement;
}

class ControlledTestEstimator final : public estimation::StateEstimator {
public:
    explicit ControlledTestEstimator(std::string name) : name_(std::move(name)) {}

    void set_process_delay(std::chrono::milliseconds delay) {
        process_delay_ = delay;
    }

    void set_failure_after(uint64_t accepted_processes) {
        failure_after_ = accepted_processes;
    }

    void set_position_bias(const Eigen::Vector3d& bias) {
        position_bias_ = bias;
    }

    void set_sequence_offset(int64_t offset) {
        sequence_offset_ = offset;
    }

    void set_external_reset_count(std::shared_ptr<std::atomic<uint64_t>> reset_count) {
        external_reset_count_ = std::move(reset_count);
    }

    [[nodiscard]] uint64_t reset_count() const {
        return reset_count_.load();
    }

    void reset(const estimation::EstimatorInitialState& initial) override {
        std::lock_guard lock(mutex_);
        ++reset_count_;
        if (external_reset_count_) {
            external_reset_count_->store(reset_count_.load());
        }
        process_count_ = 0;
        snapshot_ = {};
        snapshot_.pose.position = initial.position;
        snapshot_.pose.velocity = initial.velocity;
        snapshot_.pose.orientation = initial.orientation;
        snapshot_.pose.timestamp = 0.0;
        snapshot_.diagnostics.health_state = vio::EstimatorHealthState::INITIALIZING;
    }

    estimation::EstimatorUpdateResult process(
        const estimation::EstimatorMeasurement& measurement) override {
        if (process_delay_.count() > 0) {
            std::this_thread::sleep_for(process_delay_);
        }

        std::lock_guard lock(mutex_);
        ++process_count_;
        if (failure_after_.has_value() && process_count_ > *failure_after_) {
            throw std::runtime_error("controlled shadow failure");
        }

        snapshot_.pose.timestamp = measurement.header.timestamp_s;
        snapshot_.pose.position =
            position_bias_ +
            Eigen::Vector3d{static_cast<double>(measurement.header.sequence_id), 0.0, 0.0};
        snapshot_.pose.velocity = Eigen::Vector3d::Constant(0.01 * snapshot_.pose.position.x());
        snapshot_.pose.orientation = Eigen::Quaterniond::Identity();
        snapshot_.pose.localization_confidence = 0.91;
        snapshot_.diagnostics.health_state = vio::EstimatorHealthState::NOMINAL;
        snapshot_.last_sequence_id = static_cast<uint64_t>(
            static_cast<int64_t>(measurement.header.sequence_id) + sequence_offset_);

        estimation::EstimatorUpdateResult result;
        result.status = estimation::EstimatorUpdateStatus::ACCEPTED;
        result.validation.consumed = true;
        result.validation.status = estimation::MeasurementValidationStatus::ACCEPTED;
        result.snapshot = snapshot_;
        return result;
    }

    [[nodiscard]] estimation::EstimatorSnapshot snapshot() const override {
        std::lock_guard lock(mutex_);
        return snapshot_;
    }

    [[nodiscard]] vio::EstimatorDiagnostics diagnostics() const override {
        std::lock_guard lock(mutex_);
        return snapshot_.diagnostics;
    }

    [[nodiscard]] estimation::EstimatorCapabilities capabilities() const override {
        estimation::EstimatorCapabilities capabilities;
        capabilities.set(estimation::EstimatorCapability::IMU_PROPAGATION);
        return capabilities;
    }

    [[nodiscard]] std::string_view name() const noexcept override {
        return name_;
    }

private:
    std::string name_;
    std::chrono::milliseconds process_delay_{0};
    std::optional<uint64_t> failure_after_{};
    Eigen::Vector3d position_bias_{Eigen::Vector3d::Zero()};
    int64_t sequence_offset_{0};
    std::shared_ptr<std::atomic<uint64_t>> external_reset_count_{};

    mutable std::mutex mutex_;
    estimation::EstimatorSnapshot snapshot_{};
    std::atomic<uint64_t> reset_count_{0};
    uint64_t process_count_{0};
};

std::filesystem::path write_runtime_file(const std::string& name, const std::string& body) {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream output(path.string(), std::ios::trunc);
    EXPECT_TRUE(output.is_open());
    output << body;
    output.close();
    return path;
}

} // namespace

TEST(EstimatorAdapter, MatchesDirectEkfBehavior) {
    vio::EKFEstimator direct;
    direct.reset();

    estimation::MinimalEskfAdapter adapter;
    adapter.reset({});

    for (int i = 0; i < 50; ++i) {
        const double timestamp_s = 0.0025 * static_cast<double>(i + 1);
        const auto imu = make_imu_measurement(static_cast<uint64_t>(i + 1), timestamp_s,
                                              Eigen::Vector3d{0.01, 0.0, 9.81},
                                              Eigen::Vector3d{0.0, 0.0, 0.01});
        direct.propagate_imu(Eigen::Vector3d{0.01, 0.0, 9.81}, Eigen::Vector3d{0.0, 0.0, 0.01},
                             0.0025);
        static_cast<void>(adapter.process(imu));
    }

    direct.update_visual_pose(Eigen::Vector3d{0.2, -0.1, 0.05}, Eigen::Vector3d{0.05, 0.0, 0.0},
                              0.2, 0.2);
    direct.update_depth(0.05, 0.1);
    static_cast<void>(adapter.process(make_visual_measurement(60, 0.13)));

    estimation::EstimatorMeasurement depth;
    depth.header.timestamp_s = 0.131;
    depth.header.sequence_id = 61;
    depth.header.source = estimation::SensorSource::DEPTH_SENSOR;
    depth.header.frame = estimation::CoordinateFrame::WORLD;
    depth.header.type = estimation::MeasurementType::ALTITUDE;
    depth.data = estimation::AltitudeMeasurementData{0.05, 0.1};
    static_cast<void>(adapter.process(depth));

    const auto direct_state = direct.state();
    const auto adapter_state = adapter.snapshot().pose;
    EXPECT_TRUE(direct_state.position.isApprox(adapter_state.position, 1e-9));
    EXPECT_TRUE(direct_state.velocity.isApprox(adapter_state.velocity, 1e-9));
}

TEST(MeasurementAdapters, PreservesVisualOrientationWithoutConsumption) {
    const auto adapted = estimation::MeasurementAdapters::adapt_visual_pose(
        1.0, Eigen::Vector3d{1.0, 2.0, 3.0}, Eigen::Vector3d{0.1, 0.0, 0.0},
        Eigen::Quaterniond::Identity(), 0.8, true, 7, false);

    ASSERT_TRUE(adapted.measurement.has_value());
    EXPECT_FALSE(adapted.validation.orientation_consumed);
    const auto& visual = std::get<estimation::VisualPoseMeasurementData>(adapted.measurement->data);
    ASSERT_TRUE(visual.orientation.has_value());
    EXPECT_TRUE(
        visual.orientation->coeffs().isApprox(Eigen::Quaterniond::Identity().coeffs(), 1e-12));
}

TEST(MeasurementAdapters, RejectsInvalidTdoaCandidate) {
    const auto adapted = estimation::MeasurementAdapters::adapt_tdoa_position_candidate(
        1.0, Eigen::Vector3d{1.0, 2.0, 3.0}, 1.5, 4);
    EXPECT_FALSE(adapted.measurement.has_value());
    EXPECT_EQ(adapted.validation.status, estimation::MeasurementValidationStatus::REJECTED_INVALID);
}

TEST(EstimatorCoordinator, ShadowDisabledLeavesActiveOutputUnchanged) {
    auto active = std::make_shared<estimation::MinimalEskfAdapter>();
    estimation::EstimatorCoordinator coordinator(
        std::static_pointer_cast<estimation::StateEstimator>(active));
    coordinator.reset({});

    for (int i = 0; i < 20; ++i) {
        static_cast<void>(coordinator.process(make_imu_measurement(
            static_cast<uint64_t>(i + 1), 0.0025 * static_cast<double>(i + 1),
            Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero())));
    }

    const auto coordinated = coordinator.active_snapshot().pose;

    estimation::MinimalEskfAdapter direct;
    direct.reset({});
    for (int i = 0; i < 20; ++i) {
        static_cast<void>(direct.process(make_imu_measurement(
            static_cast<uint64_t>(i + 1), 0.0025 * static_cast<double>(i + 1),
            Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero())));
    }
    const auto direct_state = direct.snapshot().pose;
    EXPECT_TRUE(coordinated.position.isApprox(direct_state.position, 1e-12));
    EXPECT_TRUE(coordinated.velocity.isApprox(direct_state.velocity, 1e-12));
}

TEST(EstimatorCoordinator, ShadowEnabledDoesNotChangeActiveOutputAndTracksOverflow) {
    auto active = std::make_shared<ControlledTestEstimator>("active_test");
    estimation::EstimatorCoordinator coordinator(
        std::static_pointer_cast<estimation::StateEstimator>(active));
    coordinator.reset({});

    estimation::ShadowEstimatorConfig shadow_config;
    shadow_config.enabled = true;
    shadow_config.max_queue_depth = 2;
    shadow_config.max_lag_ms = 0.0;
    auto shadow = std::make_unique<ControlledTestEstimator>("shadow_test");
    shadow->set_process_delay(std::chrono::milliseconds(10));
    coordinator.configure_shadow(std::move(shadow_config), std::move(shadow));

    auto baseline = std::make_shared<ControlledTestEstimator>("baseline_test");
    baseline->reset({});
    for (int i = 0; i < 30; ++i) {
        const auto imu = make_imu_measurement(static_cast<uint64_t>(i + 1),
                                              0.0025 * static_cast<double>(i + 1),
                                              Eigen::Vector3d{0.0, 0.0, 9.81},
                                              Eigen::Vector3d::Zero());
        static_cast<void>(coordinator.process(imu));
        static_cast<void>(baseline->process(imu));
    }
    ASSERT_TRUE(coordinator.wait_for_shadow_idle(std::chrono::milliseconds(5000)));

    const auto active_state = coordinator.active_snapshot().pose;
    const auto baseline_state = baseline->snapshot().pose;
    const auto telemetry = coordinator.telemetry();

    EXPECT_TRUE(active_state.position.isApprox(baseline_state.position, 1e-12));
    EXPECT_TRUE(active_state.velocity.isApprox(baseline_state.velocity, 1e-12));
    EXPECT_GT(telemetry.dropped_events, 0u);
    coordinator.stop_shadow();
}

TEST(EstimatorCoordinator, ShadowFailureDoesNotStopActiveProcessing) {
    auto active = std::make_shared<ControlledTestEstimator>("active_test");
    estimation::EstimatorCoordinator coordinator(
        std::static_pointer_cast<estimation::StateEstimator>(active));
    coordinator.reset({});

    estimation::ShadowEstimatorConfig shadow_config;
    shadow_config.enabled = true;
    auto shadow = std::make_unique<ControlledTestEstimator>("shadow_test");
    shadow->set_failure_after(0);
    coordinator.configure_shadow(std::move(shadow_config), std::move(shadow));

    const auto first = coordinator.process(
        make_imu_measurement(1, 0.01, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()));
    const auto second = coordinator.process(
        make_imu_measurement(2, 0.02, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero()));
    ASSERT_TRUE(coordinator.wait_for_shadow_idle(std::chrono::milliseconds(5000)));

    EXPECT_EQ(first.status, estimation::EstimatorUpdateStatus::ACCEPTED);
    EXPECT_EQ(second.status, estimation::EstimatorUpdateStatus::ACCEPTED);
    EXPECT_EQ(coordinator.active_snapshot().last_sequence_id, 2u);
    EXPECT_EQ(coordinator.telemetry().shadow_health, estimation::ShadowHealthState::FAILED);
}

TEST(EstimatorCoordinator, ActiveProcessingDoesNotWaitForDelayedShadow) {
    auto active = std::make_shared<ControlledTestEstimator>("active_test");
    estimation::EstimatorCoordinator coordinator(
        std::static_pointer_cast<estimation::StateEstimator>(active));
    coordinator.reset({});

    estimation::ShadowEstimatorConfig shadow_config;
    shadow_config.enabled = true;
    shadow_config.max_lag_ms = 5.0;
    auto shadow = std::make_unique<ControlledTestEstimator>("shadow_test");
    shadow->set_process_delay(std::chrono::milliseconds(75));
    coordinator.configure_shadow(std::move(shadow_config), std::move(shadow));

    const auto started_at = std::chrono::steady_clock::now();
    static_cast<void>(coordinator.process(
        make_imu_measurement(1, 0.01, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero())));
    const auto elapsed_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started_at)
                                .count();

    EXPECT_LT(elapsed_ms, 30.0);
    ASSERT_TRUE(coordinator.wait_for_shadow_idle(std::chrono::milliseconds(5000)));
    EXPECT_EQ(coordinator.telemetry().shadow_health, estimation::ShadowHealthState::LAGGING);
}

TEST(EstimatorCoordinator, ShadowQueueDropsBecomeStaleAfterDrain) {
    auto active = std::make_shared<ControlledTestEstimator>("active_test");
    estimation::EstimatorCoordinator coordinator(
        std::static_pointer_cast<estimation::StateEstimator>(active));
    coordinator.reset({});

    estimation::ShadowEstimatorConfig shadow_config;
    shadow_config.enabled = true;
    shadow_config.max_queue_depth = 1;
    shadow_config.max_lag_ms = 1.0;
    auto shadow = std::make_unique<ControlledTestEstimator>("shadow_test");
    shadow->set_process_delay(std::chrono::milliseconds(20));
    coordinator.configure_shadow(std::move(shadow_config), std::move(shadow));

    for (uint64_t i = 1; i <= 16; ++i) {
        static_cast<void>(coordinator.process(
            make_imu_measurement(i, 0.01 * static_cast<double>(i), Eigen::Vector3d{0.0, 0.0, 9.81},
                                 Eigen::Vector3d::Zero())));
    }

    ASSERT_TRUE(coordinator.wait_for_shadow_idle(std::chrono::milliseconds(5000)));
    const auto telemetry = coordinator.telemetry();
    EXPECT_GT(telemetry.dropped_events, 0u);
    EXPECT_EQ(telemetry.shadow_health, estimation::ShadowHealthState::STALE);
    EXPECT_GE(telemetry.queue_high_water_mark, 1u);
}

TEST(EstimatorCoordinator, ReconfigureShadowResetsNewShadowInstance) {
    auto active = std::make_shared<ControlledTestEstimator>("active_test");
    estimation::EstimatorCoordinator coordinator(
        std::static_pointer_cast<estimation::StateEstimator>(active));
    coordinator.reset({});

    auto first_shadow = std::make_unique<ControlledTestEstimator>("shadow_one");
    auto first_shadow_reset_count = std::make_shared<std::atomic<uint64_t>>(0);
    auto second_shadow_reset_count = std::make_shared<std::atomic<uint64_t>>(0);

    estimation::ShadowEstimatorConfig shadow_config;
    shadow_config.enabled = true;
    first_shadow->set_external_reset_count(first_shadow_reset_count);
    coordinator.configure_shadow(shadow_config, std::move(first_shadow));

    auto second_shadow = std::make_unique<ControlledTestEstimator>("shadow_two");
    second_shadow->set_external_reset_count(second_shadow_reset_count);
    coordinator.configure_shadow(shadow_config, std::move(second_shadow));

    EXPECT_GE(first_shadow_reset_count->load(), 1u);
    EXPECT_GE(second_shadow_reset_count->load(), 1u);
    EXPECT_EQ(coordinator.telemetry().shadow_estimator_name, "shadow_two");
}

TEST(EstimatorCoordinator, StopBeforeStartAndDoubleStopAreSafe) {
    auto active = std::make_shared<ControlledTestEstimator>("active_test");
    estimation::EstimatorCoordinator coordinator(
        std::static_pointer_cast<estimation::StateEstimator>(active));

    coordinator.stop_shadow();
    coordinator.stop_shadow();

    estimation::ShadowEstimatorConfig shadow_config;
    shadow_config.enabled = true;
    coordinator.reset({});
    coordinator.configure_shadow(std::move(shadow_config),
                                 std::make_unique<ControlledTestEstimator>("shadow_test"));
    coordinator.stop_shadow();
    coordinator.stop_shadow();

    EXPECT_EQ(coordinator.telemetry().shadow_health, estimation::ShadowHealthState::STOPPED);
}

TEST(EstimatorCoordinator, TelemetryReadsRemainSafeDuringConcurrentProcessing) {
    auto active = std::make_shared<ControlledTestEstimator>("active_test");
    estimation::EstimatorCoordinator coordinator(
        std::static_pointer_cast<estimation::StateEstimator>(active));
    coordinator.reset({});

    estimation::ShadowEstimatorConfig shadow_config;
    shadow_config.enabled = true;
    auto shadow = std::make_unique<ControlledTestEstimator>("shadow_test");
    shadow->set_process_delay(std::chrono::milliseconds(3));
    coordinator.configure_shadow(std::move(shadow_config), std::move(shadow));

    std::jthread reader([&coordinator](std::stop_token stop_token) {
        while (!stop_token.stop_requested()) {
            static_cast<void>(coordinator.telemetry());
            static_cast<void>(coordinator.active_snapshot());
            static_cast<void>(coordinator.shadow_snapshot());
        }
    });

    for (uint64_t i = 1; i <= 64; ++i) {
        static_cast<void>(coordinator.process(
            make_imu_measurement(i, 0.0025 * static_cast<double>(i),
                                 Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero())));
    }

    ASSERT_TRUE(coordinator.wait_for_shadow_idle(std::chrono::milliseconds(5000)));
    reader.request_stop();
    EXPECT_GE(coordinator.telemetry().queue_high_water_mark, 1u);
}

TEST(EstimatorCoordinator, MismatchedShadowHistorySkipsComparison) {
    auto active = std::make_shared<ControlledTestEstimator>("active_test");
    estimation::EstimatorCoordinator coordinator(
        std::static_pointer_cast<estimation::StateEstimator>(active));
    coordinator.reset({});

    estimation::ShadowEstimatorConfig shadow_config;
    shadow_config.enabled = true;
    auto shadow = std::make_unique<ControlledTestEstimator>("shadow_test");
    shadow->set_sequence_offset(-1);
    coordinator.configure_shadow(std::move(shadow_config), std::move(shadow));

    static_cast<void>(coordinator.process(
        make_imu_measurement(5, 0.05, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero())));
    ASSERT_TRUE(coordinator.wait_for_shadow_idle(std::chrono::milliseconds(5000)));

    const auto telemetry = coordinator.telemetry();
    EXPECT_FALSE(telemetry.comparable);
    EXPECT_GT(telemetry.skipped_comparison_count, 0u);
}

TEST(EstimatorCoordinator, ShadowPoseNeverBecomesAuthoritativePose) {
    auto active = std::make_shared<ControlledTestEstimator>("active_test");
    estimation::EstimatorCoordinator coordinator(
        std::static_pointer_cast<estimation::StateEstimator>(active));
    coordinator.reset({});

    estimation::ShadowEstimatorConfig shadow_config;
    shadow_config.enabled = true;
    auto shadow = std::make_unique<ControlledTestEstimator>("shadow_test");
    shadow->set_position_bias(Eigen::Vector3d{100.0, 0.0, 0.0});
    coordinator.configure_shadow(std::move(shadow_config), std::move(shadow));

    static_cast<void>(coordinator.process(
        make_imu_measurement(3, 0.03, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero())));
    ASSERT_TRUE(coordinator.wait_for_shadow_idle(std::chrono::milliseconds(5000)));

    EXPECT_NEAR(coordinator.active_snapshot().pose.position.x(), 3.0, 1e-12);
    ASSERT_TRUE(coordinator.shadow_snapshot().has_value());
    EXPECT_GT(coordinator.shadow_snapshot()->pose.position.x(), 100.0);
}

TEST(RuntimeMode, RejectsUnsupportedHybridActiveConfiguration) {
    const auto temp_path = write_runtime_file(
        "runtime_hybrid_active_phase16.json",
        R"({
  "runtime_mode": "production",
  "anchor_config_path": "config/anchors.json",
  "estimator": {
    "mode": "hybrid_active",
    "enable_experimental_hybrid": false,
    "enable_fej": false,
    "enable_msckf": false,
    "enable_loop_closure_correction": false,
    "enable_automatic_zupt": false
  }
})");

    const auto result = runtime::load_runtime_file(temp_path.string());
    EXPECT_TRUE(result.loaded);
    EXPECT_FALSE(result.estimator_config_valid);
    std::filesystem::remove(temp_path);
}

TEST(RuntimeMode, ShadowRemainsDisabledByDefault) {
    const auto temp_path = write_runtime_file(
        "runtime_phase16_default_shadow.json",
        R"({
  "runtime_mode": "bench",
  "anchor_config_path": "config/anchors.json",
  "estimator": {
    "mode": "minimal"
  }
})");

    const auto result = runtime::load_runtime_file(temp_path.string());
    EXPECT_TRUE(result.loaded);
    EXPECT_FALSE(result.estimator_shadow_requested);
    EXPECT_FALSE(result.estimator_enable_shadow_estimator);
    EXPECT_FALSE(result.estimator_shadow_effective);
    std::filesystem::remove(temp_path);
}

TEST(RuntimeMode, RejectsUnknownEstimatorMode) {
    const auto temp_path = write_runtime_file(
        "runtime_phase16_unknown_mode.json",
        R"({
  "runtime_mode": "bench",
  "anchor_config_path": "config/anchors.json",
  "estimator": {
    "mode": "mystery"
  }
})");

    const auto result = runtime::load_runtime_file(temp_path.string());
    EXPECT_FALSE(result.estimator_config_valid);
    std::filesystem::remove(temp_path);
}

TEST(RuntimeMode, RejectsInvalidShadowLimitsAndImplementation) {
    const auto temp_path = write_runtime_file(
        "runtime_phase16_bad_shadow.json",
        R"({
  "runtime_mode": "bench",
  "anchor_config_path": "config/anchors.json",
  "estimator": {
    "mode": "minimal",
    "shadow": {
      "enabled": true,
      "implementation": "mystery_shadow",
      "max_queue_depth": 0,
      "max_lag_ms": -5.0,
      "position_divergence_m": 0.25,
      "velocity_divergence_mps": 0.2,
      "orientation_divergence_deg": 5.0,
      "required_consecutive_divergent_samples": 0
    }
  }
})");

    const auto result = runtime::load_runtime_file(temp_path.string());
    EXPECT_FALSE(result.estimator_config_valid);
    EXPECT_TRUE(result.estimator_shadow_requested);
    EXPECT_FALSE(result.estimator_shadow_effective);
    std::filesystem::remove(temp_path);
}

TEST(RuntimeMode, RejectsDisabledExperimentalFeatures) {
    const auto temp_path = write_runtime_file(
        "runtime_phase16_bad_experimental_flags.json",
        R"({
  "runtime_mode": "bench",
  "anchor_config_path": "config/anchors.json",
  "estimator": {
    "mode": "minimal",
    "enable_fej": true,
    "enable_msckf": true,
    "enable_loop_closure_correction": true,
    "enable_automatic_zupt": true
  }
})");

    const auto result = runtime::load_runtime_file(temp_path.string());
    EXPECT_FALSE(result.estimator_config_valid);
    std::filesystem::remove(temp_path);
}

TEST(RuntimeMode, ShadowReportsEffectiveWhenValidAndSupported) {
    const auto temp_path = write_runtime_file(
        "runtime_phase16_shadow_enabled.json",
        R"({
  "runtime_mode": "bench",
  "anchor_config_path": "config/anchors.json",
  "estimator": {
    "mode": "minimal",
    "shadow": {
      "enabled": true,
      "comparison_enabled": true,
      "implementation": "minimal_eskf_clone",
      "max_queue_depth": 8,
      "max_lag_ms": 100.0,
      "position_divergence_m": 0.25,
      "velocity_divergence_mps": 0.2,
      "orientation_divergence_deg": 5.0,
      "required_consecutive_divergent_samples": 10
    }
  }
})");

    const auto result = runtime::load_runtime_file(temp_path.string());
    EXPECT_TRUE(result.estimator_config_valid);
    EXPECT_TRUE(result.estimator_shadow_requested);
    EXPECT_EQ(result.estimator_enable_shadow_estimator,
              result.estimator_shadow_compile_time_supported);
    EXPECT_EQ(result.estimator_shadow_effective, result.estimator_shadow_compile_time_supported);
    std::filesystem::remove(temp_path);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
