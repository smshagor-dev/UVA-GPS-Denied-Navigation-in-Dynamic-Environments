#include <gtest/gtest.h>

#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

using namespace drone::vio;

namespace {

using Clock = std::chrono::steady_clock;

EstimatorValidationConfig make_phase16_cfg(bool shadow_enabled = false, uint32_t queue_depth = 16) {
    EstimatorValidationConfig cfg;
    cfg.enable_shadow_estimator = shadow_enabled;
    cfg.shadow_comparison_enabled = shadow_enabled;
    cfg.shadow_max_queue_depth = queue_depth;
    cfg.shadow_max_lag_ms = 25.0;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    return cfg;
}

MeasurementEnvelope imu_env(double timestamp_s, uint64_t sequence_id,
                            const Eigen::Vector3d& accel = Eigen::Vector3d{0.0, 0.0, 9.81},
                            const Eigen::Vector3d& gyro = Eigen::Vector3d::Zero()) {
    MeasurementEnvelope envelope;
    envelope.type = MeasurementType::Imu;
    envelope.source_id = "imu";
    envelope.timestamp_s = timestamp_s;
    envelope.sequence_id = sequence_id;
    envelope.frame = MeasurementFrame::Body;
    envelope.payload = ImuMeasurementPayload{accel, gyro};
    return envelope;
}

struct BlockingGate {
    std::mutex mutex;
    std::condition_variable cv;
    bool first_call_seen{false};
    bool released{false};
};

class DelayedShadowEstimator final : public StateEstimator {
public:
    explicit DelayedShadowEstimator(std::chrono::milliseconds delay = std::chrono::milliseconds{0},
                                    std::shared_ptr<BlockingGate> gate = {})
        : inner_(EKFConfig{}, "delayed_shadow", "phase16-test"), delay_(delay),
          gate_(std::move(gate)) {}

    void configure_validation(const EstimatorValidationConfig& cfg) override {
        inner_.configure_validation(cfg);
    }

    void reset(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
               const Eigen::Vector3d& v0) override {
        inner_.reset(p0, q0, v0);
    }

    EstimatorOperationResult process_measurement(const MeasurementEnvelope& envelope) override {
        ++process_calls_;
        if (gate_) {
            std::unique_lock lock(gate_->mutex);
            if (!gate_->first_call_seen) {
                gate_->first_call_seen = true;
                gate_->cv.notify_all();
                gate_->cv.wait(lock, [this] { return gate_->released; });
            }
        }
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

    uint64_t process_calls() const {
        return process_calls_.load();
    }

private:
    EKFStateEstimatorAdapter inner_;
    std::chrono::milliseconds delay_;
    std::shared_ptr<BlockingGate> gate_;
    std::atomic<uint64_t> process_calls_{0};
};

class ThrowingShadowEstimator final : public StateEstimator {
public:
    void configure_validation(const EstimatorValidationConfig&) override {}
    void reset(const Eigen::Vector3d&, const Eigen::Quaterniond&, const Eigen::Vector3d&) override {
        initialized_ = true;
    }
    EstimatorOperationResult process_measurement(const MeasurementEnvelope&) override {
        throw std::runtime_error("shadow failure");
    }
    EstimatorStateSnapshot snapshot() const override {
        EstimatorStateSnapshot out;
        out.initialized = initialized_;
        out.health = initialized_ ? EstimatorHealth::Healthy : EstimatorHealth::Uninitialized;
        out.estimator_name = "throwing_shadow";
        return out;
    }
    EKFDiagnostics diagnostics() const override {
        return {};
    }
    bool is_initialized() const override {
        return initialized_;
    }
    std::string estimator_name() const override {
        return "throwing_shadow";
    }
    std::string estimator_version() const override {
        return "test";
    }

private:
    bool initialized_{false};
};

class NonFiniteSnapshotEstimator final : public StateEstimator {
public:
    NonFiniteSnapshotEstimator() : inner_(EKFConfig{}, "nonfinite_shadow", "phase16-test") {}

    void configure_validation(const EstimatorValidationConfig& cfg) override {
        inner_.configure_validation(cfg);
    }

    void reset(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
               const Eigen::Vector3d& v0) override {
        inner_.reset(p0, q0, v0);
    }

    EstimatorOperationResult process_measurement(const MeasurementEnvelope& envelope) override {
        return inner_.process_measurement(envelope);
    }

    EstimatorStateSnapshot snapshot() const override {
        auto out = inner_.snapshot();
        out.position_m.x() = std::numeric_limits<double>::quiet_NaN();
        return out;
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
};

} // namespace

TEST(Phase16Adapter, DirectAndAdapterMatchForImuPropagation) {
    EKFEstimator direct;
    direct.configure_validation(make_phase16_cfg(false));
    direct.reset();

    EKFStateEstimatorAdapter adapter;
    adapter.configure_validation(make_phase16_cfg(false));
    adapter.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    for (int i = 0; i < 200; ++i) {
        const double timestamp = i * 0.0025;
        ASSERT_EQ(direct.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                 Eigen::Vector3d::Zero(), timestamp),
                  adapter.process_measurement(imu_env(timestamp, static_cast<uint64_t>(i))));
    }

    const auto direct_state = direct.state();
    const auto adapter_state = adapter.snapshot();
    EXPECT_NEAR((direct_state.position - adapter_state.position_m).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((direct_state.velocity - adapter_state.velocity_mps).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR(std::abs(direct_state.orientation.dot(adapter_state.orientation)), 1.0, 1.0e-12);
}

TEST(Phase16Coordinator, StopBeforeStartIsSafe) {
    EstimatorCoordinator coordinator(std::make_unique<EKFStateEstimatorAdapter>(),
                                     std::make_unique<EKFStateEstimatorAdapter>());
    coordinator.configure_validation(make_phase16_cfg(true, 8));
    coordinator.stop();
    const auto diagnostics = coordinator.diagnostics();
    EXPECT_FALSE(diagnostics.worker_running);
}

TEST(Phase16Coordinator, RepeatedStartStopIsSafe) {
    EstimatorCoordinator coordinator(std::make_unique<EKFStateEstimatorAdapter>(),
                                     std::make_unique<EKFStateEstimatorAdapter>());
    coordinator.configure_validation(make_phase16_cfg(true, 8));
    coordinator.initialize();
    EXPECT_TRUE(coordinator.start());
    EXPECT_TRUE(coordinator.start());
    coordinator.stop();
    coordinator.stop();
    EXPECT_TRUE(coordinator.start());
    coordinator.stop();
    const auto diagnostics = coordinator.diagnostics();
    EXPECT_GE(diagnostics.shadow_restart_count, 1u);
    EXPECT_FALSE(diagnostics.worker_running);
}

TEST(Phase16Coordinator, ShadowDisabledKeepsActiveDeterministic) {
    auto active_only = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active_a"),
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_shadow_a"));
    active_only.configure_validation(make_phase16_cfg(false));
    active_only.initialize();

    auto with_shadow = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active_b"),
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_shadow_b"));
    with_shadow.configure_validation(make_phase16_cfg(true, 32));
    with_shadow.initialize();
    ASSERT_TRUE(with_shadow.start());

    for (int i = 0; i < 400; ++i) {
        const double timestamp = i * 0.0025;
        ASSERT_EQ(active_only.process_measurement(imu_env(timestamp, static_cast<uint64_t>(i))),
                  with_shadow.process_measurement(imu_env(timestamp, static_cast<uint64_t>(i))));
    }
    with_shadow.flush_shadow();

    const auto a = active_only.active_snapshot();
    const auto b = with_shadow.active_snapshot();
    EXPECT_NEAR((a.position_m - b.position_m).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((a.velocity_mps - b.velocity_mps).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR(std::abs(a.orientation.dot(b.orientation)), 1.0, 1.0e-12);
}

TEST(Phase16Coordinator, ResetInvalidatesQueuedMeasurementsAndClearsQueue) {
    auto gate = std::make_shared<BlockingGate>();
    EstimatorCoordinator coordinator(
        std::make_unique<EKFStateEstimatorAdapter>(),
        std::make_unique<DelayedShadowEstimator>(std::chrono::milliseconds{0}, gate),
        ShadowCoordinatorConfig{});
    coordinator.configure_validation(make_phase16_cfg(true, 4));
    coordinator.initialize();
    ASSERT_TRUE(coordinator.start());

    EXPECT_EQ(coordinator.process_measurement(imu_env(0.0, 0)), EstimatorOperationResult::Accepted);
    {
        std::unique_lock lock(gate->mutex);
        ASSERT_TRUE(gate->cv.wait_for(lock, std::chrono::seconds(1),
                                      [&] { return gate->first_call_seen; }));
    }

    for (int i = 1; i < 8; ++i) {
        EXPECT_EQ(coordinator.process_measurement(imu_env(i * 0.0025, static_cast<uint64_t>(i))),
                  EstimatorOperationResult::Accepted);
    }

    coordinator.reset();
    {
        std::lock_guard lock(gate->mutex);
        gate->released = true;
    }
    gate->cv.notify_all();
    coordinator.flush_shadow();

    const auto diagnostics = coordinator.diagnostics();
    EXPECT_EQ(coordinator.queue_depth(), 0u);
    EXPECT_FALSE(coordinator.shadow_snapshot().has_value());
    EXPECT_FALSE(diagnostics.last_comparison.valid);
    EXPECT_EQ(diagnostics.last_comparison.generation, 0u);
    EXPECT_EQ(diagnostics.reset_generation, 2u);
}

TEST(Phase16Coordinator, StopWithPendingQueueDrainsSafely) {
    EstimatorCoordinator coordinator(
        std::make_unique<EKFStateEstimatorAdapter>(),
        std::make_unique<DelayedShadowEstimator>(std::chrono::milliseconds{2}),
        ShadowCoordinatorConfig{});
    coordinator.configure_validation(make_phase16_cfg(true, 8));
    coordinator.initialize();
    ASSERT_TRUE(coordinator.start());
    for (int i = 0; i < 20; ++i) {
        EXPECT_EQ(coordinator.process_measurement(imu_env(i * 0.0025, static_cast<uint64_t>(i))),
                  EstimatorOperationResult::Accepted);
    }
    coordinator.stop();
    const auto diagnostics = coordinator.diagnostics();
    EXPECT_FALSE(diagnostics.worker_running);
    EXPECT_LE(diagnostics.queue.current_depth, diagnostics.queue.capacity);
}

TEST(Phase16Coordinator, DedicatedOverloadDropOldestRemainsNonBlocking) {
    auto gate = std::make_shared<BlockingGate>();
    constexpr uint32_t kQueueCapacity = 4;
    constexpr int kSamples = 32;
    EstimatorCoordinator coordinator(
        std::make_unique<EKFStateEstimatorAdapter>(),
        std::make_unique<DelayedShadowEstimator>(std::chrono::milliseconds{0}, gate),
        ShadowCoordinatorConfig{});
    coordinator.configure_validation(make_phase16_cfg(true, kQueueCapacity));
    coordinator.initialize();
    ASSERT_TRUE(coordinator.start());

    EXPECT_EQ(coordinator.process_measurement(imu_env(0.0, 0)), EstimatorOperationResult::Accepted);
    {
        std::unique_lock lock(gate->mutex);
        ASSERT_TRUE(gate->cv.wait_for(lock, std::chrono::seconds(1),
                                      [&] { return gate->first_call_seen; }));
    }

    const auto begin = Clock::now();
    for (int i = 1; i < kSamples; ++i) {
        EXPECT_EQ(coordinator.process_measurement(imu_env(i * 0.0025, static_cast<uint64_t>(i))),
                  EstimatorOperationResult::Accepted);
    }
    const auto producer_duration_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - begin).count();

    {
        std::lock_guard lock(gate->mutex);
        gate->released = true;
    }
    gate->cv.notify_all();
    coordinator.flush_shadow();

    const auto diagnostics = coordinator.diagnostics();
    EXPECT_LT(producer_duration_ms, 100);
    EXPECT_EQ(diagnostics.active_processed_count, static_cast<uint64_t>(kSamples));
    EXPECT_LE(diagnostics.queue.peak_depth, static_cast<size_t>(kQueueCapacity));
    EXPECT_EQ(diagnostics.queue.dropped_count,
              static_cast<uint64_t>(kSamples - (kQueueCapacity + 1)));
    EXPECT_EQ(diagnostics.shadow_processed_count, static_cast<uint64_t>(kQueueCapacity + 1));
    EXPECT_EQ(coordinator.queue_depth(), 0u);
}

TEST(Phase16Coordinator, UnsupportedEnvelopeRejectedButActiveAlive) {
    EstimatorCoordinator coordinator(std::make_unique<EKFStateEstimatorAdapter>(),
                                     std::make_unique<EKFStateEstimatorAdapter>());
    coordinator.configure_validation(make_phase16_cfg(false));
    coordinator.initialize();

    auto unsupported = make_lidar_depth_envelope(MeasurementStamp{0.01, 1}, 3.0, 0.05, false);
    EXPECT_EQ(coordinator.process_measurement(unsupported),
              EstimatorOperationResult::RejectedUnsupportedMeasurement);
    EXPECT_TRUE(coordinator.active_snapshot().initialized);
}

TEST(Phase16Coordinator, ShadowFailureIsContained) {
    EstimatorCoordinator coordinator(std::make_unique<EKFStateEstimatorAdapter>(),
                                     std::make_unique<ThrowingShadowEstimator>());
    coordinator.configure_validation(make_phase16_cfg(true, 8));
    coordinator.initialize();
    ASSERT_TRUE(coordinator.start());

    EXPECT_EQ(coordinator.process_measurement(imu_env(0.0, 0)), EstimatorOperationResult::Accepted);
    coordinator.flush_shadow();

    const auto snapshot = coordinator.active_snapshot();
    const auto diagnostics = coordinator.diagnostics();
    EXPECT_TRUE(snapshot.initialized);
    EXPECT_GE(diagnostics.shadow_failure_count, 1u);
    EXPECT_EQ(diagnostics.shadow_health, EstimatorHealth::Failed);
    EXPECT_EQ(diagnostics.lifecycle_state, ShadowLifecycleState::Failed);
}

TEST(Phase16Coordinator, InvalidShadowSnapshotPreventsValidComparison) {
    EstimatorCoordinator coordinator(std::make_unique<EKFStateEstimatorAdapter>(),
                                     std::make_unique<NonFiniteSnapshotEstimator>());
    coordinator.configure_validation(make_phase16_cfg(true, 8));
    coordinator.initialize();
    ASSERT_TRUE(coordinator.start());

    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(coordinator.process_measurement(imu_env(i * 0.0025, static_cast<uint64_t>(i))),
                  EstimatorOperationResult::Accepted);
    }
    coordinator.flush_shadow();

    const auto diagnostics = coordinator.diagnostics();
    EXPECT_FALSE(diagnostics.last_comparison.valid);
    EXPECT_EQ(diagnostics.last_comparison.reason, "invalid_snapshot");
    EXPECT_EQ(diagnostics.invalid_comparison_count, 4u);
}
