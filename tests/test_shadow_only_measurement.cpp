#include <gtest/gtest.h>

#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"
#include "vio/EstimatorPromotionReadiness.hpp"
#include "vio/EstimatorPromotionSoakMonitor.hpp"

#include <atomic>
#include <limits>
#include <memory>

using namespace drone::vio;

namespace {

EstimatorValidationConfig shadow_cfg(bool enabled = true) {
    EstimatorValidationConfig cfg;
    cfg.enable_shadow_estimator = enabled;
    cfg.shadow_comparison_enabled = false;
    cfg.shadow_max_queue_depth = 16;
    cfg.shadow_max_lag_ms = 100.0;
    return cfg;
}

class RecordingShadowEstimator final : public StateEstimator {
public:
    void configure_validation(const EstimatorValidationConfig&) override {}

    void reset(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
               const Eigen::Vector3d& v0) override {
        snapshot_ = {};
        snapshot_.initialized = true;
        snapshot_.health = EstimatorHealth::Healthy;
        snapshot_.position_m = p0;
        snapshot_.orientation = q0;
        snapshot_.velocity_mps = v0;
        snapshot_.estimator_name = "recording_shadow";
        snapshot_.frame = MeasurementFrame::World;
    }

    EstimatorOperationResult process_measurement(const MeasurementEnvelope& envelope) override {
        ++process_count_;
        last_type_.store(static_cast<uint8_t>(envelope.type));
        snapshot_.timestamp_s = envelope.timestamp_s;
        return EstimatorOperationResult::Accepted;
    }

    [[nodiscard]] EstimatorStateSnapshot snapshot() const override {
        return snapshot_;
    }

    [[nodiscard]] EKFDiagnostics diagnostics() const override {
        return {};
    }

    [[nodiscard]] bool is_initialized() const override {
        return snapshot_.initialized;
    }

    [[nodiscard]] std::string estimator_name() const override {
        return "recording_shadow";
    }

    [[nodiscard]] std::string estimator_version() const override {
        return "shadow-only-test";
    }

    [[nodiscard]] uint64_t process_count() const {
        return process_count_.load();
    }

    [[nodiscard]] MeasurementType last_type() const {
        return static_cast<MeasurementType>(last_type_.load());
    }

private:
    EstimatorStateSnapshot snapshot_{};
    std::atomic<uint64_t> process_count_{0};
    std::atomic<uint8_t> last_type_{static_cast<uint8_t>(MeasurementType::Imu)};
};

MeasurementEnvelope valid_feature_envelope() {
    VisualFeatureMeasurementPayload payload;
    payload.K = Eigen::Matrix3d::Identity();
    payload.K(0, 0) = 320.0;
    payload.K(1, 1) = 320.0;
    payload.K(0, 2) = 320.0;
    payload.K(1, 2) = 240.0;
    payload.z_pixels.push_back(Eigen::Vector2d{320.0, 240.0});
    payload.p_world.push_back(Eigen::Vector3d{0.0, 0.0, 4.0});
    return make_visual_features_envelope(payload, MeasurementStamp{0.01, 1u});
}

CoordinatorDiagnostics promotion_ready_diagnostics() {
    CoordinatorDiagnostics diagnostics;
    diagnostics.shadow_enabled = true;
    diagnostics.worker_running = true;
    diagnostics.lifecycle_state = ShadowLifecycleState::Running;
    diagnostics.active_health = EstimatorHealth::Healthy;
    diagnostics.shadow_health = EstimatorHealth::Healthy;
    diagnostics.valid_comparison_count = 250;
    diagnostics.last_comparison.valid = true;
    diagnostics.last_comparison.position_delta_norm_m = 0.05;
    diagnostics.last_comparison.velocity_delta_norm_mps = 0.06;
    diagnostics.last_comparison.orientation_delta_deg = 0.5;
    diagnostics.last_comparison.covariance_trace_delta = 0.1;
    return diagnostics;
}

} // namespace

TEST(ShadowOnlyMeasurement, FeatureSubmissionNeverTouchesActiveEstimator) {
    auto shadow = std::make_unique<RecordingShadowEstimator>();
    auto* shadow_ptr = shadow.get();
    EstimatorCoordinator coordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "active_baseline"),
        std::move(shadow));
    coordinator.configure_validation(shadow_cfg(true));
    coordinator.initialize();
    ASSERT_TRUE(coordinator.start());

    const auto active_before = coordinator.active_snapshot();
    const auto active_count_before = coordinator.diagnostics().active_processed_count;

    ASSERT_EQ(coordinator.submit_shadow_measurement(valid_feature_envelope()),
              EstimatorOperationResult::Accepted);
    coordinator.flush_shadow();

    const auto active_after = coordinator.active_snapshot();
    const auto diagnostics = coordinator.diagnostics();
    EXPECT_EQ(diagnostics.active_processed_count, active_count_before);
    EXPECT_EQ(diagnostics.shadow_only_submission_count, 1u);
    EXPECT_EQ(diagnostics.shadow_only_rejected_count, 0u);
    EXPECT_EQ(shadow_ptr->process_count(), 1u);
    EXPECT_EQ(shadow_ptr->last_type(), MeasurementType::VisualFeatures);
    EXPECT_NEAR((active_after.position_m - active_before.position_m).norm(), 0.0, 1.0e-15);
    EXPECT_NEAR((active_after.velocity_mps - active_before.velocity_mps).norm(), 0.0, 1.0e-15);
    EXPECT_NEAR(std::abs(active_after.orientation.dot(active_before.orientation)), 1.0, 1.0e-15);
}

TEST(ShadowOnlyMeasurement, InvalidEnvelopeFailsBeforeQueuePublication) {
    EstimatorCoordinator coordinator(
        std::make_unique<EKFStateEstimatorAdapter>(),
        std::make_unique<RecordingShadowEstimator>());
    coordinator.configure_validation(shadow_cfg(true));
    coordinator.initialize();
    ASSERT_TRUE(coordinator.start());

    auto envelope = valid_feature_envelope();
    auto& payload = std::get<VisualFeatureMeasurementPayload>(envelope.payload);
    payload.z_pixels.front().x() = std::numeric_limits<double>::quiet_NaN();

    EXPECT_EQ(coordinator.submit_shadow_measurement(envelope),
              EstimatorOperationResult::RejectedNonFiniteInput);
    const auto diagnostics = coordinator.diagnostics();
    EXPECT_EQ(diagnostics.shadow_only_submission_count, 0u);
    EXPECT_EQ(diagnostics.shadow_only_rejected_count, 1u);
    EXPECT_EQ(diagnostics.queue.enqueued_count, 0u);
}

TEST(ShadowOnlyMeasurement, DisabledShadowFailsClosedWithoutActiveMutation) {
    EstimatorCoordinator coordinator(
        std::make_unique<EKFStateEstimatorAdapter>(),
        std::make_unique<RecordingShadowEstimator>());
    coordinator.configure_validation(shadow_cfg(false));
    coordinator.initialize();

    const auto active_before = coordinator.active_snapshot();
    EXPECT_EQ(coordinator.submit_shadow_measurement(valid_feature_envelope()),
              EstimatorOperationResult::RejectedInvalidConfiguration);
    const auto active_after = coordinator.active_snapshot();
    const auto diagnostics = coordinator.diagnostics();
    EXPECT_EQ(diagnostics.shadow_only_submission_count, 0u);
    EXPECT_EQ(diagnostics.shadow_only_rejected_count, 1u);
    EXPECT_EQ(diagnostics.active_processed_count, 0u);
    EXPECT_NEAR((active_after.position_m - active_before.position_m).norm(), 0.0, 1.0e-15);
}

TEST(ShadowPromotionReadiness, HealthyBoundedComparisonWindowCanBecomeReady) {
    const auto result = assess_promotion_readiness(promotion_ready_diagnostics());
    EXPECT_TRUE(result.ready);
    EXPECT_EQ(result.reason, PromotionBlockReason::None);
    EXPECT_EQ(to_string(result.reason), "ready");
}

TEST(ShadowPromotionReadiness, FailsClosedWhenEvidenceIsInsufficient) {
    auto diagnostics = promotion_ready_diagnostics();
    diagnostics.valid_comparison_count = 99;

    const auto result = assess_promotion_readiness(diagnostics);
    EXPECT_FALSE(result.ready);
    EXPECT_EQ(result.reason, PromotionBlockReason::InsufficientComparisons);
}

TEST(ShadowPromotionReadiness, FailsClosedOnQueueDrop) {
    auto diagnostics = promotion_ready_diagnostics();
    diagnostics.queue.dropped_count = 1;

    const auto result = assess_promotion_readiness(diagnostics);
    EXPECT_FALSE(result.ready);
    EXPECT_EQ(result.reason, PromotionBlockReason::QueueDropsObserved);
}

TEST(ShadowPromotionReadiness, FailsClosedWhenComparisonExceedsConfiguredBound) {
    auto diagnostics = promotion_ready_diagnostics();
    diagnostics.last_comparison.position_delta_norm_m = 0.30;

    const auto result = assess_promotion_readiness(diagnostics);
    EXPECT_FALSE(result.ready);
    EXPECT_EQ(result.reason, PromotionBlockReason::PositionDeltaExceeded);
}

TEST(ShadowPromotionReadiness, DoesNotTreatNegativeCovarianceDeltaAsUnbounded) {
    auto diagnostics = promotion_ready_diagnostics();
    diagnostics.last_comparison.covariance_trace_delta = -1.5;

    const auto result = assess_promotion_readiness(diagnostics);
    EXPECT_FALSE(result.ready);
    EXPECT_EQ(result.reason, PromotionBlockReason::CovarianceTraceDeltaExceeded);
}

TEST(ShadowPromotionSoak, RequiresConsecutiveReadySamples) {
    PromotionSoakMonitor monitor(PromotionSoakConfig{3, 0, true});
    const auto ready = assess_promotion_readiness(promotion_ready_diagnostics());

    EXPECT_FALSE(monitor.observe(ready).sustained_ready);
    EXPECT_FALSE(monitor.observe(ready).sustained_ready);
    const auto& state = monitor.observe(ready);
    EXPECT_TRUE(state.sustained_ready);
    EXPECT_EQ(state.consecutive_ready_samples, 3u);
    EXPECT_EQ(state.ready_samples, 3u);
}

TEST(ShadowPromotionSoak, BlockedSampleResetsReadinessProgress) {
    PromotionSoakMonitor monitor(PromotionSoakConfig{3, 0, true});
    const auto ready = assess_promotion_readiness(promotion_ready_diagnostics());
    auto blocked_diagnostics = promotion_ready_diagnostics();
    blocked_diagnostics.queue.dropped_count = 1;
    const auto blocked = assess_promotion_readiness(blocked_diagnostics);

    monitor.observe(ready);
    monitor.observe(ready);
    const auto& blocked_state = monitor.observe(blocked);
    EXPECT_FALSE(blocked_state.sustained_ready);
    EXPECT_EQ(blocked_state.consecutive_ready_samples, 0u);
    EXPECT_EQ(blocked_state.reset_count, 1u);
    EXPECT_EQ(blocked_state.last_block_reason, PromotionBlockReason::QueueDropsObserved);

    EXPECT_FALSE(monitor.observe(ready).sustained_ready);
    EXPECT_FALSE(monitor.observe(ready).sustained_ready);
    EXPECT_TRUE(monitor.observe(ready).sustained_ready);
}

TEST(ShadowPromotionSoak, DiagnosticsPathUsesExistingFailClosedReadinessRules) {
    PromotionSoakMonitor monitor(PromotionSoakConfig{2, 0, true});
    auto diagnostics = promotion_ready_diagnostics();

    EXPECT_FALSE(monitor.observe(diagnostics).sustained_ready);
    EXPECT_TRUE(monitor.observe(diagnostics).sustained_ready);

    diagnostics.shadow_health = EstimatorHealth::Degraded;
    const auto& state = monitor.observe(diagnostics);
    EXPECT_FALSE(state.sustained_ready);
    EXPECT_EQ(state.last_block_reason, PromotionBlockReason::ShadowUnhealthy);
}

TEST(ShadowPromotionSoak, ResetClearsAllAccumulatedEvidence) {
    PromotionSoakMonitor monitor(PromotionSoakConfig{2, 0, true});
    const auto ready = assess_promotion_readiness(promotion_ready_diagnostics());
    monitor.observe(ready);
    monitor.observe(ready);
    ASSERT_TRUE(monitor.state().sustained_ready);

    monitor.reset();
    EXPECT_EQ(monitor.state().total_samples, 0u);
    EXPECT_EQ(monitor.state().ready_samples, 0u);
    EXPECT_EQ(monitor.state().consecutive_ready_samples, 0u);
    EXPECT_FALSE(monitor.state().sustained_ready);
}
