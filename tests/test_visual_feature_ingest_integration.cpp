#include <gtest/gtest.h>

#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"
#include "vio/VisualFeatureTrackManager.hpp"

#include <atomic>
#include <memory>
#include <vector>

using namespace drone::vio;

namespace {

class RecordingFeatureShadow final : public StateEstimator {
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
        snapshot_.estimator_name = "feature_shadow";
        snapshot_.frame = MeasurementFrame::World;
        process_count_.store(0);
        visual_feature_count_.store(0);
    }

    EstimatorOperationResult process_measurement(const MeasurementEnvelope& envelope) override {
        ++process_count_;
        snapshot_.timestamp_s = envelope.timestamp_s;
        if (envelope.type == MeasurementType::VisualFeatures) {
            const auto& payload = std::get<VisualFeatureMeasurementPayload>(envelope.payload);
            visual_feature_count_.fetch_add(payload.z_pixels.size());
        }
        return EstimatorOperationResult::Accepted;
    }

    [[nodiscard]] EstimatorStateSnapshot snapshot() const override { return snapshot_; }
    [[nodiscard]] EKFDiagnostics diagnostics() const override { return {}; }
    [[nodiscard]] bool is_initialized() const override { return snapshot_.initialized; }
    [[nodiscard]] std::string estimator_name() const override { return "feature_shadow"; }
    [[nodiscard]] std::string estimator_version() const override { return "visual-feature-ingest"; }
    [[nodiscard]] uint64_t process_count() const { return process_count_.load(); }
    [[nodiscard]] uint64_t visual_feature_count() const { return visual_feature_count_.load(); }

private:
    EstimatorStateSnapshot snapshot_{};
    std::atomic<uint64_t> process_count_{0};
    std::atomic<uint64_t> visual_feature_count_{0};
};

EstimatorValidationConfig enabled_shadow_config() {
    EstimatorValidationConfig cfg;
    cfg.enable_shadow_estimator = true;
    cfg.shadow_comparison_enabled = false;
    cfg.shadow_max_queue_depth = 32;
    cfg.shadow_max_lag_ms = 100.0;
    return cfg;
}

Eigen::Matrix3d camera_matrix() {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 320.0;
    K(1, 1) = 320.0;
    K(0, 2) = 320.0;
    K(1, 2) = 240.0;
    return K;
}

PoseEstimate pose(double x_m) {
    PoseEstimate out;
    out.position = Eigen::Vector3d{x_m, 0.0, 0.0};
    out.orientation = Eigen::Quaterniond::Identity();
    out.velocity = Eigen::Vector3d::Zero();
    return out;
}

std::vector<Eigen::Vector2d> previous_pixels() {
    return {{320.0, 240.0}, {352.0, 240.0}, {288.0, 240.0}, {320.0, 272.0}};
}

std::vector<Eigen::Vector2d> current_pixels() {
    return {{304.0, 240.0}, {336.0, 240.0}, {272.0, 240.0}, {304.0, 272.0}};
}

VisualFeatureMeasurementPayload payload_from_batch(const VisualFeatureTrackBatch& batch,
                                                   const Eigen::Matrix3d& K) {
    VisualFeatureMeasurementPayload payload;
    payload.K = K;
    payload.z_pixels.reserve(batch.features.size());
    payload.p_world.reserve(batch.features.size());
    for (const auto& feature : batch.features) {
        payload.z_pixels.push_back(feature.pixel);
        payload.p_world.push_back(feature.world_point);
    }
    return payload;
}

} // namespace

TEST(VisualFeatureIngestIntegration, DeterministicTracksReachShadowOnlyAndPreserveActive) {
    VisualFeatureTrackManager manager;
    const auto K = camera_matrix();
    const auto prev = previous_pixels();
    const auto curr = current_pixels();
    const auto batch = manager.update(prev, curr, pose(0.0), pose(0.20), K);

    ASSERT_FALSE(batch.features.empty());
    ASSERT_EQ(batch.features.size(), batch.initialized_tracks);

    auto shadow = std::make_unique<RecordingFeatureShadow>();
    auto* shadow_ptr = shadow.get();
    EstimatorCoordinator coordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "active_baseline"),
        std::move(shadow));
    coordinator.configure_validation(enabled_shadow_config());
    coordinator.initialize();
    ASSERT_TRUE(coordinator.start());

    const auto active_before = coordinator.active_snapshot();
    const auto active_count_before = coordinator.diagnostics().active_processed_count;

    const auto payload = payload_from_batch(batch, K);
    ASSERT_EQ(coordinator.submit_shadow_measurement(make_visual_features_envelope(
                  payload, MeasurementStamp{0.02, 1u})),
              EstimatorOperationResult::Accepted);
    coordinator.flush_shadow();

    const auto active_after = coordinator.active_snapshot();
    const auto diagnostics = coordinator.diagnostics();
    EXPECT_EQ(diagnostics.active_processed_count, active_count_before);
    EXPECT_EQ(diagnostics.shadow_only_submission_count, 1u);
    EXPECT_EQ(diagnostics.shadow_only_rejected_count, 0u);
    EXPECT_EQ(shadow_ptr->process_count(), 1u);
    EXPECT_EQ(shadow_ptr->visual_feature_count(), batch.features.size());
    EXPECT_NEAR((active_after.position_m - active_before.position_m).norm(), 0.0, 1.0e-15);
    EXPECT_NEAR((active_after.velocity_mps - active_before.velocity_mps).norm(), 0.0, 1.0e-15);
    EXPECT_NEAR(std::abs(active_after.orientation.dot(active_before.orientation)), 1.0, 1.0e-15);
}

TEST(VisualFeatureIngestIntegration, RepeatedSyntheticSequenceIsDeterministic) {
    VisualFeatureTrackManager first;
    VisualFeatureTrackManager second;
    const auto K = camera_matrix();
    const auto prev = previous_pixels();
    const auto curr = current_pixels();

    const auto first_batch = first.update(prev, curr, pose(0.0), pose(0.20), K);
    const auto second_batch = second.update(prev, curr, pose(0.0), pose(0.20), K);

    ASSERT_EQ(first_batch.features.size(), second_batch.features.size());
    ASSERT_EQ(first_batch.initialized_tracks, second_batch.initialized_tracks);
    for (std::size_t i = 0; i < first_batch.features.size(); ++i) {
        EXPECT_EQ(first_batch.features[i].track_id, second_batch.features[i].track_id);
        EXPECT_NEAR((first_batch.features[i].pixel - second_batch.features[i].pixel).norm(), 0.0,
                    1.0e-15);
        EXPECT_NEAR((first_batch.features[i].world_point - second_batch.features[i].world_point).norm(),
                    0.0, 1.0e-12);
    }
}

TEST(VisualFeatureIngestIntegration, InvalidGeometryProducesNoShadowPayload) {
    VisualFeatureTrackManager manager;
    const auto K = camera_matrix();
    const auto batch = manager.update(previous_pixels(), current_pixels(), pose(0.0), pose(0.0), K);
    EXPECT_TRUE(batch.features.empty());
    EXPECT_GT(batch.rejected_geometry, 0u);
}
