#include "vio/EstimatorCoordinator.hpp"
#include "vio/Phase17ESKFEstimator.hpp"
#include "vio/VisualFeatureTrackManager.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

using namespace drone::vio;

namespace {

Eigen::Matrix3d intrinsics() {
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    K(0, 0) = 320.0;
    K(1, 1) = 320.0;
    K(0, 2) = 320.0;
    K(1, 2) = 240.0;
    K(2, 2) = 1.0;
    return K;
}

PoseEstimate pose_at(double x) {
    PoseEstimate pose;
    pose.position = Eigen::Vector3d{x, 0.0, 0.0};
    pose.orientation = Eigen::Quaterniond::Identity();
    return pose;
}

struct ReplayResult {
    std::vector<uint64_t> track_ids;
    uint64_t shadow_submissions{0};
    uint64_t shadow_processed{0};
    bool queue_drained{false};
    bool active_equivalent{false};
};

bool same_snapshot(const EstimatorStateSnapshot& lhs, const EstimatorStateSnapshot& rhs) {
    return (lhs.position_m - rhs.position_m).norm() <= 1.0e-12 &&
           (lhs.velocity_mps - rhs.velocity_mps).norm() <= 1.0e-12 &&
           std::abs(std::abs(lhs.orientation.dot(rhs.orientation)) - 1.0) <= 1.0e-12 &&
           std::abs(lhs.covariance.trace - rhs.covariance.trace) <= 1.0e-12;
}

ReplayResult run_once() {
    VisualFeatureTrackConfig track_cfg;
    track_cfg.minimum_baseline_m = 0.05;
    track_cfg.minimum_parallax_deg = 0.1;
    track_cfg.maximum_ray_gap_m = 0.5;
    VisualFeatureTrackManager manager(track_cfg);

    ShadowCoordinatorConfig shadow_cfg;
    shadow_cfg.enabled = true;
    shadow_cfg.start_worker_on_initialize = true;
    shadow_cfg.queue_capacity = 32;

    EstimatorCoordinator coordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active", "baseline"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "hardened"),
        shadow_cfg);
    coordinator.initialize();
    if (!coordinator.start()) {
        return {};
    }

    const auto active_before = coordinator.active_snapshot();
    const auto K = intrinsics();
    const std::vector<Eigen::Vector2d> p0{{320.0, 240.0}, {340.0, 240.0}, {300.0, 240.0}};
    const std::vector<Eigen::Vector2d> p1{{312.0, 240.0}, {332.0, 240.0}, {292.0, 240.0}};
    const std::vector<Eigen::Vector2d> p2{{304.0, 240.0}, {324.0, 240.0}, {284.0, 240.0}};

    ReplayResult result;
    const auto submit_batch = [&](const VisualFeatureTrackBatch& batch, double timestamp,
                                  uint64_t sequence) {
        VisualFeatureMeasurementPayload payload;
        payload.K = K;
        for (const auto& feature : batch.features) {
            result.track_ids.push_back(feature.track_id);
            payload.z_pixels.push_back(feature.pixel);
            payload.p_world.push_back(feature.world_point);
        }
        if (payload.z_pixels.empty()) {
            return false;
        }
        return coordinator.submit_shadow_measurement(make_visual_features_envelope(
                   payload, MeasurementStamp{timestamp, sequence})) ==
               EstimatorOperationResult::Accepted;
    };

    const auto first = manager.update(p0, p1, pose_at(0.0), pose_at(0.10), K);
    const auto second = manager.update(p1, p2, pose_at(0.10), pose_at(0.20), K);
    if (!submit_batch(first, 0.10, 1u) || !submit_batch(second, 0.20, 2u)) {
        coordinator.stop();
        return {};
    }

    coordinator.flush_shadow();
    const auto active_after = coordinator.active_snapshot();
    const auto diagnostics = coordinator.diagnostics();
    result.shadow_submissions = diagnostics.shadow_only_submission_count;
    result.shadow_processed = diagnostics.shadow_processed_count;
    result.queue_drained = coordinator.queue_depth() == 0u;
    result.active_equivalent = same_snapshot(active_before, active_after);
    coordinator.stop();
    return result;
}

} // namespace

int main() {
    const auto first = run_once();
    const auto second = run_once();

    const bool deterministic = !first.track_ids.empty() && first.track_ids == second.track_ids &&
                               first.shadow_submissions == second.shadow_submissions &&
                               first.shadow_processed == second.shadow_processed;
    const bool authority_preserved = first.active_equivalent && second.active_equivalent;
    const bool shadow_path_ok = first.shadow_submissions == 2u && second.shadow_submissions == 2u &&
                                first.shadow_processed >= 2u && second.shadow_processed >= 2u &&
                                first.queue_drained && second.queue_drained;

    std::cout << "deterministic=" << (deterministic ? "true" : "false") << '\n';
    std::cout << "active_equivalent=" << (authority_preserved ? "true" : "false") << '\n';
    std::cout << "shadow_path_ok=" << (shadow_path_ok ? "true" : "false") << '\n';
    std::cout << "track_samples=" << first.track_ids.size() << '\n';
    std::cout << "shadow_submissions=" << first.shadow_submissions << '\n';
    std::cout << "shadow_processed=" << first.shadow_processed << '\n';

    return deterministic && authority_preserved && shadow_path_ok ? 0 : 1;
}
