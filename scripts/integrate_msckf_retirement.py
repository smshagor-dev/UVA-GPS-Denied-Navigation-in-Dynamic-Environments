from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


# Diagnostics surfaces.
p = Path("include/vio/EKFEstimator.hpp")
s = p.read_text(encoding="utf-8-sig")
old = """    uint64_t msckf_deterministic_evictions{0};
    uint64_t triangulation_attempts{0};"""
new = """    uint64_t msckf_deterministic_evictions{0};
    uint64_t marginalization_attempts{0};
    uint64_t marginalizations_completed{0};
    uint64_t marginalization_failures{0};
    uint64_t marginalization_retiring_state_id{0};
    uint64_t marginalization_affected_tracks{0};
    uint64_t marginalization_constraint_candidates{0};
    uint64_t marginalization_covariance_dim_before{0};
    uint64_t marginalization_covariance_dim_after{0};
    uint64_t marginalization_stale_references{0};
    double marginalization_covariance_symmetry_error{0.0};
    double marginalization_covariance_min_eigenvalue{0.0};
    uint64_t triangulation_attempts{0};"""
if old in s:
    s = replace_once(s, old, new, "EKFDiagnostics fields")
p.write_text(s, encoding="utf-8")

p = Path("include/vio/StateEstimator.hpp")
s = p.read_text()
old = """    uint64_t msckf_deterministic_evictions{0};
    uint64_t triangulation_attempts{0};"""
new = """    uint64_t msckf_deterministic_evictions{0};
    uint64_t marginalization_attempts{0};
    uint64_t marginalizations_completed{0};
    uint64_t marginalization_failures{0};
    uint64_t marginalization_retiring_state_id{0};
    uint64_t marginalization_affected_tracks{0};
    uint64_t marginalization_constraint_candidates{0};
    uint64_t marginalization_covariance_dim_before{0};
    uint64_t marginalization_covariance_dim_after{0};
    uint64_t marginalization_stale_references{0};
    double marginalization_covariance_symmetry_error{0.0};
    double marginalization_covariance_min_eigenvalue{0.0};
    uint64_t triangulation_attempts{0};"""
if old in s:
    s = replace_once(s, old, new, "EstimatorStateSnapshot fields")
p.write_text(s)

# Production estimator integration.
p = Path("src/vio/Phase17ESKFEstimator.cpp")
s = p.read_text()
old = '#include "vio/Phase17ESKFEstimator.hpp"\n\n#include "vio/MeasurementEnvelope.hpp"'
if '#include "vio/MsckfMarginalization.hpp"' not in s:
    new = '#include "vio/Phase17ESKFEstimator.hpp"\n\n#include "vio/MeasurementEnvelope.hpp"\n#include "vio/MsckfMarginalization.hpp"\n#include "vio/MsckfRetirementTransaction.hpp"'
    s = replace_once(s, old, new, "marginalization includes")

old = """    diagnostics_.msckf_oldest_state_age_s = 0.0;
    diagnostics_.msckf_deterministic_evictions = 0;
}"""
if "diagnostics_.marginalization_attempts = 0;" not in s:
    new = """    diagnostics_.msckf_oldest_state_age_s = 0.0;
    diagnostics_.msckf_deterministic_evictions = 0;
    diagnostics_.marginalization_attempts = 0;
    diagnostics_.marginalizations_completed = 0;
    diagnostics_.marginalization_failures = 0;
    diagnostics_.marginalization_retiring_state_id = 0;
    diagnostics_.marginalization_affected_tracks = 0;
    diagnostics_.marginalization_constraint_candidates = 0;
    diagnostics_.marginalization_covariance_dim_before = 0;
    diagnostics_.marginalization_covariance_dim_after = 0;
    diagnostics_.marginalization_stale_references = 0;
    diagnostics_.marginalization_covariance_symmetry_error = 0.0;
    diagnostics_.marginalization_covariance_min_eigenvalue = 0.0;
}"""
    s = replace_once(s, old, new, "clear marginalization diagnostics")

old = """void Phase17ESKFEstimator::evict_msckf_state_locked() {
    if (msckf_state_order_.empty()) {
        refresh_msckf_oldest_state_age_locked();
        return;
    }
    const uint64_t oldest_id = msckf_state_order_.front();
    msckf_state_order_.pop_front();
    if (const auto it = msckf_camera_states_.find(oldest_id); it != msckf_camera_states_.end()) {
        remove_clone_covariance_block_locked(oldest_id);
        msckf_timestamp_to_state_id_.erase(it->second.timestamp_key);
        prune_feature_tracks_for_state_locked(oldest_id);
        msckf_camera_states_.erase(it);
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.msckf_states_removed;
            ++diagnostics_.msckf_deterministic_evictions;
        }
    }
    refresh_msckf_oldest_state_age_locked();
}"""
if old in s:
    new = """void Phase17ESKFEstimator::evict_msckf_state_locked() {
    if (msckf_state_order_.empty()) {
        refresh_msckf_oldest_state_age_locked();
        return;
    }

    const uint64_t oldest_id = msckf_state_order_.front();
    const auto state_it = msckf_camera_states_.find(oldest_id);
    if (state_it == msckf_camera_states_.end()) {
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.marginalization_failures;
        }
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        refresh_msckf_oldest_state_age_locked();
        return;
    }

    std::vector<MarginalizationTrackSummary> summaries;
    summaries.reserve(feature_tracks_.size());
    for (const auto& [key, track] : feature_tracks_) {
        (void)key;
        MarginalizationTrackSummary summary;
        summary.track_id = track.track_id;
        summary.landmark_initialized = track.landmark_initialized;
        summary.observation_state_ids.reserve(track.observations.size());
        for (const auto& observation : track.observations) {
            summary.observation_state_ids.push_back(observation.state_id);
        }
        summaries.push_back(std::move(summary));
    }
    const auto plan = MsckfMarginalization::build_plan(
        oldest_id, summaries, msckf_cfg_.update.minimum_track_length);
    const std::vector<uint64_t> ordered_clone_ids(msckf_state_order_.begin(),
                                                   msckf_state_order_.end());

    MsckfRetirementRequest request;
    request.retiring_state_id = oldest_id;
    request.base_error_dim = static_cast<std::size_t>(AugmentedStateLayout::kBaseErrorDim);
    request.clone_error_dim = static_cast<std::size_t>(AugmentedStateLayout::kCloneErrorDim);
    request.symmetry_tolerance = validation_cfg_.covariance_symmetry_tolerance;
    request.negativity_tolerance = validation_cfg_.variance_negativity_tolerance;

    if (msckf_diagnostics_enabled_locked()) {
        ++diagnostics_.marginalization_attempts;
        diagnostics_.marginalization_retiring_state_id = oldest_id;
        diagnostics_.marginalization_affected_tracks = plan.affected_track_ids.size();
        diagnostics_.marginalization_constraint_candidates =
            plan.constraint_candidate_track_ids.size();
        diagnostics_.marginalization_covariance_dim_before =
            static_cast<uint64_t>(augmented_covariance_.rows());
    }

    const auto prepared = MsckfRetirementTransaction::prepare(
        request, ordered_clone_ids, augmented_covariance_);
    if (!prepared.has_value() || !prepared->committed) {
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.marginalization_failures;
        }
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        refresh_msckf_oldest_state_age_locked();
        return;
    }

    // Build the complete metadata candidate before publishing any retirement mutation.
    auto candidate_order = msckf_state_order_;
    auto candidate_states = msckf_camera_states_;
    auto candidate_timestamp_map = msckf_timestamp_to_state_id_;
    auto candidate_tracks = feature_tracks_;
    candidate_order.pop_front();
    candidate_timestamp_map.erase(state_it->second.timestamp_key);
    candidate_states.erase(oldest_id);
    for (auto track_it = candidate_tracks.begin(); track_it != candidate_tracks.end();) {
        auto& observations = track_it->second.observations;
        observations.erase(std::remove_if(observations.begin(), observations.end(),
                                          [oldest_id](const FeatureObservation& observation) {
                                              return observation.state_id == oldest_id;
                                          }),
                           observations.end());
        if (observations.empty()) {
            track_it = candidate_tracks.erase(track_it);
        } else {
            ++track_it;
        }
    }

    uint64_t stale_references = 0;
    for (const auto& [key, track] : candidate_tracks) {
        (void)key;
        stale_references += static_cast<uint64_t>(std::count_if(
            track.observations.begin(), track.observations.end(),
            [oldest_id](const FeatureObservation& observation) {
                return observation.state_id == oldest_id;
            }));
    }
    const auto health = MsckfMarginalization::covariance_health(
        prepared->retained_covariance, validation_cfg_.covariance_symmetry_tolerance,
        validation_cfg_.variance_negativity_tolerance);
    const std::size_t expected_dim = static_cast<std::size_t>(kErrorDim) +
        candidate_order.size() * static_cast<std::size_t>(AugmentedStateLayout::kCloneErrorDim);
    const bool candidate_valid =
        health.finite && health.symmetric && health.psd_within_tolerance &&
        prepared->retained_covariance.rows() == static_cast<Eigen::Index>(expected_dim) &&
        prepared->retained_covariance.cols() == static_cast<Eigen::Index>(expected_dim) &&
        stale_references == 0u;
    if (!candidate_valid) {
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.marginalization_failures;
        }
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        refresh_msckf_oldest_state_age_locked();
        return;
    }

    const uint64_t removed_track_count =
        static_cast<uint64_t>(feature_tracks_.size() - candidate_tracks.size());
    msckf_state_order_ = std::move(candidate_order);
    msckf_camera_states_ = std::move(candidate_states);
    msckf_timestamp_to_state_id_ = std::move(candidate_timestamp_map);
    feature_tracks_ = std::move(candidate_tracks);
    augmented_covariance_ = prepared->retained_covariance;
    P_ = augmented_covariance_.block(0, 0, kErrorDim, kErrorDim);
    P_ = 0.5 * (P_ + P_.transpose());

    if (msckf_diagnostics_enabled_locked()) {
        ++diagnostics_.marginalizations_completed;
        ++diagnostics_.msckf_states_removed;
        ++diagnostics_.msckf_deterministic_evictions;
        diagnostics_.feature_tracks_removed += removed_track_count;
        diagnostics_.marginalization_covariance_dim_after =
            static_cast<uint64_t>(augmented_covariance_.rows());
        diagnostics_.marginalization_stale_references = stale_references;
        diagnostics_.marginalization_covariance_symmetry_error = health.symmetry_error;
        diagnostics_.marginalization_covariance_min_eigenvalue = health.minimum_eigenvalue;
    }
    refresh_triangulation_diagnostics_locked();
    refresh_msckf_oldest_state_age_locked();
}"""
    s = replace_once(s, old, new, "production eviction boundary")

old = """    while (msckf_state_order_.size() > msckf_cfg_.max_camera_states) {
        evict_msckf_state_locked();
    }
    refresh_msckf_oldest_state_age_locked();
    return state.state_id;"""
if old in s:
    s = replace_once(
        s,
        old,
        """    // Retirement is deferred until the caller has consumed the current frame's
    // feature information. Non-feature callers retire in maybe_capture_msckf_camera_state_locked().
    refresh_msckf_oldest_state_age_locked();
    return state.state_id;""",
        "defer capture retirement",
    )

old = """void Phase17ESKFEstimator::maybe_capture_msckf_camera_state_locked(double capture_timestamp_s) {
    (void)capture_or_get_msckf_camera_state_locked(capture_timestamp_s);
}"""
if old in s:
    new = """void Phase17ESKFEstimator::maybe_capture_msckf_camera_state_locked(double capture_timestamp_s) {
    (void)capture_or_get_msckf_camera_state_locked(capture_timestamp_s);
    while (msckf_state_order_.size() > msckf_cfg_.max_camera_states) {
        const auto before = msckf_state_order_.size();
        evict_msckf_state_locked();
        if (msckf_state_order_.size() >= before) {
            break;
        }
    }
}"""
    s = replace_once(s, old, new, "non-feature retirement")

old = """            try_initialize_triangulated_landmarks_locked(K);
            try_apply_msckf_feature_updates_locked(K, state_id);
        }
    }"""
if old in s:
    new = """            try_initialize_triangulated_landmarks_locked(K);
            try_apply_msckf_feature_updates_locked(K, state_id);
            while (msckf_state_order_.size() > msckf_cfg_.max_camera_states) {
                const auto before = msckf_state_order_.size();
                evict_msckf_state_locked();
                if (msckf_state_order_.size() >= before) {
                    break;
                }
            }
        }
    }"""
    s = replace_once(s, old, new, "vision post-constraint retirement")

old = """    out.msckf_deterministic_evictions = diag.msckf_deterministic_evictions;
    out.triangulation_attempts = diag.triangulation_attempts;"""
if old in s:
    new = """    out.msckf_deterministic_evictions = diag.msckf_deterministic_evictions;
    out.marginalization_attempts = diag.marginalization_attempts;
    out.marginalizations_completed = diag.marginalizations_completed;
    out.marginalization_failures = diag.marginalization_failures;
    out.marginalization_retiring_state_id = diag.marginalization_retiring_state_id;
    out.marginalization_affected_tracks = diag.marginalization_affected_tracks;
    out.marginalization_constraint_candidates = diag.marginalization_constraint_candidates;
    out.marginalization_covariance_dim_before = diag.marginalization_covariance_dim_before;
    out.marginalization_covariance_dim_after = diag.marginalization_covariance_dim_after;
    out.marginalization_stale_references = diag.marginalization_stale_references;
    out.marginalization_covariance_symmetry_error =
        diag.marginalization_covariance_symmetry_error;
    out.marginalization_covariance_min_eigenvalue =
        diag.marginalization_covariance_min_eigenvalue;
    out.triangulation_attempts = diag.triangulation_attempts;"""
    s = replace_once(s, old, new, "snapshot diagnostics")

p.write_text(s)

# Focused production-path integration tests.
p = Path("tests/CMakeLists.txt")
s = p.read_text()
anchor = 'drone_register_gtest(test_phase22_msckf_update "unit;navigation;replay;phase22")\n'
if "test_msckf_marginalization_integration" not in s:
    addition = anchor + """
add_executable(test_msckf_marginalization_integration test_msckf_marginalization_integration.cpp)
target_link_libraries(test_msckf_marginalization_integration PRIVATE
    drone_test_support
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_register_gtest(test_msckf_marginalization_integration "unit;navigation;replay;marginalization")
"""
    s = replace_once(s, anchor, addition, "integration test target")
p.write_text(s)

Path("tests/test_msckf_marginalization_integration.cpp").write_text(
    r'''#include "vio/Phase17ESKFEstimator.hpp"

#include <gtest/gtest.h>

namespace drone::vio {
namespace {

MsckfConfig make_config(bool diagnostics = true) {
    MsckfConfig cfg;
    cfg.enabled = true;
    cfg.max_camera_states = 2;
    cfg.diagnostics_enabled = diagnostics;
    cfg.triangulation.enabled = false;
    cfg.update.enabled = false;
    return cfg;
}

void drive_three_visual_pose_clones(Phase17ESKFEstimator& estimator) {
    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.0),
              EstimatorOperationResult::Accepted);
    for (int i = 1; i <= 3; ++i) {
        const double timestamp = 0.01 * static_cast<double>(i);
        ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                    Eigen::Vector3d::Zero(), timestamp),
                  EstimatorOperationResult::Accepted);
        const auto pose = estimator.state();
        estimator.update_visual_pose(pose.position, pose.velocity, 0.35, 0.45);
    }
}

TEST(MsckfMarginalizationIntegration, TransactionalOldestRetirementKeepsWindowBounded) {
    Phase17ESKFEstimator estimator;
    estimator.reset();
    estimator.configure_msckf(make_config());
    drive_three_visual_pose_clones(estimator);

    EXPECT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{2u, 3u}));
    const auto covariance = estimator.augmented_covariance_for_test();
    EXPECT_EQ(covariance.rows(), 27);
    EXPECT_EQ(covariance.cols(), 27);
    EXPECT_TRUE(covariance.array().isFinite().all());

    const auto diagnostics = estimator.diagnostics();
    EXPECT_EQ(diagnostics.marginalization_attempts, 1u);
    EXPECT_EQ(diagnostics.marginalizations_completed, 1u);
    EXPECT_EQ(diagnostics.marginalization_failures, 0u);
    EXPECT_EQ(diagnostics.marginalization_retiring_state_id, 1u);
    EXPECT_EQ(diagnostics.marginalization_covariance_dim_before, 33u);
    EXPECT_EQ(diagnostics.marginalization_covariance_dim_after, 27u);
    EXPECT_EQ(diagnostics.marginalization_stale_references, 0u);
    EXPECT_LE(diagnostics.marginalization_covariance_symmetry_error, 1.0e-9);
}

TEST(MsckfMarginalizationIntegration, DiagnosticsDisabledDoesNotDisableRetirement) {
    Phase17ESKFEstimator estimator;
    estimator.reset();
    estimator.configure_msckf(make_config(false));
    drive_three_visual_pose_clones(estimator);

    EXPECT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{2u, 3u}));
    EXPECT_EQ(estimator.augmented_covariance_for_test().rows(), 27);
    const auto diagnostics = estimator.diagnostics();
    EXPECT_EQ(diagnostics.marginalization_attempts, 0u);
    EXPECT_EQ(diagnostics.marginalizations_completed, 0u);
}

} // namespace
} // namespace drone::vio
'''
)

print("MSCKF retirement integration patch prepared")
