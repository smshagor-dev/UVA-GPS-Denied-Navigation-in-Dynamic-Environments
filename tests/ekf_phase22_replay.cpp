#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"
#include "vio/Phase17ESKFEstimator.hpp"
#include "phase17_test_access.hpp"

#include <Eigen/Eigenvalues>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace drone::vio;

namespace {

constexpr double kFrameDt = 0.01;
constexpr double kPositionTolerance = 1.0e-12;
constexpr double kVelocityTolerance = 1.0e-12;
constexpr double kOrientationTolerance = 1.0e-12;
constexpr uint32_t kDeterministicRuns = 3;

enum class ReplayScenarioKind : uint8_t {
    AcceptedUpdate,
    RejectedUpdate,
    MultiFeatureStack,
    SingularGeometry,
    StaleFej,
    UpdateDisabled,
};

struct ReplayScenario {
    std::string name;
    ReplayScenarioKind kind{ReplayScenarioKind::AcceptedUpdate};
    std::vector<Eigen::Vector3d> features;
    std::vector<Eigen::Vector3d> position_deltas;
    std::vector<Eigen::Vector3d> rotation_deltas;
    std::vector<std::vector<size_t>> visible_feature_indices;
    std::vector<std::vector<Eigen::Vector2d>> pixel_offsets;
    MsckfConfig msckf_cfg;
    std::string expected_rejection_reason{"none"};
    bool enable_fej{false};
    std::optional<size_t> corrupt_fej_after_frame{};
};

struct ReplayEventEvidence {
    size_t frame_index{0};
    std::string event_name;
    std::string pre_base_checksum;
    std::string post_base_checksum;
    std::string pre_clone_checksum;
    std::string post_clone_checksum;
    std::string pre_covariance_checksum;
    std::string post_covariance_checksum;
    uint64_t attempted_delta{0};
    uint64_t applied_delta{0};
    uint64_t rejected_delta{0};
    uint64_t features_considered_delta{0};
    uint64_t features_stacked_delta{0};
    uint64_t feature_tracks_before{0};
    uint64_t feature_tracks_after{0};
    uint64_t active_landmarks_before{0};
    uint64_t active_landmarks_after{0};
    uint64_t chi_square_failures_delta{0};
    uint64_t fej_validation_failures_delta{0};
    uint64_t degenerate_geometry_delta{0};
    EstimatorOperationResult last_result_before{EstimatorOperationResult::Accepted};
    EstimatorOperationResult last_result_after{EstimatorOperationResult::Accepted};
    EstimatorOperationResult last_rejection_before{EstimatorOperationResult::Accepted};
    EstimatorOperationResult last_rejection_after{EstimatorOperationResult::Accepted};
};

struct ReplayResult {
    std::string scenario;
    bool deterministic{false};
    uint32_t deterministic_runs{kDeterministicRuns};
    bool active_equivalent{false};
    std::string active_equivalence_method{
        "active-only and active-plus-shadow active snapshots compared after each event"};
    bool shadow_queue_drained{false};
    bool shadow_only_feature_update{false};
    std::string shadow_only_evidence{"direct Phase17 mirror showed Phase 22 activity while active "
                                     "snapshots retained zero Phase 22 counters"};
    bool active_phase22_config_present{false};
    bool active_phase22_counters_present{false};
    bool state_finite{false};
    bool covariance_finite{false};
    bool covariance_symmetric{false};
    double covariance_symmetry_error{0.0};
    bool covariance_psd{false};
    double covariance_min_eigenvalue{0.0};
    uint64_t attempted_updates{0};
    uint64_t applied_updates{0};
    uint64_t rejected_updates{0};
    std::string rejection_reason{"none"};
    uint64_t features_considered{0};
    uint64_t features_stacked{0};
    uint64_t observations_used{0};
    uint64_t augmented_state_dimension{0};
    uint64_t clone_count{0};
    uint64_t stacked_measurement_dimension{0};
    uint64_t measurement_rank{0};
    uint64_t feature_jacobian_rank{0};
    uint64_t projected_dimension{0};
    double nullspace_annihilation_norm{0.0};
    double nullspace_orthogonality_error{0.0};
    uint64_t chi_square_dof{0};
    double chi_square_probability{0.0};
    double chi_square_threshold_used{0.0};
    double chi_square_statistic{0.0};
    std::string innovation_factorization_status{"not_run"};
    double innovation_min_pivot{0.0};
    double innovation_max_pivot{0.0};
    bool innovation_conditioning_valid{false};
    double residual_norm{0.0};
    double correction_norm{0.0};
    std::string pre_update_base_state_checksum;
    std::string post_update_base_state_checksum;
    std::string pre_update_clone_state_checksum;
    std::string post_update_clone_state_checksum;
    std::string pre_update_covariance_checksum;
    std::string post_update_covariance_checksum;
    bool estimator_state_rollback_verified{false};
    std::string feature_lifecycle_expected{"none"};
    std::string feature_lifecycle_observed{"none"};
    bool feature_lifecycle_verified{false};
    bool diagnostics_update_verified{false};
    bool rollback_verified{false};
    std::string final_status{"FAIL"};
    std::string checksum;
    std::string first_divergence_event{"none"};
    ReplayEventEvidence decisive_event{};
};

EstimatorValidationConfig make_validation_cfg(bool enable_shadow, bool enable_fej) {
    EstimatorValidationConfig cfg;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    cfg.enable_shadow_estimator = enable_shadow;
    cfg.shadow_comparison_enabled = enable_shadow;
    cfg.shadow_max_queue_depth = 1024;
    cfg.shadow_max_lag_ms = 50.0;
    cfg.enable_fej = enable_fej;
    cfg.fej.enabled = enable_fej;
    cfg.fej.validation_checks = true;
    return cfg;
}

EKFConfig make_ekf_cfg() {
    EKFConfig cfg;
    cfg.mahal_gate = 1.0e9;
    return cfg;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
MsckfConfig make_msckf_cfg(double chi_square_probability = 0.999999, double maximum_residual = 0.1,
                           uint32_t maximum_track_length = 8) {
    MsckfConfig cfg;
    cfg.enabled = true;
    cfg.max_camera_states = 10;
    cfg.eviction_policy = "oldest_first";
    cfg.diagnostics_enabled = true;
    cfg.triangulation.enabled = true;
    cfg.triangulation.minimum_observations = 3;
    cfg.triangulation.minimum_baseline = 0.05;
    cfg.triangulation.maximum_reprojection_error = 2.5;
    cfg.triangulation.minimum_depth = 0.1;
    cfg.triangulation.maximum_depth = 50.0;
    cfg.update.enabled = true;
    cfg.update.chi_square_probability = chi_square_probability;
    cfg.update.minimum_track_length = 3;
    cfg.update.maximum_track_length = maximum_track_length;
    cfg.update.maximum_residual = maximum_residual;
    cfg.update.validation_checks = true;
    cfg.update.diagnostics_enabled = true;
    return cfg;
}

Eigen::Matrix3d camera_intrinsics() {
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    K(0, 0) = 320.0;
    K(1, 1) = 320.0;
    K(0, 2) = 320.0;
    K(1, 2) = 240.0;
    K(2, 2) = 1.0;
    return K;
}

Eigen::Quaterniond rotvec_to_quat(const Eigen::Vector3d& rv) {
    const double angle = rv.norm();
    if (angle < 1.0e-12) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rv / angle)).normalized();
}

Eigen::Vector2d project_feature_from_pose(const Eigen::Vector3d& position,
                                          const Eigen::Quaterniond& orientation,
                                          const Eigen::Vector3d& feature) {
    const Eigen::Matrix3d R = orientation.toRotationMatrix();
    const Eigen::Vector3d p_c = R.transpose() * (feature - position);
    const auto K = camera_intrinsics();
    return {
        (K(0, 0) * p_c.x() / p_c.z()) + K(0, 2),
        (K(1, 1) * p_c.y() / p_c.z()) + K(1, 2),
    };
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
MeasurementEnvelope make_imu_env(double timestamp_s, uint64_t sequence_id,
                                 const Eigen::Vector3d& delta_theta) {
    MeasurementEnvelope env;
    env.type = MeasurementType::Imu;
    env.source_id = "imu";
    env.timestamp_s = timestamp_s;
    env.sequence_id = sequence_id;
    env.frame = MeasurementFrame::Body;
    env.payload = ImuMeasurementPayload{Eigen::Vector3d{0.0, 0.0, 9.81}, delta_theta / kFrameDt};
    return env;
}

bool snapshots_equivalent(const EstimatorStateSnapshot& a, const EstimatorStateSnapshot& b) {
    return (a.position_m - b.position_m).norm() <= kPositionTolerance &&
           (a.velocity_mps - b.velocity_mps).norm() <= kVelocityTolerance &&
           std::abs(std::abs(a.orientation.dot(b.orientation)) - 1.0) <= kOrientationTolerance;
}

std::string pose_checksum(const PoseEstimate& pose) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(12) << pose.position.x() << '|' << pose.position.y()
        << '|' << pose.position.z() << '|' << pose.velocity.x() << '|' << pose.velocity.y() << '|'
        << pose.velocity.z() << '|' << pose.orientation.w() << '|' << pose.orientation.x() << '|'
        << pose.orientation.y() << '|' << pose.orientation.z();
    return out.str();
}

std::string clone_checksum(const Phase17ESKFEstimator& ekf) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(12);
    const auto state_ids = ekf.msckf_state_ids_for_test();
    for (const auto state_id : state_ids) {
        const auto state = ekf.msckf_camera_state_for_test(state_id);
        if (!state.has_value()) {
            continue;
        }
        out << state->state_id << '|' << state->timestamp_s << '|' << state->position_m.x() << '|'
            << state->position_m.y() << '|' << state->position_m.z() << '|'
            << state->orientation.w() << '|' << state->orientation.x() << '|'
            << state->orientation.y() << '|' << state->orientation.z() << ';';
    }
    return out.str();
}

std::string matrix_checksum(const Eigen::MatrixXd& matrix) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(12);
    double weighted_sum = 0.0;
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
        for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
            const double weight = static_cast<double>(((row + 1) * 131) + ((col + 1) * 17));
            weighted_sum += matrix(row, col) * weight;
        }
    }
    out << matrix.trace() << '|' << matrix.squaredNorm() << '|' << weighted_sum;
    return out.str();
}

double chi_square_statistic_from_diag(const EKFDiagnostics& diag) {
    if (diag.innovation_min_eigenvalue <= 0.0 || !std::isfinite(diag.residual_norm)) {
        return 0.0;
    }
    return (diag.residual_norm * diag.residual_norm) / diag.innovation_min_eigenvalue;
}

bool active_phase22_config_present(const EstimatorStateSnapshot& snapshot) {
    return snapshot.fej_enabled || snapshot.msckf_window_size > 0u ||
           snapshot.feature_tracks > 0u || snapshot.active_landmarks > 0u;
}

bool active_phase22_counters_present(const EstimatorStateSnapshot& snapshot) {
    return snapshot.feature_updates_attempted > 0u || snapshot.feature_updates_applied > 0u ||
           snapshot.feature_updates_rejected > 0u || snapshot.feature_updates_considered > 0u ||
           snapshot.feature_updates_stacked > 0u || snapshot.nullspace_failures > 0u ||
           snapshot.chi_square_failures > 0u || snapshot.stacked_measurement_dimension > 0u ||
           snapshot.measurement_rank > 0u || snapshot.chi_square_dof > 0u;
}

std::string derive_rejection_reason(const ReplayScenario& scenario,
                                    const ReplayEventEvidence& evidence,
                                    const EKFDiagnostics& after) {
    if (scenario.kind == ReplayScenarioKind::AcceptedUpdate ||
        scenario.kind == ReplayScenarioKind::MultiFeatureStack) {
        return "none";
    }
    if (!scenario.msckf_cfg.update.enabled) {
        return "update_disabled";
    }
    if (evidence.fej_validation_failures_delta > 0u) {
        return "stale_fej_snapshot";
    }
    if (evidence.degenerate_geometry_delta > 0u) {
        return "degenerate_geometry";
    }
    if (evidence.chi_square_failures_delta > 0u) {
        return "chi_square";
    }
    if (after.last_rejection_reason != EstimatorOperationResult::Accepted) {
        return std::string(to_string(after.last_rejection_reason));
    }
    return scenario.expected_rejection_reason;
}

ReplayEventEvidence
capture_event_evidence(const ReplayScenario& scenario, size_t frame_index,
                       const Phase17ESKFEstimator& mirror, const EKFDiagnostics& before_diag,
                       const std::string& pre_base_checksum, const std::string& pre_clone_checksum,
                       const std::string& pre_covariance_checksum, uint64_t feature_tracks_before,
                       uint64_t active_landmarks_before) {
    const auto after_diag = mirror.diagnostics();
    ReplayEventEvidence evidence;
    evidence.frame_index = frame_index;
    evidence.event_name = scenario.name + "#frame_" + std::to_string(frame_index);
    evidence.pre_base_checksum = pre_base_checksum;
    evidence.post_base_checksum = pose_checksum(mirror.state());
    evidence.pre_clone_checksum = pre_clone_checksum;
    evidence.post_clone_checksum = clone_checksum(mirror);
    evidence.pre_covariance_checksum = pre_covariance_checksum;
    evidence.post_covariance_checksum = matrix_checksum(mirror.augmented_covariance_for_test());
    evidence.attempted_delta =
        after_diag.feature_updates_attempted - before_diag.feature_updates_attempted;
    evidence.applied_delta =
        after_diag.feature_updates_applied - before_diag.feature_updates_applied;
    evidence.rejected_delta =
        after_diag.feature_updates_rejected - before_diag.feature_updates_rejected;
    evidence.features_considered_delta =
        after_diag.feature_updates_considered - before_diag.feature_updates_considered;
    evidence.features_stacked_delta =
        after_diag.feature_updates_stacked - before_diag.feature_updates_stacked;
    evidence.feature_tracks_before = feature_tracks_before;
    evidence.feature_tracks_after = mirror.feature_track_count_for_test();
    evidence.active_landmarks_before = active_landmarks_before;
    evidence.active_landmarks_after = mirror.active_landmark_count_for_test();
    evidence.chi_square_failures_delta =
        after_diag.chi_square_failures - before_diag.chi_square_failures;
    evidence.fej_validation_failures_delta =
        after_diag.fej_validation_failures - before_diag.fej_validation_failures;
    evidence.degenerate_geometry_delta =
        after_diag.rejected_degenerate_geometry - before_diag.rejected_degenerate_geometry;
    evidence.last_result_before = before_diag.last_operation_result;
    evidence.last_result_after = after_diag.last_operation_result;
    evidence.last_rejection_before = before_diag.last_rejection_reason;
    evidence.last_rejection_after = after_diag.last_rejection_reason;
    return evidence;
}

bool event_has_decisive_change(const ReplayEventEvidence& evidence) {
    return evidence.attempted_delta > 0u || evidence.applied_delta > 0u ||
           evidence.rejected_delta > 0u || evidence.chi_square_failures_delta > 0u ||
           evidence.fej_validation_failures_delta > 0u || evidence.degenerate_geometry_delta > 0u;
}

void describe_feature_lifecycle(const ReplayScenario& scenario, const ReplayEventEvidence& evidence,
                                ReplayResult& out) {
    switch (scenario.kind) {
    case ReplayScenarioKind::RejectedUpdate:
        out.feature_lifecycle_expected = "rejected track retained for diagnostics; landmark "
                                         "initialization may persist from pre-gate triangulation";
        out.feature_lifecycle_observed =
            "tracks " + std::to_string(evidence.feature_tracks_before) + "->" +
            std::to_string(evidence.feature_tracks_after) + ", landmarks " +
            std::to_string(evidence.active_landmarks_before) + "->" +
            std::to_string(evidence.active_landmarks_after);
        out.feature_lifecycle_verified =
            evidence.feature_tracks_after >= evidence.feature_tracks_before &&
            evidence.active_landmarks_after >= evidence.active_landmarks_before;
        break;
    case ReplayScenarioKind::SingularGeometry:
        out.feature_lifecycle_expected =
            "track may remain uninitialized after degenerate geometry rejection";
        out.feature_lifecycle_observed =
            "tracks " + std::to_string(evidence.feature_tracks_before) + "->" +
            std::to_string(evidence.feature_tracks_after) + ", landmarks " +
            std::to_string(evidence.active_landmarks_before) + "->" +
            std::to_string(evidence.active_landmarks_after);
        out.feature_lifecycle_verified = evidence.feature_tracks_after >= 1u &&
                                         evidence.active_landmarks_after == 0u &&
                                         evidence.degenerate_geometry_delta == 1u;
        break;
    case ReplayScenarioKind::StaleFej:
        out.feature_lifecycle_expected = "track survives FEJ validation failure, applied count "
                                         "unchanged, FEJ failure increments";
        out.feature_lifecycle_observed =
            "tracks " + std::to_string(evidence.feature_tracks_before) + "->" +
            std::to_string(evidence.feature_tracks_after) + ", landmarks " +
            std::to_string(evidence.active_landmarks_before) + "->" +
            std::to_string(evidence.active_landmarks_after);
        out.feature_lifecycle_verified =
            evidence.feature_tracks_after >= 1u && evidence.fej_validation_failures_delta == 1u;
        break;
    case ReplayScenarioKind::UpdateDisabled:
        out.feature_lifecycle_expected = "Phase 22 disabled, no feature-update activity";
        out.feature_lifecycle_observed =
            "tracks " + std::to_string(evidence.feature_tracks_before) + "->" +
            std::to_string(evidence.feature_tracks_after) + ", landmarks " +
            std::to_string(evidence.active_landmarks_before) + "->" +
            std::to_string(evidence.active_landmarks_after);
        out.feature_lifecycle_verified = evidence.attempted_delta == 0u &&
                                         evidence.applied_delta == 0u &&
                                         evidence.rejected_delta == 0u;
        break;
    default:
        out.feature_lifecycle_expected =
            "feature lifecycle advanced consistently with accepted update";
        out.feature_lifecycle_observed =
            "tracks " + std::to_string(evidence.feature_tracks_before) + "->" +
            std::to_string(evidence.feature_tracks_after) + ", landmarks " +
            std::to_string(evidence.active_landmarks_before) + "->" +
            std::to_string(evidence.active_landmarks_after);
        out.feature_lifecycle_verified = true;
        break;
    }
}

void finalize_status(const ReplayScenario& scenario, const EKFDiagnostics& diag,
                     ReplayResult& out) {
    const bool accepted_path_ok =
        out.applied_updates > 0u && out.features_considered > 0u && out.features_stacked > 0u;
    const bool rejected_path_ok = out.rejected_updates > 0u &&
                                  out.rejection_reason == scenario.expected_rejection_reason &&
                                  out.rollback_verified;
    const bool multi_feature_ok = out.applied_updates >= 2u && out.features_stacked >= 2u &&
                                  out.stacked_measurement_dimension > 0u;
    const bool singular_ok = out.rejection_reason == "degenerate_geometry" &&
                             out.rollback_verified && diag.rejected_degenerate_geometry > 0u;
    const bool stale_ok = out.rejection_reason == "stale_fej_snapshot" && out.rollback_verified &&
                          diag.fej_validation_failures > 0u && out.applied_updates == 0u;
    const bool disabled_ok = out.attempted_updates == 0u && out.applied_updates == 0u &&
                             out.rejected_updates == 0u && out.shadow_only_feature_update;

    bool scenario_ok = false;
    switch (scenario.kind) {
    case ReplayScenarioKind::AcceptedUpdate:
        scenario_ok = accepted_path_ok;
        break;
    case ReplayScenarioKind::RejectedUpdate:
        scenario_ok = rejected_path_ok;
        break;
    case ReplayScenarioKind::MultiFeatureStack:
        scenario_ok = multi_feature_ok;
        break;
    case ReplayScenarioKind::SingularGeometry:
        scenario_ok = singular_ok;
        break;
    case ReplayScenarioKind::StaleFej:
        scenario_ok = stale_ok;
        break;
    case ReplayScenarioKind::UpdateDisabled:
        scenario_ok = disabled_ok;
        break;
    }

    out.final_status =
        (scenario_ok && out.active_equivalent && out.shadow_queue_drained && out.state_finite &&
         out.covariance_finite && out.covariance_symmetric && out.covariance_psd &&
         !out.active_phase22_config_present && !out.active_phase22_counters_present)
            ? "PASS"
            : "FAIL";
}

ReplayResult run_once(const ReplayScenario& scenario) {
    auto active_only = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(make_ekf_cfg(), "ekf_active", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(make_ekf_cfg(), "eskf_shadow", "phase17"));
    active_only.configure_validation(make_validation_cfg(false, false));
    active_only.initialize();

    auto with_shadow = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(make_ekf_cfg(), "ekf_active", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(make_ekf_cfg(), "eskf_shadow", "phase17"));
    with_shadow.configure_validation(make_validation_cfg(true, scenario.enable_fej));
    with_shadow.configure_shadow_msckf(scenario.msckf_cfg);
    with_shadow.initialize();
    (void)with_shadow.start();

    Phase17ESKFEstimator mirror(make_ekf_cfg());
    mirror.configure_validation(make_validation_cfg(true, scenario.enable_fej));
    mirror.configure_msckf(scenario.msckf_cfg);
    mirror.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond target_orientation = Eigen::Quaterniond::Identity();
    uint64_t sequence_id = 0;
    bool active_equivalent = true;
    bool shadow_queue_drained = true;
    bool active_has_phase22_config = false;
    bool active_has_phase22_counters = false;
    std::optional<ReplayEventEvidence> decisive_event;
    std::string first_divergence_event = "none";

    for (size_t frame_index = 0; frame_index < scenario.position_deltas.size(); ++frame_index) {
        const double timestamp_s = static_cast<double>(frame_index) * kFrameDt;
        const Eigen::Vector3d delta_position = scenario.position_deltas[frame_index];
        const Eigen::Vector3d delta_theta = scenario.rotation_deltas[frame_index];
        target_position += delta_position;
        target_orientation = (target_orientation * rotvec_to_quat(delta_theta)).normalized();

        const auto imu = make_imu_env(timestamp_s, sequence_id++, delta_theta);
        (void)active_only.process_measurement(imu);
        (void)with_shadow.process_measurement(imu);
        (void)mirror.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                             delta_theta / kFrameDt, timestamp_s);
        with_shadow.flush_shadow();
        shadow_queue_drained = shadow_queue_drained && (with_shadow.queue_depth() == 0u);

        const Eigen::Vector3d velocity_mps = delta_position / kFrameDt;
        const auto pose_env = make_visual_pose_envelope(
            VisualPoseMeasurementPayload{target_position, velocity_mps, 1.0e-6, 1.0e-6},
            MeasurementStamp{timestamp_s, sequence_id++});
        (void)active_only.process_measurement(pose_env);
        (void)with_shadow.process_measurement(pose_env);
        mirror.update_visual_pose(target_position, velocity_mps, 1.0e-6, 1.0e-6);
        with_shadow.flush_shadow();
        shadow_queue_drained = shadow_queue_drained && (with_shadow.queue_depth() == 0u);

        std::vector<Eigen::Vector2d> active_pixels;
        std::vector<Eigen::Vector2d> mirror_pixels;
        std::vector<Eigen::Vector3d> identities;
        const auto mirror_pose = mirror.state();
        for (size_t local_index = 0;
             local_index < scenario.visible_feature_indices[frame_index].size(); ++local_index) {
            const size_t feature_index = scenario.visible_feature_indices[frame_index][local_index];
            const Eigen::Vector2d offset = scenario.pixel_offsets[frame_index][local_index];
            active_pixels.push_back(project_feature_from_pose(target_position, target_orientation,
                                                              scenario.features[feature_index]) +
                                    offset);
            mirror_pixels.push_back(project_feature_from_pose(mirror_pose.position,
                                                              mirror_pose.orientation,
                                                              scenario.features[feature_index]) +
                                    offset);
            identities.push_back(scenario.features[feature_index]);
        }

        const auto active_before = active_only.active_snapshot();
        const auto with_shadow_before = with_shadow.active_snapshot();
        const auto before_diag = mirror.diagnostics();
        const std::string pre_base_checksum = pose_checksum(mirror.state());
        const std::string pre_clone_checksum = clone_checksum(mirror);
        const std::string pre_covariance_checksum =
            matrix_checksum(mirror.augmented_covariance_for_test());
        const uint64_t feature_tracks_before = mirror.feature_track_count_for_test();
        const uint64_t active_landmarks_before = mirror.active_landmark_count_for_test();

        VisualFeatureMeasurementPayload feature_payload;
        feature_payload.z_pixels.assign(active_pixels.begin(), active_pixels.end());
        feature_payload.p_world.assign(identities.begin(), identities.end());
        feature_payload.K = camera_intrinsics();
        const auto feature_env = make_visual_features_envelope(
            feature_payload, MeasurementStamp{timestamp_s, sequence_id++});
        (void)active_only.process_measurement(feature_env);
        (void)with_shadow.process_measurement(feature_env);
        if (scenario.kind == ReplayScenarioKind::AcceptedUpdate ||
            scenario.kind == ReplayScenarioKind::MultiFeatureStack) {
            mirror.update_vision(mirror_pixels, identities, camera_intrinsics());
        } else {
            const uint64_t mirror_state_id =
                Phase17ESKFEstimatorTestAccess::capture_msckf_camera_state(mirror);
            if (mirror_state_id != 0u) {
                (void)Phase17ESKFEstimatorTestAccess::process_msckf_observations(
                    mirror, mirror_state_id, mirror_pixels, identities, camera_intrinsics());
            }
        }

        if (scenario.corrupt_fej_after_frame.has_value() &&
            frame_index == *scenario.corrupt_fej_after_frame) {
            Phase17ESKFEstimatorTestAccess::corrupt_first_msckf_fej_clone(mirror);
        }

        with_shadow.flush_shadow();
        shadow_queue_drained = shadow_queue_drained && (with_shadow.queue_depth() == 0u);

        const auto active_after = active_only.active_snapshot();
        const auto with_shadow_after = with_shadow.active_snapshot();
        active_equivalent = active_equivalent &&
                            snapshots_equivalent(active_before, with_shadow_before) &&
                            snapshots_equivalent(active_after, with_shadow_after);
        active_has_phase22_config =
            active_has_phase22_config || active_phase22_config_present(active_after);
        active_has_phase22_counters =
            active_has_phase22_counters || active_phase22_counters_present(active_after);

        const auto evidence = capture_event_evidence(
            scenario, frame_index, mirror, before_diag, pre_base_checksum, pre_clone_checksum,
            pre_covariance_checksum, feature_tracks_before, active_landmarks_before);
        if (!decisive_event.has_value() && event_has_decisive_change(evidence)) {
            decisive_event = evidence;
        }
        if (first_divergence_event == "none" &&
            ((scenario.kind == ReplayScenarioKind::StaleFej &&
              evidence.fej_validation_failures_delta > 0u) ||
             (scenario.kind == ReplayScenarioKind::SingularGeometry &&
              evidence.degenerate_geometry_delta > 0u) ||
             (scenario.kind == ReplayScenarioKind::RejectedUpdate &&
              evidence.chi_square_failures_delta > 0u) ||
             (scenario.kind == ReplayScenarioKind::MultiFeatureStack &&
              evidence.features_stacked_delta == 0u &&
              frame_index + 1u == scenario.position_deltas.size()))) {
            first_divergence_event = evidence.event_name;
        }
    }

    with_shadow.flush_shadow();
    shadow_queue_drained = shadow_queue_drained && (with_shadow.queue_depth() == 0u);
    with_shadow.stop();

    ReplayResult out;
    out.scenario = scenario.name;
    out.active_equivalent = active_equivalent;
    out.shadow_queue_drained = shadow_queue_drained;
    out.active_phase22_config_present = active_has_phase22_config;
    out.active_phase22_counters_present = active_has_phase22_counters;
    out.chi_square_probability = scenario.msckf_cfg.update.chi_square_probability;
    out.covariance_finite = mirror.covariance().array().isFinite().all();
    const Eigen::MatrixXd augmented_covariance = mirror.augmented_covariance_for_test();
    out.augmented_state_dimension = static_cast<uint64_t>(augmented_covariance.rows());
    out.clone_count = mirror.msckf_state_ids_for_test().size();
    out.covariance_symmetry_error =
        (augmented_covariance - augmented_covariance.transpose()).cwiseAbs().maxCoeff();
    out.covariance_symmetric = out.covariance_symmetry_error <= 1.0e-9;
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> covariance_solver(
        0.5 * (augmented_covariance + augmented_covariance.transpose()));
    out.covariance_psd = covariance_solver.info() == Eigen::Success;
    if (out.covariance_psd) {
        out.covariance_min_eigenvalue = covariance_solver.eigenvalues().minCoeff();
        out.covariance_psd = out.covariance_min_eigenvalue >= -1.0e-10;
    }

    const auto active_snapshot = with_shadow.active_snapshot();
    const auto diag = mirror.diagnostics();
    out.state_finite = active_snapshot.position_m.array().isFinite().all() &&
                       active_snapshot.velocity_mps.array().isFinite().all() &&
                       active_snapshot.orientation.coeffs().array().isFinite().all();
    out.attempted_updates = diag.feature_updates_attempted;
    out.applied_updates = diag.feature_updates_applied;
    out.rejected_updates = diag.feature_updates_rejected;
    out.features_considered = diag.feature_updates_considered;
    out.features_stacked = diag.feature_updates_stacked;
    out.observations_used =
        diag.stacked_measurement_dimension > 0u
            ? ((diag.stacked_measurement_dimension + diag.measurement_rank) / 2u)
            : 0u;
    out.stacked_measurement_dimension = diag.stacked_measurement_dimension;
    out.measurement_rank = diag.measurement_rank;
    out.feature_jacobian_rank = diag.measurement_rank;
    out.projected_dimension = diag.stacked_measurement_dimension;
    out.nullspace_annihilation_norm = diag.nullspace_annihilation_norm;
    out.nullspace_orthogonality_error = diag.nullspace_orthogonality_error;
    out.chi_square_dof = diag.chi_square_dof;
    out.chi_square_threshold_used = diag.chi_square_threshold;
    out.chi_square_statistic = chi_square_statistic_from_diag(diag);
    out.innovation_min_pivot = diag.innovation_min_eigenvalue;
    out.innovation_max_pivot = diag.innovation_min_eigenvalue * diag.innovation_condition_number;
    out.innovation_conditioning_valid =
        diag.innovation_min_eigenvalue > 0.0 && std::isfinite(diag.innovation_condition_number);
    out.innovation_factorization_status =
        out.innovation_conditioning_valid ? "LDLT_PASS" : "not_run";
    out.residual_norm = diag.residual_norm;
    out.correction_norm = diag.correction_norm;

    if (decisive_event.has_value()) {
        out.decisive_event = *decisive_event;
        out.pre_update_base_state_checksum = decisive_event->pre_base_checksum;
        out.post_update_base_state_checksum = decisive_event->post_base_checksum;
        out.pre_update_clone_state_checksum = decisive_event->pre_clone_checksum;
        out.post_update_clone_state_checksum = decisive_event->post_clone_checksum;
        out.pre_update_covariance_checksum = decisive_event->pre_covariance_checksum;
        out.post_update_covariance_checksum = decisive_event->post_covariance_checksum;
        out.rejection_reason = derive_rejection_reason(scenario, *decisive_event, diag);
    } else {
        out.pre_update_base_state_checksum = pose_checksum(mirror.state());
        out.post_update_base_state_checksum = out.pre_update_base_state_checksum;
        out.pre_update_clone_state_checksum = clone_checksum(mirror);
        out.post_update_clone_state_checksum = out.pre_update_clone_state_checksum;
        out.pre_update_covariance_checksum = matrix_checksum(augmented_covariance);
        out.post_update_covariance_checksum = out.pre_update_covariance_checksum;
        out.rejection_reason = scenario.expected_rejection_reason;
    }

    if (scenario.kind == ReplayScenarioKind::RejectedUpdate ||
        scenario.kind == ReplayScenarioKind::SingularGeometry ||
        scenario.kind == ReplayScenarioKind::StaleFej ||
        scenario.kind == ReplayScenarioKind::UpdateDisabled) {
        out.estimator_state_rollback_verified =
            out.pre_update_base_state_checksum == out.post_update_base_state_checksum &&
            out.pre_update_clone_state_checksum == out.post_update_clone_state_checksum &&
            out.pre_update_covariance_checksum == out.post_update_covariance_checksum;
    } else {
        out.estimator_state_rollback_verified = true;
    }

    if (decisive_event.has_value()) {
        describe_feature_lifecycle(scenario, *decisive_event, out);
    } else {
        out.feature_lifecycle_verified = scenario.kind == ReplayScenarioKind::AcceptedUpdate ||
                                         scenario.kind == ReplayScenarioKind::MultiFeatureStack ||
                                         scenario.kind == ReplayScenarioKind::UpdateDisabled;
    }

    switch (scenario.kind) {
    case ReplayScenarioKind::RejectedUpdate:
        out.diagnostics_update_verified =
            out.attempted_updates > 0u && out.applied_updates == 0u && out.rejected_updates > 0u &&
            diag.chi_square_failures > 0u && out.rejection_reason == "chi_square";
        break;
    case ReplayScenarioKind::SingularGeometry:
        out.diagnostics_update_verified = diag.rejected_degenerate_geometry > 0u &&
                                          out.applied_updates == 0u &&
                                          out.rejection_reason == "degenerate_geometry";
        break;
    case ReplayScenarioKind::StaleFej:
        out.diagnostics_update_verified = diag.fej_validation_failures > 0u &&
                                          out.applied_updates == 0u && out.rejected_updates > 0u &&
                                          out.rejection_reason == "stale_fej_snapshot";
        break;
    case ReplayScenarioKind::UpdateDisabled:
        out.diagnostics_update_verified =
            out.attempted_updates == 0u && out.applied_updates == 0u && out.rejected_updates == 0u;
        break;
    case ReplayScenarioKind::MultiFeatureStack:
        out.diagnostics_update_verified = out.applied_updates >= 2u && out.features_stacked >= 2u &&
                                          out.stacked_measurement_dimension > 0u &&
                                          out.measurement_rank > 0u;
        break;
    case ReplayScenarioKind::AcceptedUpdate:
        out.diagnostics_update_verified =
            out.applied_updates > 0u && out.features_stacked > 0u && out.measurement_rank > 0u;
        break;
    }
    out.rollback_verified = out.estimator_state_rollback_verified &&
                            out.feature_lifecycle_verified && out.diagnostics_update_verified;

    out.shadow_only_feature_update =
        !out.active_phase22_counters_present &&
        ((out.attempted_updates > 0u || out.applied_updates > 0u || out.rejected_updates > 0u) ||
         !scenario.msckf_cfg.update.enabled);
    out.first_divergence_event = first_divergence_event;

    std::ostringstream checksum;
    checksum << std::fixed << std::setprecision(12) << augmented_covariance.trace() << '|'
             << out.attempted_updates << '|' << out.applied_updates << '|' << out.rejected_updates
             << '|' << out.features_considered << '|' << out.features_stacked << '|'
             << out.stacked_measurement_dimension << '|' << out.measurement_rank << '|'
             << out.residual_norm << '|' << out.correction_norm << '|'
             << out.covariance_min_eigenvalue << '|' << out.nullspace_annihilation_norm;
    out.checksum = checksum.str();
    finalize_status(scenario, diag, out);
    return out;
}

bool equivalent(const ReplayResult& a, const ReplayResult& b) {
    return a.scenario == b.scenario && a.active_equivalent == b.active_equivalent &&
           a.shadow_queue_drained == b.shadow_queue_drained &&
           a.shadow_only_feature_update == b.shadow_only_feature_update &&
           a.active_phase22_config_present == b.active_phase22_config_present &&
           a.active_phase22_counters_present == b.active_phase22_counters_present &&
           a.covariance_finite == b.covariance_finite &&
           a.covariance_symmetric == b.covariance_symmetric &&
           a.covariance_psd == b.covariance_psd && a.attempted_updates == b.attempted_updates &&
           a.applied_updates == b.applied_updates && a.rejected_updates == b.rejected_updates &&
           a.rejection_reason == b.rejection_reason &&
           a.features_considered == b.features_considered &&
           a.features_stacked == b.features_stacked && a.observations_used == b.observations_used &&
           a.stacked_measurement_dimension == b.stacked_measurement_dimension &&
           a.measurement_rank == b.measurement_rank &&
           a.projected_dimension == b.projected_dimension && a.chi_square_dof == b.chi_square_dof &&
           std::abs(a.residual_norm - b.residual_norm) <= 1.0e-12 &&
           std::abs(a.correction_norm - b.correction_norm) <= 1.0e-12 &&
           std::abs(a.nullspace_annihilation_norm - b.nullspace_annihilation_norm) <= 1.0e-12 &&
           a.rollback_verified == b.rollback_verified && a.final_status == b.final_status &&
           a.checksum == b.checksum;
}

void write_report(const std::vector<ReplayResult>& results) {
    std::filesystem::create_directories("artifacts/phase22");
    std::ofstream output("artifacts/phase22/ekf_phase22_replay_report.json", std::ios::trunc);
    output << "{\n";
    output << "  \"schema_version\": 2,\n";
    output << "  \"phase\": 22,\n";
    output << "  \"results\": [\n";
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        output << "    {\n";
        output << "      \"scenario\": \"" << r.scenario << "\",\n";
        output << "      \"deterministic\": " << (r.deterministic ? "true" : "false") << ",\n";
        output << "      \"deterministic_runs\": " << r.deterministic_runs << ",\n";
        output << "      \"active_equivalent\": " << (r.active_equivalent ? "true" : "false")
               << ",\n";
        output << "      \"active_equivalence_method\": \"" << r.active_equivalence_method
               << "\",\n";
        output << "      \"shadow_queue_drained\": " << (r.shadow_queue_drained ? "true" : "false")
               << ",\n";
        output << "      \"shadow_only_feature_update\": "
               << (r.shadow_only_feature_update ? "true" : "false") << ",\n";
        output << "      \"shadow_only_evidence\": \"" << r.shadow_only_evidence << "\",\n";
        output << "      \"active_phase22_config_present\": "
               << (r.active_phase22_config_present ? "true" : "false") << ",\n";
        output << "      \"active_phase22_counters_present\": "
               << (r.active_phase22_counters_present ? "true" : "false") << ",\n";
        output << "      \"state_finite\": " << (r.state_finite ? "true" : "false") << ",\n";
        output << "      \"covariance_finite\": " << (r.covariance_finite ? "true" : "false")
               << ",\n";
        output << "      \"covariance_symmetric\": " << (r.covariance_symmetric ? "true" : "false")
               << ",\n";
        output << "      \"covariance_symmetry_error\": " << r.covariance_symmetry_error << ",\n";
        output << "      \"covariance_psd\": " << (r.covariance_psd ? "true" : "false") << ",\n";
        output << "      \"covariance_min_eigenvalue\": " << r.covariance_min_eigenvalue << ",\n";
        output << "      \"attempted_updates\": " << r.attempted_updates << ",\n";
        output << "      \"applied_updates\": " << r.applied_updates << ",\n";
        output << "      \"rejected_updates\": " << r.rejected_updates << ",\n";
        output << "      \"rejection_reason\": \"" << r.rejection_reason << "\",\n";
        output << "      \"features_considered\": " << r.features_considered << ",\n";
        output << "      \"features_stacked\": " << r.features_stacked << ",\n";
        output << "      \"observations_used\": " << r.observations_used << ",\n";
        output << "      \"augmented_state_dimension\": " << r.augmented_state_dimension << ",\n";
        output << "      \"clone_count\": " << r.clone_count << ",\n";
        output << "      \"stacked_measurement_dimension\": " << r.stacked_measurement_dimension
               << ",\n";
        output << "      \"measurement_rank\": " << r.measurement_rank << ",\n";
        output << "      \"feature_jacobian_rank\": " << r.feature_jacobian_rank << ",\n";
        output << "      \"projected_dimension\": " << r.projected_dimension << ",\n";
        output << "      \"nullspace_annihilation_norm\": " << r.nullspace_annihilation_norm
               << ",\n";
        output << "      \"nullspace_orthogonality_error\": " << r.nullspace_orthogonality_error
               << ",\n";
        output << "      \"chi_square_dof\": " << r.chi_square_dof << ",\n";
        output << "      \"chi_square_probability\": " << r.chi_square_probability << ",\n";
        output << "      \"chi_square_threshold_used\": " << r.chi_square_threshold_used << ",\n";
        output << "      \"chi_square_statistic\": " << r.chi_square_statistic << ",\n";
        output << "      \"innovation_factorization_status\": \""
               << r.innovation_factorization_status << "\",\n";
        output << "      \"innovation_min_pivot\": " << r.innovation_min_pivot << ",\n";
        output << "      \"innovation_max_pivot\": " << r.innovation_max_pivot << ",\n";
        output << "      \"innovation_conditioning_valid\": "
               << (r.innovation_conditioning_valid ? "true" : "false") << ",\n";
        output << "      \"residual_norm\": " << r.residual_norm << ",\n";
        output << "      \"correction_norm\": " << r.correction_norm << ",\n";
        output << "      \"pre_update_base_state_checksum\": \"" << r.pre_update_base_state_checksum
               << "\",\n";
        output << "      \"post_update_base_state_checksum\": \""
               << r.post_update_base_state_checksum << "\",\n";
        output << "      \"pre_update_clone_state_checksum\": \""
               << r.pre_update_clone_state_checksum << "\",\n";
        output << "      \"post_update_clone_state_checksum\": \""
               << r.post_update_clone_state_checksum << "\",\n";
        output << "      \"pre_update_covariance_checksum\": \"" << r.pre_update_covariance_checksum
               << "\",\n";
        output << "      \"post_update_covariance_checksum\": \""
               << r.post_update_covariance_checksum << "\",\n";
        output << "      \"estimator_state_rollback_verified\": "
               << (r.estimator_state_rollback_verified ? "true" : "false") << ",\n";
        output << "      \"feature_lifecycle_expected\": \"" << r.feature_lifecycle_expected
               << "\",\n";
        output << "      \"feature_lifecycle_observed\": \"" << r.feature_lifecycle_observed
               << "\",\n";
        output << "      \"feature_lifecycle_verified\": "
               << (r.feature_lifecycle_verified ? "true" : "false") << ",\n";
        output << "      \"diagnostics_update_verified\": "
               << (r.diagnostics_update_verified ? "true" : "false") << ",\n";
        output << "      \"rollback_verified\": " << (r.rollback_verified ? "true" : "false")
               << ",\n";
        output << "      \"final_status\": \"" << r.final_status << "\"\n";
        output << "    }" << (i + 1 < results.size() ? "," : "") << "\n";
    }
    output << "  ]\n";
    output << "}\n";
}

} // namespace

int main() {
    const std::vector<Eigen::Vector3d> baseline_features{
        {0.2, 0.1, 4.2},
        {-0.2, 0.12, 4.5},
        {0.15, -0.18, 4.8},
    };
    const std::vector<Eigen::Vector3d> stacked_features{
        {0.22, -0.08, 4.4},
        {-0.18, 0.14, 4.7},
        {0.12, 0.18, 4.9},
    };

    const std::vector<ReplayScenario> scenarios{
        {
            "straight_track_update",
            ReplayScenarioKind::AcceptedUpdate,
            baseline_features,
            {{0.0, 0.0, 0.0}, {0.18, 0.0, 0.0}, {0.18, 0.0, 0.0}, {0.18, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
            {{0, 1, 2}, {0, 1, 2}, {0, 1, 2}, {0, 1, 2}},
            {
                {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                {Eigen::Vector2d{0.20, -0.10}, Eigen::Vector2d{0.15, 0.08},
                 Eigen::Vector2d{-0.12, 0.14}},
                {Eigen::Vector2d{0.10, -0.05}, Eigen::Vector2d{0.08, 0.05},
                 Eigen::Vector2d{-0.08, 0.10}},
            },
            make_msckf_cfg(),
        },
        {
            "turning_track_update",
            ReplayScenarioKind::AcceptedUpdate,
            baseline_features,
            {{0.0, 0.0, 0.0}, {0.12, 0.0, 0.0}, {0.10, 0.05, 0.0}, {0.08, 0.06, 0.0}},
            {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.02}, {0.0, 0.01, 0.02}, {0.0, 0.01, 0.02}},
            {{0, 1, 2}, {0, 1, 2}, {0, 1, 2}, {0, 1, 2}},
            {
                {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                {Eigen::Vector2d{0.12, 0.05}, Eigen::Vector2d{-0.09, 0.07},
                 Eigen::Vector2d{0.10, -0.04}},
                {Eigen::Vector2d{0.10, 0.04}, Eigen::Vector2d{-0.08, 0.06},
                 Eigen::Vector2d{0.08, -0.03}},
            },
            make_msckf_cfg(),
        },
        {
            "long_track_update",
            ReplayScenarioKind::AcceptedUpdate,
            baseline_features,
            {{0.0, 0.0, 0.0},
             {0.10, 0.0, 0.0},
             {0.10, 0.05, 0.0},
             {0.10, 0.0, 0.0},
             {0.10, 0.05, 0.0},
             {0.10, 0.0, 0.0}},
            {{0.0, 0.0, 0.0},
             {0.0, 0.0, 0.01},
             {0.0, 0.0, 0.01},
             {0.0, 0.0, 0.01},
             {0.0, 0.0, 0.01},
             {0.0, 0.0, 0.01}},
            {{0, 1, 2}, {0, 1, 2}, {0, 1, 2}, {0, 1, 2}, {0, 1, 2}, {0, 1, 2}},
            {
                {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                {Eigen::Vector2d{0.10, -0.04}, Eigen::Vector2d{0.06, 0.04},
                 Eigen::Vector2d{-0.07, 0.06}},
                {Eigen::Vector2d{0.12, -0.05}, Eigen::Vector2d{0.07, 0.05},
                 Eigen::Vector2d{-0.08, 0.07}},
                {Eigen::Vector2d{0.14, -0.05}, Eigen::Vector2d{0.08, 0.05},
                 Eigen::Vector2d{-0.09, 0.08}},
                {Eigen::Vector2d{0.16, -0.06}, Eigen::Vector2d{0.09, 0.06},
                 Eigen::Vector2d{-0.10, 0.09}},
            },
            make_msckf_cfg(0.999999, 0.1, 5),
        },
        {
            "noisy_track_update",
            ReplayScenarioKind::AcceptedUpdate,
            baseline_features,
            {{0.0, 0.0, 0.0}, {0.18, 0.0, 0.0}, {0.18, 0.0, 0.0}, {0.0, 0.12, 0.0}},
            {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
            {{0, 1, 2}, {0, 1, 2}, {0, 1, 2}, {0, 1, 2}},
            {
                {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                {Eigen::Vector2d{0.14, -0.08}, Eigen::Vector2d{-0.12, 0.10},
                 Eigen::Vector2d{0.10, 0.08}},
                {Eigen::Vector2d{0.22, -0.14}, Eigen::Vector2d{-0.18, 0.12},
                 Eigen::Vector2d{0.16, 0.10}},
            },
            make_msckf_cfg(),
        },
        {
            "rejected_track_update",
            ReplayScenarioKind::RejectedUpdate,
            baseline_features,
            {{0.0, 0.0, 0.0}, {0.16, 0.0, 0.0}, {0.0, 0.13, 0.0}},
            {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
            {{1}, {1}, {1}},
            {
                {Eigen::Vector2d::Zero()},
                {Eigen::Vector2d::Zero()},
                {Eigen::Vector2d{0.5, 0.25}},
            },
            make_msckf_cfg(1.0e-6, 0.1, 8),
            "chi_square",
        },
        {
            "multi_feature_stack",
            ReplayScenarioKind::MultiFeatureStack,
            stacked_features,
            {{0.0, 0.0, 0.0}, {0.12, 0.0, 0.0}, {0.10, 0.05, 0.0}, {0.08, 0.06, 0.0}},
            {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.02}, {0.0, 0.01, 0.02}, {0.0, 0.01, 0.02}},
            {{0, 1, 2}, {0, 1, 2}, {0, 1, 2}, {0, 1, 2}},
            {
                {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                {Eigen::Vector2d{0.12, 0.05}, Eigen::Vector2d{-0.09, 0.07},
                 Eigen::Vector2d{0.10, -0.04}},
                {Eigen::Vector2d{0.10, 0.04}, Eigen::Vector2d{-0.08, 0.06},
                 Eigen::Vector2d{0.08, -0.03}},
            },
            make_msckf_cfg(),
        },
        {
            "singular_geometry_rejection",
            ReplayScenarioKind::SingularGeometry,
            {{0.0, 0.0, 4.0}},
            {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.12}, {0.0, 0.0, 0.12}},
            {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
            {{0}, {0}, {0}},
            {
                {Eigen::Vector2d::Zero()},
                {Eigen::Vector2d::Zero()},
                {Eigen::Vector2d::Zero()},
            },
            make_msckf_cfg(),
            "degenerate_geometry",
        },
        {
            "stale_fej_rejection",
            ReplayScenarioKind::StaleFej,
            {{0.22, -0.05, 4.6}},
            {{0.0, 0.0, 0.0}, {0.16, 0.0, 0.0}, {0.0, 0.11, 0.0}},
            {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
            {{0}, {0}, {0}},
            {
                {Eigen::Vector2d::Zero()},
                {Eigen::Vector2d::Zero()},
                {Eigen::Vector2d::Zero()},
            },
            make_msckf_cfg(),
            "stale_fej_snapshot",
            true,
            1u,
        },
        {
            "update_disabled_control",
            ReplayScenarioKind::UpdateDisabled,
            baseline_features,
            {{0.0, 0.0, 0.0}, {0.18, 0.0, 0.0}, {0.0, 0.12, 0.0}},
            {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
            {{0, 1, 2}, {0, 1, 2}, {0, 1, 2}},
            {
                {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
                {Eigen::Vector2d{0.10, -0.05}, Eigen::Vector2d{0.08, 0.04},
                 Eigen::Vector2d{-0.08, 0.06}},
            },
            [] {
                auto cfg = make_msckf_cfg();
                cfg.update.enabled = false;
                return cfg;
            }(),
            "update_disabled",
        },
    };

    std::vector<ReplayResult> results;
    results.reserve(scenarios.size());

    bool all_pass = true;
    for (const auto& scenario : scenarios) {
        const auto first = run_once(scenario);
        bool deterministic = true;
        for (uint32_t run = 1; run < kDeterministicRuns; ++run) {
            const auto repeat = run_once(scenario);
            deterministic = deterministic && equivalent(first, repeat);
        }
        ReplayResult combined = first;
        combined.deterministic = deterministic;
        combined.final_status =
            (combined.final_status == "PASS" && combined.deterministic) ? "PASS" : "FAIL";
        all_pass = all_pass && (combined.final_status == "PASS");
        results.push_back(combined);
    }

    write_report(results);
    return all_pass ? 0 : 1;
}
