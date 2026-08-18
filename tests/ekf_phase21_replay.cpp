#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"
#include "vio/Phase17ESKFEstimator.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

using namespace drone::vio;

namespace {

constexpr double kFrameDt = 0.01;
constexpr double kPositionTolerance = 1.0e-12;
constexpr double kVelocityTolerance = 1.0e-12;
constexpr double kOrientationTolerance = 1.0e-12;

struct Phase21ReplayMsckfOptions {
    uint32_t max_camera_states{10};
    uint32_t minimum_observations{2};
    double minimum_baseline{0.05};
    double maximum_reprojection_error{2.5};
    double minimum_depth{0.1};
    double maximum_depth{50.0};
};

struct ReplayScenario {
    std::string name;
    std::vector<Eigen::Vector3d> features;
    std::vector<Eigen::Vector3d> position_deltas;
    std::vector<Eigen::Vector3d> rotation_deltas;
    MsckfConfig msckf_cfg;
    std::vector<std::vector<size_t>> visible_feature_indices;
    std::optional<size_t> dropped_feature_index{};
};

struct ReplayResult {
    std::string scenario;
    bool deterministic{false};
    bool active_equivalent{false};
    std::string active_equivalence_method;
    bool shadow_only_triangulation{false};
    std::string shadow_only_evidence;
    bool covariance_finite{false};
    bool landmarks_finite{false};
    uint64_t feature_tracks_created{0};
    uint64_t feature_tracks_removed{0};
    uint64_t triangulation_attempts{0};
    uint64_t triangulation_successes{0};
    uint64_t rejected_insufficient_observations{0};
    uint64_t rejected_small_baseline{0};
    uint64_t rejected_negative_depth{0};
    uint64_t rejected_depth_range{0};
    uint64_t rejected_degenerate_geometry{0};
    uint64_t rejected_non_finite_input{0};
    uint64_t rejected_reprojection{0};
    uint64_t active_landmarks_before_cleanup{0};
    uint64_t active_landmarks_after_cleanup{0};
    bool stale_tracks_found{false};
    bool stale_landmarks_found{false};
    std::optional<uint64_t> dropped_feature_track_id{};
    std::optional<size_t> cleanup_frame_index{};
    std::string checksum;
    std::string final_status{"FAIL"};
};

EstimatorValidationConfig make_cfg() {
    EstimatorValidationConfig cfg;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    cfg.enable_shadow_estimator = true;
    cfg.shadow_comparison_enabled = true;
    cfg.shadow_max_queue_depth = 1024;
    cfg.shadow_max_lag_ms = 50.0;
    return cfg;
}

EstimatorValidationConfig make_active_only_cfg() {
    EstimatorValidationConfig cfg = make_cfg();
    cfg.enable_shadow_estimator = false;
    cfg.shadow_comparison_enabled = false;
    return cfg;
}

MsckfConfig make_msckf_cfg(const Phase21ReplayMsckfOptions& options = {}) {
    MsckfConfig cfg;
    cfg.enabled = true;
    cfg.max_camera_states = options.max_camera_states;
    cfg.eviction_policy = "oldest_first";
    cfg.diagnostics_enabled = true;
    cfg.triangulation.enabled = true;
    cfg.triangulation.minimum_observations = options.minimum_observations;
    cfg.triangulation.minimum_baseline = options.minimum_baseline;
    cfg.triangulation.maximum_reprojection_error = options.maximum_reprojection_error;
    cfg.triangulation.minimum_depth = options.minimum_depth;
    cfg.triangulation.maximum_depth = options.maximum_depth;
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

ReplayResult run_once(const ReplayScenario& scenario) {
    auto active_only = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "phase17"));
    active_only.configure_validation(make_active_only_cfg());
    active_only.initialize();

    auto with_shadow = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "phase17"));
    with_shadow.configure_validation(make_cfg());
    with_shadow.configure_shadow_msckf(scenario.msckf_cfg);
    with_shadow.initialize();
    (void)with_shadow.start();

    Phase17ESKFEstimator mirror;
    mirror.configure_validation(make_cfg());
    mirror.configure_msckf(scenario.msckf_cfg);
    mirror.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond target_orientation = Eigen::Quaterniond::Identity();
    uint64_t sequence_id = 0;
    bool active_equivalent = true;
    bool finite_landmarks = true;
    uint64_t active_landmarks_before_cleanup = 0;
    std::optional<uint64_t> dropped_feature_track_id{};
    std::optional<size_t> cleanup_frame_index{};

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

        const Eigen::Vector3d velocity_mps = delta_position / kFrameDt;
        const auto pose_env = make_visual_pose_envelope(
            VisualPoseMeasurementPayload{target_position, velocity_mps, 0.01, 0.01},
            MeasurementStamp{timestamp_s, sequence_id++});
        (void)active_only.process_measurement(pose_env);
        (void)with_shadow.process_measurement(pose_env);
        mirror.update_visual_pose(target_position, velocity_mps, 0.01, 0.01);
        with_shadow.flush_shadow();

        std::vector<Eigen::Vector2d> pixels;
        std::vector<Eigen::Vector3d> identities;
        for (const size_t feature_index : scenario.visible_feature_indices[frame_index]) {
            pixels.push_back(project_feature_from_pose(target_position, target_orientation,
                                                       scenario.features[feature_index]));
            identities.push_back(scenario.features[feature_index]);
        }

        const auto active_before = active_only.active_snapshot();
        const auto with_shadow_before = with_shadow.active_snapshot();
        VisualFeatureMeasurementPayload feature_payload;
        feature_payload.z_pixels.assign(pixels.begin(), pixels.end());
        feature_payload.p_world.assign(identities.begin(), identities.end());
        feature_payload.K = camera_intrinsics();
        const auto feature_env = make_visual_features_envelope(
            feature_payload, MeasurementStamp{timestamp_s, sequence_id++});
        (void)active_only.process_measurement(feature_env);
        (void)with_shadow.process_measurement(feature_env);
        mirror.update_vision(pixels, identities, camera_intrinsics());
        with_shadow.flush_shadow();

        const auto active_after = active_only.active_snapshot();
        const auto with_shadow_after = with_shadow.active_snapshot();
        active_equivalent = active_equivalent &&
                            snapshots_equivalent(active_before, with_shadow_before) &&
                            snapshots_equivalent(active_after, with_shadow_after);

        active_landmarks_before_cleanup =
            std::max(active_landmarks_before_cleanup, mirror.active_landmark_count_for_test());
        if (scenario.dropped_feature_index.has_value()) {
            const auto& dropped_feature = scenario.features[*scenario.dropped_feature_index];
            if (!dropped_feature_track_id.has_value()) {
                dropped_feature_track_id =
                    mirror.feature_track_id_for_feature_for_test(dropped_feature);
            }
            const bool track_present =
                mirror.feature_track_id_for_feature_for_test(dropped_feature).has_value();
            const bool landmark_present =
                mirror.triangulated_landmark_for_feature_for_test(dropped_feature).has_value();
            if (!cleanup_frame_index.has_value() && !track_present && !landmark_present &&
                dropped_feature_track_id.has_value()) {
                cleanup_frame_index = frame_index;
            }
        }
    }

    with_shadow.flush_shadow();
    with_shadow.stop();

    const auto active_snapshot = with_shadow.active_snapshot();
    const auto shadow_snapshot = with_shadow.shadow_snapshot();
    const auto cov = mirror.covariance();

    bool stale_tracks_found = false;
    bool stale_landmarks_found = false;
    if (scenario.dropped_feature_index.has_value()) {
        const auto& dropped_feature = scenario.features[*scenario.dropped_feature_index];
        stale_tracks_found =
            mirror.feature_track_id_for_feature_for_test(dropped_feature).has_value();
        stale_landmarks_found =
            mirror.triangulated_landmark_for_feature_for_test(dropped_feature).has_value();
    }

    for (const auto& feature : scenario.features) {
        if (const auto landmark = mirror.triangulated_landmark_for_feature_for_test(feature);
            landmark.has_value()) {
            finite_landmarks = finite_landmarks && landmark->array().isFinite().all();
        }
    }

    ReplayResult out;
    out.scenario = scenario.name;
    out.active_equivalent = active_equivalent;
    out.active_equivalence_method =
        "Coordinator scenario traffic equality after each visual-features event: "
        "position norm <= 1e-12, velocity norm <= 1e-12, |dot(q)| delta <= 1e-12";
    out.covariance_finite = cov.array().isFinite().all() && std::isfinite(cov.trace()) &&
                            (cov - cov.transpose()).cwiseAbs().maxCoeff() < 1.0e-8;
    out.landmarks_finite = finite_landmarks;
    out.active_landmarks_before_cleanup = active_landmarks_before_cleanup;
    out.active_landmarks_after_cleanup = mirror.active_landmark_count_for_test();
    out.stale_tracks_found = stale_tracks_found;
    out.stale_landmarks_found = stale_landmarks_found;
    out.dropped_feature_track_id = dropped_feature_track_id;
    out.cleanup_frame_index = cleanup_frame_index;

    if (shadow_snapshot.has_value()) {
        out.feature_tracks_created = shadow_snapshot->feature_tracks_created;
        out.feature_tracks_removed = shadow_snapshot->feature_tracks_removed;
        out.triangulation_attempts = shadow_snapshot->triangulation_attempts;
        out.triangulation_successes = shadow_snapshot->triangulation_successes;
        out.rejected_insufficient_observations =
            shadow_snapshot->rejected_insufficient_observations;
        out.rejected_small_baseline = shadow_snapshot->rejected_small_baseline;
        out.rejected_negative_depth = shadow_snapshot->rejected_negative_depth;
        out.rejected_depth_range = shadow_snapshot->rejected_depth_range;
        out.rejected_degenerate_geometry = shadow_snapshot->rejected_degenerate_geometry;
        out.rejected_non_finite_input = shadow_snapshot->rejected_non_finite_input;
        out.rejected_reprojection = shadow_snapshot->rejected_reprojection;
    }

    const bool active_has_no_triangulation_state =
        active_snapshot.triangulation_attempts == 0u &&
        active_snapshot.triangulation_successes == 0u && active_snapshot.active_landmarks == 0u &&
        active_snapshot.feature_tracks == 0u && active_snapshot.msckf_window_size == 0u;
    const bool shadow_changed_state =
        out.feature_tracks_created > 0u || out.triangulation_attempts > 0u ||
        out.rejected_small_baseline > 0u || out.active_landmarks_before_cleanup > 0u;
    out.shadow_only_triangulation = out.active_equivalent && active_has_no_triangulation_state &&
                                    shadow_changed_state && shadow_snapshot.has_value();

    {
        std::ostringstream evidence;
        evidence
            << "shadow snapshot counters changed via coordinator traffic, active snapshot kept "
               "triangulation fields at zero, active outputs matched active-only coordinator";
        if (out.cleanup_frame_index.has_value()) {
            evidence << ", dropped feature track removed by frame " << *out.cleanup_frame_index;
        }
        out.shadow_only_evidence = evidence.str();
    }

    {
        std::ostringstream checksum;
        checksum << std::fixed << std::setprecision(12) << active_snapshot.position_m.transpose()
                 << '|' << active_snapshot.velocity_mps.transpose() << '|'
                 << active_snapshot.orientation.coeffs().transpose() << '|' << cov.trace() << '|'
                 << out.feature_tracks_created << '|' << out.feature_tracks_removed << '|'
                 << out.triangulation_attempts << '|' << out.triangulation_successes << '|'
                 << out.rejected_insufficient_observations << '|' << out.rejected_small_baseline
                 << '|' << out.rejected_negative_depth << '|' << out.rejected_depth_range << '|'
                 << out.rejected_degenerate_geometry << '|' << out.rejected_non_finite_input << '|'
                 << out.rejected_reprojection << '|' << out.active_landmarks_before_cleanup << '|'
                 << out.active_landmarks_after_cleanup << '|' << (out.stale_tracks_found ? 1 : 0)
                 << '|' << (out.stale_landmarks_found ? 1 : 0);
        if (out.cleanup_frame_index.has_value()) {
            checksum << '|' << *out.cleanup_frame_index;
        }
        out.checksum = checksum.str();
    }

    const bool cleanup_ok = !out.stale_tracks_found && !out.stale_landmarks_found;
    out.final_status = (out.active_equivalent && out.shadow_only_triangulation &&
                        out.covariance_finite && out.landmarks_finite && cleanup_ok)
                           ? "PASS"
                           : "FAIL";
    return out;
}

void write_report(const std::vector<ReplayResult>& results) {
    const std::filesystem::path out_dir = std::filesystem::path("artifacts") / "phase21";
    std::filesystem::create_directories(out_dir);
    const std::filesystem::path out_path = out_dir / "ekf_phase21_replay_report.json";

    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"schema_version\": 2,\n";
    oss << "  \"date\": \"2026-07-18\",\n";
    oss << "  \"scenarios\": [\n";
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        oss << "    {\n";
        oss << "      \"scenario\": \"" << r.scenario << "\",\n";
        oss << "      \"deterministic\": " << (r.deterministic ? "true" : "false") << ",\n";
        oss << "      \"active_equivalent\": " << (r.active_equivalent ? "true" : "false") << ",\n";
        oss << "      \"active_equivalence_method\": \"" << r.active_equivalence_method << "\",\n";
        oss << "      \"shadow_only_triangulation\": "
            << (r.shadow_only_triangulation ? "true" : "false") << ",\n";
        oss << "      \"shadow_only_evidence\": \"" << r.shadow_only_evidence << "\",\n";
        oss << "      \"covariance_finite\": " << (r.covariance_finite ? "true" : "false") << ",\n";
        oss << "      \"landmarks_finite\": " << (r.landmarks_finite ? "true" : "false") << ",\n";
        oss << "      \"feature_tracks_created\": " << r.feature_tracks_created << ",\n";
        oss << "      \"feature_tracks_removed\": " << r.feature_tracks_removed << ",\n";
        oss << "      \"triangulation_attempts\": " << r.triangulation_attempts << ",\n";
        oss << "      \"triangulation_successes\": " << r.triangulation_successes << ",\n";
        oss << "      \"rejected_insufficient_observations\": "
            << r.rejected_insufficient_observations << ",\n";
        oss << "      \"rejected_small_baseline\": " << r.rejected_small_baseline << ",\n";
        oss << "      \"rejected_negative_depth\": " << r.rejected_negative_depth << ",\n";
        oss << "      \"rejected_depth_range\": " << r.rejected_depth_range << ",\n";
        oss << "      \"rejected_degenerate_geometry\": " << r.rejected_degenerate_geometry
            << ",\n";
        oss << "      \"rejected_non_finite_input\": " << r.rejected_non_finite_input << ",\n";
        oss << "      \"rejected_reprojection\": " << r.rejected_reprojection << ",\n";
        oss << "      \"active_landmarks_before_cleanup\": " << r.active_landmarks_before_cleanup
            << ",\n";
        oss << "      \"active_landmarks_after_cleanup\": " << r.active_landmarks_after_cleanup
            << ",\n";
        oss << "      \"stale_tracks_found\": " << (r.stale_tracks_found ? "true" : "false")
            << ",\n";
        oss << "      \"stale_landmarks_found\": " << (r.stale_landmarks_found ? "true" : "false")
            << ",\n";
        if (r.dropped_feature_track_id.has_value()) {
            oss << "      \"dropped_feature_track_id\": " << *r.dropped_feature_track_id << ",\n";
        }
        if (r.cleanup_frame_index.has_value()) {
            oss << "      \"cleanup_frame_index\": " << *r.cleanup_frame_index << ",\n";
        }
        oss << "      \"checksum\": \"" << r.checksum << "\",\n";
        oss << "      \"final_status\": \"" << r.final_status << "\"\n";
        oss << "    }" << (i + 1 < results.size() ? "," : "") << "\n";
    }
    oss << "  ]\n";
    oss << "}\n";

    std::ofstream out(out_path, std::ios::binary);
    out << oss.str();
}

} // namespace

int main() {
    const std::vector<Eigen::Vector3d> base_features{
        {0.2, 0.1, 4.5},
        {-0.15, 0.12, 5.0},
        {0.05, -0.18, 4.2},
    };

    const std::vector<ReplayScenario> scenarios{
        {"straight_motion", base_features,
         std::vector<Eigen::Vector3d>(5, Eigen::Vector3d{0.14, 0.0, 0.0}),
         std::vector<Eigen::Vector3d>(5, Eigen::Vector3d::Zero()), make_msckf_cfg(),
         std::vector<std::vector<size_t>>(5, std::vector<size_t>{0, 1, 2}), std::nullopt},
        {"lateral_motion", base_features,
         std::vector<Eigen::Vector3d>(5, Eigen::Vector3d{0.0, 0.14, 0.0}),
         std::vector<Eigen::Vector3d>(5, Eigen::Vector3d::Zero()), make_msckf_cfg(),
         std::vector<std::vector<size_t>>(5, std::vector<size_t>{0, 1, 2}), std::nullopt},
        {"rotating_camera", base_features,
         std::vector<Eigen::Vector3d>(5, Eigen::Vector3d{0.04, 0.0, 0.0}),
         std::vector<Eigen::Vector3d>(5, Eigen::Vector3d{0.0, 0.05, 0.0}), make_msckf_cfg(),
         std::vector<std::vector<size_t>>(5, std::vector<size_t>{0, 1, 2}), std::nullopt},
        {"low_parallax", base_features,
         std::vector<Eigen::Vector3d>(5, Eigen::Vector3d{0.01, 0.0, 0.0}),
         std::vector<Eigen::Vector3d>(5, Eigen::Vector3d::Zero()),
         make_msckf_cfg(Phase21ReplayMsckfOptions{.minimum_baseline = 0.15}),
         std::vector<std::vector<size_t>>(5, std::vector<size_t>{0, 1, 2}), std::nullopt},
        {"feature_dropout",
         base_features,
         std::vector<Eigen::Vector3d>(6, Eigen::Vector3d{0.12, 0.08, 0.0}),
         std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero()),
         make_msckf_cfg(Phase21ReplayMsckfOptions{.max_camera_states = 3}),
         {
             {0, 1, 2},
             {0, 1, 2},
             {1, 2},
             {1, 2},
             {1, 2},
             {1, 2},
         },
         0u},
    };

    std::vector<ReplayResult> results;
    results.reserve(scenarios.size());
    bool ok = true;
    for (const auto& scenario : scenarios) {
        auto first = run_once(scenario);
        auto second = run_once(scenario);
        first.deterministic =
            first.checksum == second.checksum &&
            first.active_equivalent == second.active_equivalent &&
            first.shadow_only_triangulation == second.shadow_only_triangulation &&
            first.covariance_finite == second.covariance_finite &&
            first.landmarks_finite == second.landmarks_finite &&
            first.feature_tracks_created == second.feature_tracks_created &&
            first.feature_tracks_removed == second.feature_tracks_removed &&
            first.triangulation_attempts == second.triangulation_attempts &&
            first.triangulation_successes == second.triangulation_successes &&
            first.rejected_insufficient_observations == second.rejected_insufficient_observations &&
            first.rejected_small_baseline == second.rejected_small_baseline &&
            first.rejected_negative_depth == second.rejected_negative_depth &&
            first.rejected_depth_range == second.rejected_depth_range &&
            first.rejected_degenerate_geometry == second.rejected_degenerate_geometry &&
            first.rejected_non_finite_input == second.rejected_non_finite_input &&
            first.rejected_reprojection == second.rejected_reprojection &&
            first.active_landmarks_before_cleanup == second.active_landmarks_before_cleanup &&
            first.active_landmarks_after_cleanup == second.active_landmarks_after_cleanup &&
            first.stale_tracks_found == second.stale_tracks_found &&
            first.stale_landmarks_found == second.stale_landmarks_found &&
            first.cleanup_frame_index == second.cleanup_frame_index &&
            first.final_status == second.final_status;
        ok = ok && first.deterministic && first.final_status == "PASS";
        results.push_back(std::move(first));
    }

    write_report(results);
    return ok ? 0 : 1;
}
