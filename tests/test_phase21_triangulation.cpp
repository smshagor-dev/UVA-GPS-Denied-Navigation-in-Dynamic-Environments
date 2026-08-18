#include <gtest/gtest.h>

#include "vio/Phase17ESKFEstimator.hpp"

using namespace drone::vio;

namespace {

struct Phase21MsckfOptions {
    bool triangulation_enabled{true};
    uint32_t max_camera_states{8};
    uint32_t minimum_observations{2};
    double minimum_baseline{0.05};
    double maximum_reprojection_error{2.5};
    double minimum_depth{0.1};
    double maximum_depth{50.0};
    bool diagnostics_enabled{true};
};

struct FeatureObservationSpec {
    Eigen::Vector3d feature_identity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d projection_feature{Eigen::Vector3d::Zero()};
    Eigen::Vector2d pixel_offset{Eigen::Vector2d::Zero()};
};

EstimatorValidationConfig make_phase21_validation_cfg() {
    EstimatorValidationConfig cfg;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    return cfg;
}

MsckfConfig make_phase21_msckf_cfg(const Phase21MsckfOptions& options = {}) {
    MsckfConfig cfg;
    cfg.enabled = true;
    cfg.max_camera_states = options.max_camera_states;
    cfg.eviction_policy = "oldest_first";
    cfg.diagnostics_enabled = options.diagnostics_enabled;
    cfg.triangulation.enabled = options.triangulation_enabled;
    cfg.triangulation.minimum_observations = options.minimum_observations;
    cfg.triangulation.minimum_baseline = options.minimum_baseline;
    cfg.triangulation.maximum_reprojection_error = options.maximum_reprojection_error;
    cfg.triangulation.minimum_depth = options.minimum_depth;
    cfg.triangulation.maximum_depth = options.maximum_depth;
    return cfg;
}

EKFConfig make_permissive_vision_cfg() {
    EKFConfig cfg;
    cfg.sigma_px = 100.0;
    cfg.mahal_gate = 1.0e9;
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

Eigen::Vector2d project_feature(const Phase17ESKFEstimator& ekf, const Eigen::Vector3d& feature) {
    const auto state = ekf.state();
    return project_feature_from_pose(state.position, state.orientation, feature);
}

void advance_frame(Phase17ESKFEstimator& ekf, double& timestamp_s,
                   const Eigen::Vector3d& delta_position = Eigen::Vector3d::Zero(),
                   const Eigen::Vector3d& delta_theta = Eigen::Vector3d::Zero()) {
    ASSERT_EQ(ekf.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero(),
                                          timestamp_s),
              EstimatorOperationResult::Accepted);
    timestamp_s += 0.01;

    if (!delta_position.isZero(0.0) || !delta_theta.isZero(0.0)) {
        ErrorVec dx = ErrorVec::Zero();
        dx.segment<3>(0) = delta_position;
        dx.segment<3>(6) = delta_theta;
        ASSERT_EQ(ekf.inject_error_for_test(dx), EstimatorOperationResult::Accepted);
    }
}

void observe_feature(Phase17ESKFEstimator& ekf, const FeatureObservationSpec& observation) {
    const Eigen::Vector2d pixel =
        project_feature(ekf, observation.projection_feature) + observation.pixel_offset;
    ekf.update_vision({pixel}, {observation.feature_identity}, camera_intrinsics());
}

void observe_pixels(Phase17ESKFEstimator& ekf, const std::vector<Eigen::Vector2d>& pixels,
                    const std::vector<Eigen::Vector3d>& identities) {
    ekf.update_vision(pixels, identities, camera_intrinsics());
}

void expect_vec_near(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected,
                     double tolerance) {
    EXPECT_NEAR(actual.x(), expected.x(), tolerance);
    EXPECT_NEAR(actual.y(), expected.y(), tolerance);
    EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

struct DeterministicResult {
    Eigen::Vector3d landmark{Eigen::Vector3d::Zero()};
    uint64_t attempts{0};
    uint64_t successes{0};
    uint64_t failures{0};
    uint64_t active_landmarks{0};
};

DeterministicResult run_deterministic_case() {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg(Phase21MsckfOptions{.minimum_observations = 3}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.3, 0.2, 4.2};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.15, 0.0, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.0, 0.12, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});

    DeterministicResult result;
    const auto landmark = ekf.triangulated_landmark_for_feature_for_test(feature);
    if (landmark.has_value()) {
        result.landmark = *landmark;
    }
    const auto diag = ekf.diagnostics();
    result.attempts = diag.triangulation_attempts;
    result.successes = diag.triangulation_successes;
    result.failures = diag.triangulation_failures;
    result.active_landmarks = diag.active_landmarks;
    return result;
}

} // namespace

TEST(Phase21TriangulationTest, SuccessfulTriangulationCreatesActiveLandmark) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg());
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.2, 0.1, 4.0};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    EXPECT_EQ(ekf.active_landmark_count_for_test(), 0u);

    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.25, 0.0, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});

    const auto landmark = ekf.triangulated_landmark_for_feature_for_test(feature);
    ASSERT_TRUE(landmark.has_value());
    if (!landmark.has_value()) {
        return;
    }
    expect_vec_near(*landmark, feature, 1.0e-2);
    const auto diag = ekf.diagnostics();
    EXPECT_EQ(diag.triangulation_attempts, 1u);
    EXPECT_EQ(diag.triangulation_successes, 1u);
    EXPECT_EQ(diag.triangulation_failures, 0u);
    EXPECT_EQ(diag.active_landmarks, 1u);
    EXPECT_EQ(diag.feature_tracks, 1u);
}

TEST(Phase21TriangulationTest, TwoViewTriangulationIsAccurate) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg());
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{-0.25, 0.15, 5.0};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.3, 0.0, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});

    const auto landmark = ekf.triangulated_landmark_for_feature_for_test(feature);
    ASSERT_TRUE(landmark.has_value());
    if (!landmark.has_value()) {
        return;
    }
    expect_vec_near(*landmark, feature, 1.0e-2);
}

TEST(Phase21TriangulationTest, MultiViewTriangulationIsAccurate) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg(Phase21MsckfOptions{.minimum_observations = 3}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.35, -0.2, 4.5};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.12, 0.0, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.0, 0.18, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});

    const auto landmark = ekf.triangulated_landmark_for_feature_for_test(feature);
    ASSERT_TRUE(landmark.has_value());
    if (!landmark.has_value()) {
        return;
    }
    expect_vec_near(*landmark, feature, 1.0e-2);
}

TEST(Phase21TriangulationTest, InsufficientObservationCountIsRejected) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg(Phase21MsckfOptions{.minimum_observations = 3}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.2, 0.1, 4.0};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});

    EXPECT_FALSE(ekf.triangulated_landmark_for_feature_for_test(feature).has_value());
    const auto diag = ekf.diagnostics();
    EXPECT_GE(diag.rejected_insufficient_observations, 1u);
    EXPECT_EQ(diag.triangulation_attempts, 0u);
}

TEST(Phase21TriangulationTest, InsufficientBaselineIsRejected) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg(Phase21MsckfOptions{.minimum_baseline = 0.75}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.1, 0.0, 4.0};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.05, 0.0, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});

    EXPECT_FALSE(ekf.triangulated_landmark_for_feature_for_test(feature).has_value());
    const auto diag = ekf.diagnostics();
    EXPECT_EQ(diag.active_landmarks, 0u);
    EXPECT_GE(diag.rejected_small_baseline, 1u);
    EXPECT_GE(diag.triangulation_failures, 1u);
}

TEST(Phase21TriangulationTest, NegativeDepthIsRejected) {
    Phase17ESKFEstimator ekf(make_permissive_vision_cfg());
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg());
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature_identity{0.1, 0.0, 4.0};
    const Eigen::Vector3d behind_camera{0.0, 0.0, -4.0};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(ekf, FeatureObservationSpec{.feature_identity = feature_identity,
                                                .projection_feature = behind_camera});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.25, 0.0, 0.0});
    observe_feature(ekf, FeatureObservationSpec{.feature_identity = feature_identity,
                                                .projection_feature = behind_camera});

    EXPECT_FALSE(ekf.triangulated_landmark_for_feature_for_test(feature_identity).has_value());
    const auto diag = ekf.diagnostics();
    EXPECT_GE(diag.rejected_negative_depth, 1u);
}

TEST(Phase21TriangulationTest, MaximumDepthIsRejected) {
    Phase17ESKFEstimator ekf(make_permissive_vision_cfg());
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg(
        Phase21MsckfOptions{.maximum_reprojection_error = 10.0, .maximum_depth = 5.0}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature_identity{0.1, 0.0, 4.0};
    const Eigen::Vector3d very_far_feature{0.1, 0.0, 30.0};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(ekf, FeatureObservationSpec{.feature_identity = feature_identity,
                                                .projection_feature = very_far_feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.4, 0.0, 0.0});
    observe_feature(ekf, FeatureObservationSpec{.feature_identity = feature_identity,
                                                .projection_feature = very_far_feature});

    EXPECT_FALSE(ekf.triangulated_landmark_for_feature_for_test(feature_identity).has_value());
    const auto diag = ekf.diagnostics();
    EXPECT_GE(diag.rejected_depth_range, 1u);
}

TEST(Phase21TriangulationTest, DegenerateGeometryIsRejected) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg(
        Phase21MsckfOptions{.minimum_baseline = 0.0, .maximum_reprojection_error = 10.0}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.0, 0.0, 4.0};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.0, 0.0, 0.3});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});

    EXPECT_FALSE(ekf.triangulated_landmark_for_feature_for_test(feature).has_value());
    const auto diag = ekf.diagnostics();
    EXPECT_GE(diag.rejected_degenerate_geometry, 1u);
}

TEST(Phase21TriangulationTest, NonFinitePixelInputIsRejected) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg());
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d valid_feature{0.2, 0.1, 4.0};
    const Eigen::Vector3d invalid_feature{-0.2, 0.1, 4.2};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_pixels(ekf,
                   {project_feature(ekf, valid_feature),
                    Eigen::Vector2d{std::numeric_limits<double>::quiet_NaN(), 240.0}},
                   {valid_feature, invalid_feature});

    const auto diag = ekf.diagnostics();
    EXPECT_GE(diag.rejected_non_finite_input, 1u);
    EXPECT_EQ(ekf.feature_track_count_for_test(), 1u);
}

TEST(Phase21TriangulationTest, MinimumDepthFailureIsRejected) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(
        make_phase21_msckf_cfg(Phase21MsckfOptions{.minimum_depth = 5.0, .maximum_depth = 10.0}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.1, 0.0, 4.0};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.3, 0.0, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});

    EXPECT_FALSE(ekf.triangulated_landmark_for_feature_for_test(feature).has_value());
    const auto diag = ekf.diagnostics();
    EXPECT_GE(diag.rejected_negative_depth, 1u);
}

TEST(Phase21TriangulationTest, ExcessiveReprojectionErrorIsRejected) {
    Phase17ESKFEstimator ekf(make_permissive_vision_cfg());
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg(
        Phase21MsckfOptions{.minimum_observations = 3, .maximum_reprojection_error = 0.5}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.0, 0.1, 4.5};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.25, 0.0, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.0, 0.2, 0.0});
    observe_feature(ekf, FeatureObservationSpec{.feature_identity = feature,
                                                .projection_feature = feature,
                                                .pixel_offset = Eigen::Vector2d{12.0, 0.0}});

    EXPECT_FALSE(ekf.triangulated_landmark_for_feature_for_test(feature).has_value());
    const auto diag = ekf.diagnostics();
    EXPECT_GE(diag.rejected_reprojection, 1u);
}

TEST(Phase21TriangulationTest, DisabledTriangulationPreservesShadowWindow) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(
        make_phase21_msckf_cfg(Phase21MsckfOptions{.triangulation_enabled = false}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.2, 0.1, 4.0};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.3, 0.0, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});

    const auto diag = ekf.diagnostics();
    EXPECT_FALSE(ekf.triangulated_landmark_for_feature_for_test(feature).has_value());
    EXPECT_EQ(diag.triangulation_attempts, 0u);
    EXPECT_EQ(diag.triangulation_successes, 0u);
    EXPECT_GE(ekf.msckf_state_ids_for_test().size(), 2u);
    EXPECT_EQ(ekf.feature_track_count_for_test(), 0u);
}

TEST(Phase21TriangulationTest, DiagnosticsDisabledSuppressesPublicationOnly) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg(Phase21MsckfOptions{.diagnostics_enabled = false}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.2, 0.1, 4.0};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.25, 0.0, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});

    ASSERT_TRUE(ekf.triangulated_landmark_for_feature_for_test(feature).has_value());
    EXPECT_EQ(ekf.feature_track_count_for_test(), 1u);
    EXPECT_EQ(ekf.active_landmark_count_for_test(), 1u);
    EXPECT_GE(ekf.msckf_state_ids_for_test().size(), 2u);

    const auto diag = ekf.diagnostics();
    EXPECT_EQ(diag.triangulation_attempts, 0u);
    EXPECT_EQ(diag.triangulation_successes, 0u);
    EXPECT_EQ(diag.active_landmarks, 0u);
    EXPECT_EQ(diag.feature_tracks, 0u);
}

TEST(Phase21TriangulationTest, FeatureTrackRemovalOccursOnWindowEviction) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg(
        Phase21MsckfOptions{.max_camera_states = 2, .minimum_observations = 3}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.1, 0.0, 4.0};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.15, 0.0, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    ASSERT_EQ(ekf.feature_track_count_for_test(), 1u);

    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.15, 0.0, 0.0});
    observe_pixels(ekf, {project_feature(ekf, Eigen::Vector3d{0.4, 0.0, 4.2})},
                   {Eigen::Vector3d{0.4, 0.0, 4.2}});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.15, 0.0, 0.0});
    observe_pixels(ekf, {project_feature(ekf, Eigen::Vector3d{0.5, 0.0, 4.1})},
                   {Eigen::Vector3d{0.5, 0.0, 4.1}});

    EXPECT_FALSE(ekf.feature_track_id_for_feature_for_test(feature).has_value());
    EXPECT_FALSE(ekf.triangulated_landmark_for_feature_for_test(feature).has_value());
    EXPECT_GE(ekf.diagnostics().feature_tracks_removed, 1u);
}

TEST(Phase21TriangulationTest, ResetClearsTracksAndLandmarks) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg());
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.2, -0.1, 4.5};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.25, 0.0, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    ASSERT_TRUE(ekf.triangulated_landmark_for_feature_for_test(feature).has_value());

    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    EXPECT_EQ(ekf.feature_track_count_for_test(), 0u);
    EXPECT_EQ(ekf.active_landmark_count_for_test(), 0u);
    EXPECT_TRUE(ekf.msckf_state_ids_for_test().empty());
    EXPECT_FALSE(ekf.triangulated_landmark_for_feature_for_test(feature).has_value());
}

TEST(Phase21TriangulationTest, EvictionRemovesStaleLandmarkState) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg(Phase21MsckfOptions{.max_camera_states = 2}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.2, 0.1, 4.0};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.25, 0.0, 0.0});
    observe_feature(
        ekf, FeatureObservationSpec{.feature_identity = feature, .projection_feature = feature});
    ASSERT_TRUE(ekf.triangulated_landmark_for_feature_for_test(feature).has_value());
    EXPECT_EQ(ekf.active_landmark_count_for_test(), 1u);

    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.25, 0.0, 0.0});
    observe_pixels(ekf, {project_feature(ekf, Eigen::Vector3d{0.6, 0.1, 4.0})},
                   {Eigen::Vector3d{0.6, 0.1, 4.0}});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.25, 0.0, 0.0});
    observe_pixels(ekf, {project_feature(ekf, Eigen::Vector3d{0.8, -0.1, 4.0})},
                   {Eigen::Vector3d{0.8, -0.1, 4.0}});

    EXPECT_FALSE(ekf.triangulated_landmark_for_feature_for_test(feature).has_value());
    EXPECT_EQ(ekf.active_landmark_count_for_test(), 0u);
}

TEST(Phase21TriangulationTest, FeatureAndLandmarkIdsRestartDeterministicallyAfterReset) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg());

    const Eigen::Vector3d feature{0.2, -0.15, 4.8};
    auto run_once = [&]() {
        ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
        double timestamp_s = 0.0;
        advance_frame(ekf, timestamp_s);
        observe_feature(ekf, FeatureObservationSpec{.feature_identity = feature,
                                                    .projection_feature = feature});
        const auto first_state_ids = ekf.msckf_state_ids_for_test();
        const auto first_track_id = ekf.feature_track_id_for_feature_for_test(feature);
        advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.22, 0.0, 0.0});
        observe_feature(ekf, FeatureObservationSpec{.feature_identity = feature,
                                                    .projection_feature = feature});
        const auto second_state_ids = ekf.msckf_state_ids_for_test();
        const auto landmark = ekf.triangulated_landmark_for_feature_for_test(feature);
        return std::tuple{first_state_ids, first_track_id, second_state_ids, landmark};
    };

    const auto [first_initial_states, first_track_id, first_states, first_landmark] = run_once();
    const auto [second_initial_states, second_track_id, second_states, second_landmark] =
        run_once();

    ASSERT_TRUE(first_track_id.has_value());
    ASSERT_TRUE(second_track_id.has_value());
    ASSERT_TRUE(first_landmark.has_value());
    ASSERT_TRUE(second_landmark.has_value());
    if (!first_track_id.has_value() || !second_track_id.has_value() ||
        !first_landmark.has_value() || !second_landmark.has_value()) {
        return;
    }
    EXPECT_EQ(first_initial_states, second_initial_states);
    EXPECT_EQ(first_states, second_states);
    EXPECT_EQ(*first_track_id, *second_track_id);
    expect_vec_near(*first_landmark, *second_landmark, 1.0e-12);
}

TEST(Phase21TriangulationTest, DeterministicResultsRepeatAcrossEquivalentRuns) {
    const auto first = run_deterministic_case();
    const auto second = run_deterministic_case();

    expect_vec_near(first.landmark, second.landmark, 1.0e-12);
    EXPECT_EQ(first.attempts, second.attempts);
    EXPECT_EQ(first.successes, second.successes);
    EXPECT_EQ(first.failures, second.failures);
    EXPECT_EQ(first.active_landmarks, second.active_landmarks);
}

TEST(Phase21TriangulationTest, FiniteLandmarkCoordinatesArePreserved) {
    const auto result = run_deterministic_case();
    EXPECT_TRUE(result.landmark.array().isFinite().all());
}

TEST(Phase21TriangulationTest, InvalidConfigurationFailsValidation) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg(Phase21MsckfOptions{.minimum_observations = 1}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    EXPECT_EQ(
        ekf.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero(), 0.0),
        EstimatorOperationResult::RejectedInvalidConfiguration);
}

TEST(Phase21TriangulationTest, RepeatedInitializationRemainsStable) {
    Phase17ESKFEstimator ekf;
    ekf.configure_validation(make_phase21_validation_cfg());
    ekf.configure_msckf(make_phase21_msckf_cfg());

    const Eigen::Vector3d feature{0.2, -0.15, 4.8};
    auto run_once = [&]() {
        ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
        double timestamp_s = 0.0;
        advance_frame(ekf, timestamp_s);
        observe_feature(ekf, FeatureObservationSpec{.feature_identity = feature,
                                                    .projection_feature = feature});
        advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.22, 0.0, 0.0});
        observe_feature(ekf, FeatureObservationSpec{.feature_identity = feature,
                                                    .projection_feature = feature});
        return ekf.triangulated_landmark_for_feature_for_test(feature);
    };

    const auto first = run_once();
    const auto second = run_once();
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    if (!first.has_value() || !second.has_value()) {
        return;
    }
    expect_vec_near(*first, *second, 1.0e-12);
    EXPECT_EQ(ekf.feature_track_count_for_test(), 1u);
    EXPECT_EQ(ekf.active_landmark_count_for_test(), 1u);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
