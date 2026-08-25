// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

// test_ekf.cpp    GoogleTest suite for EKFEstimator

#include <gtest/gtest.h>
#include "vio/EKFEstimator.hpp"
#include <Eigen/Core>
#include <cmath>
#include <numbers>

using namespace drone::vio;

//  Helper: propagate N IMU samples at constant acceleration â”€
static PoseEstimate propagate_constant(EKFEstimator& ekf, const Eigen::Vector3d& accel,
                                       const Eigen::Vector3d& gyro, double dt, int steps) {
    for (int i = 0; i < steps; ++i)
        ekf.propagate_imu(accel, gyro, dt);
    return ekf.state();
}

static EstimatorValidationConfig make_validation_config() {
    EstimatorValidationConfig cfg;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    cfg.lidar_depth_correction_enabled = false;
    return cfg;
}

static void feed_timestamped_imu(EKFEstimator& ekf, const Eigen::Vector3d& accel,
                                 const Eigen::Vector3d& gyro, double dt, int steps,
                                 double start_timestamp = 0.0) {
    double timestamp = start_timestamp;
    for (int i = 0; i < steps; ++i) {
        (void)ekf.process_imu_measurement(accel, gyro, timestamp);
        timestamp += dt;
    }
}

class EKFTest : public ::testing::Test {
protected:
    void SetUp() override {
        ekf_.configure_validation(make_validation_config());
        ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                   Eigen::Vector3d::Zero());
    }
    EKFEstimator ekf_;
};

//  1. After reset, state should be identity/zero â”€
TEST_F(EKFTest, ResetYieldsZeroState) {
    auto s = ekf_.state();
    EXPECT_TRUE(s.position.isZero(1e-10));
    EXPECT_TRUE(s.velocity.isZero(1e-10));
    // Quaternion should be identity
    EXPECT_NEAR(s.orientation.w(), 1.0, 1e-10);
    EXPECT_NEAR(s.orientation.x(), 0.0, 1e-10);
}

//  2. Free-fall: 1 second of gravity should give ~4.9 m/s downward
TEST_F(EKFTest, FreeFallVelocity) {
    // In body frame, gravity sensed as +9.81 upward (reaction force)
    // For a truly free-falling IMU the accel reading is zero.
    const Eigen::Vector3d accel_freefall{0, 0, 0};
    const Eigen::Vector3d gyro_zero{0, 0, 0};
    const double dt = 0.0025; // 400 Hz
    const int steps = 400;    // 1 second

    auto s = propagate_constant(ekf_, accel_freefall, gyro_zero, dt, steps);
    // velocity in -z should be ~9.81 m/s (gravity accumulation)
    EXPECT_NEAR(s.velocity.z(), -9.81, 0.05);
}

//  3. Constant velocity: no accel (minus gravity compensation)
TEST_F(EKFTest, ConstantVelocityPositionGrowth) {
    // Stationary on ground: accel â‰ˆ [0,0,+g] in world frame
    const Eigen::Vector3d accel_static{0, 0, 9.81};
    const Eigen::Vector3d gyro_zero{0, 0, 0};
    const double dt = 0.0025;
    const int steps = 400; // 1 second

    auto s = propagate_constant(ekf_, accel_static, gyro_zero, dt, steps);
    EXPECT_NEAR(s.velocity.norm(), 0.0, 0.05);
    EXPECT_NEAR(s.position.norm(), 0.0, 0.05);
}

//  4. Pure rotation about Z axis â”€
TEST_F(EKFTest, PureYawRotation) {
    const double yaw_rate_rads = std::numbers::pi_v<double> / 4.0; // 45 deg/s
    const Eigen::Vector3d accel_static{0, 0, 9.81};
    const Eigen::Vector3d gyro{0, 0, yaw_rate_rads};
    const double dt = 0.0025;
    const int steps = 400; // 1 second  45 degrees

    auto s = propagate_constant(ekf_, accel_static, gyro, dt, steps);
    const auto euler = s.euler_zyx_deg();
    EXPECT_NEAR(euler(0), 45.0, 1.0); // yaw
    EXPECT_NEAR(euler(1), 0.0, 1.0);  // pitch
}

//  5. ZUPT should drive velocity to zero
TEST_F(EKFTest, ZUPTClearsVelocity) {
    // Give it some velocity first
    ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
               Eigen::Vector3d{1.0, 0.5, 0.0});

    ekf_.update_zupt();
    auto s = ekf_.state();
    EXPECT_NEAR(s.velocity.norm(), 0.0, 0.01);
}

TEST_F(EKFTest, DepthUpdateDisabledByDefault) {
    // propagate briefly then check depth correction
    propagate_constant(ekf_, Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 0.0025, 100);

    const auto before = ekf_.state();
    ekf_.update_depth(5.0, 0.02);
    auto s = ekf_.state();
    const auto diag = ekf_.diagnostics();
    EXPECT_NEAR(s.position.z(), before.position.z(), 1.0e-12);
    EXPECT_EQ(diag.disabled_lidar_correction_count, 1u);
    EXPECT_EQ(diag.last_operation_result, EstimatorOperationResult::RejectedUnsupportedMeasurement);
}

TEST_F(EKFTest, DepthUpdateCanBeExplicitlyEnabled) {
    EstimatorValidationConfig cfg = make_validation_config();
    cfg.lidar_depth_correction_enabled = true;
    ekf_.configure_validation(cfg);

    ekf_.update_depth(5.0, 0.02);
    auto s = ekf_.state();
    EXPECT_NEAR(s.position.z(), 5.0, 0.2);
}

//  7. Covariance must remain symmetric positive-definite
TEST_F(EKFTest, CovarianceRemainsValid) {
    propagate_constant(ekf_, Eigen::Vector3d{0.1, 0.2, 9.81}, Eigen::Vector3d{0.01, -0.01, 0.005},
                       0.0025, 1000);

    // We can't access P_ directly, but we validate via pos_std > 0
    auto s = ekf_.state();
    for (int i = 0; i < 3; ++i)
        EXPECT_GT(s.pos_std(i), 0.0) << "pos_std[" << i << "] <= 0";
}

//  8. Vision update with known point should reduce position uncertainty
TEST_F(EKFTest, VisionUpdateReducesUncertainty) {
    propagate_constant(ekf_, Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 0.0025, 400);

    auto s_before = ekf_.state();

    // Place a map point 5m ahead and compute its pixel projection
    Eigen::Matrix3d K;
    K << 800, 0, 320, 0, 800, 240, 0, 0, 1;

    Eigen::Vector3d p_world{0, 0, 5.0}; // 5m in front (along +Z in world)
    // projected pixel (approx center for zero-pose drone)
    std::vector<Eigen::Vector2d> z{{320, 240}};
    std::vector<Eigen::Vector3d> pts{p_world};

    ekf_.update_vision(z, pts, K);
    auto s_after = ekf_.state();

    // Uncertainty should not increase
    EXPECT_LE(s_after.pos_std.sum(), s_before.pos_std.sum() + 1e-6);
}

TEST_F(EKFTest, VisualPoseUpdateChangesState) {
    propagate_constant(ekf_, Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 0.0025, 200);

    const auto before = ekf_.state();
    ekf_.update_visual_pose(before.position + Eigen::Vector3d{0.4, -0.2, 0.1},
                            Eigen::Vector3d{0.2, 0.0, 0.0}, 0.15, 0.2);
    const auto after = ekf_.state();

    EXPECT_GT((after.position - before.position).norm(), 1.0e-3);
    EXPECT_GT((after.velocity - before.velocity).norm(), 1.0e-3);
}

TEST_F(EKFTest, DuplicateTimestampDoesNotMutateState) {
    const auto accepted =
        ekf_.process_imu_measurement(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 1.0);
    ASSERT_EQ(accepted, EstimatorOperationResult::Accepted);
    const auto before = ekf_.state();

    const auto result =
        ekf_.process_imu_measurement(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 1.0);
    const auto after = ekf_.state();
    const auto diag = ekf_.diagnostics();

    EXPECT_EQ(result, EstimatorOperationResult::RejectedDuplicateTimestamp);
    EXPECT_NEAR((after.position - before.position).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((after.velocity - before.velocity).norm(), 0.0, 1.0e-12);
    EXPECT_EQ(diag.duplicate_timestamp_count, 1u);
}

TEST_F(EKFTest, BackwardTimestampDoesNotMutateState) {
    (void)ekf_.process_imu_measurement(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 1.0);
    (void)ekf_.process_imu_measurement(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 1.01);
    const auto before = ekf_.state();

    const auto result =
        ekf_.process_imu_measurement(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 0.99);
    const auto after = ekf_.state();
    const auto diag = ekf_.diagnostics();

    EXPECT_EQ(result, EstimatorOperationResult::RejectedBackwardTimestamp);
    EXPECT_NEAR((after.position - before.position).norm(), 0.0, 1.0e-12);
    EXPECT_EQ(diag.backward_timestamp_count, 1u);
}

TEST_F(EKFTest, OversizedTimeStepRejected) {
    (void)ekf_.process_imu_measurement(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 1.0);
    const auto before = ekf_.state();

    const auto result =
        ekf_.process_imu_measurement(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 2.0);
    const auto after = ekf_.state();

    EXPECT_EQ(result, EstimatorOperationResult::RejectedTimeStepTooLarge);
    EXPECT_NEAR((after.position - before.position).norm(), 0.0, 1.0e-12);
}

TEST_F(EKFTest, NonFiniteMeasurementRejected) {
    (void)ekf_.process_imu_measurement(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 1.0);
    const auto before = ekf_.state();

    const auto result = ekf_.process_imu_measurement(
        Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 9.81},
        Eigen::Vector3d::Zero(), 1.01);
    const auto after = ekf_.state();
    const auto diag = ekf_.diagnostics();

    EXPECT_EQ(result, EstimatorOperationResult::RejectedNonFiniteInput);
    EXPECT_NEAR((after.position - before.position).norm(), 0.0, 1.0e-12);
    EXPECT_EQ(diag.non_finite_input_count, 1u);
}

TEST_F(EKFTest, InvalidInitialQuaternionIsRejected) {
    EKFEstimator local;
    local.configure_validation(make_validation_config());
    local.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond{0.0, 0.0, 0.0, 0.0},
                Eigen::Vector3d::Zero());
    EXPECT_FALSE(local.is_initialized());
    EXPECT_EQ(local.diagnostics().last_operation_result,
              EstimatorOperationResult::RejectedInvalidQuaternion);
}

TEST_F(EKFTest, TimestampedPropagationMatchesDeterministicExpectation) {
    feed_timestamped_imu(ekf_, Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 0.0025, 401,
                         10.0);
    const auto s = ekf_.state();
    EXPECT_NEAR(s.velocity.norm(), 0.0, 0.05);
    EXPECT_NEAR(s.position.norm(), 0.0, 0.05);
}

TEST_F(EKFTest, RepeatedZUPTKeepsCovarianceFiniteAndSymmetric) {
    propagate_constant(ekf_, Eigen::Vector3d{0.2, 0.0, 9.81}, Eigen::Vector3d::Zero(), 0.0025, 200);
    for (int i = 0; i < 50; ++i) {
        ekf_.update_zupt();
    }

    const auto cov = ekf_.covariance();
    EXPECT_TRUE(cov.array().isFinite().all());
    EXPECT_LT((cov - cov.transpose()).cwiseAbs().maxCoeff(), 1.0e-8);
    EXPECT_EQ(ekf_.diagnostics().zupt_accepted_count, 50u);
}

TEST_F(EKFTest, LongDurationPropagationRemainsFinite) {
    feed_timestamped_imu(ekf_, Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d{0.0, 0.0, 0.001},
                         0.0025, 4001, 1.0);
    const auto s = ekf_.state();
    const auto cov = ekf_.covariance();
    EXPECT_TRUE(s.position.array().isFinite().all());
    EXPECT_TRUE(s.velocity.array().isFinite().all());
    EXPECT_TRUE(std::isfinite(s.orientation.norm()));
    EXPECT_TRUE(cov.array().isFinite().all());
    EXPECT_LT((cov - cov.transpose()).cwiseAbs().maxCoeff(), 1.0e-8);
}

TEST_F(EKFTest, DiagnosticsTrackAcceptedAndRejectedOperations) {
    (void)ekf_.process_imu_measurement(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 1.0);
    (void)ekf_.process_imu_measurement(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 1.01);
    (void)ekf_.process_imu_measurement(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 1.01);
    ekf_.update_depth(3.0, 0.05);

    const auto diag = ekf_.diagnostics();
    EXPECT_EQ(diag.accepted_propagation_count, 1u);
    EXPECT_EQ(diag.duplicate_timestamp_count, 1u);
    EXPECT_EQ(diag.disabled_lidar_correction_count, 1u);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
