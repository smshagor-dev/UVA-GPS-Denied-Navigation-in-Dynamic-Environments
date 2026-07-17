// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

// test_ekf.cpp    GoogleTest suite for EKFEstimator

#include <gtest/gtest.h>
#include "vio/EKFEstimator.hpp"
#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <numbers>

using namespace drone::vio;

//  Helper: propagate N IMU samples at constant acceleration â”€
static PoseEstimate propagate_constant(EKFEstimator& ekf, const Eigen::Vector3d& accel,
                                       const Eigen::Vector3d& gyro, double dt, int steps) {
    for (int i = 0; i < steps; ++i)
        ekf.propagate_imu(accel, gyro, dt);
    return ekf.state();
}

class EKFTest : public ::testing::Test {
protected:
    void SetUp() override {
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
    EXPECT_NEAR(s.velocity.norm(), 0.0, 0.05);
}

TEST_F(EKFTest, LongStationaryPropagationRemainsFiniteAndNormalized) {
    const auto s = propagate_constant(
        ekf_, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero(), 0.0025, 4000);
    const auto diagnostics = ekf_.diagnostics();

    EXPECT_NEAR(s.position.norm(), 0.0, 0.1);
    EXPECT_NEAR(s.velocity.norm(), 0.0, 0.1);
    EXPECT_NEAR(s.orientation.norm(), 1.0, 1e-9);
    EXPECT_FALSE(diagnostics.has_non_finite_state);
    EXPECT_FALSE(diagnostics.has_non_finite_covariance);
}

//  6. Depth update should correct z position â”€
TEST_F(EKFTest, DepthUpdateCorrection) {
    // propagate briefly then check depth correction
    propagate_constant(ekf_, Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 0.0025, 100);

    const double true_depth = 0.12;
    ekf_.update_depth(true_depth, 0.02);
    auto s = ekf_.state();
    EXPECT_NEAR(s.position.z(), true_depth, 0.2);
}

TEST_F(EKFTest, DepthUpdateOnlyCorrectsForIndependentMeasurement) {
    propagate_constant(ekf_, Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 0.0025, 100);
    const auto before = ekf_.state();

    ekf_.update_depth(before.position.z(), 0.02);
    const auto unchanged = ekf_.state();
    EXPECT_NEAR((unchanged.position - before.position).norm(), 0.0, 1.0e-9);

    ekf_.update_depth(before.position.z() + 0.12, 0.05);
    const auto corrected = ekf_.state();
    EXPECT_GT(std::abs(corrected.position.z() - before.position.z()), 0.015);
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

TEST_F(EKFTest, TimestampGoingBackwardsIsRejected) {
    ekf_.propagate_imu(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 0.01);
    const auto before = ekf_.state();

    ekf_.propagate_imu(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), -0.01);
    const auto after = ekf_.state();
    const auto diagnostics = ekf_.diagnostics();

    EXPECT_TRUE(after.position.isApprox(before.position, 1e-12));
    EXPECT_EQ(diagnostics.timestamp_rejection_count, 1u);
}

TEST_F(EKFTest, ExcessiveImuTimeStepIsRejected) {
    const auto before = ekf_.state();
    ekf_.propagate_imu(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 0.2);
    const auto after = ekf_.state();
    const auto diagnostics = ekf_.diagnostics();

    EXPECT_TRUE(after.position.isApprox(before.position, 1e-12));
    EXPECT_EQ(diagnostics.timestamp_rejection_count, 1u);
}

TEST_F(EKFTest, NaNImuInputIsRejected) {
    const auto before = ekf_.state();
    ekf_.propagate_imu(
        Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 9.81},
        Eigen::Vector3d::Zero(), 0.01);
    const auto after = ekf_.state();
    const auto diagnostics = ekf_.diagnostics();

    EXPECT_TRUE(after.position.isApprox(before.position, 1e-12));
    EXPECT_EQ(diagnostics.invalid_input_count, 1u);
}

TEST_F(EKFTest, InfiniteMeasurementInputIsRejected) {
    const auto before = ekf_.state();
    ekf_.update_depth(std::numeric_limits<double>::infinity(), 0.02);
    const auto after = ekf_.state();
    const auto diagnostics = ekf_.diagnostics();

    EXPECT_TRUE(after.position.isApprox(before.position, 1e-12));
    EXPECT_EQ(diagnostics.invalid_input_count, 1u);
}

TEST_F(EKFTest, VisionOutlierRejectionLeavesStateUnchanged) {
    propagate_constant(ekf_, Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 0.0025, 200);
    const auto before = ekf_.state();
    ekf_.update_visual_pose(before.position + Eigen::Vector3d{50.0, -50.0, 20.0},
                            Eigen::Vector3d{30.0, 0.0, 0.0}, 0.05, 0.05);
    const auto after = ekf_.state();
    const auto diagnostics = ekf_.diagnostics();

    EXPECT_TRUE(after.position.isApprox(before.position, 1e-9));
    EXPECT_TRUE(after.velocity.isApprox(before.velocity, 1e-9));
    EXPECT_GE(diagnostics.rejected_updates[static_cast<size_t>(EstimatorSensorType::VISUAL_POSE)],
              1u);
}

TEST_F(EKFTest, DepthMeasurementOutlierIsRejected) {
    const auto before = ekf_.state();
    ekf_.update_depth(500.0, 0.01);
    const auto after = ekf_.state();
    const auto diagnostics = ekf_.diagnostics();

    EXPECT_TRUE(after.position.isApprox(before.position, 1e-9));
    EXPECT_GE(diagnostics.rejected_updates[static_cast<size_t>(EstimatorSensorType::DEPTH)], 1u);
}

TEST_F(EKFTest, ZuptPreservesPositionAttitudeAndBiases) {
    ekf_.reset(Eigen::Vector3d{1.0, 2.0, 3.0},
               Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())),
               Eigen::Vector3d{0.4, -0.2, 0.1});
    const auto before = ekf_.state();

    ekf_.update_zupt();
    const auto after = ekf_.state();
    const auto diagnostics = ekf_.diagnostics();

    EXPECT_TRUE(after.position.isApprox(before.position, 1e-9));
    EXPECT_TRUE(after.orientation.coeffs().isApprox(before.orientation.coeffs(), 1e-9));
    EXPECT_TRUE(after.accel_bias.isApprox(before.accel_bias, 1e-9));
    EXPECT_TRUE(after.gyro_bias.isApprox(before.gyro_bias, 1e-9));
    EXPECT_LT(after.velocity.norm(), before.velocity.norm());
    EXPECT_LT(diagnostics.covariance_symmetry_error, 1e-9);
}

TEST_F(EKFTest, CovarianceSymmetryHoldsAfterRepeatedUpdates) {
    Eigen::Matrix3d K;
    K << 800, 0, 320, 0, 800, 240, 0, 0, 1;
    std::vector<Eigen::Vector2d> z{{320, 240}};
    std::vector<Eigen::Vector3d> pts{{0.0, 0.0, 5.0}};

    for (int i = 0; i < 100; ++i) {
        ekf_.propagate_imu(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d{0.0, 0.0, 0.01}, 0.01);
        ekf_.update_vision(z, pts, K);
        ekf_.update_depth(0.0, 0.05);
    }

    const auto diagnostics = ekf_.diagnostics();
    EXPECT_LT(diagnostics.covariance_symmetry_error, 1e-8);
    EXPECT_GT(diagnostics.covariance_min_diagonal, 0.0);
}

TEST_F(EKFTest, CovarianceRemainsFiniteDuringLongPropagation) {
    for (int i = 0; i < 20000; ++i) {
        ekf_.propagate_imu(Eigen::Vector3d{0.01, -0.02, 9.81},
                           Eigen::Vector3d{0.001, 0.002, -0.003}, 0.0025);
    }
    const auto diagnostics = ekf_.diagnostics();
    EXPECT_FALSE(diagnostics.has_non_finite_covariance);
    EXPECT_FALSE(diagnostics.has_non_finite_state);
}

TEST_F(EKFTest, DeterministicOutputForIdenticalReplayInput) {
    EKFEstimator ekf_a;
    EKFEstimator ekf_b;
    ekf_a.reset();
    ekf_b.reset();

    for (int i = 0; i < 400; ++i) {
        const Eigen::Vector3d accel{0.02, 0.01, 9.81};
        const Eigen::Vector3d gyro{0.0, 0.0, 0.01};
        ekf_a.propagate_imu(accel, gyro, 0.0025);
        ekf_b.propagate_imu(accel, gyro, 0.0025);
    }
    ekf_a.update_visual_pose(Eigen::Vector3d{0.3, 0.1, 0.0}, Eigen::Vector3d{0.1, 0.0, 0.0});
    ekf_b.update_visual_pose(Eigen::Vector3d{0.3, 0.1, 0.0}, Eigen::Vector3d{0.1, 0.0, 0.0});
    ekf_a.update_depth(0.0, 0.05);
    ekf_b.update_depth(0.0, 0.05);

    const auto a = ekf_a.state();
    const auto b = ekf_b.state();
    EXPECT_TRUE(a.position.isApprox(b.position, 1e-12));
    EXPECT_TRUE(a.velocity.isApprox(b.velocity, 1e-12));
    EXPECT_TRUE(a.orientation.coeffs().isApprox(b.orientation.coeffs(), 1e-12));
}

TEST_F(EKFTest, InvalidMeasurementDoesNotModifyState) {
    const auto before = ekf_.state();
    ekf_.update_visual_pose(Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
                            Eigen::Vector3d::Zero(), 0.1, 0.1);
    const auto after = ekf_.state();
    EXPECT_TRUE(after.position.isApprox(before.position, 1e-12));
    EXPECT_TRUE(after.velocity.isApprox(before.velocity, 1e-12));
}

TEST_F(EKFTest, ResetClearsEstimatorHistoryAndDiagnostics) {
    ekf_.propagate_imu(Eigen::Vector3d{0, 0, 9.81}, Eigen::Vector3d::Zero(), 0.01);
    ekf_.update_depth(0.2, 0.05);
    ASSERT_GT(ekf_.diagnostics().propagation_count, 0u);

    ekf_.reset();
    const auto state = ekf_.state();
    const auto diagnostics = ekf_.diagnostics();

    EXPECT_TRUE(state.position.isZero(1e-12));
    EXPECT_EQ(diagnostics.propagation_count, 0u);
    EXPECT_EQ(diagnostics.invalid_input_count, 0u);
    EXPECT_EQ(diagnostics.timestamp_rejection_count, 0u);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
