// ─────────────────────────────────────────────────────────────────────────────
// test_ekf.cpp  —  GoogleTest suite for EKFEstimator
// ─────────────────────────────────────────────────────────────────────────────
#include <gtest/gtest.h>
#include "vio/EKFEstimator.hpp"
#include <Eigen/Core>
#include <cmath>

using namespace drone::vio;

// ─── Helper: propagate N IMU samples at constant acceleration ────────────────
static PoseEstimate propagate_constant(
        EKFEstimator& ekf,
        const Eigen::Vector3d& accel,
        const Eigen::Vector3d& gyro,
        double dt, int steps) {
    for (int i = 0; i < steps; ++i)
        ekf.propagate_imu(accel, gyro, dt);
    return ekf.state();
}

// ─────────────────────────────────────────────────────────────────────────────
class EKFTest : public ::testing::Test {
protected:
    void SetUp() override {
        ekf_.reset(Eigen::Vector3d::Zero(),
                   Eigen::Quaterniond::Identity(),
                   Eigen::Vector3d::Zero());
    }
    EKFEstimator ekf_;
};

// ── 1. After reset, state should be identity/zero ─────────────────────────
TEST_F(EKFTest, ResetYieldsZeroState) {
    auto s = ekf_.state();
    EXPECT_TRUE(s.position.isZero(1e-10));
    EXPECT_TRUE(s.velocity.isZero(1e-10));
    // Quaternion should be identity
    EXPECT_NEAR(s.orientation.w(), 1.0, 1e-10);
    EXPECT_NEAR(s.orientation.x(), 0.0, 1e-10);
}

// ── 2. Free-fall: 1 second of gravity should give ~4.9 m/s downward ──────
TEST_F(EKFTest, FreeFallVelocity) {
    // In body frame, gravity sensed as +9.81 upward (reaction force)
    // For a truly free-falling IMU the accel reading is zero.
    const Eigen::Vector3d accel_freefall{0, 0, 0};
    const Eigen::Vector3d gyro_zero{0, 0, 0};
    const double dt    = 0.0025;   // 400 Hz
    const int    steps = 400;      // 1 second

    auto s = propagate_constant(ekf_, accel_freefall, gyro_zero, dt, steps);
    // velocity in -z should be ~9.81 m/s (gravity accumulation)
    EXPECT_NEAR(s.velocity.z(), -9.81, 0.05);
}

// ── 3. Constant velocity: no accel (minus gravity compensation) ───────────
TEST_F(EKFTest, ConstantVelocityPositionGrowth) {
    // Stationary on ground: accel ≈ [0,0,+g] in world frame
    const Eigen::Vector3d accel_static{0, 0, 9.81};
    const Eigen::Vector3d gyro_zero{0, 0, 0};
    const double dt = 0.0025;
    const int steps = 400;  // 1 second

    auto s = propagate_constant(ekf_, accel_static, gyro_zero, dt, steps);
    EXPECT_NEAR(s.velocity.norm(), 0.0, 0.05);
    EXPECT_NEAR(s.position.norm(), 0.0, 0.05);
}

// ── 4. Pure rotation about Z axis ────────────────────────────────────────
TEST_F(EKFTest, PureYawRotation) {
    const double yaw_rate_rads = M_PI / 4.0;  // 45 deg/s
    const Eigen::Vector3d accel_static{0, 0, 9.81};
    const Eigen::Vector3d gyro{0, 0, yaw_rate_rads};
    const double dt = 0.0025;
    const int    steps = 400;  // 1 second → 45 degrees

    auto s = propagate_constant(ekf_, accel_static, gyro, dt, steps);
    const auto euler = s.euler_zyx_deg();
    EXPECT_NEAR(euler(0), 45.0, 1.0);  // yaw
    EXPECT_NEAR(euler(1),  0.0, 1.0);  // pitch
}

// ── 5. ZUPT should drive velocity to zero ────────────────────────────────
TEST_F(EKFTest, ZUPTClearsVelocity) {
    // Give it some velocity first
    ekf_.reset(Eigen::Vector3d::Zero(),
               Eigen::Quaterniond::Identity(),
               Eigen::Vector3d{1.0, 0.5, 0.0});

    ekf_.update_zupt();
    auto s = ekf_.state();
    EXPECT_NEAR(s.velocity.norm(), 0.0, 0.01);
}

// ── 6. Depth update should correct z position ────────────────────────────
TEST_F(EKFTest, DepthUpdateCorrection) {
    // propagate briefly then check depth correction
    propagate_constant(ekf_,
        Eigen::Vector3d{0,0,9.81}, Eigen::Vector3d::Zero(),
        0.0025, 100);

    const double true_depth = 5.0;
    ekf_.update_depth(true_depth, 0.02);
    auto s = ekf_.state();
    EXPECT_NEAR(s.position.z(), true_depth, 0.2);
}

// ── 7. Covariance must remain symmetric positive-definite ─────────────────
TEST_F(EKFTest, CovarianceRemainsValid) {
    propagate_constant(ekf_,
        Eigen::Vector3d{0.1, 0.2, 9.81},
        Eigen::Vector3d{0.01, -0.01, 0.005},
        0.0025, 1000);

    // We can't access P_ directly, but we validate via pos_std > 0
    auto s = ekf_.state();
    for (int i = 0; i < 3; ++i)
        EXPECT_GT(s.pos_std(i), 0.0) << "pos_std[" << i << "] <= 0";
}

// ── 8. Vision update with known point should reduce position uncertainty ──
TEST_F(EKFTest, VisionUpdateReducesUncertainty) {
    propagate_constant(ekf_,
        Eigen::Vector3d{0,0,9.81}, Eigen::Vector3d::Zero(),
        0.0025, 400);

    auto s_before = ekf_.state();

    // Place a map point 5m ahead and compute its pixel projection
    Eigen::Matrix3d K;
    K << 800, 0, 320,
         0, 800, 240,
         0, 0, 1;

    Eigen::Vector3d p_world{0, 0, 5.0};  // 5m in front (along +Z in world)
    // projected pixel (approx center for zero-pose drone)
    std::vector<Eigen::Vector2d> z{{320, 240}};
    std::vector<Eigen::Vector3d> pts{p_world};

    ekf_.update_vision(z, pts, K);
    auto s_after = ekf_.state();

    // Uncertainty should not increase
    EXPECT_LE(s_after.pos_std.sum(), s_before.pos_std.sum() + 1e-6);
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
