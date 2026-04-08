// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// EKFEstimator.hpp  â€”  Extended Kalman Filter for Visual-Inertial Odometry
// State: [pos(3), vel(3), quat(4), ba(3), bg(3)] = 16-dim
// Drone Swarm Sensor Fusion  |  Phase 2 â€” VIO Pipeline
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <mutex>
#include <numbers>
#include <optional>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace drone::vio {

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// State vector layout  (16-dim)
//   [0:2]  position      p  (m, world frame)
//   [3:5]  velocity      v  (m/s, world frame)
//   [6:9]  quaternion    q  (w,x,y,z  â€” worldâ†body)
//   [10:12] accel bias   ba (m/sÂ²)
//   [13:15] gyro bias    bg (rad/s)
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
constexpr int kStateDim    = 16;
constexpr int kErrorDim    = 15;  // error-state (quaternion â†’ 3-param)
constexpr int kImuNoiseDim = 12;  // na, ng, nba, nbg (3 each)

using StateVec     = Eigen::Matrix<double, kStateDim,    1>;
using ErrorVec     = Eigen::Matrix<double, kErrorDim,    1>;
using CovMat       = Eigen::Matrix<double, kErrorDim, kErrorDim>;
using FMat         = Eigen::Matrix<double, kErrorDim, kErrorDim>;
using GMat         = Eigen::Matrix<double, kErrorDim, kImuNoiseDim>;
using QNoiseMat    = Eigen::Matrix<double, kImuNoiseDim, kImuNoiseDim>;

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
struct EKFConfig {
    // IMU noise parameters
    double sigma_na{0.02};     // accel noise  (m/sÂ²)
    double sigma_ng{0.005};    // gyro  noise  (rad/s)
    double sigma_nba{1.0e-4};  // accel bias walk
    double sigma_nbg{1.0e-5};  // gyro  bias walk

    // Vision noise
    double sigma_px{1.5};      // pixel reprojection noise
    double sigma_pz{0.05};     // depth measurement noise (if available)

    // Initial covariance
    double init_pos_std{0.1};
    double init_vel_std{0.05};
    double init_att_std{0.05};
    double init_ba_std{0.01};
    double init_bg_std{0.001};

    // Outlier rejection gate (chiÂ² threshold, 3 dof)
    double mahal_gate{7.815};
};

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// Pose output
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
struct PoseEstimate {
    double           timestamp{0.0};
    Eigen::Vector3d  position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d  velocity{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d  accel_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d  gyro_bias{Eigen::Vector3d::Zero()};

    // Uncertainty (diagonal of position block)
    Eigen::Vector3d pos_std{Eigen::Vector3d::Ones() * 0.1};

    // Derived
    [[nodiscard]] Eigen::Matrix3d R_wb() const { return orientation.toRotationMatrix(); }
    [[nodiscard]] Eigen::Vector3d euler_zyx_deg() const {
        return orientation.toRotationMatrix().eulerAngles(2,1,0) *
               (180.0 / std::numbers::pi_v<double>);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
class EKFEstimator {
public:
    explicit EKFEstimator(EKFConfig cfg = {});

    // â”€â”€ Initialize with known pose â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    void reset(const Eigen::Vector3d& p0       = Eigen::Vector3d::Zero(),
               const Eigen::Quaterniond& q0    = Eigen::Quaterniond::Identity(),
               const Eigen::Vector3d& v0       = Eigen::Vector3d::Zero());

    // â”€â”€ IMU propagation â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //   Call at IMU rate (~400 Hz).  dt in seconds.
    void propagate_imu(const Eigen::Vector3d& accel_mps2,
                       const Eigen::Vector3d& gyro_rads,
                       double dt);

    // â”€â”€ Camera update (feature-based) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //   z_pixels: Nx2 observed pixel coordinates
    //   p_world:  Nx3 corresponding 3-D map points
    //   K:        3x3 camera intrinsic matrix
    void update_vision(const std::vector<Eigen::Vector2d>& z_pixels,
                       const std::vector<Eigen::Vector3d>& p_world,
                       const Eigen::Matrix3d& K);

    // â”€â”€ Depth update (from LiDAR plane fit) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    void update_depth(double z_depth_m, double sigma_m = 0.05);

    // â”€â”€ Zero velocity update (on ground detection) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    void update_zupt();

    // â”€â”€ Query â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    [[nodiscard]] PoseEstimate state() const;
    [[nodiscard]] double       total_drift_m() const { return total_drift_; }
    [[nodiscard]] bool         is_initialized() const { return initialized_; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // â”€â”€ Core EKF math â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    FMat compute_F(const Eigen::Vector3d& accel_body,
                   const Eigen::Matrix3d& R_wb,
                   double dt) const;

    GMat compute_G(const Eigen::Matrix3d& R_wb) const;

    // Quaternion kinematics
    static Eigen::Quaterniond propagate_quat(const Eigen::Quaterniond& q,
                                              const Eigen::Vector3d& omega,
                                              double dt);
    // Box-plus / box-minus for error-state
    static Eigen::Vector3d quat_to_rotvec(const Eigen::Quaterniond& q);
    static Eigen::Quaterniond rotvec_to_quat(const Eigen::Vector3d& rv);

    // Nominal state
    Eigen::Vector3d    pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d    vel_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d    ba_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d    bg_{Eigen::Vector3d::Zero()};

    // Error-state covariance
    CovMat P_{CovMat::Identity() * 0.01};

    // IMU noise matrix (constant)
    QNoiseMat Q_imu_{QNoiseMat::Zero()};

    double timestamp_{0.0};
    double total_drift_{0.0};
    bool   initialized_{false};

    EKFConfig cfg_;
    mutable std::mutex mtx_;

    std::shared_ptr<spdlog::logger> logger_{spdlog::get("EKF")};
};

} // namespace drone::vio
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
