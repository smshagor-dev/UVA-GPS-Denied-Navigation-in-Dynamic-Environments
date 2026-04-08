// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// VIOPipeline.hpp  â€”  Visual-Inertial Odometry orchestrator
// Manages the EKF, feature tracking, and multi-sensor data flow
// Drone Swarm Sensor Fusion  |  Phase 2
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
#include "vio/EKFEstimator.hpp"
#include "sensors/IMUSensor.hpp"
#include "sensors/CameraSensor.hpp"
#include "sensors/LidarSensor.hpp"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <queue>
#include <thread>
#include <variant>

namespace drone::vio {

// â”€â”€â”€ Variant message for sensor event queue â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
using SensorEvent = std::variant<
    sensors::ImuMeasurement,
    sensors::CameraFrame,
    sensors::LidarMeasurement
>;

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
class VIOPipeline {
public:
    using PoseCallback = std::function<void(const PoseEstimate&)>;

    explicit VIOPipeline(EKFConfig cfg = {})
        : ekf_(cfg) {}

    ~VIOPipeline() { stop(); }

    // â”€â”€ Sensor registration â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    void attach_imu   (std::shared_ptr<sensors::IMUSensor>    imu);
    void attach_camera(std::shared_ptr<sensors::CameraSensor> cam);
    void attach_lidar (std::shared_ptr<sensors::LidarSensor>  lidar);

    // â”€â”€ Pipeline control â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    bool start();
    void stop();
    void reset();

    // â”€â”€ State query â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    [[nodiscard]] PoseEstimate  current_pose() const { return ekf_.state(); }
    [[nodiscard]] double        drift_m()      const { return ekf_.total_drift_m(); }

    // â”€â”€ Output callback â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    void set_pose_callback(PoseCallback cb) { pose_cb_ = std::move(cb); }

    // â”€â”€ Configuration â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    void set_camera_matrix(const Eigen::Matrix3d& K) { K_ = K; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    void enqueue(SensorEvent evt);
    void processing_loop();

    void handle(const sensors::ImuMeasurement& imu);
    void handle(const sensors::CameraFrame& frame);
    void handle(const sensors::LidarMeasurement&);

    EKFEstimator ekf_;

    std::shared_ptr<sensors::IMUSensor>    imu_;
    std::shared_ptr<sensors::CameraSensor> cam_;
    std::shared_ptr<sensors::LidarSensor>  lidar_;

    // Thread-safe event queue
    std::queue<SensorEvent>  event_queue_;
    mutable std::mutex       queue_mutex_;
    std::condition_variable  queue_cv_;
    std::thread              proc_thread_;
    std::atomic<bool>        running_{false};

    Eigen::Matrix3d  K_{Eigen::Matrix3d::Identity()};
    PoseCallback     pose_cb_;
    double           last_imu_ts_{-1.0};

    std::shared_ptr<spdlog::logger> logger_{spdlog::get("VIO")};
};

} // namespace drone::vio
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
