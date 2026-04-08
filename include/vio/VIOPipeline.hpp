// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once
 
// VIOPipeline.hpp    Visual-Inertial Odometry orchestrator
// Manages the EKF, feature tracking, and multi-sensor data flow
// Drone Swarm Sensor Fusion  |  Phase 2
 
#include "vio/EKFEstimator.hpp"
#include "sensors/IMUSensor.hpp"
#include "sensors/CameraSensor.hpp"
#include "sensors/LidarSensor.hpp"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <queue>
#include <string>
#include <thread>
#include <variant>

namespace drone::vio {

//  Variant message for sensor event queue 
using SensorEvent = std::variant<
    sensors::ImuMeasurement,
    sensors::CameraFrame,
    sensors::LidarMeasurement
>;

struct RuntimeTelemetry {
    double localization_confidence_trend{0.0};
    double sync_confidence{1.0};
    double imu_camera_offset_ms{0.0};
    double peer_clock_offset_ms{0.0};
    double occupancy_ratio{0.0};
    double anchor_visibility_ratio{0.0};
    double tdoa_weight{0.0};
    double tdoa_confidence{0.0};
    size_t relocalization_count{0};
    size_t visible_anchor_count{0};
    size_t planned_waypoint_count{0};
    uint64_t last_relocalized_keyframe{0};
    std::string localization_state{"nominal"};
    std::string localization_source{"vision-inertial"};
};

 
class VIOPipeline {
public:
    using PoseCallback = std::function<void(const PoseEstimate&)>;

    explicit VIOPipeline(EKFConfig cfg = {})
        : ekf_(cfg) {}

    ~VIOPipeline() { stop(); }

    //  Sensor registration 
    void attach_imu   (std::shared_ptr<sensors::IMUSensor>    imu);
    void attach_camera(std::shared_ptr<sensors::CameraSensor> cam);
    void attach_lidar (std::shared_ptr<sensors::LidarSensor>  lidar);

    //  Pipeline control 
    bool start();
    void stop();
    void reset();

    //  State query â”€
    [[nodiscard]] PoseEstimate  current_pose() const { return ekf_.state(); }
    [[nodiscard]] double        drift_m()      const { return ekf_.total_drift_m(); }
    [[nodiscard]] RuntimeTelemetry runtime_telemetry() const {
        std::lock_guard lock(runtime_mutex_);
        return runtime_telemetry_;
    }

    //  Output callback 
    void set_pose_callback(PoseCallback cb) { pose_cb_ = std::move(cb); }

    //  Configuration 
    void set_camera_matrix(const Eigen::Matrix3d& K) { K_ = K; }
    void set_runtime_telemetry(RuntimeTelemetry telemetry) {
        std::lock_guard lock(runtime_mutex_);
        runtime_telemetry_ = std::move(telemetry);
    }

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
    mutable std::mutex runtime_mutex_;
    RuntimeTelemetry runtime_telemetry_{};

    std::shared_ptr<spdlog::logger> logger_{spdlog::get("VIO")};
};

} // namespace drone::vio
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
