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
#include "runtime/RuntimeMode.hpp"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <opencv2/core.hpp>
#include <queue>
#include <string>
#include <thread>
#include <variant>
#include <vector>

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
    size_t peer_count{0};
    size_t stale_peer_count{0};
    std::string local_consensus_state{"single_node"};
    uint64_t local_consensus_epoch{0};
    double mesh_bandwidth_kbps{0.0};
    std::string edge_serialization_mode{"json"};
    double edge_average_packet_size_bytes{0.0};
    double edge_bandwidth_savings_estimate_pct{0.0};
    double edge_packet_encode_latency_us{0.0};
    bool disconnected_operation{false};
    std::string edge_health_status{"nominal"};
    std::string edge_autonomy_state{"backend_assisted"};
    std::string edge_inference_status{"idle"};
    double edge_inference_fps{0.0};
    double edge_inference_confidence{0.0};
    int local_obstacle_count{0};
    int shared_obstacle_count{0};
    size_t relocalization_count{0};
    size_t visible_anchor_count{0};
    size_t planned_waypoint_count{0};
    uint64_t last_relocalized_keyframe{0};
    std::string localization_state{"nominal"};
    std::string localization_source{"vision-inertial"};
    std::string localization_data_source{"unavailable"};
    std::string security_state{"TRUSTED"};
    std::string security_summary{"All trust signals nominal"};
    std::string security_transition_reason{"initial-trust"};
    bool remote_command_allowed{true};
    bool telemetry_uplink_allowed{true};
    double link_integrity_score{1.0};
    uint64_t trust_epoch{1};
    double last_auth_failure_at_s{0.0};
    double tamper_score{0.0};
    std::string firmware_measurement{"lab-local-build"};
    std::string firmware_version{"0.0.0"};
    std::string secure_boot_state{"LAB_BOOT"};
    std::string boot_trust_summary{"Lab boot trust bypassed"};
    uint64_t rollback_counter{0};
    bool maintenance_mode{false};
    std::string update_channel_state{"idle"};
    std::string last_remote_command_status{"no remote command"};
    std::vector<std::string> health_flags{};
    size_t tracked_feature_count{0};
    double inlier_ratio{0.0};
    double reprojection_error{0.0};
    double visual_update_confidence{0.0};
    bool visual_frontend_valid{false};
    bool visual_placeholder_active{false};
};

struct VisualFrontendMetrics {
    size_t tracked_feature_count{0};
    double inlier_ratio{0.0};
    double reprojection_error{1.0e6};
    double visual_update_confidence{0.0};
    bool update_accepted{false};
    bool used_placeholder{false};
};

struct VisualFrontendResult {
    Eigen::Vector3d observed_position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d observed_velocity{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond relative_orientation{Eigen::Quaterniond::Identity()};
    VisualFrontendMetrics metrics{};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

[[nodiscard]] bool visual_placeholder_allowed(drone::runtime::RuntimeMode mode);
[[nodiscard]] double compute_visual_update_confidence(const VisualFrontendMetrics& metrics);
[[nodiscard]] VisualFrontendResult run_visual_frontend(
    const cv::Mat& previous_gray,
    const cv::Mat& current_gray,
    const Eigen::Matrix3d& K,
    const PoseEstimate& previous_pose,
    const PoseEstimate& current_predicted_pose,
    double dt_s);
[[nodiscard]] VisualFrontendResult build_placeholder_visual_frontend_result(
    const sensors::CameraFrame& frame,
    const PoseEstimate& pose,
    const Eigen::Matrix3d& K);

 
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
    void set_runtime_mode(drone::runtime::RuntimeMode mode) { runtime_mode_ = mode; }

    //  State query â”€
    [[nodiscard]] PoseEstimate  current_pose() const;
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
        telemetry.tracked_feature_count = runtime_telemetry_.tracked_feature_count;
        telemetry.inlier_ratio = runtime_telemetry_.inlier_ratio;
        telemetry.reprojection_error = runtime_telemetry_.reprojection_error;
        telemetry.visual_update_confidence = runtime_telemetry_.visual_update_confidence;
        telemetry.visual_frontend_valid = runtime_telemetry_.visual_frontend_valid;
        telemetry.visual_placeholder_active = runtime_telemetry_.visual_placeholder_active;
        runtime_telemetry_ = std::move(telemetry);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    void enqueue(SensorEvent evt);
    void processing_loop();

    void handle(const sensors::ImuMeasurement& imu);
    void handle(const sensors::CameraFrame& frame);
    void handle(const sensors::LidarMeasurement&);
    void apply_visual_quality_to_pose(PoseEstimate& pose) const;

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
    double           last_camera_ts_{-1.0};
    cv::Mat          previous_gray_frame_;
    PoseEstimate     previous_camera_pose_{};
    bool             previous_camera_pose_valid_{false};
    drone::runtime::RuntimeMode runtime_mode_{drone::runtime::RuntimeMode::SIMULATION};
    mutable std::mutex visual_metrics_mutex_;
    VisualFrontendMetrics last_visual_metrics_{};
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
