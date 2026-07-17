// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

// VIOPipeline.cpp    Multi-sensor orchestration with lock-free event queue
// Drone Swarm Sensor Fusion  |  Phase 2

#include "vio/VIOPipeline.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <memory>
#include <numeric>

namespace drone::vio {

namespace {

constexpr size_t kMinTrackedFeatures = 24;
constexpr double kMinInlierRatio = 0.55;
constexpr double kMaxReprojectionErrorPx = 3.5;
constexpr size_t kMaxFeatures = 300;
constexpr double kDefaultVisualConfidenceOnFailure = 0.18;
constexpr double kDefaultVisualConfidenceOnPlaceholder = 0.42;

bool thread_sanitizer_enabled() {
#if defined(__has_feature)
#if __has_feature(thread_sanitizer)
    return true;
#endif
#endif
#if defined(__SANITIZE_THREAD__)
    return true;
#endif
    return false;
}

void configure_opencv_threads_for_tsan() {
    static std::once_flag once;
    if (!thread_sanitizer_enabled()) {
        return;
    }
    std::call_once(once, [] { cv::setNumThreads(1); });
}

cv::Mat to_gray(const cv::Mat& image) {
    if (image.empty()) {
        return {};
    }
    if (image.channels() == 1) {
        return image.clone();
    }
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

double mean_optical_flow_error(const std::vector<float>& errors, const std::vector<uchar>& status) {
    double sum = 0.0;
    size_t count = 0;
    for (size_t i = 0; i < errors.size() && i < status.size(); ++i) {
        if (!status[i])
            continue;
        sum += static_cast<double>(errors[i]);
        ++count;
    }
    return count > 0 ? (sum / static_cast<double>(count)) : 1.0e6;
}

} // namespace

VIOPipeline::VIOPipeline(EKFConfig cfg)
    : ekf_config_(cfg), active_estimator_(std::make_shared<estimation::MinimalEskfAdapter>(cfg)),
      coordinator_(std::static_pointer_cast<estimation::StateEstimator>(active_estimator_)) {
    active_estimator_->set_validation_config(estimator_validation_config_);
}

bool visual_placeholder_allowed(drone::runtime::RuntimeMode mode) {
    return mode == drone::runtime::RuntimeMode::SIMULATION;
}

double compute_visual_update_confidence(const VisualFrontendMetrics& metrics) {
    if (metrics.tracked_feature_count == 0) {
        return kDefaultVisualConfidenceOnFailure;
    }
    const double feature_score =
        std::clamp(static_cast<double>(metrics.tracked_feature_count) / 140.0, 0.0, 1.0);
    const double inlier_score = std::clamp(metrics.inlier_ratio, 0.0, 1.0);
    const double reprojection_score =
        std::clamp(1.0 - (metrics.reprojection_error / 8.0), 0.0, 1.0);
    double confidence =
        (feature_score * 0.35) + (inlier_score * 0.45) + (reprojection_score * 0.20);
    if (!metrics.update_accepted) {
        confidence *= 0.45;
    }
    if (metrics.used_placeholder) {
        confidence = std::min(confidence, kDefaultVisualConfidenceOnPlaceholder);
    }
    return std::clamp(confidence, 0.0, 1.0);
}

VisualFrontendResult run_visual_frontend(const cv::Mat& previous_gray, const cv::Mat& current_gray,
                                         const Eigen::Matrix3d& K,
                                         const PoseEstimate& previous_pose,
                                         const PoseEstimate& current_predicted_pose, double dt_s) {
    configure_opencv_threads_for_tsan();
    VisualFrontendResult result;
    if (previous_gray.empty() || current_gray.empty() || dt_s <= 0.0) {
        result.metrics.visual_update_confidence = kDefaultVisualConfidenceOnFailure;
        return result;
    }

    std::vector<cv::Point2f> previous_points;
    cv::goodFeaturesToTrack(previous_gray, previous_points, static_cast<int>(kMaxFeatures), 0.01,
                            8.0);
    if (previous_points.size() < 8) {
        result.metrics.tracked_feature_count = previous_points.size();
        result.metrics.visual_update_confidence = kDefaultVisualConfidenceOnFailure;
        return result;
    }

    std::vector<cv::Point2f> current_points;
    std::vector<uchar> status;
    std::vector<float> errors;
    cv::calcOpticalFlowPyrLK(previous_gray, current_gray, previous_points, current_points, status,
                             errors, cv::Size(21, 21), 3);

    std::vector<cv::Point2f> tracked_prev;
    std::vector<cv::Point2f> tracked_curr;
    tracked_prev.reserve(previous_points.size());
    tracked_curr.reserve(previous_points.size());
    for (size_t i = 0; i < previous_points.size() && i < status.size(); ++i) {
        if (!status[i])
            continue;
        tracked_prev.push_back(previous_points[i]);
        tracked_curr.push_back(current_points[i]);
    }

    result.metrics.tracked_feature_count = tracked_curr.size();
    result.metrics.reprojection_error = mean_optical_flow_error(errors, status);
    if (tracked_curr.size() < 8) {
        result.metrics.visual_update_confidence = compute_visual_update_confidence(result.metrics);
        return result;
    }

    cv::Mat K_cv(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            K_cv.at<double>(r, c) = K(r, c);
        }
    }

    cv::Mat inlier_mask;
    const cv::Mat essential =
        cv::findEssentialMat(tracked_prev, tracked_curr, K_cv, cv::RANSAC, 0.999, 1.5, inlier_mask);
    if (essential.empty() || inlier_mask.empty()) {
        result.metrics.visual_update_confidence = compute_visual_update_confidence(result.metrics);
        return result;
    }

    int inlier_count = 0;
    for (int i = 0; i < inlier_mask.rows; ++i) {
        if (inlier_mask.at<uchar>(i, 0) != 0) {
            ++inlier_count;
        }
    }
    result.metrics.inlier_ratio =
        tracked_curr.empty()
            ? 0.0
            : static_cast<double>(inlier_count) / static_cast<double>(tracked_curr.size());

    cv::Mat R_cv;
    cv::Mat t_cv;
    cv::Mat recover_mask;
    const int recovered =
        cv::recoverPose(essential, tracked_prev, tracked_curr, K_cv, R_cv, t_cv, recover_mask);
    if (recovered < static_cast<int>(kMinTrackedFeatures / 2)) {
        result.metrics.visual_update_confidence = compute_visual_update_confidence(result.metrics);
        return result;
    }

    const Eigen::Vector3d predicted_delta =
        current_predicted_pose.position - previous_pose.position;
    const double predicted_scale =
        std::max(predicted_delta.norm(), current_predicted_pose.velocity.norm() * dt_s);
    if (predicted_scale < 1.0e-4) {
        result.metrics.visual_update_confidence = compute_visual_update_confidence(result.metrics);
        return result;
    }

    Eigen::Matrix3d relative_rotation = Eigen::Matrix3d::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            relative_rotation(r, c) = R_cv.at<double>(r, c);
        }
    }
    Eigen::Vector3d translation_direction{
        t_cv.at<double>(0, 0),
        t_cv.at<double>(1, 0),
        t_cv.at<double>(2, 0),
    };
    if (translation_direction.norm() < 1.0e-6) {
        result.metrics.visual_update_confidence = compute_visual_update_confidence(result.metrics);
        return result;
    }
    translation_direction.normalize();

    const Eigen::Vector3d world_translation =
        previous_pose.R_wb() * translation_direction * predicted_scale;
    result.observed_position = previous_pose.position + world_translation;
    result.observed_velocity = world_translation / dt_s;
    result.relative_orientation = Eigen::Quaterniond(relative_rotation).normalized();

    result.metrics.update_accepted = result.metrics.tracked_feature_count >= kMinTrackedFeatures &&
                                     result.metrics.inlier_ratio >= kMinInlierRatio &&
                                     result.metrics.reprojection_error <= kMaxReprojectionErrorPx;
    result.metrics.visual_update_confidence = compute_visual_update_confidence(result.metrics);
    return result;
}

VisualFrontendResult build_placeholder_visual_frontend_result(const sensors::CameraFrame& frame,
                                                              const PoseEstimate& pose,
                                                              const Eigen::Matrix3d& K) {
    VisualFrontendResult result;
    result.metrics.used_placeholder = true;

    std::vector<Eigen::Vector2d> z_pixels;
    std::vector<Eigen::Vector3d> p_world;
    for (const auto& det : frame.detections) {
        if (det.confidence < 0.6f)
            continue;
        const float cx_px =
            (det.bbox.x + det.bbox.width * 0.5f) * static_cast<float>(K(0, 2)) * 2.0f;
        const float cy_px =
            (det.bbox.y + det.bbox.height * 0.5f) * static_cast<float>(K(1, 2)) * 2.0f;
        z_pixels.push_back({cx_px, cy_px});

        const Eigen::Vector3d forward = pose.R_wb() * Eigen::Vector3d{0.0, 0.0, 1.0};
        p_world.push_back(pose.position + forward * 5.0);
    }

    result.metrics.tracked_feature_count = z_pixels.size();
    result.metrics.inlier_ratio = z_pixels.empty() ? 0.0 : 1.0;
    result.metrics.reprojection_error = z_pixels.empty() ? 1.0e6 : 0.5;
    result.metrics.update_accepted = !z_pixels.empty();
    result.metrics.visual_update_confidence = compute_visual_update_confidence(result.metrics);
    result.observed_position = pose.position;
    result.observed_velocity = pose.velocity;
    return result;
}

void VIOPipeline::attach_imu(std::shared_ptr<sensors::IMUSensor> imu) {
    imu_ = std::move(imu);
    imu_->set_data_callback([this](const sensors::ImuMeasurement& m) { enqueue(m); });
}

void VIOPipeline::attach_camera(std::shared_ptr<sensors::CameraSensor> cam) {
    cam_ = std::move(cam);
    cam_->set_data_callback([this](const sensors::CameraFrame& f) { enqueue(f); });
}

void VIOPipeline::attach_lidar(std::shared_ptr<sensors::LidarSensor> lidar) {
    lidar_ = std::move(lidar);
    lidar_->set_data_callback([this](const sensors::LidarMeasurement& m) { enqueue(m); });
}

bool VIOPipeline::start() {
    if (running_.exchange(true))
        return true;

    coordinator_.reset({});
    if (estimator_validation_config_.enable_shadow_estimator) {
        estimation::ShadowEstimatorConfig shadow_config;
        shadow_config.enabled = true;
        shadow_config.comparison_enabled = estimator_validation_config_.shadow_comparison_enabled;
        shadow_config.max_queue_depth = estimator_validation_config_.shadow_max_queue_depth;
        shadow_config.max_lag_ms = estimator_validation_config_.shadow_max_lag_ms;
        shadow_config.thresholds.position_divergence_m =
            estimator_validation_config_.shadow_position_divergence_m;
        shadow_config.thresholds.velocity_divergence_mps =
            estimator_validation_config_.shadow_velocity_divergence_mps;
        shadow_config.thresholds.orientation_divergence_deg =
            estimator_validation_config_.shadow_orientation_divergence_deg;
        shadow_config.thresholds.required_consecutive_divergent_samples =
            estimator_validation_config_.shadow_required_consecutive_divergent_samples;
        auto shadow = std::make_unique<estimation::MinimalEskfAdapter>(ekf_config_);
        shadow->set_validation_config(estimator_validation_config_);
        coordinator_.configure_shadow(std::move(shadow_config), std::move(shadow));
    } else {
        coordinator_.stop_shadow();
    }

    proc_thread_ = std::thread([this] { processing_loop(); });

    if (logger_)
        logger_->info("VIO pipeline started");
    return true;
}

void VIOPipeline::stop() {
    if (!running_.exchange(false))
        return;
    queue_cv_.notify_all();
    if (proc_thread_.joinable())
        proc_thread_.join();
    coordinator_.stop_shadow();
    if (logger_)
        logger_->info("VIO pipeline stopped");
}

void VIOPipeline::reset() {
    std::lock_guard lock(queue_mutex_);
    while (!event_queue_.empty())
        event_queue_.pop();
    coordinator_.reset({});
    last_imu_ts_ = -1.0;
    last_camera_ts_ = -1.0;
    measurement_sequence_ = 0;
    previous_gray_frame_.release();
    previous_camera_pose_valid_ = false;
    {
        std::lock_guard visual_lock(visual_metrics_mutex_);
        last_visual_metrics_ = {};
    }
}

PoseEstimate VIOPipeline::current_pose() const {
    auto pose = coordinator_.active_snapshot().pose;
    apply_visual_quality_to_pose(pose);
    return pose;
}

void VIOPipeline::enqueue(SensorEvent evt) {
    {
        std::lock_guard lock(queue_mutex_);
        if (event_queue_.size() > 2000) {
            event_queue_.pop();
            if (logger_)
                logger_->warn("VIO queue overflow  dropping oldest event");
        }
        event_queue_.push(std::move(evt));
    }
    queue_cv_.notify_one();
}

void VIOPipeline::processing_loop() {
    while (running_.load()) {
        SensorEvent evt;
        {
            std::unique_lock lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return !event_queue_.empty() || !running_.load(); });
            if (!running_.load() && event_queue_.empty())
                return;
            evt = std::move(event_queue_.front());
            event_queue_.pop();
        }

        std::visit([this](auto&& e) { handle(e); }, evt);

        if (pose_cb_) {
            pose_cb_(current_pose());
        }
    }
}

void VIOPipeline::handle(const sensors::ImuMeasurement& imu) {
    if (coordinator_.active_snapshot().diagnostics.health_state ==
            EstimatorHealthState::INITIALIZING &&
        last_imu_ts_ < 0.0) {
        coordinator_.reset({});
        last_imu_ts_ = imu.timestamp;
        return;
    }

    if (!imu.accel_mps2.array().isFinite().all() || !imu.gyro_rads.array().isFinite().all() ||
        !std::isfinite(imu.timestamp)) {
        return;
    }

    const double dt = (last_imu_ts_ > 0.0) ? (imu.timestamp - last_imu_ts_) : 0.0025;
    if (last_imu_ts_ > 0.0 && dt <= 0.0) {
        return;
    }
    if (dt >= 0.1) {
        last_imu_ts_ = imu.timestamp;
        return;
    }
    last_imu_ts_ = imu.timestamp;
    const auto adapted = estimation::MeasurementAdapters::adapt_imu(imu, ++measurement_sequence_);
    if (adapted.measurement.has_value()) {
        static_cast<void>(coordinator_.process(*adapted.measurement));
    }
    update_shadow_runtime_telemetry();
}

void VIOPipeline::handle(const sensors::CameraFrame& frame) {
    if (coordinator_.active_snapshot().diagnostics.health_state ==
        EstimatorHealthState::INITIALIZING)
        return;
    if (frame.detections.empty() && frame.image.empty())
        return;

    const auto predicted_pose = coordinator_.active_snapshot().pose;
    VisualFrontendResult frontend_result;
    cv::Mat current_gray = to_gray(frame.image);
    const double dt = (last_camera_ts_ > 0.0) ? (frame.timestamp - last_camera_ts_) : 0.0;

    if (!previous_gray_frame_.empty() && !current_gray.empty() && previous_camera_pose_valid_ &&
        dt > 0.0) {
        frontend_result = run_visual_frontend(previous_gray_frame_, current_gray, K_,
                                              previous_camera_pose_, predicted_pose, dt);
    } else {
        frontend_result.metrics.visual_update_confidence = kDefaultVisualConfidenceOnFailure;
    }

    if (frontend_result.metrics.update_accepted) {
        const auto adapted = estimation::MeasurementAdapters::adapt_visual_pose(
            frame.timestamp, frontend_result.observed_position, frontend_result.observed_velocity,
            frontend_result.relative_orientation, frontend_result.metrics.visual_update_confidence,
            frontend_result.metrics.update_accepted, ++measurement_sequence_, false);
        if (adapted.measurement.has_value()) {
            static_cast<void>(coordinator_.process(*adapted.measurement));
        }
    } else if (visual_placeholder_allowed(runtime_mode_) && !frame.detections.empty()) {
        const auto placeholder =
            build_placeholder_visual_frontend_result(frame, predicted_pose, K_);
        frontend_result = placeholder;

        std::vector<Eigen::Vector2d> z_pixels;
        std::vector<Eigen::Vector3d> p_world;
        for (const auto& det : frame.detections) {
            if (det.confidence < 0.6f)
                continue;
            const float cx_px =
                (det.bbox.x + det.bbox.width * 0.5f) * static_cast<float>(K_(0, 2)) * 2.0f;
            const float cy_px =
                (det.bbox.y + det.bbox.height * 0.5f) * static_cast<float>(K_(1, 2)) * 2.0f;
            z_pixels.push_back({cx_px, cy_px});

            const Eigen::Vector3d forward = predicted_pose.R_wb() * Eigen::Vector3d{0.0, 0.0, 1.0};
            p_world.push_back(predicted_pose.position + forward * 5.0);
        }
        if (!z_pixels.empty()) {
            active_estimator_->estimator().update_vision(z_pixels, p_world, K_);
        }
    }

    {
        std::lock_guard lock(visual_metrics_mutex_);
        last_visual_metrics_ = frontend_result.metrics;
    }
    {
        std::lock_guard lock(runtime_mutex_);
        runtime_telemetry_.tracked_feature_count = frontend_result.metrics.tracked_feature_count;
        runtime_telemetry_.inlier_ratio = frontend_result.metrics.inlier_ratio;
        runtime_telemetry_.reprojection_error = frontend_result.metrics.reprojection_error;
        runtime_telemetry_.visual_update_confidence =
            frontend_result.metrics.visual_update_confidence;
        runtime_telemetry_.visual_frontend_valid = frontend_result.metrics.update_accepted;
        runtime_telemetry_.visual_placeholder_active = frontend_result.metrics.used_placeholder;
    }

    previous_gray_frame_ = current_gray;
    previous_camera_pose_ = coordinator_.active_snapshot().pose;
    previous_camera_pose_valid_ = true;
    last_camera_ts_ = frame.timestamp;
    update_shadow_runtime_telemetry();
}

void VIOPipeline::handle(const sensors::LidarMeasurement& lidar) {
    if (coordinator_.active_snapshot().diagnostics.health_state ==
            EstimatorHealthState::INITIALIZING ||
        !lidar.cloud)
        return;

    if (lidar.cloud->empty())
        return;

    std::vector<float> z_vals;
    z_vals.reserve(lidar.cloud->size());
    for (const auto& pt : *lidar.cloud)
        if (pt.z > -20.0f && pt.z < 0.5f)
            z_vals.push_back(pt.z);

    if (z_vals.empty())
        return;

    std::nth_element(z_vals.begin(), z_vals.begin() + z_vals.size() / 2, z_vals.end());
    const double ground_z = z_vals[z_vals.size() / 2];

    const auto pose = coordinator_.active_snapshot().pose;
    const double height = pose.position.z() - ground_z;
    if (!std::isfinite(height) || height <= 0.3 || height >= 100.0) {
        return;
    }
    // Phase 15 safety baseline: this plane fit only provides a local sensor-frame ground-relative
    // height. Without explicit frame calibration and observability to world-frame altitude, it is
    // unsafe to feed it into the active EKF depth update.
    update_shadow_runtime_telemetry();
}

void VIOPipeline::apply_visual_quality_to_pose(PoseEstimate& pose) const {
    VisualFrontendMetrics metrics;
    {
        std::lock_guard lock(visual_metrics_mutex_);
        metrics = last_visual_metrics_;
    }

    if (metrics.visual_update_confidence <= 0.0) {
        return;
    }

    pose.localization_confidence = std::clamp(
        pose.localization_confidence * std::clamp(metrics.visual_update_confidence, 0.18, 1.0), 0.0,
        1.0);

    if (metrics.tracked_feature_count < kMinTrackedFeatures ||
        metrics.inlier_ratio < kMinInlierRatio ||
        metrics.reprojection_error > kMaxReprojectionErrorPx) {
        pose.localization_degraded = true;
        pose.localization_source =
            metrics.used_placeholder ? "simulation-placeholder-vision" : "low-visual-quality";
    }
    if (pose.localization_confidence < 0.22) {
        pose.localization_lost = true;
    }
}

void VIOPipeline::update_shadow_runtime_telemetry() {
    const auto telemetry = coordinator_.telemetry();
    {
        std::lock_guard lock(runtime_mutex_);
        runtime_telemetry_.active_estimator_name = telemetry.active_estimator_name;
        runtime_telemetry_.shadow_estimator_name = telemetry.shadow_estimator_name;
        runtime_telemetry_.active_estimator_health =
            estimator_health_to_string(telemetry.active_health);
        runtime_telemetry_.shadow_estimator_health =
            shadow_health_to_string(telemetry.shadow_health);
        runtime_telemetry_.shadow_enabled = telemetry.enabled;
        runtime_telemetry_.shadow_lag_ms = telemetry.lag_ms;
        runtime_telemetry_.shadow_queue_depth = telemetry.queue_depth;
        runtime_telemetry_.shadow_queue_high_water_mark = telemetry.queue_high_water_mark;
        runtime_telemetry_.shadow_dropped_events = telemetry.dropped_events;
        runtime_telemetry_.shadow_position_delta_m = telemetry.position_delta_m;
        runtime_telemetry_.shadow_velocity_delta_mps = telemetry.velocity_delta_mps;
        runtime_telemetry_.shadow_orientation_delta_deg = telemetry.orientation_delta_deg;
        runtime_telemetry_.shadow_divergence_active = telemetry.divergence_active;
        runtime_telemetry_.shadow_last_failure_reason = telemetry.last_failure_reason;
        runtime_telemetry_.shadow_comparable_snapshot_count = telemetry.comparable_snapshot_count;
    }
}

std::string VIOPipeline::estimator_health_to_string(EstimatorHealthState state) {
    switch (state) {
    case EstimatorHealthState::INITIALIZING:
        return "initializing";
    case EstimatorHealthState::NOMINAL:
        return "nominal";
    case EstimatorHealthState::DEGRADED:
        return "degraded";
    case EstimatorHealthState::REJECTING_MEASUREMENTS:
        return "rejecting_measurements";
    case EstimatorHealthState::NUMERICAL_WARNING:
        return "numerical_warning";
    case EstimatorHealthState::INVALID:
        return "invalid";
    }
    return "unknown";
}

std::string VIOPipeline::shadow_health_to_string(estimation::ShadowHealthState state) {
    switch (state) {
    case estimation::ShadowHealthState::DISABLED:
        return "disabled";
    case estimation::ShadowHealthState::STARTING:
        return "starting";
    case estimation::ShadowHealthState::SYNCHRONIZED:
        return "synchronized";
    case estimation::ShadowHealthState::LAGGING:
        return "lagging";
    case estimation::ShadowHealthState::STALE:
        return "stale";
    case estimation::ShadowHealthState::DIVERGED:
        return "diverged";
    case estimation::ShadowHealthState::FAILED:
        return "failed";
    case estimation::ShadowHealthState::STOPPED:
        return "stopped";
    }
    return "unknown";
}

} // namespace drone::vio
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
