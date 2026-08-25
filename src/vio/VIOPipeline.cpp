// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "vio/VIOPipeline.hpp"
#include "vio/Phase17ESKFEstimator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>
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
    if (!thread_sanitizer_enabled()) return;
    std::call_once(once, [] { cv::setNumThreads(1); });
}

cv::Mat to_gray(const cv::Mat& image) {
    if (image.empty()) return {};
    if (image.channels() == 1) return image.clone();
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

double mean_optical_flow_error(const std::vector<float>& errors, const std::vector<uchar>& status) {
    double sum = 0.0;
    size_t count = 0;
    for (size_t i = 0; i < errors.size() && i < status.size(); ++i) {
        if (!status[i]) continue;
        sum += static_cast<double>(errors[i]);
        ++count;
    }
    return count > 0 ? sum / static_cast<double>(count) : 1.0e6;
}
} // namespace

VIOPipeline::VIOPipeline(EKFConfig cfg) : ekf_cfg_(cfg) {
    ShadowCoordinatorConfig shadow_cfg{};
    coordinator_ = std::make_unique<EstimatorCoordinator>(
        std::make_unique<EKFStateEstimatorAdapter>(ekf_cfg_, "ekf_active", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(ekf_cfg_, "eskf_shadow", "phase17"), shadow_cfg);
}

bool visual_placeholder_allowed(drone::runtime::RuntimeMode mode) {
    return mode == drone::runtime::RuntimeMode::SIMULATION;
}

double compute_visual_update_confidence(const VisualFrontendMetrics& metrics) {
    if (metrics.tracked_feature_count == 0) return kDefaultVisualConfidenceOnFailure;
    const double feature_score =
        std::clamp(static_cast<double>(metrics.tracked_feature_count) / 140.0, 0.0, 1.0);
    const double inlier_score = std::clamp(metrics.inlier_ratio, 0.0, 1.0);
    const double reprojection_score = std::clamp(1.0 - metrics.reprojection_error / 8.0, 0.0, 1.0);
    double confidence = feature_score * 0.35 + inlier_score * 0.45 + reprojection_score * 0.20;
    if (!metrics.update_accepted) confidence *= 0.45;
    if (metrics.used_placeholder) confidence = std::min(confidence, kDefaultVisualConfidenceOnPlaceholder);
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
    cv::goodFeaturesToTrack(previous_gray, previous_points, static_cast<int>(kMaxFeatures), 0.01, 8.0);
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
        if (!status[i]) continue;
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
        for (int c = 0; c < 3; ++c) K_cv.at<double>(r, c) = K(r, c);
    }

    cv::Mat inlier_mask;
    const cv::Mat essential = cv::findEssentialMat(tracked_prev, tracked_curr, K_cv, cv::RANSAC,
                                                    0.999, 1.5, inlier_mask);
    if (essential.empty() || inlier_mask.empty()) {
        result.metrics.visual_update_confidence = compute_visual_update_confidence(result.metrics);
        return result;
    }

    int inlier_count = 0;
    for (int i = 0; i < inlier_mask.rows; ++i) {
        if (inlier_mask.at<uchar>(i, 0) != 0) ++inlier_count;
    }
    result.metrics.inlier_ratio = tracked_curr.empty()
                                      ? 0.0
                                      : static_cast<double>(inlier_count) /
                                            static_cast<double>(tracked_curr.size());

    cv::Mat R_cv;
    cv::Mat t_cv;
    cv::Mat recover_mask;
    const int recovered = cv::recoverPose(essential, tracked_prev, tracked_curr, K_cv, R_cv, t_cv,
                                          recover_mask);
    if (recovered < static_cast<int>(kMinTrackedFeatures / 2)) {
        result.metrics.visual_update_confidence = compute_visual_update_confidence(result.metrics);
        return result;
    }

    result.previous_inlier_pixels.reserve(tracked_prev.size());
    result.current_inlier_pixels.reserve(tracked_curr.size());
    for (size_t i = 0; i < tracked_prev.size(); ++i) {
        const bool essential_inlier = i < static_cast<size_t>(inlier_mask.rows) &&
                                      inlier_mask.at<uchar>(static_cast<int>(i), 0) != 0;
        const bool pose_inlier = i < static_cast<size_t>(recover_mask.rows) &&
                                 recover_mask.at<uchar>(static_cast<int>(i), 0) != 0;
        if (!essential_inlier || !pose_inlier) continue;
        result.previous_inlier_pixels.emplace_back(tracked_prev[i].x, tracked_prev[i].y);
        result.current_inlier_pixels.emplace_back(tracked_curr[i].x, tracked_curr[i].y);
    }

    const Eigen::Vector3d predicted_delta = current_predicted_pose.position - previous_pose.position;
    const double predicted_scale =
        std::max(predicted_delta.norm(), current_predicted_pose.velocity.norm() * dt_s);
    if (predicted_scale < 1.0e-4) {
        result.metrics.visual_update_confidence = compute_visual_update_confidence(result.metrics);
        return result;
    }

    Eigen::Matrix3d relative_rotation = Eigen::Matrix3d::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) relative_rotation(r, c) = R_cv.at<double>(r, c);
    }
    Eigen::Vector3d translation_direction{t_cv.at<double>(0, 0), t_cv.at<double>(1, 0),
                                          t_cv.at<double>(2, 0)};
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
    size_t accepted = 0;
    for (const auto& det : frame.detections) {
        if (det.confidence >= 0.6f) ++accepted;
    }
    result.metrics.tracked_feature_count = accepted;
    result.metrics.inlier_ratio = accepted == 0 ? 0.0 : 1.0;
    result.metrics.reprojection_error = accepted == 0 ? 1.0e6 : 0.5;
    result.metrics.update_accepted = accepted != 0;
    result.metrics.visual_update_confidence = compute_visual_update_confidence(result.metrics);
    result.observed_position = pose.position;
    result.observed_velocity = pose.velocity;
    (void)K;
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
    if (running_.exchange(true)) return true;
    if (coordinator_) {
        coordinator_->initialize();
        (void)coordinator_->start();
    }
    proc_thread_ = std::thread([this] { processing_loop(); });
    if (logger_) logger_->info("VIO pipeline started");
    return true;
}

void VIOPipeline::stop() {
    if (!running_.exchange(false)) return;
    queue_cv_.notify_all();
    if (proc_thread_.joinable()) proc_thread_.join();
    if (coordinator_) coordinator_->stop();
    if (logger_) logger_->info("VIO pipeline stopped");
}

void VIOPipeline::reset() {
    std::lock_guard lock(queue_mutex_);
    while (!event_queue_.empty()) event_queue_.pop();
    if (coordinator_) coordinator_->reset();
    last_imu_ts_ = -1.0;
    last_camera_ts_ = -1.0;
    measurement_sequence_ = 0;
    previous_gray_frame_.release();
    previous_camera_pose_valid_ = false;
    visual_feature_tracks_.reset();
    {
        std::lock_guard visual_lock(visual_metrics_mutex_);
        last_visual_metrics_ = {};
    }
    {
        std::lock_guard runtime_lock(runtime_mutex_);
        runtime_telemetry_.persistent_feature_tracks = 0;
        runtime_telemetry_.initialized_feature_tracks = 0;
        runtime_telemetry_.rejected_feature_geometry = 0;
        runtime_telemetry_.shadow_feature_batches_submitted = 0;
    }
}

PoseEstimate VIOPipeline::current_pose() const {
    auto pose = coordinator_ ? coordinator_->active_pose() : PoseEstimate{};
    apply_visual_quality_to_pose(pose);
    if (const auto snapshot = coordinator_ ? std::optional{coordinator_->active_snapshot()}
                                           : std::optional<EstimatorStateSnapshot>{}) {
        pose.drift_m = snapshot->covariance.position_std_m.norm();
    }
    return pose;
}

void VIOPipeline::enqueue(SensorEvent evt) {
    {
        std::lock_guard lock(queue_mutex_);
        if (event_queue_.size() > 2000) event_queue_.pop();
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
            if (!running_.load() && event_queue_.empty()) return;
            evt = std::move(event_queue_.front());
            event_queue_.pop();
        }
        std::visit([this](auto&& e) { handle(e); }, evt);
        if (pose_cb_) pose_cb_(current_pose());
    }
}

void VIOPipeline::handle(const sensors::ImuMeasurement& imu) {
    if (!coordinator_) return;
    if (!coordinator_->active_snapshot().initialized) coordinator_->initialize();
    last_imu_ts_ = imu.timestamp;
    (void)coordinator_->process_measurement(make_imu_envelope(imu, measurement_sequence_++));
    refresh_runtime_telemetry_from_coordinator();
}

void VIOPipeline::submit_tracked_features_to_shadow(const VisualFrontendResult& frontend_result,
                                                     const PoseEstimate& previous_pose,
                                                     const PoseEstimate& current_pose,
                                                     double timestamp_s) {
    if (!coordinator_ || frontend_result.previous_inlier_pixels.empty() ||
        frontend_result.previous_inlier_pixels.size() !=
            frontend_result.current_inlier_pixels.size()) {
        return;
    }

    const auto batch = visual_feature_tracks_.update(frontend_result.previous_inlier_pixels,
                                                     frontend_result.current_inlier_pixels,
                                                     previous_pose, current_pose, K_);
    {
        std::lock_guard lock(runtime_mutex_);
        runtime_telemetry_.persistent_feature_tracks = visual_feature_tracks_.track_count();
        runtime_telemetry_.initialized_feature_tracks += batch.initialized_tracks;
        runtime_telemetry_.rejected_feature_geometry += batch.rejected_geometry;
    }
    if (batch.features.empty()) return;

    VisualFeatureMeasurementPayload payload;
    payload.K = K_;
    payload.z_pixels.reserve(batch.features.size());
    payload.p_world.reserve(batch.features.size());
    for (const auto& feature : batch.features) {
        payload.z_pixels.push_back(feature.pixel);
        payload.p_world.push_back(feature.world_point);
    }

    const auto result = coordinator_->submit_shadow_measurement(make_visual_features_envelope(
        payload, MeasurementStamp{timestamp_s, measurement_sequence_++}));
    if (result == EstimatorOperationResult::Accepted) {
        std::lock_guard lock(runtime_mutex_);
        ++runtime_telemetry_.shadow_feature_batches_submitted;
    }
}

void VIOPipeline::handle(const sensors::CameraFrame& frame) {
    if (!coordinator_ || !coordinator_->active_snapshot().initialized) return;
    if (frame.detections.empty() && frame.image.empty()) return;

    const auto predicted_pose = coordinator_->active_pose();
    VisualFrontendResult frontend_result;
    cv::Mat current_gray = to_gray(frame.image);
    const double dt = last_camera_ts_ > 0.0 ? frame.timestamp - last_camera_ts_ : 0.0;
    if (!previous_gray_frame_.empty() && !current_gray.empty() && previous_camera_pose_valid_ &&
        dt > 0.0) {
        frontend_result = run_visual_frontend(previous_gray_frame_, current_gray, K_,
                                              previous_camera_pose_, predicted_pose, dt);
    } else {
        frontend_result.metrics.visual_update_confidence = kDefaultVisualConfidenceOnFailure;
    }

    if (frontend_result.metrics.update_accepted) {
        VisualPoseMeasurementPayload payload;
        payload.position_m = frontend_result.observed_position;
        payload.velocity_mps = frontend_result.observed_velocity;
        payload.sigma_position_m = std::clamp(
            0.50 - frontend_result.metrics.visual_update_confidence * 0.28, 0.14, 0.50);
        payload.sigma_velocity_mps = std::clamp(
            0.65 - frontend_result.metrics.visual_update_confidence * 0.30, 0.18, 0.65);
        (void)coordinator_->process_measurement(make_visual_pose_envelope(
            payload, MeasurementStamp{frame.timestamp, measurement_sequence_++}));
        submit_tracked_features_to_shadow(frontend_result, previous_camera_pose_, predicted_pose,
                                          frame.timestamp);
    } else if (visual_placeholder_allowed(runtime_mode_) && !frame.detections.empty()) {
        frontend_result = build_placeholder_visual_frontend_result(frame, predicted_pose, K_);
        (void)coordinator_->process_measurement(make_visual_pose_envelope(
            VisualPoseMeasurementPayload{predicted_pose.position, predicted_pose.velocity, 0.35,
                                         0.45},
            MeasurementStamp{frame.timestamp, measurement_sequence_++}));
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
    previous_camera_pose_ = coordinator_->active_pose();
    previous_camera_pose_valid_ = true;
    last_camera_ts_ = frame.timestamp;
    refresh_runtime_telemetry_from_coordinator();
}

void VIOPipeline::handle(const sensors::LidarMeasurement& lidar) {
    if (!coordinator_ || !coordinator_->active_snapshot().initialized || !lidar.cloud ||
        lidar.cloud->empty()) {
        return;
    }
    std::vector<float> z_vals;
    z_vals.reserve(lidar.cloud->size());
    for (const auto& pt : *lidar.cloud) {
        if (pt.z > -20.0f && pt.z < 0.5f) z_vals.push_back(pt.z);
    }
    if (z_vals.empty()) return;
    const auto median_index = z_vals.size() / 2;
    std::nth_element(z_vals.begin(),
                     z_vals.begin() + static_cast<std::vector<float>::difference_type>(median_index),
                     z_vals.end());
    const double ground_z = z_vals[median_index];
    const auto pose = coordinator_->active_pose();
    const double height = pose.position.z() - ground_z;
    if (height > 0.3 && height < 100.0) {
        (void)coordinator_->process_measurement(make_lidar_depth_envelope(
            MeasurementStamp{lidar.timestamp, measurement_sequence_++}, pose.position.z(), 0.05,
            estimator_validation_cfg_.lidar_depth_correction_enabled));
    }
    refresh_runtime_telemetry_from_coordinator();
}

void VIOPipeline::apply_visual_quality_to_pose(PoseEstimate& pose) const {
    VisualFrontendMetrics metrics;
    {
        std::lock_guard lock(visual_metrics_mutex_);
        metrics = last_visual_metrics_;
    }
    if (metrics.visual_update_confidence <= 0.0) return;
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
    if (pose.localization_confidence < 0.22) pose.localization_lost = true;
}

void VIOPipeline::set_estimator_validation_config(const EstimatorValidationConfig& cfg) {
    estimator_validation_cfg_ = cfg;
    reconfigure_coordinator();
}

void VIOPipeline::set_shadow_msckf_config(const MsckfConfig& cfg) {
    shadow_msckf_cfg_ = cfg;
    reconfigure_coordinator();
}

void VIOPipeline::reconfigure_coordinator() {
    if (!coordinator_) return;
    coordinator_->configure_validation(estimator_validation_cfg_);
    coordinator_->configure_shadow_msckf(shadow_msckf_cfg_);
    coordinator_->stop();
    coordinator_->initialize();
    visual_feature_tracks_.reset();
    if (running_.load()) (void)coordinator_->start();
}

double VIOPipeline::drift_m() const {
    return current_pose().drift_m;
}

void VIOPipeline::refresh_runtime_telemetry_from_coordinator() {
    if (!coordinator_) return;
    const auto diagnostics = coordinator_->diagnostics();
    std::lock_guard lock(runtime_mutex_);
    runtime_telemetry_.active_estimator_name = diagnostics.active_estimator_name;
    runtime_telemetry_.shadow_estimator_name = diagnostics.shadow_estimator_name;
    runtime_telemetry_.active_estimator_health = std::string(to_string(diagnostics.active_health));
    runtime_telemetry_.shadow_estimator_health = std::string(to_string(diagnostics.shadow_health));
    runtime_telemetry_.shadow_enabled = diagnostics.shadow_enabled;
    runtime_telemetry_.shadow_lag_ms = diagnostics.shadow_lag_ms;
    runtime_telemetry_.shadow_queue_depth = diagnostics.queue.current_depth;
    runtime_telemetry_.shadow_queue_high_water_mark = diagnostics.queue.peak_depth;
    runtime_telemetry_.shadow_dropped_events = diagnostics.queue.dropped_count;
    runtime_telemetry_.shadow_position_delta_m = diagnostics.last_comparison.position_delta_norm_m;
    runtime_telemetry_.shadow_velocity_delta_mps = diagnostics.last_comparison.velocity_delta_norm_mps;
    runtime_telemetry_.shadow_orientation_delta_deg = diagnostics.last_comparison.orientation_delta_deg;
    runtime_telemetry_.shadow_divergence_active =
        diagnostics.last_comparison.valid &&
        (diagnostics.last_comparison.position_delta_norm_m >
             estimator_validation_cfg_.shadow_position_divergence_m ||
         diagnostics.last_comparison.velocity_delta_norm_mps >
             estimator_validation_cfg_.shadow_velocity_divergence_mps ||
         diagnostics.last_comparison.orientation_delta_deg >
             estimator_validation_cfg_.shadow_orientation_divergence_deg);
    runtime_telemetry_.shadow_last_failure_reason = diagnostics.last_shadow_failure_reason;
    runtime_telemetry_.shadow_comparable_snapshot_count = diagnostics.valid_comparison_count;
}

} // namespace drone::vio
