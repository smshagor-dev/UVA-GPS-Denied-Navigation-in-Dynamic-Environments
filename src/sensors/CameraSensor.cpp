// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "sensors/CameraSensor.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <filesystem>

namespace drone::sensors {

bool CameraSensor::initialize() {
    if (logger_) {
        logger_->info("[{}] initialize stream={} resolution={}x{}",
                      id_, stream_url_, intrinsics_.width, intrinsics_.height);
    }
    precompute_undistort_maps();
    cap_.open(stream_url_);
    if (!cap_.isOpened()) {
        report_error("Camera stream unavailable: " + stream_url_);
        set_state(SensorState::DEGRADED);
        return false;
    }

    poll_rate_hz_ = 30;
    set_state(SensorState::RUNNING);
    return true;
}

bool CameraSensor::reconfigure(const std::string&) {
    if (logger_) {
        logger_->info("[{}] reconfigure camera intrinsics", id_);
    }
    precompute_undistort_maps();
    return true;
}

void CameraSensor::poll() {
    if (!cap_.isOpened()) {
        if (logger_) {
            logger_->warn("[{}] poll skipped because stream is not opened", id_);
        }
        return;
    }

    cv::Mat raw;
    if (!cap_.read(raw) || raw.empty()) {
        if (logger_) {
            logger_->debug("[{}] frame read failed or empty", id_);
        }
        return;
    }

    CameraFrame frame;
    frame.timestamp = now_sec();
    frame.source_id = id_;
    frame.frame_id = ++frame_counter_;
    frame.image = undistort(raw);
    frame.detections = run_inference(frame.image);

    if (logger_) {
        logger_->debug("[{}] frame={} detections={}",
                       id_, frame.frame_id, frame.detections.size());
    }

    {
        std::lock_guard lock(data_mutex_);
        latest_ = frame;
    }

    std::lock_guard lock(cb_mutex_);
    if (data_cb_) {
        data_cb_(frame);
    }
}

bool CameraSensor::load_yolo_model(const std::string& engine_path,
                                   float conf_thresh,
                                   float nms_thresh) {
    if (logger_) {
        logger_->info("[{}] load_yolo_model path={} conf={} nms={}",
                      id_, engine_path, conf_thresh, nms_thresh);
    }
    conf_thresh_ = conf_thresh;
    nms_thresh_ = nms_thresh;

    if (!std::filesystem::exists(engine_path)) {
        inference_ready_ = false;
        if (logger_) {
            logger_->warn("[{}] YOLO model not found at {}", id_, engine_path);
        }
        return false;
    }

#ifdef DRONE_HAS_TENSORRT
    inference_ready_ = true;
    return true;
#else
    try {
        dnn_net_ = cv::dnn::readNet(engine_path);
        inference_ready_ = !dnn_net_.empty();
        if (logger_) {
            logger_->info("[{}] OpenCV DNN fallback model load {}", id_, inference_ready_ ? "succeeded" : "failed");
        }
    } catch (...) {
        inference_ready_ = false;
        if (logger_) {
            logger_->error("[{}] OpenCV DNN fallback model load threw exception", id_);
        }
    }
    return inference_ready_;
#endif
}

void CameraSensor::precompute_undistort_maps() {
    if (logger_) {
        logger_->debug("[{}] precompute undistort maps", id_);
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        intrinsics_.fx, 0.0, intrinsics_.cx,
        0.0, intrinsics_.fy, intrinsics_.cy,
        0.0, 0.0, 1.0);

    cv::Mat D(1, 5, CV_64F);
    for (int i = 0; i < 5; ++i) {
        D.at<double>(0, i) = intrinsics_.dist_coeffs[static_cast<size_t>(i)];
    }

    cv::initUndistortRectifyMap(
        K, D, cv::Mat(), K,
        cv::Size(intrinsics_.width, intrinsics_.height),
        CV_32FC1, map1_, map2_);
}

cv::Mat CameraSensor::undistort(const cv::Mat& raw) const {
    if (map1_.empty() || map2_.empty()) {
        return raw.clone();
    }

    cv::Mat out;
    cv::remap(raw, out, map1_, map2_, cv::INTER_LINEAR);
    return out;
}

std::vector<Detection> CameraSensor::run_inference(const cv::Mat& frame) {
    if (!inference_ready_) {
        if (logger_) {
            logger_->debug("[{}] run_inference skipped because inference is disabled", id_);
        }
        return {};
    }

#ifdef DRONE_HAS_TENSORRT
    return {};
#else
    return run_inference_dnn_fallback(frame);
#endif
}

std::vector<Detection> CameraSensor::run_inference_dnn_fallback(const cv::Mat&) {
    if (logger_) {
        logger_->debug("[{}] run_inference_dnn_fallback placeholder invoked", id_);
    }
    return {};
}

} // namespace drone::sensors
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
