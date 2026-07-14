// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "sensors/CameraSensor.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>

namespace drone::sensors {

namespace {

std::string lowercase(std::string_view value) {
    std::string out(value.begin(), value.end());
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

} // namespace

std::unordered_map<int, std::string> load_detector_label_map_json(const std::string& path) {
    std::unordered_map<int, std::string> label_map;
    if (path.empty() || !std::filesystem::exists(path)) {
        return label_map;
    }

    std::ifstream input(path);
    if (!input.is_open()) {
        return label_map;
    }

    std::stringstream buffer;
    buffer << input.rdbuf();
    const std::string content = buffer.str();

    const std::regex pair_pattern("\"([0-9]+)\"\\s*:\\s*\"([^\"]+)\"", std::regex::icase);
    std::sregex_iterator it(content.begin(), content.end(), pair_pattern);
    std::sregex_iterator end;
    for (; it != end; ++it) {
        const auto& match = *it;
        if (match.size() < 3) {
            continue;
        }
        try {
            const int class_id = std::stoi(match[1].str());
            label_map[class_id] = lowercase(match[2].str());
        } catch (...) {
        }
    }
    return label_map;
}

std::string resolve_detector_label(int class_id,
                                   const std::unordered_map<int, std::string>& label_map) {
    if (const auto found = label_map.find(class_id);
        found != label_map.end() && !found->second.empty()) {
        return found->second;
    }
    return "unknown_class_" + std::to_string(class_id);
}

bool CameraSensor::initialize() {
    if (logger_) {
        logger_->info("[{}] initialize stream={} resolution={}x{}", id_, stream_url_,
                      intrinsics_.width, intrinsics_.height);
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

CameraSensor::TelemetryStats CameraSensor::telemetry_stats() const {
    std::lock_guard lock(data_mutex_);
    TelemetryStats stats;
    stats.stream_active = state() == SensorState::RUNNING && cap_.isOpened();
    stats.simulated = false;
    stats.fps = fps_estimate_hz_;
    stats.frame_age_ms =
        latest_.has_value() ? std::max(0.0, (now_sec() - latest_->timestamp) * 1000.0) : 0.0;
    stats.dropped_frames = dropped_frames_;
    stats.width = latest_width_;
    stats.height = latest_height_;
    stats.latest_frame_id = latest_frame_id_;
    if (latest_frame_id_ > 0) {
        stats.latest_frame_ref = "frame-" + std::to_string(latest_frame_id_);
    }
    return stats;
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
        ++dropped_frames_;
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
        logger_->debug("[{}] frame={} detections={}", id_, frame.frame_id, frame.detections.size());
    }

    {
        std::lock_guard lock(data_mutex_);
        if (last_frame_timestamp_ > 0.0) {
            const double dt = frame.timestamp - last_frame_timestamp_;
            if (dt > 1.0e-6) {
                fps_estimate_hz_ = 1.0 / dt;
            }
        }
        last_frame_timestamp_ = frame.timestamp;
        latest_frame_id_ = frame.frame_id;
        latest_width_ = frame.image.cols;
        latest_height_ = frame.image.rows;
        latest_ = frame;
    }

    std::lock_guard lock(cb_mutex_);
    if (data_cb_) {
        data_cb_(frame);
    }
}

bool CameraSensor::load_yolo_model(const std::string& engine_path, float conf_thresh,
                                   float nms_thresh) {
    if (logger_) {
        logger_->info("[{}] load_yolo_model path={} conf={} nms={}", id_, engine_path, conf_thresh,
                      nms_thresh);
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
            logger_->info("[{}] OpenCV DNN fallback model load {}", id_,
                          inference_ready_ ? "succeeded" : "failed");
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

bool CameraSensor::load_detector_labels(const std::string& labels_path) {
    label_map_path_ = labels_path;
    label_map_ = load_detector_label_map_json(labels_path);
    if (logger_) {
        logger_->info("[{}] detector label map path={} entries={}", id_,
                      labels_path.empty() ? std::string("<none>") : labels_path, label_map_.size());
    }
    return !label_map_.empty();
}

void CameraSensor::precompute_undistort_maps() {
    if (logger_) {
        logger_->debug("[{}] precompute undistort maps", id_);
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << intrinsics_.fx, 0.0, intrinsics_.cx, 0.0, intrinsics_.fy,
                 intrinsics_.cy, 0.0, 0.0, 1.0);

    cv::Mat D(1, 5, CV_64F);
    for (int i = 0; i < 5; ++i) {
        D.at<double>(0, i) = intrinsics_.dist_coeffs[static_cast<size_t>(i)];
    }

    cv::initUndistortRectifyMap(K, D, cv::Mat(), K, cv::Size(intrinsics_.width, intrinsics_.height),
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

std::vector<Detection> CameraSensor::run_inference_dnn_fallback(const cv::Mat& frame) {
    if (dnn_net_.empty()) {
        return {};
    }

    cv::Mat blob =
        cv::dnn::blobFromImage(frame, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);

    if (blob.empty()) {
        return {};
    }

    dnn_net_.setInput(blob);
    std::vector<cv::Mat> outputs;
    dnn_net_.forward(outputs, dnn_net_.getUnconnectedOutLayersNames());
    if (outputs.empty()) {
        return {};
    }

    cv::Mat out = outputs.front();
    if (out.dims == 3 && out.size[1] < out.size[2]) {
        out = out.reshape(1, out.size[2]);
    } else if (out.dims == 3) {
        cv::transpose(out.reshape(1, out.size[1]), out);
    }
    if (out.cols < 6) {
        return {};
    }

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int row = 0; row < out.rows; ++row) {
        const float* data = out.ptr<float>(row);
        if (!data) {
            continue;
        }

        const int class_count = out.cols - 4;
        cv::Mat scores(1, class_count, CV_32F, const_cast<float*>(data + 4));
        cv::Point class_id_point;
        double max_class_score = 0.0;
        cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id_point);
        if (max_class_score < conf_thresh_) {
            continue;
        }

        const float cx = data[0];
        const float cy = data[1];
        const float w = data[2];
        const float h = data[3];
        boxes.emplace_back(
            static_cast<int>((cx - (w * 0.5f)) * static_cast<float>(intrinsics_.width) / 640.0f),
            static_cast<int>((cy - (h * 0.5f)) * static_cast<float>(intrinsics_.height) / 640.0f),
            static_cast<int>(w * static_cast<float>(intrinsics_.width) / 640.0f),
            static_cast<int>(h * static_cast<float>(intrinsics_.height) / 640.0f));
        confidences.push_back(static_cast<float>(max_class_score));
        class_ids.push_back(class_id_point.x);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, confidences, conf_thresh_, nms_thresh_, keep);

    std::vector<Detection> detections;
    detections.reserve(keep.size());
    for (const int index : keep) {
        const auto& box = boxes[static_cast<size_t>(index)];
        Detection detection;
        detection.class_id = class_ids[static_cast<size_t>(index)];
        detection.confidence = confidences[static_cast<size_t>(index)];
        detection.label = resolve_detector_label(detection.class_id, label_map_);
        detection.bbox = cv::Rect2f(
            std::clamp(static_cast<float>(box.x) / static_cast<float>(intrinsics_.width), 0.0f,
                       1.0f),
            std::clamp(static_cast<float>(box.y) / static_cast<float>(intrinsics_.height), 0.0f,
                       1.0f),
            std::clamp(static_cast<float>(box.width) / static_cast<float>(intrinsics_.width), 0.0f,
                       1.0f),
            std::clamp(static_cast<float>(box.height) / static_cast<float>(intrinsics_.height),
                       0.0f, 1.0f));
        detections.push_back(std::move(detection));
    }

    return detections;
}

} // namespace drone::sensors
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
