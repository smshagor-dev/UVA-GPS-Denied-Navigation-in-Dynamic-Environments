#include "localization/TimeSyncTracker.hpp"

#include <algorithm>
#include <cmath>

namespace drone::localization {

TimeSyncTracker::TimeSyncTracker()
    : TimeSyncTracker(Config{}) {}

TimeSyncTracker::TimeSyncTracker(Config cfg)
    : cfg_(cfg) {}

void TimeSyncTracker::observe_imu(double timestamp_s) {
    last_imu_ts_ = timestamp_s;
    if (last_camera_ts_ > 0.0) {
        push_sample(imu_camera_offsets_ms_, (last_camera_ts_ - last_imu_ts_) * 1000.0);
    }
}

void TimeSyncTracker::observe_camera(double timestamp_s) {
    last_camera_ts_ = timestamp_s;
    if (last_imu_ts_ > 0.0) {
        push_sample(imu_camera_offsets_ms_, (last_camera_ts_ - last_imu_ts_) * 1000.0);
    }
}

void TimeSyncTracker::observe_anchor(uint32_t, double arrival_time_s, double reference_time_s) {
    push_sample(anchor_offsets_ms_, (arrival_time_s - reference_time_s) * 1000.0);
}

void TimeSyncTracker::observe_peer(uint32_t, double remote_timestamp_s, double local_receive_time_s) {
    push_sample(peer_offsets_ms_, (local_receive_time_s - remote_timestamp_s) * 1000.0);
}

TimeSyncStatus TimeSyncTracker::status() const {
    TimeSyncStatus out;
    out.imu_camera_offset_ms = average(imu_camera_offsets_ms_);
    out.anchor_clock_offset_ms = average(anchor_offsets_ms_);
    out.peer_clock_offset_ms = average(peer_offsets_ms_);

    const double imu_jitter = mean_abs_deviation(imu_camera_offsets_ms_, out.imu_camera_offset_ms);
    const double anchor_jitter = mean_abs_deviation(anchor_offsets_ms_, out.anchor_clock_offset_ms);
    const double peer_jitter = mean_abs_deviation(peer_offsets_ms_, out.peer_clock_offset_ms);
    out.jitter_ms = std::max({imu_jitter, anchor_jitter, peer_jitter, 0.0});

    const double worst_offset = std::max({
        std::abs(out.imu_camera_offset_ms),
        std::abs(out.anchor_clock_offset_ms),
        std::abs(out.peer_clock_offset_ms),
        0.0
    });

    out.confidence = std::clamp(1.0 - (worst_offset / std::max(cfg_.degraded_threshold_ms, 1.0)), 0.0, 1.0);
    out.synchronized = worst_offset <= cfg_.synchronized_threshold_ms && out.jitter_ms <= cfg_.synchronized_threshold_ms;

    if (std::abs(out.anchor_clock_offset_ms) >= std::abs(out.imu_camera_offset_ms) &&
        std::abs(out.anchor_clock_offset_ms) >= std::abs(out.peer_clock_offset_ms)) {
        out.dominant_issue = "anchor-clock-offset";
    } else if (std::abs(out.peer_clock_offset_ms) >= std::abs(out.imu_camera_offset_ms)) {
        out.dominant_issue = "peer-clock-offset";
    } else {
        out.dominant_issue = "imu-camera-offset";
    }

    if (out.synchronized) {
        out.dominant_issue = "stable";
    }
    return out;
}

void TimeSyncTracker::reset() {
    imu_camera_offsets_ms_.clear();
    anchor_offsets_ms_.clear();
    peer_offsets_ms_.clear();
    last_imu_ts_ = -1.0;
    last_camera_ts_ = -1.0;
}

double TimeSyncTracker::average(const std::deque<double>& samples) {
    if (samples.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (double sample : samples) {
        sum += sample;
    }
    return sum / static_cast<double>(samples.size());
}

double TimeSyncTracker::mean_abs_deviation(const std::deque<double>& samples, double mean) {
    if (samples.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (double sample : samples) {
        sum += std::abs(sample - mean);
    }
    return sum / static_cast<double>(samples.size());
}

void TimeSyncTracker::push_sample(std::deque<double>& queue, double value) {
    queue.push_back(value);
    while (queue.size() > cfg_.max_samples) {
        queue.pop_front();
    }
}

} // namespace drone::localization
