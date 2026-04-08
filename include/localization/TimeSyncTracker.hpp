#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>

namespace drone::localization {

struct TimeSyncStatus {
    double imu_camera_offset_ms{0.0};
    double anchor_clock_offset_ms{0.0};
    double peer_clock_offset_ms{0.0};
    double jitter_ms{0.0};
    double confidence{1.0};
    bool synchronized{true};
    std::string dominant_issue{"stable"};
};

class TimeSyncTracker {
public:
    struct Config {
        size_t max_samples{128};
        double synchronized_threshold_ms{8.0};
        double degraded_threshold_ms{20.0};
    };

    explicit TimeSyncTracker(Config cfg = {});

    void observe_imu(double timestamp_s);
    void observe_camera(double timestamp_s);
    void observe_anchor(uint32_t anchor_id, double arrival_time_s, double reference_time_s);
    void observe_peer(uint32_t peer_id, double remote_timestamp_s, double local_receive_time_s);

    [[nodiscard]] TimeSyncStatus status() const;
    void reset();

private:
    static double average(const std::deque<double>& samples);
    static double mean_abs_deviation(const std::deque<double>& samples, double mean);

    void push_sample(std::deque<double>& queue, double value);

    Config cfg_;
    std::deque<double> imu_camera_offsets_ms_;
    std::deque<double> anchor_offsets_ms_;
    std::deque<double> peer_offsets_ms_;
    double last_imu_ts_{-1.0};
    double last_camera_ts_{-1.0};
};

} // namespace drone::localization
