// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once
 
// SensorBase.hpp  â€”  Abstract base for all drone sensors
// Drone Swarm Sensor Fusion  |  Phase 2: Core C++ Sensor Engine
 
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace drone::sensors {

// â”€â”€â”€ Timestamp alias â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
using Timestamp = double; // seconds since epoch

inline Timestamp now_sec() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// â”€â”€â”€ Sensor health states â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
enum class SensorState : uint8_t {
    UNINITIALIZED = 0,
    INITIALIZING,
    RUNNING,
    DEGRADED,   // partial data quality
    FAILED,
    DISCONNECTED
};

inline std::string_view to_string(SensorState s) {
    switch (s) {
    case SensorState::UNINITIALIZED:  return "UNINITIALIZED";
    case SensorState::INITIALIZING:   return "INITIALIZING";
    case SensorState::RUNNING:        return "RUNNING";
    case SensorState::DEGRADED:       return "DEGRADED";
    case SensorState::FAILED:         return "FAILED";
    case SensorState::DISCONNECTED:   return "DISCONNECTED";
    }
    return "UNKNOWN";
}

// â”€â”€â”€ Generic sensor measurement â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
struct SensorMeasurement {
    Timestamp   timestamp{0.0};
    SensorState quality{SensorState::RUNNING};
    float       confidence{1.0f};  // 0.0 â€“ 1.0
    std::string source_id;
};

// â”€â”€â”€ Callback types â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
template <typename T>
using DataCallback = std::function<void(const T&)>;
using ErrorCallback = std::function<void(const std::string&)>;

 
// SensorBase  â€”  CRTP-free abstract interface
 
class SensorBase {
public:
    explicit SensorBase(std::string sensor_id, std::string sensor_type)
        : id_(std::move(sensor_id))
        , type_(std::move(sensor_type))
        , state_(SensorState::UNINITIALIZED) {
        logger_ = spdlog::get(id_);
        if (!logger_) {
            logger_ = spdlog::stdout_color_mt(id_);
        }
    }

    virtual ~SensorBase() { stop(); }

    // Non-copyable, movable
    SensorBase(const SensorBase&)            = delete;
    SensorBase& operator=(const SensorBase&) = delete;
    SensorBase(SensorBase&&)                 = default;
    SensorBase& operator=(SensorBase&&)      = default;

    // â”€â”€ Lifecycle â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    virtual bool initialize()                = 0;
    virtual bool start();
    virtual void stop();
    virtual bool reconfigure(const std::string& config_json) = 0;

    // â”€â”€ Data access (non-blocking, returns std::optional) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    virtual void poll() = 0;  // called periodically by sensor thread

    // â”€â”€ Status â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    [[nodiscard]] SensorState   state()        const noexcept { return state_.load(); }
    [[nodiscard]] std::string_view sensor_id() const noexcept { return id_; }
    [[nodiscard]] std::string_view sensor_type() const noexcept { return type_; }
    [[nodiscard]] float         dropout_rate() const noexcept { return dropout_rate_; }
    [[nodiscard]] uint64_t      sample_count() const noexcept { return sample_count_; }

    // â”€â”€ Error callback â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    void set_error_callback(ErrorCallback cb) {
        std::lock_guard lock(cb_mutex_);
        error_cb_ = std::move(cb);
    }

protected:
    void set_state(SensorState s) {
        state_.store(s);
        logger_->info("[{}] state â†’ {}", id_, to_string(s));
    }

    void report_error(const std::string& msg) {
        logger_->error("[{}] {}", id_, msg);
        std::lock_guard lock(cb_mutex_);
        if (error_cb_) error_cb_(msg);
    }

    void increment_samples() {
        ++sample_count_;
    }

    std::shared_ptr<spdlog::logger> logger_;
    std::string                     id_;
    std::string                     type_;
    std::atomic<SensorState>        state_;

    // Acquisition thread
    std::thread                     acq_thread_;
    std::atomic<bool>               running_{false};

    mutable std::mutex              data_mutex_;
    mutable std::mutex              cb_mutex_;
    ErrorCallback                   error_cb_;

    float     dropout_rate_{0.0f};
    uint64_t  sample_count_{0};
    uint32_t  poll_rate_hz_{100};
};

// â”€â”€â”€ Inline implementations â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
inline bool SensorBase::start() {
    if (running_.exchange(true)) return true; // already running
    set_state(SensorState::RUNNING);
    acq_thread_ = std::thread([this] {
        const auto interval = std::chrono::microseconds(1'000'000 / poll_rate_hz_);
        while (running_.load()) {
            auto next = std::chrono::steady_clock::now() + interval;
            try {
                poll();
                increment_samples();
            } catch (const std::exception& e) {
                report_error(e.what());
                set_state(SensorState::DEGRADED);
            }
            std::this_thread::sleep_until(next);
        }
    });
    logger_->info("[{}] acquisition thread started @ {}Hz", id_, poll_rate_hz_);
    return true;
}

inline void SensorBase::stop() {
    if (!running_.exchange(false)) return;
    if (acq_thread_.joinable()) acq_thread_.join();
    set_state(SensorState::DISCONNECTED);
    logger_->info("[{}] stopped. {} samples collected", id_, sample_count_);
}

} // namespace drone::sensors
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
