// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

 
// IMUSensor.cpp    IMU acquisition, calibration, and I2C integration
// Drone Swarm Sensor Fusion  |  Phase 2
 
#include "sensors/IMUSensor.hpp"
#include <cmath>
#include <cstring>
#include <numbers>
#include <random>
#include <stdexcept>

// Linux I2C
#ifdef __linux__
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace drone::sensors {

// MPU-6050 register map
static constexpr uint8_t MPU_PWR_MGMT_1  = 0x6B;
static constexpr uint8_t MPU_SMPLRT_DIV  = 0x19;
static constexpr uint8_t MPU_CONFIG      = 0x1A;
static constexpr uint8_t MPU_GYRO_CONFIG = 0x1B;
static constexpr uint8_t MPU_ACCEL_CONFIG= 0x1C;
static constexpr uint8_t MPU_ACCEL_XOUT  = 0x3B;
static constexpr double kAccelScale_2G  = 9.81 / 16384.0;  // m/sÂ²/LSB
static constexpr double kGyroScale_250  =
    (250.0 / 32768.0) * (std::numbers::pi_v<double> / 180.0); // rad/s/LSB

 
bool IMUSensor::initialize() {
    set_state(SensorState::INITIALIZING);

#ifdef __linux__
    fd_ = ::open(device_path_.c_str(), O_RDWR);
    if (fd_ < 0) {
        report_error("Failed to open I2C device: " + device_path_);
        set_state(SensorState::FAILED);
        return false;
    }

    if (::ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) {
        report_error("Failed to set I2C slave address");
        ::close(fd_);
        fd_ = -1;
        set_state(SensorState::FAILED);
        return false;
    }

    // Wake up MPU-6050 (clear sleep bit)
    uint8_t buf[2] = {MPU_PWR_MGMT_1, 0x00};
    if (::write(fd_, buf, 2) != 2) {
        report_error("Failed to wake up MPU-6050");
        set_state(SensorState::FAILED);
        return false;
    }

    auto write_register = [this](uint8_t reg, uint8_t value, std::string_view step) {
        const uint8_t command[2] = {reg, value};
        if (::write(fd_, command, sizeof(command)) != static_cast<ssize_t>(sizeof(command))) {
            report_error("Failed to configure MPU-6050 " + std::string(step));
            set_state(SensorState::FAILED);
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        return true;
    };

    if (!write_register(MPU_SMPLRT_DIV, 0x00, "sample rate") ||
        !write_register(MPU_CONFIG, 0x02, "DLPF") ||
        !write_register(MPU_GYRO_CONFIG, 0x00, "gyro range") ||
        !write_register(MPU_ACCEL_CONFIG, 0x00, "accel range")) {
        return false;
    }
#else
    // Simulation mode on non-Linux (x86 dev/CI)
    logger_->warn("[{}] Non-Linux platform  running in simulation mode", id_);
#endif

    poll_rate_hz_ = 400;
    set_state(SensorState::RUNNING);
    logger_->info("[{}] Initialized on {} addr=0x{:02X}", id_, device_path_, i2c_addr_);
    return true;
}

IMUSensor::TelemetryStats IMUSensor::telemetry_stats() const {
    std::lock_guard lock(data_mutex_);
    TelemetryStats stats;
    stats.device_active = state() == SensorState::RUNNING;
    stats.simulated = last_measurement_simulated_;
    stats.sample_rate_hz = sample_rate_estimate_hz_;
    stats.last_sample_age_ms = latest_.has_value() ? std::max(0.0, (now_sec() - latest_->timestamp) * 1000.0) : 0.0;
    if (latest_.has_value()) {
        stats.accel_mps2 = latest_->accel_mps2;
        stats.gyro_rads = latest_->gyro_rads;
        stats.health = stats.simulated ? "simulation" : "good";
    } else {
        stats.health = "unavailable";
    }
    return stats;
}

 
void IMUSensor::poll() {
    ImuMeasurement m = read_raw();
    m = apply_calibration(m);

    {
        std::lock_guard lock(data_mutex_);
        if (last_sample_timestamp_ > 0.0) {
            const double dt = m.timestamp - last_sample_timestamp_;
            if (dt > 1.0e-6) {
                sample_rate_estimate_hz_ = 1.0 / dt;
            }
        }
        last_sample_timestamp_ = m.timestamp;
        last_measurement_simulated_ = std::abs(m.accel_mps2.z() - 9.81) < 0.25 && fd_ < 0;
        latest_ = m;
        buffer_.push_back(m);
        if (buffer_.size() > kBufferMax)
            buffer_.pop_front();
    }

    std::lock_guard lock(cb_mutex_);
    if (data_cb_) data_cb_(m);
}

 
ImuMeasurement IMUSensor::read_raw() {
    ImuMeasurement m;
    m.timestamp = now_sec();
    m.source_id = id_;

#ifdef __linux__
    if (fd_ >= 0) {
        // Burst-read 14 bytes starting at ACCEL_XOUT_H
        uint8_t reg = MPU_ACCEL_XOUT;
        if (::write(fd_, &reg, 1) == 1) {
            uint8_t raw[14]{};
            if (::read(fd_, raw, 14) == 14) {
                auto to_int16 = [](uint8_t h, uint8_t l) -> int16_t {
                    return static_cast<int16_t>((h << 8) | l);
                };

                m.accel_mps2.x() = to_int16(raw[0],  raw[1])  * kAccelScale_2G;
                m.accel_mps2.y() = to_int16(raw[2],  raw[3])  * kAccelScale_2G;
                m.accel_mps2.z() = to_int16(raw[4],  raw[5])  * kAccelScale_2G;

                const int16_t raw_temp = to_int16(raw[6], raw[7]);
                m.temperature_c = raw_temp / 340.0f + 36.53f;

                m.gyro_rads.x() = to_int16(raw[8],  raw[9])  * kGyroScale_250;
                m.gyro_rads.y() = to_int16(raw[10], raw[11]) * kGyroScale_250;
                m.gyro_rads.z() = to_int16(raw[12], raw[13]) * kGyroScale_250;

                m.quality     = SensorState::RUNNING;
                m.confidence  = 1.0f;
                return m;
            }
        }
    }
#endif
    // Deterministic simulation: stationary with small noise
    const double t = m.timestamp;
    std::mt19937_64 rng(static_cast<uint64_t>(t * 1e6));
    std::normal_distribution<double> noise(0.0, noise_.accel_noise_density);

    m.accel_mps2 = Eigen::Vector3d{noise(rng), noise(rng), 9.81 + noise(rng)};
    m.gyro_rads  = Eigen::Vector3d{noise(rng)*0.1, noise(rng)*0.1, noise(rng)*0.1};
    m.temperature_c = 45.0f;
    m.quality    = SensorState::RUNNING;
    return m;
}

 
ImuMeasurement IMUSensor::apply_calibration(ImuMeasurement m) const {
    m.accel_mps2 = accel_scale_ * (m.accel_mps2 - accel_bias_);
    m.gyro_rads  = m.gyro_rads - gyro_bias_;
    return m;
}

 
bool IMUSensor::calibrate_static(uint32_t n_samples) {
    logger_->info("[{}] Starting static calibration ({} samples)â€¦", id_, n_samples);

    if (state() != SensorState::RUNNING) {
        if (!initialize()) return false;
    }

    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_sum  = Eigen::Vector3d::Zero();

    for (uint32_t i = 0; i < n_samples; ++i) {
        ImuMeasurement m = read_raw();
        accel_sum += m.accel_mps2;
        gyro_sum  += m.gyro_rads;
        std::this_thread::sleep_for(std::chrono::microseconds(2500)); // 400Hz
    }

    const double inv = 1.0 / n_samples;
    gyro_bias_ = gyro_sum * inv;

    // Accel bias: remove gravity (assume +Z up)
    accel_bias_ = accel_sum * inv - Eigen::Vector3d{0, 0, 9.81};

    logger_->info("[{}] Calibration done. ba=[{:.4f},{:.4f},{:.4f}]  bg=[{:.5f},{:.5f},{:.5f}]",
                  id_,
                  accel_bias_.x(), accel_bias_.y(), accel_bias_.z(),
                  gyro_bias_.x(),  gyro_bias_.y(),  gyro_bias_.z());
    return true;
}

 
bool IMUSensor::reconfigure(const std::string& config_json) {
    // In production: parse JSON, update noise model / range / filter
    logger_->info("[{}] reconfigure: {}", id_, config_json);
    return true;
}

} // namespace drone::sensors
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
