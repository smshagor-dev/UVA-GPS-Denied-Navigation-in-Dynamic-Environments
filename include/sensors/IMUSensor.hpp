// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

// IMUSensor.hpp    IMU (MPU-6050 / ICM-42688-P) sensor interface
// Drone Swarm Sensor Fusion  |  Phase 2

#include "sensors/SensorBase.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <optional>

namespace drone::sensors {

//  IMU Measurement
struct ImuMeasurement : SensorMeasurement {
    Eigen::Vector3d accel_mps2{0, 0, 0}; // m/sÂ²  (body frame)
    Eigen::Vector3d gyro_rads{0, 0, 0};  // rad/s (body frame)
    Eigen::Vector3d mag_gauss{0, 0, 0};  // Gauss (optional magnetometer)
    float temperature_c{25.0f};          // Â°C (for bias compensation)

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//  IMU Noise & Bias Model
struct ImuNoiseModel {
    double accel_noise_density{2.0e-3}; // m/sÂ²/âˆšHz  (datasheet)
    double gyro_noise_density{1.6e-4};  // rad/s/âˆšHz
    double accel_bias_instability{3.0e-5};
    double gyro_bias_instability{3.0e-6};
    double accel_random_walk{3.0e-3}; // m/sÂ²Â·âˆšHz
    double gyro_random_walk{1.0e-5};  // rad/sÂ·âˆšHz
};

class IMUSensor : public SensorBase {
public:
    struct TelemetryStats {
        bool device_active{false};
        bool simulated{false};
        double sample_rate_hz{0.0};
        double last_sample_age_ms{0.0};
        Eigen::Vector3d accel_mps2{Eigen::Vector3d::Zero()};
        Eigen::Vector3d gyro_rads{Eigen::Vector3d::Zero()};
        std::string health{"unavailable"};
    };

    explicit IMUSensor(std::string id, std::string device_path = "/dev/i2c-1",
                       uint8_t i2c_addr = 0x68)
        : SensorBase(std::move(id), "IMU"), device_path_(std::move(device_path)),
          i2c_addr_(i2c_addr) {}

    //  Lifecycle
    bool initialize() override;
    bool reconfigure(const std::string& config_json) override;
    void poll() override;

    //  Data
    [[nodiscard]] std::optional<ImuMeasurement> latest() const {
        std::lock_guard lock(data_mutex_);
        return latest_;
    }

    // Returns a snapshot of the internal buffer (thread-safe)
    [[nodiscard]] std::vector<ImuMeasurement> drain_buffer() {
        std::lock_guard lock(data_mutex_);
        std::vector<ImuMeasurement> out(buffer_.begin(), buffer_.end());
        buffer_.clear();
        return out;
    }

    void set_data_callback(DataCallback<ImuMeasurement> cb) {
        std::lock_guard lock(cb_mutex_);
        data_cb_ = std::move(cb);
    }

    //  Calibration â”€
    // Collect N static samples to estimate bias
    bool calibrate_static(uint32_t samples = 2000);
    [[nodiscard]] const Eigen::Vector3d& accel_bias() const {
        return accel_bias_;
    }
    [[nodiscard]] const Eigen::Vector3d& gyro_bias() const {
        return gyro_bias_;
    }
    [[nodiscard]] const ImuNoiseModel& noise_model() const {
        return noise_;
    }
    [[nodiscard]] TelemetryStats telemetry_stats() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    ImuMeasurement read_raw();
    ImuMeasurement apply_calibration(ImuMeasurement m) const;

    std::string device_path_;
    uint8_t i2c_addr_;
    int fd_{-1}; // file descriptor for I2C

    std::optional<ImuMeasurement> latest_;
    std::deque<ImuMeasurement> buffer_;
    static constexpr size_t kBufferMax{4096};
    double sample_rate_estimate_hz_{0.0};
    double last_sample_timestamp_{0.0};
    bool last_measurement_simulated_{false};

    Eigen::Vector3d accel_bias_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d accel_scale_{Eigen::Matrix3d::Identity()};
    ImuNoiseModel noise_;

    DataCallback<ImuMeasurement> data_cb_;
};

} // namespace drone::sensors
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
