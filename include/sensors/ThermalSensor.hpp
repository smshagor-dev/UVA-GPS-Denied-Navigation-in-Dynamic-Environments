// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once
 
// ThermalSensor.hpp    MLX90640 32Ã—24 IR thermal array
// Drone Swarm Sensor Fusion  |  Phase 2
 
#include "sensors/SensorBase.hpp"
#include <array>
#include <optional>

namespace drone::sensors {

constexpr int kThermalWidth  = 32;
constexpr int kThermalHeight = 24;
constexpr int kThermalPixels = kThermalWidth * kThermalHeight;

struct ThermalFrame : SensorMeasurement {
    // Pixel temperatures in Celsius, row-major
    std::array<float, kThermalPixels> pixels{};
    float ambient_temp_c{25.0f};
    float min_temp_c{0.0f};
    float max_temp_c{0.0f};
    int   width{kThermalWidth};
    int   height{kThermalHeight};

    [[nodiscard]] float at(int row, int col) const {
        return pixels[row * kThermalWidth + col];
    }
};

class ThermalSensor : public SensorBase {
public:
    explicit ThermalSensor(std::string id,
                            std::string bus     = "/dev/i2c-1",
                            uint8_t     addr    = 0x33,
                            float       emissivity = 0.95f)
        : SensorBase(std::move(id), "Thermal")
        , bus_(std::move(bus)), addr_(addr)
        , emissivity_(emissivity) {}

    bool initialize() override;
    bool reconfigure(const std::string& config_json) override;
    void poll() override;

    [[nodiscard]] std::optional<ThermalFrame> latest() const {
        std::lock_guard lock(data_mutex_);
        return latest_;
    }

    void set_data_callback(DataCallback<ThermalFrame> cb) {
        std::lock_guard lock(cb_mutex_);
        data_cb_ = std::move(cb);
    }

private:
    ThermalFrame read_mlx90640();

    std::string  bus_;
    uint8_t      addr_;
    float        emissivity_;
    int          fd_{-1};

    std::optional<ThermalFrame> latest_;
    DataCallback<ThermalFrame>  data_cb_;

    // MLX90640 calibration data (loaded once at init)
    std::array<uint16_t, 832> eeprom_data_{};
};

} // namespace drone::sensors
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
