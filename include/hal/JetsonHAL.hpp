// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once
// JetsonHAL.hpp  â€”  Hardware Abstraction Layer
// Jetson Nano / Raspberry Pi 4  â†”  ESP32-CAM / LiDAR / IMU
// Drone Swarm Sensor Fusion  |  Phase 2 â€” Hardware Integration
#include <functional>
#include <memory>
#include <string>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>

namespace drone::hal {

// Platform detection
enum class Platform { UNKNOWN, JETSON_NANO, JETSON_ORIN, RPI4, RPI5, X86_SIM };
Platform detect_platform();
std::string_view to_string(Platform p);

 
// ESP32CamInterface  â€”  RTSP / UDP frame receiver from ESP32-CAM
//
// ESP32-CAM firmware (see /firmware/esp32_cam/) streams:
//   - RTSP at rtsp://<ip>:554/stream (H264, 30fps, 640Ã—480)
//   - OR raw MJPEG over UDP port 1234 for low-latency path
 
class ESP32CamInterface {
public:
    enum class StreamMode { RTSP, UDP_MJPEG };

    struct Config {
        std::string ip{"192.168.4.1"};     // ESP32-CAM AP default
        uint16_t    port{554};             // RTSP default
        StreamMode  mode{StreamMode::RTSP};
        int         width{640};
        int         height{480};
        int         fps_target{30};
        bool        hardware_decode{true}; // use HW H264 decoder on Jetson
        int         reconnect_attempts{5};
        double      reconnect_delay_s{2.0};
    };

    using FrameCallback = std::function<void(const cv::Mat&, double timestamp)>;

    explicit ESP32CamInterface(Config cfg = {});
    ~ESP32CamInterface();

    bool     connect();
    void     disconnect();
    bool     is_connected() const;

    // Blocking grab (use in acquisition thread)
    bool     grab_frame(cv::Mat& out, double& timestamp_sec);

    // Async path â€” callback on each new frame
    void     set_frame_callback(FrameCallback cb);
    void     start_async();
    void     stop_async();

    [[nodiscard]] float     actual_fps()    const { return fps_actual_; }
    [[nodiscard]] uint32_t  dropped_frames()const { return dropped_frames_; }
    [[nodiscard]] float     latency_ms()    const { return latency_ms_; }

private:
    std::string build_url() const;
    bool        try_reconnect();

    Config           cfg_;
    cv::VideoCapture cap_;
    bool             connected_{false};

    std::thread      async_thread_;
    std::atomic<bool>async_running_{false};
    FrameCallback    frame_cb_;

    float    fps_actual_{0.0f};
    uint32_t dropped_frames_{0};
    float    latency_ms_{0.0f};
    uint32_t frame_count_{0};
    double   fps_window_start_{0.0};

    std::shared_ptr<spdlog::logger> logger_{spdlog::get("HAL_ESP32")};
};

 
// I2CDevice  â€”  Generic I2C r/w wrapper (Linux /dev/i2c-N)
 
class I2CDevice {
public:
    explicit I2CDevice(std::string bus = "/dev/i2c-1", uint8_t addr = 0x68);
    ~I2CDevice();

    bool    open();
    void    close();
    bool    write_register(uint8_t reg, uint8_t val);
    bool    read_registers(uint8_t reg, uint8_t* buf, size_t len);
    uint8_t read_byte(uint8_t reg);
    int16_t read_word_be(uint8_t reg);  // big-endian (MPU-6050 style)

    [[nodiscard]] bool is_open() const { return fd_ >= 0; }

private:
    std::string bus_;
    uint8_t     addr_;
    int         fd_{-1};

    std::shared_ptr<spdlog::logger> logger_{spdlog::get("I2C")};
};

 
// UARTInterface  â€”  UART for RPLIDAR A3 / serial debug
 
class UARTInterface {
public:
    explicit UARTInterface(std::string device = "/dev/ttyUSB0", int baud = 115200);
    ~UARTInterface();

    bool open();
    void close();
    int  read(uint8_t* buf, size_t len, int timeout_ms = 100);
    int  write(const uint8_t* buf, size_t len);

    [[nodiscard]] bool is_open() const;

private:
    std::string device_;
    int         baud_;
    int         fd_{-1};

    std::shared_ptr<spdlog::logger> logger_{spdlog::get("UART")};
};

 
// SystemMonitor  â€”  CPU, GPU, memory, battery stats (Jetson/RPi)
 
struct SystemStats {
    float cpu_pct{0};
    float gpu_pct{0};         // Jetson only
    float mem_used_mb{0};
    float mem_total_mb{0};
    float cpu_temp_c{0};
    float gpu_temp_c{0};
    float battery_v{0};
    float battery_pct{0};
    float wifi_rssi_dbm{-100};
    float motor_health{1.0f};
    float motor_temp_c{0.0f};
};

SystemStats read_system_stats();

} // namespace drone::hal
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
