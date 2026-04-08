// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

 
// JetsonHAL.cpp    Hardware abstraction: I2C, UART, ESP32-CAM, system stats
// Drone Swarm Sensor Fusion  |  Phase 2
 
#include "hal/JetsonHAL.hpp"
#include <fstream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>

#include <spdlog/sinks/stdout_color_sinks.h>

#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#endif

namespace drone::hal {

 
// Platform detection
 
Platform detect_platform() {
#ifdef __linux__
    std::ifstream nv("/etc/nv_tegra_release");
    if (nv.is_open()) {
        std::string line;
        std::getline(nv, line);
        if (line.find("R36") != std::string::npos ||
            line.find("R35") != std::string::npos)
            return Platform::JETSON_ORIN;
        return Platform::JETSON_NANO;
    }

    std::ifstream model("/proc/device-tree/model");
    if (model.is_open()) {
        std::string m;
        std::getline(model, m);
        if (m.find("Raspberry Pi 5") != std::string::npos) return Platform::RPI5;
        if (m.find("Raspberry Pi 4") != std::string::npos) return Platform::RPI4;
    }
    return Platform::UNKNOWN;
#elif defined(__x86_64__)
    return Platform::X86_SIM;
#else
    return Platform::UNKNOWN;
#endif
}

std::string_view to_string(Platform p) {
    switch (p) {
    case Platform::JETSON_NANO:  return "Jetson Nano";
    case Platform::JETSON_ORIN:  return "Jetson Orin";
    case Platform::RPI4:         return "Raspberry Pi 4";
    case Platform::RPI5:         return "Raspberry Pi 5";
    case Platform::X86_SIM:      return "x86 Simulation";
    default:                     return "Unknown";
    }
}

 
// SystemStats
 
static float read_sysfs_float(const std::string& path, float scale = 1.0f) {
    std::ifstream f(path);
    if (!f.is_open()) return 0.0f;
    float val = 0.0f;
    f >> val;
    return val * scale;
}

SystemStats read_system_stats() {
    SystemStats s{};

#ifdef __linux__
    // CPU usage (instantaneous from /proc/stat)
    {
        static uint64_t prev_idle = 0, prev_total = 0;
        std::ifstream stat("/proc/stat");
        if (stat.is_open()) {
            std::string line;
            std::getline(stat, line);
            std::istringstream ss(line);
            std::string cpu;
            uint64_t user,nice,sys,idle,iowait,irq,softirq,steal;
            ss >> cpu >> user >> nice >> sys >> idle >> iowait >> irq >> softirq >> steal;
            uint64_t total = user+nice+sys+idle+iowait+irq+softirq+steal;
            uint64_t diff_idle  = idle  - prev_idle;
            uint64_t diff_total = total - prev_total;
            if (diff_total > 0)
                s.cpu_pct = 100.0f * (1.0f - static_cast<float>(diff_idle) / diff_total);
            prev_idle  = idle;
            prev_total = total;
        }
    }

    // Memory
    {
        std::ifstream meminfo("/proc/meminfo");
        if (meminfo.is_open()) {
            std::string key;
            uint64_t val;
            std::string unit;
            uint64_t total = 0, available = 0;
            std::string line;
            while (std::getline(meminfo, line)) {
                std::istringstream ss(line);
                ss >> key >> val >> unit;
                if (key == "MemTotal:")     total     = val;
                if (key == "MemAvailable:") available = val;
            }
            s.mem_total_mb = static_cast<float>(total)    / 1024.0f;
            s.mem_used_mb  = static_cast<float>(total - available) / 1024.0f;
        }
    }

    // CPU temperature (Jetson/RPi thermal zone)
    s.cpu_temp_c = read_sysfs_float(
        "/sys/class/thermal/thermal_zone0/temp", 0.001f);  // millideg  deg

    // Jetson GPU
    if (detect_platform() == Platform::JETSON_NANO ||
        detect_platform() == Platform::JETSON_ORIN) {
        s.gpu_temp_c = read_sysfs_float(
            "/sys/class/thermal/thermal_zone1/temp", 0.001f);
        // GPU utilization via tegra_stats (simplified)
        std::ifstream gpu_load("/sys/devices/gpu.0/load");
        if (gpu_load.is_open()) {
            float pct; gpu_load >> pct;
            s.gpu_pct = pct / 10.0f;  // tegra gives 0-1000
        }
    }

    // WiFi RSSI
    {
        std::ifstream wireless("/proc/net/wireless");
        if (wireless.is_open()) {
            std::string line;
            std::getline(wireless, line); // header 1
            std::getline(wireless, line); // header 2
            if (std::getline(wireless, line)) {
                std::istringstream ss(line);
                std::string iface;
                int status;
                float link, level, noise;
                ss >> iface >> status >> link >> level >> noise;
                s.wifi_rssi_dbm = level;  // dBm
            }
        }
    }

    // Battery: INA219 or /sys/class/power_supply
    {
        std::ifstream bat_cap("/sys/class/power_supply/BAT0/capacity");
        if (bat_cap.is_open()) {
            bat_cap >> s.battery_pct;
        } else {
            s.battery_pct = 87.5f;  // sim fallback
        }
    }
#else
    // Simulation values
    s.cpu_pct     = 32.5f;
    s.gpu_pct     = 15.0f;
    s.mem_used_mb = 1200.0f;
    s.mem_total_mb= 4096.0f;
    s.cpu_temp_c  = 52.0f;
    s.battery_pct = 85.0f;
    s.wifi_rssi_dbm = -62.0f;
    s.motor_health = 0.93f;
    s.motor_temp_c = 46.0f;
#endif

    return s;
}

 
// ESP32CamInterface
 
ESP32CamInterface::ESP32CamInterface(Config cfg)
    : cfg_(std::move(cfg)) {
    logger_ = spdlog::get("HAL_ESP32");
    if (!logger_) logger_ = spdlog::stdout_color_mt("HAL_ESP32");
}

ESP32CamInterface::~ESP32CamInterface() {
    stop_async();
    disconnect();
}

std::string ESP32CamInterface::build_url() const {
    if (cfg_.mode == StreamMode::RTSP) {
        return "rtsp://" + cfg_.ip + ":" + std::to_string(cfg_.port) + "/stream";
    }
    return "udp://" + cfg_.ip + ":" + std::to_string(cfg_.port);
}

bool ESP32CamInterface::connect() {
    const std::string url = build_url();
    logger_->info("[ESP32CAM] Connecting to: {}", url);

    // For Jetson: enable GStreamer HW decoder pipeline
    std::string pipeline = url;
#if defined(__aarch64__) && defined(HAVE_GSTREAMER)
    if (cfg_.mode == StreamMode::RTSP && cfg_.hardware_decode) {
        pipeline = "rtspsrc location=" + url +
                   " ! rtph264depay ! h264parse ! nvv4l2decoder ! "
                   "nvvidconv ! video/x-raw,format=BGRx ! "
                   "videoconvert ! video/x-raw,format=BGR ! appsink";
        cap_.open(pipeline, cv::CAP_GSTREAMER);
    }
#endif

    if (!cap_.isOpened()) {
        cap_.open(url);
    }

    if (!cap_.isOpened()) {
        logger_->error("[ESP32CAM] Failed to open stream: {}", url);
        connected_ = false;
        return false;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  cfg_.width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.height);
    cap_.set(cv::CAP_PROP_FPS,          cfg_.fps_target);
    cap_.set(cv::CAP_PROP_BUFFERSIZE,   2);  // minimize latency

    connected_ = true;
    fps_window_start_ = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    logger_->info("[ESP32CAM] Connected. {}Ã—{} @ {}fps",
                  cfg_.width, cfg_.height, cfg_.fps_target);
    return true;
}

void ESP32CamInterface::disconnect() {
    if (cap_.isOpened()) cap_.release();
    connected_ = false;
}

bool ESP32CamInterface::is_connected() const { return connected_; }

bool ESP32CamInterface::grab_frame(cv::Mat& out, double& ts) {
    if (!connected_ && !try_reconnect()) return false;

    if (!cap_.read(out) || out.empty()) {
        ++dropped_frames_;
        logger_->warn("[ESP32CAM] Dropped frame #{}", dropped_frames_);
        connected_ = false;
        return false;
    }

    ts = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    ++frame_count_;
    const double now = ts;
    const double elapsed = now - fps_window_start_;
    if (elapsed >= 1.0) {
        fps_actual_     = static_cast<float>(frame_count_) / static_cast<float>(elapsed);
        frame_count_    = 0;
        fps_window_start_ = now;
    }
    return true;
}

bool ESP32CamInterface::try_reconnect() {
    for (int i = 0; i < cfg_.reconnect_attempts; ++i) {
        logger_->warn("[ESP32CAM] Reconnect attempt {}/{}", i+1, cfg_.reconnect_attempts);
        disconnect();
        std::this_thread::sleep_for(
            std::chrono::duration<double>(cfg_.reconnect_delay_s));
        if (connect()) return true;
    }
    logger_->error("[ESP32CAM] All reconnect attempts failed");
    return false;
}

void ESP32CamInterface::start_async() {
    if (async_running_.exchange(true)) return;
    async_thread_ = std::thread([this] {
        cv::Mat frame;
        double ts;
        while (async_running_.load()) {
            if (grab_frame(frame, ts) && frame_cb_) {
                frame_cb_(frame, ts);
            }
        }
    });
}

void ESP32CamInterface::stop_async() {
    async_running_.store(false);
    if (async_thread_.joinable()) async_thread_.join();
}

void ESP32CamInterface::set_frame_callback(FrameCallback cb) {
    frame_cb_ = std::move(cb);
}

 
// I2CDevice
 
I2CDevice::I2CDevice(std::string bus, uint8_t addr)
    : bus_(std::move(bus)), addr_(addr) {}

I2CDevice::~I2CDevice() { close(); }

bool I2CDevice::open() {
#ifdef __linux__
    fd_ = ::open(bus_.c_str(), O_RDWR);
    if (fd_ < 0) { logger_->error("I2C open failed: {}", bus_); return false; }
    if (::ioctl(fd_, I2C_SLAVE, addr_) < 0) {
        logger_->error("I2C slave addr failed: 0x{:02X}", addr_);
        ::close(fd_); fd_ = -1; return false;
    }
    return true;
#else
    fd_ = 1;  // simulation
    return true;
#endif
}

void I2CDevice::close() {
#ifdef __linux__
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
#endif
}

bool I2CDevice::write_register(uint8_t reg, uint8_t val) {
#ifdef __linux__
    uint8_t buf[2] = {reg, val};
    return ::write(fd_, buf, 2) == 2;
#else
    return true;
#endif
}

bool I2CDevice::read_registers(uint8_t reg, uint8_t* buf, size_t len) {
#ifdef __linux__
    if (::write(fd_, &reg, 1) != 1) return false;
    return ::read(fd_, buf, len) == static_cast<ssize_t>(len);
#else
    std::memset(buf, 0, len);
    return true;
#endif
}

uint8_t I2CDevice::read_byte(uint8_t reg) {
    uint8_t v = 0;
    read_registers(reg, &v, 1);
    return v;
}

int16_t I2CDevice::read_word_be(uint8_t reg) {
    uint8_t buf[2]{};
    read_registers(reg, buf, 2);
    return static_cast<int16_t>((buf[0] << 8) | buf[1]);
}

} // namespace drone::hal
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
