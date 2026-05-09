// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "sensors/LidarSensor.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <regex>
#include <sstream>

#ifdef _WIN32
#define NOMINMAX
#include <Winsock2.h>
#include <Ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace drone::sensors {

namespace {

constexpr size_t kMaxUdpPacketBytes = 65535;
constexpr std::array<char, 4> kGenericPacketMagic{{'L', 'D', 'R', '1'}};
constexpr std::array<char, 4> kSimulationPacketMagic{{'S', 'I', 'M', 'L'}};
constexpr float kGenericPointEps = 1.0e-6f;
constexpr float kPi = 3.14159265358979323846f;

#ifdef _WIN32
using SocketLength = int;
#else
using SocketLength = socklen_t;
#endif

void close_socket_fd(int& socket_fd) {
    if (socket_fd < 0) {
        return;
    }
#ifdef _WIN32
    closesocket(static_cast<SOCKET>(socket_fd));
#else
    close(socket_fd);
#endif
    socket_fd = -1;
}

bool ensure_socket_runtime_ready() {
#ifdef _WIN32
    static bool initialized = false;
    if (!initialized) {
        WSADATA wsa_data;
        if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
            return false;
        }
        initialized = true;
    }
#endif
    return true;
}

std::string socket_error_string(const std::string& prefix) {
#ifdef _WIN32
    return prefix + " (winsock=" + std::to_string(WSAGetLastError()) + ")";
#else
    return prefix + " (" + std::strerror(errno) + ")";
#endif
}

std::string lowercase(std::string_view value) {
    std::string out(value.begin(), value.end());
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

std::optional<std::string> extract_json_string(const std::string& content, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]+)\"", std::regex::icase);
    std::smatch match;
    if (std::regex_search(content, match, pattern) && match.size() >= 2) {
        return match[1].str();
    }
    return std::nullopt;
}

std::optional<double> extract_json_number(const std::string& content, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)", std::regex::icase);
    std::smatch match;
    if (!(std::regex_search(content, match, pattern) && match.size() >= 2)) {
        return std::nullopt;
    }
    try {
        return std::stod(match[1].str());
    } catch (...) {
        return std::nullopt;
    }
}

template <typename T>
T read_packed(const std::vector<uint8_t>& bytes, size_t offset) {
    T value{};
    std::memcpy(&value, bytes.data() + offset, sizeof(T));
    return value;
}

class GenericCartesianLidarParser final : public ILidarParser {
public:
    explicit GenericCartesianLidarParser(std::string frame_id)
        : frame_id_(std::move(frame_id)) {}

    [[nodiscard]] std::string_view model_name() const noexcept override {
        return "generic_udp_cartesian_v1";
    }

    [[nodiscard]] std::optional<LidarScan> parse(const RawLidarPacket& packet) const override {
        constexpr size_t kHeaderBytes = 8;
        constexpr size_t kPointStride = sizeof(float) * 4;
        if (packet.bytes.size() < kHeaderBytes) {
            return std::nullopt;
        }
        if (!std::equal(kGenericPacketMagic.begin(), kGenericPacketMagic.end(), packet.bytes.begin())) {
            return std::nullopt;
        }

        const uint16_t version = read_packed<uint16_t>(packet.bytes, 4);
        const uint16_t point_count = read_packed<uint16_t>(packet.bytes, 6);
        if (version != 1) {
            return std::nullopt;
        }
        const size_t expected_size = kHeaderBytes + (static_cast<size_t>(point_count) * kPointStride);
        if (packet.bytes.size() != expected_size) {
            return std::nullopt;
        }

        LidarScan scan;
        scan.timestamp = packet.timestamp;
        scan.frame_id = frame_id_;
        scan.points.reserve(point_count);

        size_t offset = kHeaderBytes;
        for (uint16_t i = 0; i < point_count; ++i) {
            const float x = read_packed<float>(packet.bytes, offset);
            const float y = read_packed<float>(packet.bytes, offset + sizeof(float));
            const float z = read_packed<float>(packet.bytes, offset + (sizeof(float) * 2));
            const float intensity = read_packed<float>(packet.bytes, offset + (sizeof(float) * 3));
            offset += kPointStride;

            if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(intensity))) {
                return std::nullopt;
            }

            LidarPoint point;
            point.xyz = Eigen::Vector3f{x, y, z};
            point.intensity = intensity;
            point.range_m = point.xyz.norm();
            if (point.range_m <= kGenericPointEps) {
                continue;
            }
            point.azimuth_deg = std::atan2(y, x) * 180.0f / kPi;
            point.elevation_deg = std::atan2(z, std::sqrt((x * x) + (y * y))) * 180.0f / kPi;
            scan.points.push_back(point);
        }

        if (scan.points.empty()) {
            return std::nullopt;
        }
        return scan;
    }

private:
    std::string frame_id_;
};

class SimulationPlaceholderLidarParser final : public ILidarParser {
public:
    explicit SimulationPlaceholderLidarParser(std::string frame_id)
        : frame_id_(std::move(frame_id)) {}

    [[nodiscard]] std::string_view model_name() const noexcept override {
        return "simulation_placeholder";
    }

    [[nodiscard]] std::optional<LidarScan> parse(const RawLidarPacket& packet) const override {
        if (packet.bytes.size() < kSimulationPacketMagic.size()) {
            return std::nullopt;
        }
        if (!std::equal(kSimulationPacketMagic.begin(), kSimulationPacketMagic.end(), packet.bytes.begin())) {
            return std::nullopt;
        }

        LidarScan scan;
        scan.timestamp = packet.timestamp;
        scan.frame_id = frame_id_;
        scan.simulated = true;
        scan.points = {
            {Eigen::Vector3f{1.2f, 0.0f, 0.0f}, 1.0f, 1.2f, 0.0f, 0.0f},
            {Eigen::Vector3f{1.4f, 0.2f, 0.0f}, 0.8f, std::sqrt(2.0f), 8.1f, 0.0f},
            {Eigen::Vector3f{1.0f, -0.3f, 0.1f}, 0.7f, 1.05f, -16.7f, 5.7f},
        };
        return scan;
    }

private:
    std::string frame_id_;
};

} // namespace

std::optional<RawLidarPacket> receive_lidar_udp_packet(int socket_fd, size_t max_bytes, int timeout_ms) {
    if (socket_fd < 0) {
        return std::nullopt;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(static_cast<unsigned int>(socket_fd), &read_fds);

    timeval timeout{};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    const int ready = select(socket_fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0) {
        return std::nullopt;
    }

    std::vector<uint8_t> buffer(std::max<size_t>(1, max_bytes));
    sockaddr_in source_addr{};
    SocketLength source_len = sizeof(source_addr);
    const int received = recvfrom(
        socket_fd,
        reinterpret_cast<char*>(buffer.data()),
        static_cast<int>(buffer.size()),
        0,
        reinterpret_cast<sockaddr*>(&source_addr),
        &source_len);
    if (received <= 0) {
        return std::nullopt;
    }

    buffer.resize(static_cast<size_t>(received));

    char host_buffer[INET_ADDRSTRLEN] = {};
    const char* address = inet_ntop(AF_INET, &source_addr.sin_addr, host_buffer, sizeof(host_buffer));

    RawLidarPacket packet;
    packet.bytes = std::move(buffer);
    packet.source_host = address ? std::string(address) : std::string{};
    packet.source_port = ntohs(source_addr.sin_port);
    packet.timestamp = now_sec();
    return packet;
}

std::unique_ptr<ILidarParser> create_lidar_parser(std::string_view model,
                                                  const std::string& frame_id,
                                                  bool allow_placeholder_parser) {
    const auto normalized = lowercase(model);
    if (normalized == "generic_udp_cartesian_v1" || normalized == "generic" || normalized == "udp_cartesian") {
        return std::make_unique<GenericCartesianLidarParser>(frame_id);
    }
    if (allow_placeholder_parser &&
        (normalized == "simulation_placeholder" || normalized == "sim" || normalized == "demo")) {
        return std::make_unique<SimulationPlaceholderLidarParser>(frame_id);
    }
    return {};
}

PointCloudPtr point_cloud_from_scan(const LidarScan& scan, float min_range_m, float max_range_m) {
    auto cloud = PointCloudPtr(new PointCloud());
    cloud->reserve(scan.points.size());
    for (const auto& point : scan.points) {
        if (!(std::isfinite(point.range_m) &&
              point.range_m >= min_range_m &&
              point.range_m <= max_range_m)) {
            continue;
        }
        cloud->push_back(pcl::PointXYZI{
            point.xyz.x(),
            point.xyz.y(),
            point.xyz.z(),
            point.intensity,
        });
    }
    return cloud;
}

bool LidarSensor::initialize() {
    if (!ensure_socket_runtime_ready()) {
        report_error("LiDAR socket runtime initialization failed");
        set_state(SensorState::FAILED);
        set_status("socket runtime initialization failed");
        return false;
    }

    close_socket_fd(udp_sock_);
    udp_sock_ = static_cast<int>(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
    if (udp_sock_ < 0) {
        report_error(socket_error_string("LiDAR UDP socket creation failed"));
        set_state(SensorState::FAILED);
        set_status("socket creation failed");
        return false;
    }

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(udp_port_);
    const std::string bind_host = bind_host_.empty() ? std::string("0.0.0.0") : bind_host_;
    if (inet_pton(AF_INET, bind_host.c_str(), &bind_addr.sin_addr) != 1) {
        close_socket_fd(udp_sock_);
        report_error("LiDAR bind host is invalid: " + bind_host);
        set_state(SensorState::FAILED);
        set_status("invalid bind host");
        return false;
    }

    if (bind(static_cast<int>(udp_sock_), reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
        const std::string error = socket_error_string("LiDAR UDP bind failed on " + bind_host + ":" + std::to_string(udp_port_));
        close_socket_fd(udp_sock_);
        report_error(error);
        set_state(SensorState::FAILED);
        set_status("udp bind failed");
        return false;
    }

    parser_ = create_lidar_parser(
        model_,
        frame_id_,
        runtime_mode_ == drone::runtime::RuntimeMode::SIMULATION);
    if (!parser_) {
        close_socket_fd(udp_sock_);
        report_error("No LiDAR parser available for model \"" + model_ + "\" in " +
                     std::string(drone::runtime::to_string(runtime_mode_)) + " mode");
        set_state(SensorState::FAILED);
        set_status("parser unavailable");
        return false;
    }

    if (logger_) {
        logger_->info("[{}] initialize lidar endpoint={} model={} frame={} range=[{:.1f},{:.1f}] timeout={}ms",
                      id_, endpoint_, model_, frame_id_, range_min_, range_max_, udp_timeout_ms_);
    }
    poll_rate_hz_ = 10;
    set_state(SensorState::RUNNING);
    set_status("listening for UDP packets");
    return true;
}

bool LidarSensor::reconfigure(const std::string& config_json) {
    if (const auto host = extract_json_string(config_json, "host")) {
        bind_host_ = *host;
    }
    if (const auto port = extract_json_number(config_json, "port")) {
        if (*port >= 1.0 && *port <= 65535.0) {
            udp_port_ = static_cast<uint16_t>(*port);
        }
    }
    if (const auto model = extract_json_string(config_json, "model")) {
        model_ = *model;
    }
    if (const auto frame_id = extract_json_string(config_json, "frame_id")) {
        frame_id_ = *frame_id;
    }
    if (const auto min_range = extract_json_number(config_json, "min_range_m")) {
        range_min_ = static_cast<float>(*min_range);
    }
    if (const auto max_range = extract_json_number(config_json, "max_range_m")) {
        range_max_ = static_cast<float>(*max_range);
    }
    endpoint_ = bind_host_ + ":" + std::to_string(udp_port_);
    if (logger_) {
        logger_->info("[{}] reconfigure requested endpoint={} model={} frame={} range=[{:.1f},{:.1f}]",
                      id_, endpoint_, model_, frame_id_, range_min_, range_max_);
    }
    return true;
}

LidarSensor::TelemetryStats LidarSensor::telemetry_stats() const {
    std::lock_guard lock(data_mutex_);
    TelemetryStats stats;
    stats.scan_active = state() == SensorState::RUNNING && latest_.has_value();
    stats.simulated = latest_.has_value() && latest_->simulated;
    stats.packet_rate_hz = packet_rate_estimate_hz_;
    stats.scan_age_ms = latest_.has_value() ? std::max(0.0, (now_sec() - latest_->timestamp) * 1000.0) : 0.0;
    stats.point_count = latest_.has_value() ? latest_->num_points : 0u;
    stats.min_range_m = range_min_;
    stats.max_range_m = range_max_;
    stats.latest_points = latest_.has_value() ? latest_->points : std::vector<LidarPoint>{};
    {
        std::lock_guard status_lock(status_mutex_);
        stats.status = last_status_;
    }
    return stats;
}

void LidarSensor::poll() {
    const auto scan = receive_udp_packet();
    if (!scan.has_value() || scan->points.empty()) {
        if (logger_) {
            logger_->debug("[{}] poll no LiDAR scan available", id_);
        }
        return;
    }

    auto cloud = point_cloud_from_scan(*scan, range_min_, range_max_);
    if (!cloud || cloud->empty()) {
        set_status("packet received but all points were filtered out");
        return;
    }

    cloud = downsample(remove_outliers(cloud));

    LidarMeasurement measurement;
    measurement.timestamp = scan->timestamp;
    measurement.source_id = id_;
    measurement.cloud = cloud;
    measurement.points = scan->points;
    measurement.num_points = static_cast<uint32_t>(cloud->size());
    measurement.range_min_m = range_min_;
    measurement.range_max_m = range_max_;
    measurement.frame_id = scan->frame_id;
    measurement.simulated = scan->simulated;
    measurement.quality = scan->simulated ? SensorState::DEGRADED : SensorState::RUNNING;
    measurement.confidence = scan->simulated ? 0.35f : 0.9f;

    last_packet_timestamp_ = measurement.timestamp;
    if (previous_packet_timestamp_ > 0.0) {
        const double dt = measurement.timestamp - previous_packet_timestamp_;
        if (dt > 1.0e-6) {
            packet_rate_estimate_hz_ = 1.0 / dt;
        }
    }
    previous_packet_timestamp_ = measurement.timestamp;
    set_status(scan->simulated
        ? "simulation placeholder LiDAR scan parsed"
        : "live LiDAR packet parsed");

    if (logger_) {
        logger_->debug("[{}] cloud received points={} ts={:.3f} simulated={}",
                       id_, measurement.num_points, measurement.timestamp, measurement.simulated);
    }

    {
        std::lock_guard lock(data_mutex_);
        latest_ = measurement;
    }

    std::lock_guard lock(cb_mutex_);
    if (data_cb_) {
        data_cb_(measurement);
    }
}

void LidarSensor::stop() {
    SensorBase::stop();
    close_socket_fd(udp_sock_);
    set_status("stopped");
}

bool LidarSensor::has_recent_scan(double max_age_s) const {
    if (last_packet_timestamp_ <= 0.0) {
        return false;
    }
    return (now_sec() - last_packet_timestamp_) <= max_age_s;
}

std::optional<LidarScan> LidarSensor::receive_udp_packet() {
    const auto packet = receive_lidar_udp_packet(udp_sock_, kMaxUdpPacketBytes, udp_timeout_ms_);
    if (!packet.has_value()) {
        set_status("udp receive timeout");
        return std::nullopt;
    }

    if (!parser_) {
        set_status("no LiDAR parser configured");
        return std::nullopt;
    }

    const auto parsed = parser_->parse(*packet);
    if (!parsed.has_value()) {
        set_status("invalid LiDAR packet rejected");
        if (logger_) {
            logger_->warn("[{}] invalid LiDAR packet rejected model={} bytes={}",
                          id_, parser_->model_name(), packet->bytes.size());
        }
        return std::nullopt;
    }
    return parsed;
}

PointCloudPtr LidarSensor::downsample(PointCloudPtr in) const {
    if (logger_) {
        logger_->debug("[{}] downsample passthrough", id_);
    }
    return in;
}

PointCloudPtr LidarSensor::remove_outliers(PointCloudPtr in) const {
    if (logger_) {
        logger_->debug("[{}] remove_outliers passthrough", id_);
    }
    return in;
}

} // namespace drone::sensors

// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
