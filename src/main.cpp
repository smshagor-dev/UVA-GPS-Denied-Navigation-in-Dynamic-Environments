// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

 
// main.cpp  â€”  Drone Node Entry Point
// Boots all subsystems, starts VIO pipeline, swarm networking, and GUI bridge
// Drone Swarm Sensor Fusion
 
#include "sensors/IMUSensor.hpp"
#include "sensors/LidarSensor.hpp"
#include "sensors/CameraSensor.hpp"
#include "sensors/BarometerSensor.hpp"
#include "sensors/MotorSensor.hpp"
#include "sensors/OpticalFlowSensor.hpp"
#include "sensors/RangefinderSensor.hpp"
#include "autonomy/DecisionEngine.hpp"
#include "autonomy/ExperienceMemory.hpp"
#include "localization/LocalizationFusion.hpp"
#include "localization/TDOAIngestor.hpp"
#include "localization/TDOALocalizer.hpp"
#include "localization/TimeSyncTracker.hpp"
#include "localization/UWBSerialDriver.hpp"
#include "security/CommandPolicy.hpp"
#include "telemetry/ControlPlaneTelemetryClient.hpp"
#include "vio/VIOPipeline.hpp"
#include "slam/KeyframeManager.hpp"
#include "slam/MapPlanner.hpp"
#include "slam/OccupancyGridMap.hpp"
#include "hal/JetsonHAL.hpp"
#include "security/DroneSecurity.hpp"
#include "swarm/V2XMeshNetwork.hpp"
#include "swarm/SwarmSecurity.hpp"
#include "utils/RuntimeLogging.hpp"

#include <csignal>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <tuple>
#include <vector>
#include <iostream>
#include <thread>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

 
// Global shutdown flag
 
static std::atomic<bool> g_shutdown{false};

std::optional<std::string> env_var(std::string_view key) {
#ifdef _WIN32
    char* value = nullptr;
    size_t length = 0;
    if (_dupenv_s(&value, &length, std::string(key).c_str()) != 0 || !value) {
        return std::nullopt;
    }
    std::string out(value);
    std::free(value);
    return out;
#else
    if (const char* value = std::getenv(std::string(key).c_str())) {
        return std::string(value);
    }
    return std::nullopt;
#endif
}

void signal_handler(int sig) {
    spdlog::warn("Signal {} received â€” initiating graceful shutdownâ€¦", sig);
    g_shutdown.store(true);
}

 
void setup_logging() {
    std::filesystem::create_directories("logs");

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);

    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        "logs/drone.log", 10 * 1024 * 1024 /* 10MB */, 5 /* keep 5 */);
    file_sink->set_level(spdlog::level::debug);

    auto logger = std::make_shared<spdlog::logger>(
        "drone", spdlog::sinks_init_list{console_sink, file_sink});

    spdlog::set_default_logger(logger);
    spdlog::set_level(spdlog::level::debug);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v");

    // Create subsystem loggers sharing same sinks
    for (const char* name : {"EKF","VIO","SLAM","V2X","HAL_ESP32","I2C","UART","AI","MEMORY","TDOA","SECURITY","CAMERA","LIDAR"}) {
        auto child = std::make_shared<spdlog::logger>(name, logger->sinks().begin(), logger->sinks().end());
        child->set_level(spdlog::level::debug);
        spdlog::register_logger(child);
    }

    spdlog::flush_on(spdlog::level::info);
    spdlog::info("Logging initialized. file=logs/drone.log");
}

 
// Parse simple CLI args  --id=1  --esp32=192.168.4.1  --lidar=192.168.1.201
 
struct NodeConfig {
    uint32_t    drone_id{1};
    std::string esp32_ip{"192.168.4.1"};
    std::string camera_stream_url{};
    std::string imu_device{"/dev/i2c-1"};
    uint8_t     imu_i2c_addr{0x68};
    std::string lidar_endpoint{"192.168.1.201:2368"};
    std::string swarm_group{"239.255.0.1"};
    uint16_t    swarm_port{7400};
    std::string yolo_engine{"models/yolov8n.engine"};
    std::string tdoa_measurements_csv{};
    uint16_t    tdoa_udp_port{0};
    std::string tdoa_serial_device{};
    bool enable_imu{true};
    bool enable_camera{true};
    bool enable_lidar{true};
    bool enable_barometer{true};
    bool enable_motor{true};
    bool enable_optical_flow{true};
    bool enable_rangefinder{true};
    bool enable_uwb_serial{true};
    bool enable_tdoa_ingestor{true};
    bool enable_backend_telemetry{false};
    uint16_t backend_telemetry_interval_ms{1000};
    std::string security_profile{"lab"};
};

uint16_t parse_port_or_default(const std::string& value, uint16_t fallback) {
    try {
        return static_cast<uint16_t>(std::stoul(value));
    } catch (const std::exception&) {
        return fallback;
    }
}

bool parse_bool_or_default(const std::string& value, bool fallback) {
    std::string lower = value;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (lower == "1" || lower == "true" || lower == "yes" || lower == "on") {
        return true;
    }
    if (lower == "0" || lower == "false" || lower == "no" || lower == "off") {
        return false;
    }
    return fallback;
}

void apply_env_overrides(NodeConfig& cfg) {
    if (const auto value = env_var("DRONE_NODE_ID")) {
        try {
            cfg.drone_id = static_cast<uint32_t>(std::stoul(*value));
        } catch (const std::exception&) {
        }
    }
    if (const auto value = env_var("DRONE_ESP32_IP")) {
        cfg.esp32_ip = *value;
    }
    if (const auto value = env_var("DRONE_CAMERA_STREAM_URL")) {
        cfg.camera_stream_url = *value;
    }
    if (const auto value = env_var("DRONE_IMU_DEVICE")) {
        cfg.imu_device = *value;
    }
    if (const auto value = env_var("DRONE_IMU_ADDR")) {
        cfg.imu_i2c_addr = static_cast<uint8_t>(parse_port_or_default(*value, cfg.imu_i2c_addr));
    }
    if (const auto value = env_var("DRONE_LIDAR_ENDPOINT")) {
        cfg.lidar_endpoint = *value;
    }
    if (const auto value = env_var("DRONE_SWARM_GROUP")) {
        cfg.swarm_group = *value;
    }
    if (const auto value = env_var("DRONE_SWARM_PORT")) {
        cfg.swarm_port = parse_port_or_default(*value, cfg.swarm_port);
    }
    if (const auto value = env_var("DRONE_YOLO_ENGINE")) {
        cfg.yolo_engine = *value;
    }
    if (const auto value = env_var("DRONE_TDOA_CSV")) {
        cfg.tdoa_measurements_csv = *value;
    }
    if (const auto value = env_var("DRONE_TDOA_UDP_PORT")) {
        cfg.tdoa_udp_port = parse_port_or_default(*value, cfg.tdoa_udp_port);
    }
    if (const auto value = env_var("DRONE_TDOA_SERIAL")) {
        cfg.tdoa_serial_device = *value;
    }
    if (const auto value = env_var("DRONE_ENABLE_IMU")) {
        cfg.enable_imu = parse_bool_or_default(*value, cfg.enable_imu);
    }
    if (const auto value = env_var("DRONE_ENABLE_CAMERA")) {
        cfg.enable_camera = parse_bool_or_default(*value, cfg.enable_camera);
    }
    if (const auto value = env_var("DRONE_ENABLE_LIDAR")) {
        cfg.enable_lidar = parse_bool_or_default(*value, cfg.enable_lidar);
    }
    if (const auto value = env_var("DRONE_ENABLE_BAROMETER")) {
        cfg.enable_barometer = parse_bool_or_default(*value, cfg.enable_barometer);
    }
    if (const auto value = env_var("DRONE_ENABLE_MOTOR")) {
        cfg.enable_motor = parse_bool_or_default(*value, cfg.enable_motor);
    }
    if (const auto value = env_var("DRONE_ENABLE_OPTICAL_FLOW")) {
        cfg.enable_optical_flow = parse_bool_or_default(*value, cfg.enable_optical_flow);
    }
    if (const auto value = env_var("DRONE_ENABLE_RANGEFINDER")) {
        cfg.enable_rangefinder = parse_bool_or_default(*value, cfg.enable_rangefinder);
    }
    if (const auto value = env_var("DRONE_ENABLE_UWB_SERIAL")) {
        cfg.enable_uwb_serial = parse_bool_or_default(*value, cfg.enable_uwb_serial);
    }
    if (const auto value = env_var("DRONE_ENABLE_TDOA_INGESTOR")) {
        cfg.enable_tdoa_ingestor = parse_bool_or_default(*value, cfg.enable_tdoa_ingestor);
    }
    if (const auto value = env_var("DRONE_ENABLE_BACKEND_TELEMETRY")) {
        cfg.enable_backend_telemetry = parse_bool_or_default(*value, cfg.enable_backend_telemetry);
    }
    if (const auto value = env_var("DRONE_BACKEND_TELEMETRY_INTERVAL_MS")) {
        cfg.backend_telemetry_interval_ms = parse_port_or_default(*value, cfg.backend_telemetry_interval_ms);
    }
    if (const auto value = env_var("DRONE_SECURITY_PROFILE")) {
        cfg.security_profile = *value;
    }
}

std::string normalize_security_profile(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (value == "field") {
        return "field";
    }
    if (value == "prod" || value == "production") {
        return "production";
    }
    return "lab";
}

bool is_placeholder_secret(const std::string& secret) {
    return secret.empty() ||
           secret == "replace-with-a-strong-shared-secret" ||
           secret == "replace-with-strong-shared-secret" ||
           secret == "drone-swarm-dev-secret-change-me";
}

void apply_security_failsafe(const drone::security::DroneSecurityAssessment& security,
                             drone::autonomy::DecisionCommand& command,
                             const drone::vio::PoseEstimate& pose) {
    using drone::autonomy::BehaviorMode;
    switch (security.state) {
    case drone::security::DroneSecurityState::LAND_IMMEDIATELY:
        command.mode = BehaviorMode::EMERGENCY_LAND;
        command.requires_operator_attention = true;
        command.desired_velocity = -(pose.R_wb() * Eigen::Vector3d{0.0, 1.0, 0.0}) * 0.8;
        command.desired_yaw_rate_rads = 0.0;
        command.summary = security.summary;
        break;
    case drone::security::DroneSecurityState::SAFE_RETURN:
        if (command.mode != BehaviorMode::EMERGENCY_LAND) {
            command.mode = BehaviorMode::RETURN_HOME;
            command.requires_operator_attention = true;
            command.desired_velocity = (-pose.velocity * 0.45) + (pose.R_wb() * Eigen::Vector3d{0.0, 1.0, 0.0} * 0.18);
            command.desired_yaw_rate_rads = 0.0;
            command.summary = security.summary;
        }
        break;
    case drone::security::DroneSecurityState::ISOLATED_AUTONOMY:
    case drone::security::DroneSecurityState::CONTROL_PLANE_UNTRUSTED:
    case drone::security::DroneSecurityState::PEER_SPOOF_SUSPECT:
    case drone::security::DroneSecurityState::COMMAND_REPLAY_SUSPECT:
    case drone::security::DroneSecurityState::AUTH_SUSPECT:
        if (command.mode != BehaviorMode::EMERGENCY_LAND &&
            command.mode != BehaviorMode::SAFE_RETURN_BY_ANCHOR &&
            command.mode != BehaviorMode::LOCALIZATION_LOST) {
            command.mode = BehaviorMode::HOLD_POSITION;
            command.requires_operator_attention = true;
            command.desired_velocity = -pose.velocity * 0.35;
            command.desired_yaw_rate_rads = 0.0;
            command.summary = security.summary;
        }
        break;
    case drone::security::DroneSecurityState::DEGRADED_LINK:
    case drone::security::DroneSecurityState::TRUSTED:
        break;
    }
}

NodeConfig parse_args(int argc, char** argv) {
    NodeConfig cfg;
    apply_env_overrides(cfg);
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto eq = arg.find('=');
        if (eq == std::string::npos) continue;
        auto key = arg.substr(2, eq - 2);  // strip leading "--"
        auto val = arg.substr(eq + 1);
        if      (key == "id")      cfg.drone_id        = std::stoul(val);
        else if (key == "esp32")   cfg.esp32_ip         = val;
        else if (key == "lidar")   cfg.lidar_endpoint   = val;
        else if (key == "group")   cfg.swarm_group      = val;
        else if (key == "yolo")    cfg.yolo_engine      = val;
        else if (key == "tdoa-csv")cfg.tdoa_measurements_csv = val;
        else if (key == "tdoa-udp")cfg.tdoa_udp_port = static_cast<uint16_t>(std::stoul(val));
        else if (key == "tdoa-serial")cfg.tdoa_serial_device = val;
    }
    cfg.security_profile = normalize_security_profile(cfg.security_profile);
    spdlog::info("CLI config parsed: id={} esp32={} lidar={} group={} yolo={} tdoa_csv={} tdoa_udp={} tdoa_serial={} security_profile={} backend_telemetry={} telemetry_interval_ms={}",
                 cfg.drone_id, cfg.esp32_ip, cfg.lidar_endpoint, cfg.swarm_group, cfg.yolo_engine,
                 cfg.tdoa_measurements_csv.empty() ? std::string("<demo>") : cfg.tdoa_measurements_csv,
                 cfg.tdoa_udp_port,
                 cfg.tdoa_serial_device.empty() ? std::string("<none>") : cfg.tdoa_serial_device,
                 cfg.security_profile,
                 cfg.enable_backend_telemetry,
                 cfg.backend_telemetry_interval_ms);
    return cfg;
}

std::vector<drone::localization::TDOALocalizer::Anchor> default_tdoa_anchors() {
    using Anchor = drone::localization::TDOALocalizer::Anchor;
    return {
        Anchor{1, Eigen::Vector3d{-20.0, -20.0, 6.0}},
        Anchor{2, Eigen::Vector3d{ 20.0, -20.0, 6.0}},
        Anchor{3, Eigen::Vector3d{ 20.0,  20.0, 6.0}},
        Anchor{4, Eigen::Vector3d{-20.0,  20.0, 6.0}},
        Anchor{5, Eigen::Vector3d{  0.0,   0.0, 24.0}},
    };
}

std::vector<drone::localization::TDOALocalizer::Measurement> build_demo_tdoa_measurements(
        const std::vector<drone::localization::TDOALocalizer::Anchor>& anchors,
        const Eigen::Vector3d& position,
        double now_s) {
    std::vector<drone::localization::TDOALocalizer::Measurement> measurements;
    measurements.reserve(anchors.size());
    constexpr double kSignalSpeedMps = 299702547.0;
    constexpr double kClockBiasS = 2.5e-7;

    for (const auto& anchor : anchors) {
        const double range_m = (position - anchor.position).norm();
        const double propagation_s = range_m / kSignalSpeedMps;
        measurements.push_back({anchor.id, now_s + kClockBiasS + propagation_s});
    }
    return measurements;
}

std::optional<std::vector<drone::localization::TDOALocalizer::Measurement>> load_tdoa_measurements_from_csv(
        const std::string& csv_path) {
    static std::string last_open_warning_path;
    static std::string last_short_warning_path;
    if (csv_path.empty()) {
        return std::nullopt;
    }

    std::ifstream input(csv_path);
    if (!input.is_open()) {
        if (last_open_warning_path != csv_path) {
            spdlog::warn("TDOA CSV path could not be opened: {}", csv_path);
            last_open_warning_path = csv_path;
        }
        return std::nullopt;
    }
    last_open_warning_path.clear();

    std::vector<drone::localization::TDOALocalizer::Measurement> measurements;
    std::string line;
    size_t line_no = 0;
    while (std::getline(input, line)) {
        ++line_no;
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::replace(line.begin(), line.end(), ';', ',');
        std::stringstream ss(line);
        std::string anchor_text;
        std::string time_text;
        if (!std::getline(ss, anchor_text, ',') || !std::getline(ss, time_text, ',')) {
            spdlog::warn("Skipping malformed TDOA CSV row {}: {}", line_no, line);
            continue;
        }
        try {
            measurements.push_back({
                static_cast<uint32_t>(std::stoul(anchor_text)),
                std::stod(time_text),
            });
        } catch (const std::exception&) {
            spdlog::warn("Skipping unparsable TDOA CSV row {}: {}", line_no, line);
        }
    }

    if (measurements.size() < 4) {
        if (last_short_warning_path != csv_path) {
            spdlog::warn("TDOA CSV contained fewer than 4 usable measurements");
            last_short_warning_path = csv_path;
        }
        return std::nullopt;
    }
    last_short_warning_path.clear();
    return measurements;
}

 
int main(int argc, char** argv) {
    setup_logging();
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    const auto cfg = parse_args(argc, argv);
    const auto backend_url = env_var("DRONE_BACKEND_URL").value_or("");
    drone::telemetry::ControlPlaneTelemetryClient telemetry_client(
        cfg.enable_backend_telemetry ? backend_url : std::string{},
        cfg.backend_telemetry_interval_ms);

    spdlog::info("â•”â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•—");
    spdlog::info("â•‘  GPS-Denied Drone Swarm Node v2.0        â•‘");
    spdlog::info("â• â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•£");
    spdlog::info("â•‘  Drone ID  : {:5d}                       â•‘", cfg.drone_id);
    spdlog::info("â•‘  Platform  : {}         â•‘",
                  std::string(drone::hal::to_string(drone::hal::detect_platform())));
    spdlog::info("â•šâ•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•");

    // â”€â”€ 1. Sensors â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    auto imu = std::make_shared<drone::sensors::IMUSensor>(
        "imu0", cfg.imu_device, cfg.imu_i2c_addr);

    auto cam = std::make_shared<drone::sensors::CameraSensor>(
        "cam0",
        cfg.camera_stream_url.empty() ? ("rtsp://" + cfg.esp32_ip + ":554/stream") : cfg.camera_stream_url,
        drone::sensors::CameraIntrinsics{800, 800, 320, 240, {}, 640, 480});

    auto lidar = std::make_shared<drone::sensors::LidarSensor>(
        "lidar0", cfg.lidar_endpoint);
    auto barometer = std::make_shared<drone::sensors::BarometerSensor>("baro0");
    auto motor = std::make_shared<drone::sensors::MotorSensor>("motor0");
    auto optical_flow = std::make_shared<drone::sensors::OpticalFlowSensor>("flow0");
    auto rangefinder = std::make_shared<drone::sensors::RangefinderSensor>("range0");

    // Attempt sensor initialization
    for (auto& [name, enabled, sensor] : std::initializer_list<
            std::tuple<const char*, bool, drone::sensors::SensorBase*>>{
            {"IMU", cfg.enable_imu, imu.get()},
            {"Camera", cfg.enable_camera, cam.get()},
            {"LiDAR", cfg.enable_lidar, lidar.get()},
            {"Barometer", cfg.enable_barometer, barometer.get()},
            {"Motor", cfg.enable_motor, motor.get()},
            {"OpticalFlow", cfg.enable_optical_flow, optical_flow.get()},
            {"Rangefinder", cfg.enable_rangefinder, rangefinder.get()}}) {
        if (!enabled) {
            spdlog::warn("{} disabled by environment", name);
            continue;
        }
        if (!sensor->initialize()) {
            spdlog::error("{} initialization failed â€” running in degraded mode", name);
        }
    }

    // Load YOLOv8n (non-fatal if engine absent)
    if (cfg.enable_camera) {
        cam->load_yolo_model(cfg.yolo_engine, 0.45f, 0.5f);
    }

    // â”€â”€ 2. VIO Pipeline â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    drone::vio::EKFConfig ekf_cfg;
    auto vio = std::make_shared<drone::vio::VIOPipeline>(ekf_cfg);
    vio->attach_imu(imu);
    vio->attach_camera(cam);
    vio->attach_lidar(lidar);

    Eigen::Matrix3d K;
    K << 800, 0, 320,
         0, 800, 240,
         0, 0, 1;
    vio->set_camera_matrix(K);

    vio->set_pose_callback([&](const drone::vio::PoseEstimate& p) {
        spdlog::debug("Pose: [{:.3f},{:.3f},{:.3f}]  drift={:.4f}m",
                       p.position.x(), p.position.y(), p.position.z(),
                       vio->drift_m());
    });

    drone::autonomy::DecisionEngine ai;
    drone::autonomy::ExperienceMemory memory;
    drone::localization::LocalizationFusion localization_fusion;
    drone::localization::TimeSyncTracker time_sync;
    drone::localization::TDOALocalizer tdoa;
    const auto tdoa_anchors = default_tdoa_anchors();
    tdoa.set_anchors(tdoa_anchors);
    drone::localization::TDOAIngestor tdoa_ingestor({
        !cfg.enable_tdoa_ingestor
            ? drone::localization::TDOAIngestor::Mode::DISABLED
            : cfg.tdoa_udp_port > 0
            ? drone::localization::TDOAIngestor::Mode::UDP_TEXT
            : (!cfg.tdoa_measurements_csv.empty()
                ? drone::localization::TDOAIngestor::Mode::CSV_FILE
                : drone::localization::TDOAIngestor::Mode::DISABLED),
        cfg.tdoa_measurements_csv,
        cfg.tdoa_udp_port,
        32,
    });
    if (cfg.enable_tdoa_ingestor) {
        tdoa_ingestor.start();
    }
    drone::localization::UWBSerialDriver uwb_serial({
        cfg.enable_uwb_serial ? cfg.tdoa_serial_device : std::string{},
        115200,
        32,
    });
    if (cfg.enable_uwb_serial) {
        uwb_serial.start();
    }
    auto slam = std::make_shared<drone::slam::KeyframeManager>(cfg.drone_id, nullptr);
    drone::slam::MapPlanner planner;
    drone::slam::OccupancyGridMap occupancy_map;

    // â”€â”€ 3. V2X Swarm Network â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    auto net = std::make_shared<drone::swarm::V2XMeshNetwork>(
        cfg.drone_id, cfg.swarm_group, cfg.swarm_port);
    drone::swarm::SwarmSecurityConfig security_cfg;
    security_cfg.enabled = true;
    const bool hardened_profile = cfg.security_profile == "field" || cfg.security_profile == "production";
    bool placeholder_swarm_secret = false;
    if (const auto secret = env_var("DRONE_SWARM_SECRET")) {
        security_cfg.swarm_secret = *secret;
        placeholder_swarm_secret = is_placeholder_secret(security_cfg.swarm_secret);
        if (hardened_profile && is_placeholder_secret(security_cfg.swarm_secret)) {
            spdlog::error("DRONE_SWARM_SECRET is a placeholder value and cannot be used in {} mode", cfg.security_profile);
            return 1;
        }
    } else if (hardened_profile) {
        spdlog::error("DRONE_SWARM_SECRET is required in {} mode", cfg.security_profile);
        return 1;
    } else {
        security_cfg.swarm_secret = "drone-swarm-dev-secret-change-me";
        placeholder_swarm_secret = true;
        spdlog::warn("DRONE_SWARM_SECRET not set, using development swarm secret");
    }
    net->configure_security(std::move(security_cfg));

    slam = std::make_shared<drone::slam::KeyframeManager>(cfg.drone_id, net);

    std::mutex remote_command_mutex;
    std::optional<drone::security::RemoteCommandEnvelope> pending_remote_command;
    std::string last_remote_command_status = "no remote command";

    net->on_message([&](const drone::swarm::SwarmMessage& msg) {
        if (msg.type == drone::swarm::SwarmMessage::Type::KEYFRAME_SHARE)
            slam->on_remote_keyframe(msg);
        if (const auto remote = drone::security::command_from_swarm_message(msg)) {
            std::lock_guard lock(remote_command_mutex);
            pending_remote_command = *remote;
        }
    });

    // â”€â”€ 4. Start all subsystems â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (cfg.enable_imu) imu->start();
    if (cfg.enable_camera) cam->start();
    if (cfg.enable_lidar) lidar->start();
    if (cfg.enable_barometer) barometer->start();
    if (cfg.enable_motor) motor->start();
    if (cfg.enable_optical_flow) optical_flow->start();
    if (cfg.enable_rangefinder) rangefinder->start();
    vio->start();
    net->start();
    net->trigger_election();

    spdlog::info("All subsystems online. Running until SIGINTâ€¦");

    // â”€â”€ 5. Main loop â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    while (!g_shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        auto stats  = drone::hal::read_system_stats();
        const auto pose   = vio->current_pose();
        const auto frame  = cam->latest();
        const auto imu_state = imu->latest();
        const auto barometer_state = barometer->latest();
        const auto motor_state = motor->latest();
        const auto optical_flow_state = optical_flow->latest();
        const auto rangefinder_state = rangefinder->latest();
        const auto lidar_state = lidar->latest();
        const double now_s = drone::sensors::now_sec();
        size_t swarm_peer_count = 0;
        bool swarm_follower = false;
        occupancy_map.clear();

        if (imu_state.has_value()) {
            time_sync.observe_imu(imu_state->timestamp);
        }
        if (frame.has_value()) {
            time_sync.observe_camera(frame->timestamp);
        }

        if (motor_state.has_value()) {
            stats.motor_health = motor_state->average_health;
            stats.motor_temp_c = 0.0f;
            for (const auto& motor_sample : motor_state->motors) {
                stats.motor_temp_c = std::max(stats.motor_temp_c, motor_sample.temperature_c);
            }
        }

        drone::swarm::SwarmHealthMetrics swarm_health;
        swarm_health.battery_pct = stats.battery_pct;
        swarm_health.motor_health = motor_state.has_value() ? motor_state->average_health : stats.motor_health;
        swarm_health.link_quality = std::clamp((stats.wifi_rssi_dbm + 90.0f) / 45.0f, 0.0f, 1.0f);
        swarm_health.cpu_headroom = std::clamp(1.0f - (stats.cpu_pct / 100.0f), 0.0f, 1.0f);
        swarm_health.thermal_headroom =
            std::clamp(1.0f - std::max(0.0f, (stats.cpu_temp_c - 55.0f) / 35.0f), 0.0f, 1.0f);
        swarm_health.emergency_fault =
            (stats.battery_pct < 12.0f) ||
            (motor_state.has_value() && motor_state->critical_fault) ||
            (swarm_health.link_quality < 0.12f);
        net->set_local_health(swarm_health);
        swarm_peer_count = net->peer_count();
        swarm_follower = (net->local_role() == drone::swarm::DroneRole::FOLLOWER);

        const auto serial_tdoa_measurements = uwb_serial.poll();
        const auto ingested_tdoa_measurements = tdoa_ingestor.poll();
        const auto external_tdoa_measurements =
            serial_tdoa_measurements.has_value()
                ? serial_tdoa_measurements
                : (tdoa_ingestor.running()
                    ? ingested_tdoa_measurements
                    : load_tdoa_measurements_from_csv(cfg.tdoa_measurements_csv));
        const bool using_external_tdoa = external_tdoa_measurements.has_value();
        const auto tdoa_solution = tdoa.estimate(
            using_external_tdoa
                ? *external_tdoa_measurements
                : build_demo_tdoa_measurements(tdoa_anchors, pose.position, now_s),
            pose.position);
        if (external_tdoa_measurements.has_value() && !external_tdoa_measurements->empty()) {
            const double ref_time = external_tdoa_measurements->front().arrival_time_s;
            for (const auto& measurement : *external_tdoa_measurements) {
                time_sync.observe_anchor(measurement.anchor_id, measurement.arrival_time_s, ref_time);
            }
        }

        if (lidar_state.has_value()) {
            occupancy_map.integrate_lidar(*lidar_state, pose.position);
        }
        for (const auto& anchor : tdoa_anchors) {
            const bool visible = external_tdoa_measurements.has_value() &&
                std::any_of(external_tdoa_measurements->begin(), external_tdoa_measurements->end(),
                    [&](const auto& measurement) { return measurement.anchor_id == anchor.id; });
            occupancy_map.mark_anchor(anchor, visible);
        }

        const auto sync_status = time_sync.status();
        const auto occupancy_status = occupancy_map.status();
        auto slam_status = slam->status();
        const auto fusion = localization_fusion.update({
            pose,
            tdoa_solution,
            frame.has_value(),
            lidar_state.has_value(),
            rangefinder_state.has_value() && rangefinder_state->valid,
            optical_flow_state.has_value(),
            barometer_state.has_value(),
            tdoa_ingestor.visibility_ratio(tdoa_anchors.size()),
            sync_status,
            pose.localization_confidence,
        });
        const drone::security::DroneSecurityAssessment security = drone::security::assess_security({
            cfg.security_profile,
            net->security_enabled(),
            hardened_profile,
            placeholder_swarm_secret,
            fusion.lost,
            swarm_health.emergency_fault,
            stats.battery_pct,
            swarm_health.link_quality,
            sync_status.confidence,
            sync_status.peer_clock_offset_ms,
            swarm_peer_count,
        });

        memory.observe(
            cfg.drone_id,
            pose,
            stats,
            frame,
            swarm_peer_count,
            fusion.confidence,
            fusion.source,
            fusion.lost,
            now_s);
        auto prior = memory.summarize(cfg.drone_id);
        auto fused_pose = pose;
        fused_pose.position = fusion.fused_position;
        fused_pose.localization_confidence = fusion.confidence;
        fused_pose.localization_source = fusion.source;
        fused_pose.localization_degraded = fusion.degraded;
        fused_pose.localization_lost = fusion.lost;
        if (frame.has_value() && !frame->image.empty()) {
            slam->try_add_frame(frame->image, fused_pose.position, fused_pose.orientation, now_s);
            if (fusion.degraded || fusion.lost) {
                if (const auto relocalized = slam->attempt_relocalization(
                        frame->image,
                        fused_pose.position,
                        fused_pose.orientation)) {
                    fused_pose.position = relocalized->corrected_position;
                    fused_pose.orientation = relocalized->corrected_orientation;
                    fused_pose.localization_confidence = std::max(fused_pose.localization_confidence, relocalized->confidence);
                    fused_pose.localization_source = "loop-closure-relocalized";
                    fused_pose.localization_degraded = relocalized->confidence < 0.58;
                    fused_pose.localization_lost = relocalized->confidence < 0.22;
                    slam_status = slam->status();
                }
            }
        }

        const auto plan = planner.plan(occupancy_status, fused_pose.position, ai.home_position());

        drone::autonomy::DecisionContext decision_ctx;
        decision_ctx.pose = fused_pose;
        decision_ctx.system = stats;
        decision_ctx.frame = frame;
        decision_ctx.memory_prior = prior;
        decision_ctx.localization_confidence = fusion.confidence;
        decision_ctx.localization_source = fusion.source;
        decision_ctx.localization_degraded = fusion.degraded;
        decision_ctx.localization_lost = fusion.lost;
        decision_ctx.sync_confidence = sync_status.confidence;
        decision_ctx.visible_anchor_count = tdoa_ingestor.visible_anchor_count();
        decision_ctx.relocalization_count = slam_status.relocalization_count;
        decision_ctx.camera_tracking_nominal = frame.has_value() &&
            (!frame->detections.empty() || fusion.confidence >= 0.55 || slam_status.loop_candidates > 0);
        decision_ctx.swarm_peer_count = swarm_peer_count;
        decision_ctx.swarm_follower = swarm_follower;
        decision_ctx.inference_ready = cam->inference_enabled();
        decision_ctx.now_s = now_s;
        if (tdoa_solution.has_value()) {
            decision_ctx.tdoa_position = tdoa_solution->position;
            decision_ctx.tdoa_confidence = tdoa_solution->confidence;
        }

        vio->set_runtime_telemetry({
            fusion.confidence_trend,
            sync_status.confidence,
            sync_status.imu_camera_offset_ms,
            sync_status.peer_clock_offset_ms,
            occupancy_status.occupied_ratio,
            tdoa_ingestor.visibility_ratio(tdoa_anchors.size()),
            fusion.tdoa_weight,
            tdoa_solution.has_value() ? tdoa_solution->confidence : 0.0,
            slam_status.relocalization_count,
            tdoa_ingestor.visible_anchor_count(),
            plan.has_value() ? plan->waypoints.size() : 0u,
            slam_status.last_relocalized_keyframe,
            fusion.state,
            fusion.source,
            std::string(drone::security::to_string(security.state)),
            security.summary,
            security.remote_command_allowed,
            security.telemetry_uplink_allowed,
            security.link_integrity_score,
            last_remote_command_status,
            security.health_flags,
        });

        spdlog::info("[ HEALTH ] CPU:{:.1f}%  Temp:{:.1f}Â°C  Bat:{:.0f}%  "
                      "Pos:[{:.2f},{:.2f},{:.2f}]  Drift:{:.3f}m",
                      stats.cpu_pct, stats.cpu_temp_c, stats.battery_pct,
                      pose.position.x(), pose.position.y(), pose.position.z(),
                      vio->drift_m());
        spdlog::info("[ LOC    ] Source:{}  State:{}  Conf:{:.2f}  Trend:{:.2f}  Degraded:{}  Lost:{}",
                     fusion.source,
                     fusion.state,
                     fusion.confidence,
                     fusion.confidence_trend,
                     fusion.degraded,
                     fusion.lost);
        spdlog::info("[ SYNC   ] IMU-Cam:{:.2f}ms  Anchor:{:.2f}ms  Peer:{:.2f}ms  Conf:{:.2f}",
                     sync_status.imu_camera_offset_ms,
                     sync_status.anchor_clock_offset_ms,
                     sync_status.peer_clock_offset_ms,
                     sync_status.confidence);
        spdlog::info("[ MAP    ] Occupied:{:.2f}%  Anchors:{}/{}  Reloc:{}  LoopHints:{}",
                     occupancy_status.occupied_ratio * 100.0,
                     tdoa_ingestor.visible_anchor_count(),
                     tdoa_anchors.size(),
                     slam_status.relocalization_count,
                     slam_status.loop_candidates);
        if (plan.has_value() && !plan->waypoints.empty()) {
            const auto& next_waypoint = plan->waypoints.front();
            spdlog::info("[ PLAN   ] Waypoints:{} Cost:{:.2f} AnchorGuided:{} Next:[{:.2f},{:.2f},{:.2f}]",
                         plan->waypoints.size(),
                         plan->total_cost,
                         plan->used_anchor_guidance,
                         next_waypoint.position.x(),
                         next_waypoint.position.y(),
                         next_waypoint.position.z());
        }

        spdlog::info("[ SWARM  ] Peers:{:d}  Role:{}  AvgLatency:{:.1f}ms",
                      net->peer_count(),
                      std::string(drone::swarm::to_string(net->local_role())),
                      net->avg_latency_ms());
        spdlog::info("[ SEC    ] State:{}  Link:{:.2f}  RemoteCmd:{}  Uplink:{}  {}",
                     std::string(drone::security::to_string(security.state)),
                     security.link_integrity_score,
                     security.remote_command_allowed,
                     security.telemetry_uplink_allowed,
                     security.summary);

        if (tdoa_solution.has_value()) {
            spdlog::info("[ TDOA   ] Mode:{}  Pos:[{:.2f},{:.2f},{:.2f}]  RMS:{:.3f}m  Conf:{:.2f}",
                         using_external_tdoa ? "external" : "demo",
                         tdoa_solution->position.x(),
                         tdoa_solution->position.y(),
                         tdoa_solution->position.z(),
                         tdoa_solution->rms_residual_m,
                         tdoa_solution->confidence);
        }

        spdlog::info("[ MEMORY ] Risk:{:.2f}  DriftTrend:{:.3f}m/min  BatteryBurn:{:.2f}%/min  LocConf:{:.2f}  Label:{}  LocMode:{}",
                     prior.risk_score,
                     prior.drift_trend_m_per_min,
                     prior.battery_burn_pct_per_min,
                     prior.localization_confidence_avg,
                     prior.dominant_label.empty() ? std::string("none") : prior.dominant_label,
                     prior.dominant_localization_source.empty() ? std::string("none") : prior.dominant_localization_source);

        auto decision = ai.update(decision_ctx);
        apply_security_failsafe(security, decision, fused_pose);
        {
            std::lock_guard lock(remote_command_mutex);
            if (pending_remote_command.has_value()) {
                const auto policy = drone::security::evaluate_remote_command(security, *pending_remote_command);
                last_remote_command_status =
                    std::string(drone::security::to_string(pending_remote_command->action)) +
                    " from node " + std::to_string(pending_remote_command->src_id) +
                    (policy.accepted ? " accepted" : " rejected") +
                    " (" + policy.reason + ")";
                if (policy.accepted) {
                    drone::security::apply_remote_command(*pending_remote_command, decision, fused_pose);
                }
                pending_remote_command.reset();
            }
        }
        spdlog::info("[ REMOTE ] {}", last_remote_command_status);
        const auto telemetry_now = std::chrono::steady_clock::now();
        if (telemetry_client.enabled() && security.telemetry_uplink_allowed && telemetry_client.should_publish(telemetry_now)) {
            drone::telemetry::TelemetrySnapshot snapshot;
            std::ostringstream cluster_id;
            cluster_id << "cluster-" << std::setw(2) << std::setfill('0') << (((cfg.drone_id - 1) / 20) + 1);
            snapshot.drone_id = cfg.drone_id;
            snapshot.cluster_id = cluster_id.str();
            snapshot.role = std::string(drone::swarm::to_string(net->local_role()));
            snapshot.connectivity = security.link_integrity_score < 0.35 ? "Degraded" : "Mesh";
            snapshot.reachable = true;
            snapshot.position = fused_pose.position;
            snapshot.velocity = fused_pose.velocity;
            snapshot.attitude_rpy = fused_pose.euler_zyx_deg().cast<double>();
            snapshot.thrust_vector = decision.desired_velocity + Eigen::Vector3d{0.0, 0.0, 9.81};
            snapshot.commanded_altitude_m = fused_pose.position.z();
            snapshot.commanded_speed_mps = decision.desired_velocity.norm();
            snapshot.drift_m = vio->drift_m();
            snapshot.battery_pct = stats.battery_pct;
            snapshot.rssi_dbm = stats.wifi_rssi_dbm;
            snapshot.cpu_temp_c = stats.cpu_temp_c;
            snapshot.gpu_load_pct = stats.gpu_pct;
            snapshot.mission_state = std::string(drone::autonomy::to_string(decision.mode));
            snapshot.localization_source = fusion.source;
            snapshot.localization_state = fusion.state;
            snapshot.localization_confidence = fusion.confidence;
            snapshot.tdoa_confidence = tdoa_solution.has_value() ? tdoa_solution->confidence : 0.0;
            snapshot.confidence_trend = fusion.confidence_trend;
            snapshot.relocalization_count = static_cast<int>(slam_status.relocalization_count);
            snapshot.visible_anchor_count = static_cast<int>(tdoa_ingestor.visible_anchor_count());
            snapshot.occupancy_ratio = occupancy_status.occupied_ratio;
            snapshot.sync_confidence = sync_status.confidence;
            snapshot.imu_camera_offset_ms = sync_status.imu_camera_offset_ms;
            snapshot.security_state = std::string(drone::security::to_string(security.state));
            snapshot.security_summary = security.summary;
            snapshot.remote_command_allowed = security.remote_command_allowed;
            snapshot.telemetry_uplink_allowed = security.telemetry_uplink_allowed;
            snapshot.link_integrity_score = security.link_integrity_score;
            snapshot.health_flags = security.health_flags;
            telemetry_client.publish(snapshot, telemetry_now);
        }
        if (telemetry_client.enabled()) {
            spdlog::info("[ BACKEND] {}", telemetry_client.last_status());
        }
        spdlog::info("[ AI     ] Mode:{}  Vcmd:[{:.2f},{:.2f},{:.2f}]  Yaw:{:.2f}  {}",
                     std::string(drone::autonomy::to_string(decision.mode)),
                     decision.desired_velocity.x(),
                     decision.desired_velocity.y(),
                     decision.desired_velocity.z(),
                     decision.desired_yaw_rate_rads,
                     decision.summary);
    }

    // â”€â”€ 6. Graceful shutdown â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    spdlog::info("Shutting down subsystemsâ€¦");
    vio->stop();
    net->stop();
    if (cfg.enable_lidar) lidar->stop();
    if (cfg.enable_rangefinder) rangefinder->stop();
    if (cfg.enable_optical_flow) optical_flow->stop();
    if (cfg.enable_motor) motor->stop();
    if (cfg.enable_barometer) barometer->stop();
    if (cfg.enable_camera) cam->stop();
    if (cfg.enable_imu) imu->stop();
    tdoa_ingestor.stop();
    uwb_serial.stop();

    spdlog::info("Drone node {} shutdown complete.", cfg.drone_id);
    return EXIT_SUCCESS;
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
