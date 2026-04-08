// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

 
// main.cpp  â€”  Drone Node Entry Point
// Boots all subsystems, starts VIO pipeline, swarm networking, and GUI bridge
// Drone Swarm Sensor Fusion
 
#include "sensors/IMUSensor.hpp"
#include "sensors/LidarSensor.hpp"
#include "sensors/CameraSensor.hpp"
#include "sensors/MotorSensor.hpp"
#include "autonomy/DecisionEngine.hpp"
#include "autonomy/ExperienceMemory.hpp"
#include "localization/TDOALocalizer.hpp"
#include "vio/VIOPipeline.hpp"
#include "slam/KeyframeManager.hpp"
#include "hal/JetsonHAL.hpp"
#include "utils/RuntimeLogging.hpp"

#ifdef DRONE_HAS_FASTDDS
#include "swarm/V2XMeshNetwork.hpp"
#include "swarm/SwarmSecurity.hpp"
#endif

#include <csignal>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <vector>
#include <iostream>
#include <thread>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

 
// Global shutdown flag
 
static std::atomic<bool> g_shutdown{false};

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
    std::string lidar_endpoint{"192.168.1.201:2368"};
    std::string swarm_group{"239.255.0.1"};
    uint16_t    swarm_port{7400};
    std::string yolo_engine{"models/yolov8n.engine"};
};

NodeConfig parse_args(int argc, char** argv) {
    NodeConfig cfg;
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
    }
    spdlog::info("CLI config parsed: id={} esp32={} lidar={} group={} yolo={}",
                 cfg.drone_id, cfg.esp32_ip, cfg.lidar_endpoint, cfg.swarm_group, cfg.yolo_engine);
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

 
int main(int argc, char** argv) {
    setup_logging();
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    const auto cfg = parse_args(argc, argv);

    spdlog::info("â•”â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•—");
    spdlog::info("â•‘  GPS-Denied Drone Swarm Node v2.0        â•‘");
    spdlog::info("â• â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•£");
    spdlog::info("â•‘  Drone ID  : {:5d}                       â•‘", cfg.drone_id);
    spdlog::info("â•‘  Platform  : {}         â•‘",
                  std::string(drone::hal::to_string(drone::hal::detect_platform())));
    spdlog::info("â•šâ•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•");

    // â”€â”€ 1. Sensors â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    auto imu = std::make_shared<drone::sensors::IMUSensor>(
        "imu0", "/dev/i2c-1", 0x68);

    auto cam = std::make_shared<drone::sensors::CameraSensor>(
        "cam0",
        "rtsp://" + cfg.esp32_ip + ":554/stream",
        drone::sensors::CameraIntrinsics{800, 800, 320, 240, {}, 640, 480});

    auto lidar = std::make_shared<drone::sensors::LidarSensor>(
        "lidar0", cfg.lidar_endpoint);
    auto motor = std::make_shared<drone::sensors::MotorSensor>("motor0");

    // Attempt sensor initialization
    for (auto& [name, sensor] : std::initializer_list<
            std::pair<const char*, drone::sensors::SensorBase*>>{
            {"IMU",    imu.get()},
            {"Camera", cam.get()},
            {"LiDAR",  lidar.get()},
            {"Motor",  motor.get()}}) {
        if (!sensor->initialize()) {
            spdlog::error("{} initialization failed â€” running in degraded mode", name);
        }
    }

    // Load YOLOv8n (non-fatal if engine absent)
    cam->load_yolo_model(cfg.yolo_engine, 0.45f, 0.5f);

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
    drone::localization::TDOALocalizer tdoa;
    const auto tdoa_anchors = default_tdoa_anchors();
    tdoa.set_anchors(tdoa_anchors);

    // â”€â”€ 3. V2X Swarm Network â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
#ifdef DRONE_HAS_FASTDDS
    auto net = std::make_shared<drone::swarm::V2XMeshNetwork>(
        cfg.drone_id, cfg.swarm_group, cfg.swarm_port);
    drone::swarm::SwarmSecurityConfig security_cfg;
    security_cfg.enabled = true;
    if (const char* secret = std::getenv("DRONE_SWARM_SECRET")) {
        security_cfg.swarm_secret = secret;
    } else {
        security_cfg.swarm_secret = "drone-swarm-dev-secret-change-me";
        spdlog::warn("DRONE_SWARM_SECRET not set, using development swarm secret");
    }
    net->configure_security(std::move(security_cfg));

    auto slam = std::make_shared<drone::slam::KeyframeManager>(cfg.drone_id, net);

    net->on_message([&](const drone::swarm::SwarmMessage& msg) {
        if (msg.type == drone::swarm::SwarmMessage::Type::KEYFRAME_SHARE)
            slam->on_remote_keyframe(msg);
    });
#endif

    // â”€â”€ 4. Start all subsystems â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    imu->start();
    cam->start();
    lidar->start();
    motor->start();
    vio->start();
#ifdef DRONE_HAS_FASTDDS
    net->start();
    net->trigger_election();
#endif

    spdlog::info("All subsystems online. Running until SIGINTâ€¦");

    // â”€â”€ 5. Main loop â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    while (!g_shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        auto stats  = drone::hal::read_system_stats();
        const auto pose   = vio->current_pose();
        const auto frame  = cam->latest();
        const auto motor_state = motor->latest();
        const double now_s = drone::sensors::now_sec();
        size_t swarm_peer_count = 0;
        bool swarm_follower = false;

        if (motor_state.has_value()) {
            stats.motor_health = motor_state->average_health;
            stats.motor_temp_c = 0.0f;
            for (const auto& motor_sample : motor_state->motors) {
                stats.motor_temp_c = std::max(stats.motor_temp_c, motor_sample.temperature_c);
            }
        }

#ifdef DRONE_HAS_FASTDDS
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
#endif

        memory.observe(cfg.drone_id, pose, stats, frame, swarm_peer_count, now_s);
        auto prior = memory.summarize(cfg.drone_id);
        const auto tdoa_solution = tdoa.estimate(
            build_demo_tdoa_measurements(tdoa_anchors, pose.position, now_s),
            pose.position);

        drone::autonomy::DecisionContext decision_ctx;
        decision_ctx.pose = pose;
        decision_ctx.system = stats;
        decision_ctx.frame = frame;
        decision_ctx.memory_prior = prior;
        decision_ctx.swarm_peer_count = swarm_peer_count;
        decision_ctx.swarm_follower = swarm_follower;
        decision_ctx.inference_ready = cam->inference_enabled();
        decision_ctx.now_s = now_s;
        if (tdoa_solution.has_value()) {
            decision_ctx.tdoa_position = tdoa_solution->position;
            decision_ctx.tdoa_confidence = tdoa_solution->confidence;
        }

        spdlog::info("[ HEALTH ] CPU:{:.1f}%  Temp:{:.1f}Â°C  Bat:{:.0f}%  "
                      "Pos:[{:.2f},{:.2f},{:.2f}]  Drift:{:.3f}m",
                      stats.cpu_pct, stats.cpu_temp_c, stats.battery_pct,
                      pose.position.x(), pose.position.y(), pose.position.z(),
                      vio->drift_m());

#ifdef DRONE_HAS_FASTDDS
        spdlog::info("[ SWARM  ] Peers:{:d}  Role:{}  AvgLatency:{:.1f}ms",
                      net->peer_count(),
                      std::string(drone::swarm::to_string(net->local_role())),
                      net->avg_latency_ms());
#endif

        if (tdoa_solution.has_value()) {
            spdlog::info("[ TDOA   ] Pos:[{:.2f},{:.2f},{:.2f}]  RMS:{:.3f}m  Conf:{:.2f}",
                         tdoa_solution->position.x(),
                         tdoa_solution->position.y(),
                         tdoa_solution->position.z(),
                         tdoa_solution->rms_residual_m,
                         tdoa_solution->confidence);
        }

        spdlog::info("[ MEMORY ] Risk:{:.2f}  DriftTrend:{:.3f}m/min  BatteryBurn:{:.2f}%/min  Label:{}",
                     prior.risk_score,
                     prior.drift_trend_m_per_min,
                     prior.battery_burn_pct_per_min,
                     prior.dominant_label.empty() ? std::string("none") : prior.dominant_label);

        const auto decision = ai.update(decision_ctx);
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
#ifdef DRONE_HAS_FASTDDS
    net->stop();
#endif
    lidar->stop();
    motor->stop();
    cam->stop();
    imu->stop();

    spdlog::info("Drone node {} shutdown complete.", cfg.drone_id);
    return EXIT_SUCCESS;
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
