// ─────────────────────────────────────────────────────────────────────────────
// main.cpp  —  Drone Node Entry Point
// Boots all subsystems, starts VIO pipeline, swarm networking, and GUI bridge
// Drone Swarm Sensor Fusion
// ─────────────────────────────────────────────────────────────────────────────
#include "sensors/IMUSensor.hpp"
#include "sensors/LidarSensor.hpp"
#include "sensors/CameraSensor.hpp"
#include "autonomy/DecisionEngine.hpp"
#include "vio/VIOPipeline.hpp"
#include "slam/KeyframeManager.hpp"
#include "hal/JetsonHAL.hpp"

#ifdef DRONE_HAS_FASTDDS
#include "swarm/V2XMeshNetwork.hpp"
#endif

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

// ─────────────────────────────────────────────────────────────────────────────
// Global shutdown flag
// ─────────────────────────────────────────────────────────────────────────────
static std::atomic<bool> g_shutdown{false};

void signal_handler(int sig) {
    spdlog::warn("Signal {} received — initiating graceful shutdown…", sig);
    g_shutdown.store(true);
}

// ─────────────────────────────────────────────────────────────────────────────
void setup_logging() {
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
    for (const char* name : {"EKF","VIO","SLAM","V2X","HAL_ESP32","I2C","UART"}) {
        spdlog::stdout_color_mt(name)->set_level(spdlog::level::info);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Parse simple CLI args  --id=1  --esp32=192.168.4.1  --lidar=192.168.1.201
// ─────────────────────────────────────────────────────────────────────────────
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
    return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    setup_logging();
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    const auto cfg = parse_args(argc, argv);

    spdlog::info("╔══════════════════════════════════════════╗");
    spdlog::info("║  GPS-Denied Drone Swarm Node v2.0        ║");
    spdlog::info("╠══════════════════════════════════════════╣");
    spdlog::info("║  Drone ID  : {:5d}                       ║", cfg.drone_id);
    spdlog::info("║  Platform  : {}         ║",
                  std::string(drone::hal::to_string(drone::hal::detect_platform())));
    spdlog::info("╚══════════════════════════════════════════╝");

    // ── 1. Sensors ─────────────────────────────────────────────────────────
    auto imu = std::make_shared<drone::sensors::IMUSensor>(
        "imu0", "/dev/i2c-1", 0x68);

    auto cam = std::make_shared<drone::sensors::CameraSensor>(
        "cam0",
        "rtsp://" + cfg.esp32_ip + ":554/stream",
        drone::sensors::CameraIntrinsics{800, 800, 320, 240, {}, 640, 480});

    auto lidar = std::make_shared<drone::sensors::LidarSensor>(
        "lidar0", cfg.lidar_endpoint);

    // Attempt sensor initialization
    for (auto& [name, sensor] : std::initializer_list<
            std::pair<const char*, drone::sensors::SensorBase*>>{
            {"IMU",    imu.get()},
            {"Camera", cam.get()},
            {"LiDAR",  lidar.get()}}) {
        if (!sensor->initialize()) {
            spdlog::error("{} initialization failed — running in degraded mode", name);
        }
    }

    // Load YOLOv8n (non-fatal if engine absent)
    cam->load_yolo_model(cfg.yolo_engine, 0.45f, 0.5f);

    // ── 2. VIO Pipeline ────────────────────────────────────────────────────
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

    // ── 3. V2X Swarm Network ───────────────────────────────────────────────
#ifdef DRONE_HAS_FASTDDS
    auto net = std::make_shared<drone::swarm::V2XMeshNetwork>(
        cfg.drone_id, cfg.swarm_group, cfg.swarm_port);

    auto slam = std::make_shared<drone::slam::KeyframeManager>(cfg.drone_id, net);

    net->on_message([&](const drone::swarm::SwarmMessage& msg) {
        if (msg.type == drone::swarm::SwarmMessage::Type::KEYFRAME_SHARE)
            slam->on_remote_keyframe(msg);
    });
#endif

    // ── 4. Start all subsystems ────────────────────────────────────────────
    imu->start();
    cam->start();
    lidar->start();
    vio->start();
#ifdef DRONE_HAS_FASTDDS
    net->start();
    net->trigger_election();
#endif

    spdlog::info("All subsystems online. Running until SIGINT…");

    // ── 5. Main loop ───────────────────────────────────────────────────────
    while (!g_shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        const auto stats  = drone::hal::read_system_stats();
        const auto pose   = vio->current_pose();
        const auto frame  = cam->latest();

        drone::autonomy::DecisionContext decision_ctx;
        decision_ctx.pose = pose;
        decision_ctx.system = stats;
        decision_ctx.frame = frame;
        decision_ctx.inference_ready = cam->inference_enabled();
        decision_ctx.now_s = drone::sensors::now_sec();

        spdlog::info("[ HEALTH ] CPU:{:.1f}%  Temp:{:.1f}°C  Bat:{:.0f}%  "
                      "Pos:[{:.2f},{:.2f},{:.2f}]  Drift:{:.3f}m",
                      stats.cpu_pct, stats.cpu_temp_c, stats.battery_pct,
                      pose.position.x(), pose.position.y(), pose.position.z(),
                      vio->drift_m());

#ifdef DRONE_HAS_FASTDDS
        decision_ctx.swarm_peer_count = net->peer_count();
        decision_ctx.swarm_follower = (net->local_role() == drone::swarm::DroneRole::FOLLOWER);
        spdlog::info("[ SWARM  ] Peers:{:d}  Role:{}  AvgLatency:{:.1f}ms",
                      net->peer_count(),
                      std::string(drone::swarm::to_string(net->local_role())),
                      net->avg_latency_ms());
#endif

        const auto decision = ai.update(decision_ctx);
        spdlog::info("[ AI     ] Mode:{}  Vcmd:[{:.2f},{:.2f},{:.2f}]  Yaw:{:.2f}  {}",
                     std::string(drone::autonomy::to_string(decision.mode)),
                     decision.desired_velocity.x(),
                     decision.desired_velocity.y(),
                     decision.desired_velocity.z(),
                     decision.desired_yaw_rate_rads,
                     decision.summary);
    }

    // ── 6. Graceful shutdown ───────────────────────────────────────────────
    spdlog::info("Shutting down subsystems…");
    vio->stop();
#ifdef DRONE_HAS_FASTDDS
    net->stop();
#endif
    lidar->stop();
    cam->stop();
    imu->stop();

    spdlog::info("Drone node {} shutdown complete.", cfg.drone_id);
    return EXIT_SUCCESS;
}
