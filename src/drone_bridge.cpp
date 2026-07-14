// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

// drone_bridge.cpp    pybind11 module exposing C++ core to Python GUI
// Drone Swarm Sensor Fusion  |  Phase 4  GUI Bridge

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>

#include "vio/VIOPipeline.hpp"
#include "vio/EKFEstimator.hpp"
#include "hal/JetsonHAL.hpp"
#include "swarm/SwarmSecurity.hpp"
#include "swarm/V2XMeshNetwork.hpp"

namespace py = pybind11;
using namespace drone;

PYBIND11_MODULE(drone_bridge, m) {
    m.doc() = "GPS-Denied Drone Swarm  Python â†” C++ Bridge (pybind11)";

    //  EKFConfig
    py::class_<vio::EKFConfig>(m, "EKFConfig")
        .def(py::init<>())
        .def_readwrite("sigma_na", &vio::EKFConfig::sigma_na)
        .def_readwrite("sigma_ng", &vio::EKFConfig::sigma_ng)
        .def_readwrite("sigma_nba", &vio::EKFConfig::sigma_nba)
        .def_readwrite("sigma_nbg", &vio::EKFConfig::sigma_nbg)
        .def_readwrite("sigma_px", &vio::EKFConfig::sigma_px)
        .def_readwrite("mahal_gate", &vio::EKFConfig::mahal_gate);

    //  PoseEstimate
    py::class_<vio::PoseEstimate>(m, "PoseEstimate")
        .def(py::init<>())
        .def_readwrite("timestamp", &vio::PoseEstimate::timestamp)
        .def_readwrite("position", &vio::PoseEstimate::position)
        .def_readwrite("velocity", &vio::PoseEstimate::velocity)
        .def_readwrite("pos_std", &vio::PoseEstimate::pos_std)
        .def_readwrite("drift_m", &vio::PoseEstimate::drift_m)
        .def_readwrite("localization_confidence", &vio::PoseEstimate::localization_confidence)
        .def_readwrite("localization_source", &vio::PoseEstimate::localization_source)
        .def_readwrite("localization_degraded", &vio::PoseEstimate::localization_degraded)
        .def_readwrite("localization_lost", &vio::PoseEstimate::localization_lost)
        .def("euler_zyx_deg", &vio::PoseEstimate::euler_zyx_deg)
        .def("__repr__", [](const vio::PoseEstimate& p) {
            return py::str(
                       "PoseEstimate(t={:.3f}, pos=[{:.3f},{:.3f},{:.3f}], conf={:.2f}, source={})")
                .format(p.timestamp, p.position.x(), p.position.y(), p.position.z(),
                        p.localization_confidence, p.localization_source);
        });

    py::class_<vio::RuntimeTelemetry>(m, "RuntimeTelemetry")
        .def(py::init<>())
        .def_readwrite("localization_confidence_trend",
                       &vio::RuntimeTelemetry::localization_confidence_trend)
        .def_readwrite("sync_confidence", &vio::RuntimeTelemetry::sync_confidence)
        .def_readwrite("imu_camera_offset_ms", &vio::RuntimeTelemetry::imu_camera_offset_ms)
        .def_readwrite("peer_clock_offset_ms", &vio::RuntimeTelemetry::peer_clock_offset_ms)
        .def_readwrite("occupancy_ratio", &vio::RuntimeTelemetry::occupancy_ratio)
        .def_readwrite("anchor_visibility_ratio", &vio::RuntimeTelemetry::anchor_visibility_ratio)
        .def_readwrite("tdoa_weight", &vio::RuntimeTelemetry::tdoa_weight)
        .def_readwrite("tdoa_confidence", &vio::RuntimeTelemetry::tdoa_confidence)
        .def_readwrite("peer_count", &vio::RuntimeTelemetry::peer_count)
        .def_readwrite("stale_peer_count", &vio::RuntimeTelemetry::stale_peer_count)
        .def_readwrite("local_consensus_state", &vio::RuntimeTelemetry::local_consensus_state)
        .def_readwrite("local_consensus_epoch", &vio::RuntimeTelemetry::local_consensus_epoch)
        .def_readwrite("mesh_bandwidth_kbps", &vio::RuntimeTelemetry::mesh_bandwidth_kbps)
        .def_readwrite("edge_serialization_mode", &vio::RuntimeTelemetry::edge_serialization_mode)
        .def_readwrite("edge_average_packet_size_bytes",
                       &vio::RuntimeTelemetry::edge_average_packet_size_bytes)
        .def_readwrite("edge_bandwidth_savings_estimate_pct",
                       &vio::RuntimeTelemetry::edge_bandwidth_savings_estimate_pct)
        .def_readwrite("edge_packet_encode_latency_us",
                       &vio::RuntimeTelemetry::edge_packet_encode_latency_us)
        .def_readwrite("disconnected_operation", &vio::RuntimeTelemetry::disconnected_operation)
        .def_readwrite("edge_health_status", &vio::RuntimeTelemetry::edge_health_status)
        .def_readwrite("edge_autonomy_state", &vio::RuntimeTelemetry::edge_autonomy_state)
        .def_readwrite("edge_inference_status", &vio::RuntimeTelemetry::edge_inference_status)
        .def_readwrite("edge_inference_fps", &vio::RuntimeTelemetry::edge_inference_fps)
        .def_readwrite("edge_inference_confidence",
                       &vio::RuntimeTelemetry::edge_inference_confidence)
        .def_readwrite("local_obstacle_count", &vio::RuntimeTelemetry::local_obstacle_count)
        .def_readwrite("shared_obstacle_count", &vio::RuntimeTelemetry::shared_obstacle_count)
        .def_readwrite("relocalization_count", &vio::RuntimeTelemetry::relocalization_count)
        .def_readwrite("visible_anchor_count", &vio::RuntimeTelemetry::visible_anchor_count)
        .def_readwrite("planned_waypoint_count", &vio::RuntimeTelemetry::planned_waypoint_count)
        .def_readwrite("last_relocalized_keyframe",
                       &vio::RuntimeTelemetry::last_relocalized_keyframe)
        .def_readwrite("localization_state", &vio::RuntimeTelemetry::localization_state)
        .def_readwrite("localization_source", &vio::RuntimeTelemetry::localization_source)
        .def_readwrite("localization_data_source", &vio::RuntimeTelemetry::localization_data_source)
        .def_readwrite("security_state", &vio::RuntimeTelemetry::security_state)
        .def_readwrite("security_summary", &vio::RuntimeTelemetry::security_summary)
        .def_readwrite("security_transition_reason",
                       &vio::RuntimeTelemetry::security_transition_reason)
        .def_readwrite("remote_command_allowed", &vio::RuntimeTelemetry::remote_command_allowed)
        .def_readwrite("telemetry_uplink_allowed", &vio::RuntimeTelemetry::telemetry_uplink_allowed)
        .def_readwrite("link_integrity_score", &vio::RuntimeTelemetry::link_integrity_score)
        .def_readwrite("trust_epoch", &vio::RuntimeTelemetry::trust_epoch)
        .def_readwrite("last_auth_failure_at_s", &vio::RuntimeTelemetry::last_auth_failure_at_s)
        .def_readwrite("tamper_score", &vio::RuntimeTelemetry::tamper_score)
        .def_readwrite("firmware_measurement", &vio::RuntimeTelemetry::firmware_measurement)
        .def_readwrite("firmware_version", &vio::RuntimeTelemetry::firmware_version)
        .def_readwrite("secure_boot_state", &vio::RuntimeTelemetry::secure_boot_state)
        .def_readwrite("boot_trust_summary", &vio::RuntimeTelemetry::boot_trust_summary)
        .def_readwrite("rollback_counter", &vio::RuntimeTelemetry::rollback_counter)
        .def_readwrite("maintenance_mode", &vio::RuntimeTelemetry::maintenance_mode)
        .def_readwrite("update_channel_state", &vio::RuntimeTelemetry::update_channel_state)
        .def_readwrite("last_remote_command_status",
                       &vio::RuntimeTelemetry::last_remote_command_status)
        .def_readwrite("health_flags", &vio::RuntimeTelemetry::health_flags)
        .def_readwrite("tracked_feature_count", &vio::RuntimeTelemetry::tracked_feature_count)
        .def_readwrite("inlier_ratio", &vio::RuntimeTelemetry::inlier_ratio)
        .def_readwrite("reprojection_error", &vio::RuntimeTelemetry::reprojection_error)
        .def_readwrite("visual_update_confidence", &vio::RuntimeTelemetry::visual_update_confidence)
        .def_readwrite("visual_frontend_valid", &vio::RuntimeTelemetry::visual_frontend_valid)
        .def_readwrite("visual_placeholder_active",
                       &vio::RuntimeTelemetry::visual_placeholder_active);

    //  SystemStats
    py::class_<hal::SystemStats>(m, "SystemStats")
        .def(py::init<>())
        .def_readwrite("cpu_pct", &hal::SystemStats::cpu_pct)
        .def_readwrite("gpu_pct", &hal::SystemStats::gpu_pct)
        .def_readwrite("mem_used_mb", &hal::SystemStats::mem_used_mb)
        .def_readwrite("mem_total_mb", &hal::SystemStats::mem_total_mb)
        .def_readwrite("cpu_temp_c", &hal::SystemStats::cpu_temp_c)
        .def_readwrite("battery_pct", &hal::SystemStats::battery_pct)
        .def_readwrite("wifi_rssi_dbm", &hal::SystemStats::wifi_rssi_dbm);

    //  Free function: read_system_stats
    m.def("read_system_stats", &hal::read_system_stats,
          "Read current CPU, GPU, memory, battery, and RSSI stats");

    //  DroneRole
    py::enum_<swarm::DroneRole>(m, "DroneRole")
        .value("CANDIDATE", swarm::DroneRole::CANDIDATE)
        .value("FOLLOWER", swarm::DroneRole::FOLLOWER)
        .value("LEADER", swarm::DroneRole::LEADER)
        .value("RELAY", swarm::DroneRole::RELAY)
        .export_values();

    py::enum_<swarm::SwarmMessage::Type>(m, "SwarmMessageType")
        .value("HEARTBEAT", swarm::SwarmMessage::Type::HEARTBEAT)
        .value("POSE_UPDATE", swarm::SwarmMessage::Type::POSE_UPDATE)
        .value("KEYFRAME_SHARE", swarm::SwarmMessage::Type::KEYFRAME_SHARE)
        .value("LEADER_ELECT", swarm::SwarmMessage::Type::LEADER_ELECT)
        .value("FORMATION_CMD", swarm::SwarmMessage::Type::FORMATION_CMD)
        .value("EMERGENCY_STOP", swarm::SwarmMessage::Type::EMERGENCY_STOP)
        .value("MISSION_SYNC", swarm::SwarmMessage::Type::MISSION_SYNC)
        .export_values();

    py::enum_<swarm::FormationCommand::Formation>(m, "FormationShape")
        .value("LINE", swarm::FormationCommand::Formation::LINE)
        .value("VEE", swarm::FormationCommand::Formation::VEE)
        .value("DIAMOND", swarm::FormationCommand::Formation::DIAMOND)
        .value("WEDGE", swarm::FormationCommand::Formation::WEDGE)
        .value("FREE", swarm::FormationCommand::Formation::FREE)
        .export_values();

    py::class_<swarm::FormationCommand>(m, "FormationCommand")
        .def(py::init<>())
        .def_readwrite("shape", &swarm::FormationCommand::shape)
        .def_readwrite("spacing_m", &swarm::FormationCommand::spacing_m)
        .def_readwrite("altitude_m", &swarm::FormationCommand::altitude_m)
        .def_readwrite("leader_target", &swarm::FormationCommand::leader_target)
        .def_readwrite("velocity_mps", &swarm::FormationCommand::velocity_mps);

    py::class_<swarm::SwarmSecurityConfig>(m, "SwarmSecurityConfig")
        .def(py::init<>())
        .def_readwrite("enabled", &swarm::SwarmSecurityConfig::enabled)
        .def_readwrite("swarm_secret", &swarm::SwarmSecurityConfig::swarm_secret)
        .def_readwrite("pbkdf2_iterations", &swarm::SwarmSecurityConfig::pbkdf2_iterations);

    //  PeerInfo
    py::class_<swarm::PeerInfo>(m, "PeerInfo")
        .def(py::init<>())
        .def_readwrite("id", &swarm::PeerInfo::id)
        .def_readwrite("role", &swarm::PeerInfo::role)
        .def_readwrite("position", &swarm::PeerInfo::position)
        .def_readwrite("velocity", &swarm::PeerInfo::velocity)
        .def_readwrite("battery_pct", &swarm::PeerInfo::battery_pct)
        .def_readwrite("rssi_dbm", &swarm::PeerInfo::rssi_dbm)
        .def_readwrite("last_seen_ts", &swarm::PeerInfo::last_seen_ts)
        .def_readwrite("reachable", &swarm::PeerInfo::reachable);

    //  V2XMeshNetwork
    py::class_<swarm::V2XMeshNetwork, std::shared_ptr<swarm::V2XMeshNetwork>>(m, "V2XMeshNetwork")
        .def(py::init<uint32_t, std::string, uint16_t>(), py::arg("local_id"),
             py::arg("multicast_group") = "239.255.0.1", py::arg("port") = 7400)
        .def("start", &swarm::V2XMeshNetwork::start)
        .def("stop", &swarm::V2XMeshNetwork::stop)
        .def("local_role", &swarm::V2XMeshNetwork::local_role)
        .def("leader_id", &swarm::V2XMeshNetwork::leader_id)
        .def("peer_count", &swarm::V2XMeshNetwork::peer_count)
        .def("active_peers", &swarm::V2XMeshNetwork::active_peers)
        .def("avg_latency_ms", &swarm::V2XMeshNetwork::avg_latency_ms)
        .def("packet_loss_pct", &swarm::V2XMeshNetwork::packet_loss_pct)
        .def("trigger_election", &swarm::V2XMeshNetwork::trigger_election)
        .def("configure_security", &swarm::V2XMeshNetwork::configure_security)
        .def("security_enabled", &swarm::V2XMeshNetwork::security_enabled)
        .def("security_last_error", &swarm::V2XMeshNetwork::security_last_error)
        .def("send_formation", &swarm::V2XMeshNetwork::send_formation)
        .def("broadcast", &swarm::V2XMeshNetwork::broadcast, py::arg("type"),
             py::arg("payload") = std::vector<uint8_t>{});

    //  VIOPipeline â”€
    // Note: sensors are started separately on the C++ side;
    // the Python GUI interacts with the pipeline in read-only query mode.
    py::class_<vio::VIOPipeline, std::shared_ptr<vio::VIOPipeline>>(m, "VIOPipeline")
        .def(py::init<vio::EKFConfig>(), py::arg("cfg") = vio::EKFConfig{})
        .def("current_pose", &vio::VIOPipeline::current_pose)
        .def("drift_m", &vio::VIOPipeline::drift_m)
        .def("runtime_telemetry", &vio::VIOPipeline::runtime_telemetry)
        .def("set_pose_callback", &vio::VIOPipeline::set_pose_callback)
        .def("reset", &vio::VIOPipeline::reset);

    //  Version info
    m.attr("__version__") = "2.0.0";
    m.attr("BUILD_SWARM_V2X") = true;
    m.attr("BUILD_FASTDDS_TRANSPORT") =
#ifdef DRONE_HAS_FASTDDS_TRANSPORT
        true;
#else
        false;
#endif
    m.attr("BUILD_TENSORRT") =
#ifdef DRONE_HAS_TENSORRT
        true;
#else
        false;
#endif
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
