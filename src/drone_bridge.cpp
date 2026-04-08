// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// drone_bridge.cpp  â€”  pybind11 module exposing C++ core to Python GUI
// Drone Swarm Sensor Fusion  |  Phase 4 â€” GUI Bridge
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>

#include "vio/VIOPipeline.hpp"
#include "vio/EKFEstimator.hpp"
#include "hal/JetsonHAL.hpp"

#ifdef DRONE_HAS_FASTDDS
#include "swarm/V2XMeshNetwork.hpp"
#endif

namespace py = pybind11;
using namespace drone;

PYBIND11_MODULE(drone_bridge, m) {
    m.doc() = "GPS-Denied Drone Swarm â€” Python â†” C++ Bridge (pybind11)";

    // â”€â”€ EKFConfig â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    py::class_<vio::EKFConfig>(m, "EKFConfig")
        .def(py::init<>())
        .def_readwrite("sigma_na",       &vio::EKFConfig::sigma_na)
        .def_readwrite("sigma_ng",       &vio::EKFConfig::sigma_ng)
        .def_readwrite("sigma_nba",      &vio::EKFConfig::sigma_nba)
        .def_readwrite("sigma_nbg",      &vio::EKFConfig::sigma_nbg)
        .def_readwrite("sigma_px",       &vio::EKFConfig::sigma_px)
        .def_readwrite("mahal_gate",     &vio::EKFConfig::mahal_gate);

    // â”€â”€ PoseEstimate â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    py::class_<vio::PoseEstimate>(m, "PoseEstimate")
        .def(py::init<>())
        .def_readwrite("timestamp",   &vio::PoseEstimate::timestamp)
        .def_readwrite("position",    &vio::PoseEstimate::position)
        .def_readwrite("velocity",    &vio::PoseEstimate::velocity)
        .def_readwrite("pos_std",     &vio::PoseEstimate::pos_std)
        .def("euler_zyx_deg",         &vio::PoseEstimate::euler_zyx_deg)
        .def("__repr__", [](const vio::PoseEstimate& p) {
            return py::str("PoseEstimate(t={:.3f}, pos=[{:.3f},{:.3f},{:.3f}])")
                .format(p.timestamp,
                        p.position.x(), p.position.y(), p.position.z());
        });

    // â”€â”€ SystemStats â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    py::class_<hal::SystemStats>(m, "SystemStats")
        .def(py::init<>())
        .def_readwrite("cpu_pct",        &hal::SystemStats::cpu_pct)
        .def_readwrite("gpu_pct",        &hal::SystemStats::gpu_pct)
        .def_readwrite("mem_used_mb",    &hal::SystemStats::mem_used_mb)
        .def_readwrite("mem_total_mb",   &hal::SystemStats::mem_total_mb)
        .def_readwrite("cpu_temp_c",     &hal::SystemStats::cpu_temp_c)
        .def_readwrite("battery_pct",    &hal::SystemStats::battery_pct)
        .def_readwrite("wifi_rssi_dbm",  &hal::SystemStats::wifi_rssi_dbm);

    // â”€â”€ Free function: read_system_stats â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    m.def("read_system_stats", &hal::read_system_stats,
          "Read current CPU, GPU, memory, battery, and RSSI stats");

#ifdef DRONE_HAS_FASTDDS
    // â”€â”€ DroneRole â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    py::enum_<swarm::DroneRole>(m, "DroneRole")
        .value("CANDIDATE", swarm::DroneRole::CANDIDATE)
        .value("FOLLOWER",  swarm::DroneRole::FOLLOWER)
        .value("LEADER",    swarm::DroneRole::LEADER)
        .value("RELAY",     swarm::DroneRole::RELAY)
        .export_values();

    py::enum_<swarm::SwarmMessage::Type>(m, "SwarmMessageType")
        .value("HEARTBEAT",      swarm::SwarmMessage::Type::HEARTBEAT)
        .value("POSE_UPDATE",    swarm::SwarmMessage::Type::POSE_UPDATE)
        .value("KEYFRAME_SHARE", swarm::SwarmMessage::Type::KEYFRAME_SHARE)
        .value("LEADER_ELECT",   swarm::SwarmMessage::Type::LEADER_ELECT)
        .value("FORMATION_CMD",  swarm::SwarmMessage::Type::FORMATION_CMD)
        .value("EMERGENCY_STOP", swarm::SwarmMessage::Type::EMERGENCY_STOP)
        .value("MISSION_SYNC",   swarm::SwarmMessage::Type::MISSION_SYNC)
        .export_values();

    py::enum_<swarm::FormationCommand::Formation>(m, "FormationShape")
        .value("LINE",    swarm::FormationCommand::Formation::LINE)
        .value("VEE",     swarm::FormationCommand::Formation::VEE)
        .value("DIAMOND", swarm::FormationCommand::Formation::DIAMOND)
        .value("WEDGE",   swarm::FormationCommand::Formation::WEDGE)
        .value("FREE",    swarm::FormationCommand::Formation::FREE)
        .export_values();

    py::class_<swarm::FormationCommand>(m, "FormationCommand")
        .def(py::init<>())
        .def_readwrite("shape",         &swarm::FormationCommand::shape)
        .def_readwrite("spacing_m",     &swarm::FormationCommand::spacing_m)
        .def_readwrite("altitude_m",    &swarm::FormationCommand::altitude_m)
        .def_readwrite("leader_target", &swarm::FormationCommand::leader_target)
        .def_readwrite("velocity_mps",  &swarm::FormationCommand::velocity_mps);

    // â”€â”€ PeerInfo â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    py::class_<swarm::PeerInfo>(m, "PeerInfo")
        .def(py::init<>())
        .def_readwrite("id",           &swarm::PeerInfo::id)
        .def_readwrite("role",         &swarm::PeerInfo::role)
        .def_readwrite("position",     &swarm::PeerInfo::position)
        .def_readwrite("velocity",     &swarm::PeerInfo::velocity)
        .def_readwrite("battery_pct",  &swarm::PeerInfo::battery_pct)
        .def_readwrite("rssi_dbm",     &swarm::PeerInfo::rssi_dbm)
        .def_readwrite("last_seen_ts", &swarm::PeerInfo::last_seen_ts)
        .def_readwrite("reachable",    &swarm::PeerInfo::reachable);

    // â”€â”€ V2XMeshNetwork â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    py::class_<swarm::V2XMeshNetwork,
               std::shared_ptr<swarm::V2XMeshNetwork>>(m, "V2XMeshNetwork")
        .def(py::init<uint32_t, std::string, uint16_t>(),
             py::arg("local_id"),
             py::arg("multicast_group") = "239.255.0.1",
             py::arg("port") = 7400)
        .def("start",           &swarm::V2XMeshNetwork::start)
        .def("stop",            &swarm::V2XMeshNetwork::stop)
        .def("local_role",      &swarm::V2XMeshNetwork::local_role)
        .def("leader_id",       &swarm::V2XMeshNetwork::leader_id)
        .def("peer_count",      &swarm::V2XMeshNetwork::peer_count)
        .def("active_peers",    &swarm::V2XMeshNetwork::active_peers)
        .def("avg_latency_ms",  &swarm::V2XMeshNetwork::avg_latency_ms)
        .def("packet_loss_pct", &swarm::V2XMeshNetwork::packet_loss_pct)
        .def("trigger_election",&swarm::V2XMeshNetwork::trigger_election)
        .def("send_formation",  &swarm::V2XMeshNetwork::send_formation)
        .def("broadcast",
             &swarm::V2XMeshNetwork::broadcast,
             py::arg("type"),
             py::arg("payload") = std::vector<uint8_t>{});
#endif

    // â”€â”€ VIOPipeline â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // Note: sensors are started separately on the C++ side;
    // the Python GUI interacts with the pipeline in read-only query mode.
    py::class_<vio::VIOPipeline,
               std::shared_ptr<vio::VIOPipeline>>(m, "VIOPipeline")
        .def(py::init<vio::EKFConfig>(), py::arg("cfg") = vio::EKFConfig{})
        .def("current_pose", &vio::VIOPipeline::current_pose)
        .def("drift_m",      &vio::VIOPipeline::drift_m)
        .def("set_pose_callback", &vio::VIOPipeline::set_pose_callback)
        .def("reset",        &vio::VIOPipeline::reset);

    // â”€â”€ Version info â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    m.attr("__version__") = "2.0.0";
    m.attr("BUILD_FASTDDS") =
#ifdef DRONE_HAS_FASTDDS
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
