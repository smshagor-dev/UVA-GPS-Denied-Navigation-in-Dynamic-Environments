// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once
 
// V2XMeshNetwork.hpp    Decentralized mesh networking for drone swarm
// Leader-Follower topology over Fast-DDS / UDP multicast fallback
// Drone Swarm Sensor Fusion  |  Phase 3  Swarm V2X
 
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "swarm/EdgeConsensusManager.hpp"
#include "swarm/SwarmStateCache.hpp"
#include "sensors/LidarSensor.hpp"

namespace drone::swarm {

struct SwarmSecurityConfig;
class SwarmSecurityContext;

struct SwarmHealthMetrics {
    float battery_pct{100.0f};
    float motor_health{1.0f};
    float link_quality{1.0f};
    float cpu_headroom{1.0f};
    float thermal_headroom{1.0f};
    float leadership_score{0.0f};
    bool emergency_fault{false};
};

 
// DroneRole
 
enum class DroneRole : uint8_t {
    CANDIDATE = 0,  // not yet in swarm
    FOLLOWER  = 1,
    LEADER    = 2,
    RELAY     = 3   // bandwidth relay only
};

inline std::string_view to_string(DroneRole r) {
    switch (r) {
    case DroneRole::CANDIDATE: return "CANDIDATE";
    case DroneRole::FOLLOWER:  return "FOLLOWER";
    case DroneRole::LEADER:    return "LEADER";
    case DroneRole::RELAY:     return "RELAY";
    }
    return "UNKNOWN";
}

 
// SwarmMessage    wire format for all inter-drone traffic
 
struct SwarmMessage {
    enum class Type : uint8_t {
        HEARTBEAT       = 0,
        POSE_UPDATE     = 1,
        KEYFRAME_SHARE  = 2,
        LEADER_ELECT    = 3,
        FORMATION_CMD   = 4,
        EMERGENCY_STOP  = 5,
        MISSION_SYNC    = 6,
        EDGE_PACKET     = 7,
    };

    uint32_t    src_id{0};
    uint32_t    dst_id{0xFFFFFFFF};   // broadcast
    uint32_t    seq_num{0};
    double      timestamp{0.0};
    Type        type{Type::HEARTBEAT};
    uint8_t     hop_count{0};
    uint8_t     ttl{8};
    uint16_t    payload_len{0};
    std::vector<uint8_t> payload;

    // Serialization (little-endian)
    std::vector<uint8_t> serialize() const;
    static std::optional<SwarmMessage> deserialize(const uint8_t* data, size_t len);
};

 
// PeerInfo    known state of another drone
 
struct PeerInfo {
    uint32_t    id{0};
    DroneRole   role{DroneRole::CANDIDATE};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    float       battery_pct{100.0f};
    float       rssi_dbm{-50.0f};
    double      last_seen_ts{0.0};
    uint32_t    seq_last{0};
    bool        reachable{true};
    SwarmHealthMetrics health{};

    [[nodiscard]] bool is_stale(double timeout_s = 2.0) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct AvoidanceObstacle {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    float radius_m{0.8f};
    bool dynamic{false};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

 
// FormationCommand    issued by leader to all followers
 
struct FormationCommand {
    enum class Formation : uint8_t { LINE, VEE, DIAMOND, WEDGE, FREE };

    Formation   shape{Formation::DIAMOND};
    float       spacing_m{2.5f};
    float       altitude_m{10.0f};
    Eigen::Vector3d  leader_target{Eigen::Vector3d::Zero()};
    float       velocity_mps{3.0f};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct LocalEdgeState {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    double localization_confidence{0.0};
    std::string source{"unavailable"};
    std::string edge_health_status{"nominal"};
    std::string autonomy_state{"distributed_autonomy"};
    std::string consensus_state{"single_node"};
    uint64_t consensus_epoch{0};
    int local_obstacle_count{0};
    int shared_obstacle_count{0};
    double mesh_bandwidth_kbps{0.0};
    bool disconnected_operation{false};
    uint64_t trust_epoch{0};
    std::string threat_level{"none"};
    std::string threat_summary{};
    EdgeSerializationMode serialization_mode{EdgeSerializationMode::JSON};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

 
class V2XMeshNetwork {
public:
    using MessageCallback  = std::function<void(const SwarmMessage&)>;
    using PeerCallback     = std::function<void(const PeerInfo&)>;

    explicit V2XMeshNetwork(uint32_t local_id,
                             std::string multicast_group = "239.255.0.1",
                             uint16_t    port            = 7400);
    ~V2XMeshNetwork();

    //  Lifecycle 
    bool start();
    void stop();

    //  Outbound â”€
    bool broadcast(SwarmMessage::Type type, std::vector<uint8_t> payload = {});
    bool unicast  (uint32_t dst, SwarmMessage::Type type, std::vector<uint8_t> payload);
    void configure_security(SwarmSecurityConfig cfg);
    void set_edge_serialization_mode(EdgeSerializationMode mode);
    [[nodiscard]] bool security_enabled() const;
    [[nodiscard]] std::string security_last_error() const;
    void set_local_health(SwarmHealthMetrics health);
    void set_local_edge_state(const LocalEdgeState& state);
    [[nodiscard]] SwarmHealthMetrics local_health() const;
    [[nodiscard]] static float compute_leadership_score(const SwarmHealthMetrics& health);

    //  Leader election (Bully algorithm variant) â”€
    void trigger_election();
    [[nodiscard]] DroneRole local_role() const { return role_.load(); }
    [[nodiscard]] uint32_t  leader_id()  const { return leader_id_.load(); }

    //  Formation control â”€
    bool send_formation(const FormationCommand& cmd);  // LEADER only

    //  Peer registry 
    [[nodiscard]] std::vector<PeerInfo> active_peers() const;
    [[nodiscard]] size_t                peer_count()   const;
    [[nodiscard]] size_t                stale_peer_count() const;
    [[nodiscard]] size_t                safety_eligible_peer_count() const;
    [[nodiscard]] std::string           consensus_state() const;
    [[nodiscard]] uint64_t              consensus_epoch() const;
    [[nodiscard]] std::string           edge_health_status() const;
    [[nodiscard]] std::string           autonomy_state() const;
    [[nodiscard]] float                 mesh_bandwidth_kbps() const;
    [[nodiscard]] bool                  disconnected_operation() const;
    [[nodiscard]] bool                  split_swarm_isolated() const;
    [[nodiscard]] EdgeSerializationMode edge_serialization_mode() const;
    [[nodiscard]] EdgeSerializationMetrics edge_serialization_metrics() const;

    //  Callbacks 
    void on_message(MessageCallback cb) { msg_cb_  = std::move(cb); }
    void on_peer_update(PeerCallback cb){ peer_cb_ = std::move(cb); }

    //  Metrics 
    [[nodiscard]] float    avg_latency_ms()   const { return avg_latency_ms_; }
    [[nodiscard]] uint32_t tx_count()         const { return tx_count_; }
    [[nodiscard]] uint32_t rx_count()         const { return rx_count_; }
    [[nodiscard]] float    packet_loss_pct()  const { return packet_loss_pct_; }

private:
    void recv_loop();
    void heartbeat_loop();
    void election_loop();
    void expire_stale_peers();

    void handle_message(const SwarmMessage& msg);
    void handle_heartbeat(const SwarmMessage& msg);
    void handle_election(const SwarmMessage& msg);
    void handle_pose_update(const SwarmMessage& msg);
    void handle_edge_packet(const SwarmMessage& msg);
    [[nodiscard]] bool should_force_re_election(const PeerInfo& peer) const;

    std::vector<uint8_t> build_heartbeat_payload() const;
    bool broadcast_edge_packet(const EdgePeerPacket& packet);
    EdgePeerPacket build_edge_heartbeat_packet() const;
    EdgePeerPacket build_edge_pose_packet() const;
    EdgePeerPacket build_edge_health_packet() const;
    EdgePeerPacket build_edge_obstacle_packet() const;
    EdgePeerPacket build_edge_threat_packet() const;
    EdgePeerPacket build_edge_consensus_packet() const;
    EdgePeerPacket build_edge_emergency_packet() const;
    EdgePeerPacket build_edge_goodbye_packet() const;

    uint32_t    local_id_;
    std::string multicast_group_;
    uint16_t    port_;

    int         sock_{-1};
    uint32_t    seq_counter_{0};

    std::atomic<DroneRole> role_{DroneRole::CANDIDATE};
    std::atomic<uint32_t>  leader_id_{0};
    std::atomic<bool>      election_ongoing_{false};
    mutable std::mutex      health_mutex_;
    SwarmHealthMetrics      local_health_{};
    mutable std::mutex      local_edge_state_mutex_;
    LocalEdgeState          local_edge_state_{};

    mutable std::mutex                          peers_mutex_;
    std::unordered_map<uint32_t, PeerInfo>      peers_;
    SwarmStateCache edge_state_cache_{};
    EdgeConsensusManager edge_consensus_{local_id_};

    std::thread recv_thread_;
    std::thread heartbeat_thread_;
    std::atomic<bool> running_{false};
    std::unique_ptr<SwarmSecurityContext> security_;

    MessageCallback msg_cb_;
    PeerCallback    peer_cb_;

    // Metrics
    float    avg_latency_ms_{0.0f};
    uint32_t tx_count_{0};
    uint32_t rx_count_{0};
    uint32_t rx_expected_{0};
    float    packet_loss_pct_{0.0f};
    std::atomic<uint64_t> edge_tx_bytes_{0};
    std::atomic<uint64_t> edge_rx_bytes_{0};
    mutable std::mutex edge_serialization_metrics_mutex_;
    EdgeSerializationMode edge_serialization_mode_{EdgeSerializationMode::JSON};
    EdgeSerializationMetrics edge_serialization_metrics_{};
    mutable std::atomic<uint64_t> last_edge_sequence_{0};

    std::shared_ptr<spdlog::logger> logger_{spdlog::get("V2X")};

    static constexpr double kHeartbeatInterval_s{0.5};
    static constexpr double kPeerTimeout_s{3.0};
    static constexpr size_t kMaxPayload{65507};
};

 
// LeaderFollowerController    Formation geometry and waypoint following
 
class LeaderFollowerController {
public:
    explicit LeaderFollowerController(uint32_t drone_id,
                                       std::shared_ptr<V2XMeshNetwork> net)
        : id_(drone_id), net_(std::move(net)) {}

    // Compute the desired position offset for this follower given the
    // current formation and peer positions
    [[nodiscard]] Eigen::Vector3d compute_target(
        const Eigen::Vector3d& leader_pos,
        const FormationCommand& cmd,
        uint32_t follower_index) const;

    // P-controller step  velocity command (m/s)
    [[nodiscard]] Eigen::Vector3d velocity_command(
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& current_velocity,
        const Eigen::Vector3d& target_pos,
        float kp = 1.5f,
        float max_speed_mps = 4.0f) const;

    [[nodiscard]] Eigen::Vector3d velocity_command(
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& current_velocity,
        const Eigen::Vector3d& target_pos,
        const std::vector<PeerInfo>& peers,
        const std::vector<AvoidanceObstacle>& obstacles = {},
        float kp = 1.5f,
        float max_speed_mps = 4.0f) const;

    [[nodiscard]] Eigen::Vector3d compute_avoidance_velocity(
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& current_velocity,
        const std::vector<PeerInfo>& peers,
        const std::vector<AvoidanceObstacle>& obstacles = {},
        float min_separation_m = 2.0f,
        float influence_radius_m = 6.0f,
        float max_avoid_speed_mps = 3.0f,
        float prediction_horizon_s = 2.0f) const;

    [[nodiscard]] static std::vector<AvoidanceObstacle> obstacles_from_lidar(
        const sensors::LidarMeasurement& scan,
        const Eigen::Vector3d& drone_position,
        size_t stride = 24,
        float obstacle_radius_m = 0.7f);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    uint32_t                        id_;
    std::shared_ptr<V2XMeshNetwork> net_;
};

} // namespace drone::swarm
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
