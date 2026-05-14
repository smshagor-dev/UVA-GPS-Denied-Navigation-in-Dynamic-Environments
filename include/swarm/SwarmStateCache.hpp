#pragma once

#include "swarm/EdgePeerProtocol.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace drone::swarm {

struct CachedPeerState {
    uint32_t peer_id{0};
    uint64_t last_packet_timestamp_ms{0};
    uint64_t last_heartbeat_timestamp_ms{0};
    uint64_t last_pose_timestamp_ms{0};
    uint64_t last_obstacle_digest_timestamp_ms{0};
    uint32_t last_sequence_number{0};
    uint64_t trust_epoch{0};
    double localization_confidence{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    std::string source{"unavailable"};
    std::string edge_health_status{"unknown"};
    std::string autonomy_state{"unknown"};
    std::string consensus_state{"single_node"};
    std::string threat_level{"none"};
    int peer_count{0};
    int stale_peer_count{0};
    int local_obstacle_count{0};
    int shared_obstacle_count{0};
    uint32_t obstacle_digest_freshness_ms{0};
    double mesh_bandwidth_kbps{0.0};
    bool emergency_fault{false};
    bool disconnected_operation{false};
    bool stale{false};
    bool split_swarm_isolated{false};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct SwarmStateCacheConfig {
    size_t max_entries{32};
    uint32_t stale_peer_timeout_ms{900};
    uint32_t entry_expiry_ms{2500};
};

struct SwarmStateCacheObservation {
    bool accepted{false};
    bool peer_became_stale{false};
    std::string reason{};
};

class SwarmStateCache {
public:
    explicit SwarmStateCache(SwarmStateCacheConfig config = {});

    SwarmStateCacheObservation observe_packet(const EdgePeerPacket& packet, uint64_t now_ms);
    void expire(uint64_t now_ms);
    void clear();

    [[nodiscard]] size_t peer_count() const;
    [[nodiscard]] size_t stale_peer_count() const;
    [[nodiscard]] size_t safety_eligible_peer_count() const;
    [[nodiscard]] bool disconnected_operation() const;
    [[nodiscard]] bool split_swarm_isolated() const;
    [[nodiscard]] std::optional<uint32_t> last_sequence_number(uint32_t peer_id) const;
    [[nodiscard]] std::optional<CachedPeerState> peer_state(uint32_t peer_id) const;
    [[nodiscard]] std::vector<CachedPeerState> snapshot() const;

private:
    void trim_to_limit_locked();

    SwarmStateCacheConfig config_{};
    mutable std::mutex mutex_{};
    std::unordered_map<uint32_t, CachedPeerState> peers_{};
};

} // namespace drone::swarm
