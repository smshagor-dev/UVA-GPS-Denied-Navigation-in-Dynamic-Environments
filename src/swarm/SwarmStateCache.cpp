#include "swarm/SwarmStateCache.hpp"

#include <algorithm>

namespace drone::swarm {

SwarmStateCache::SwarmStateCache(SwarmStateCacheConfig config) : config_(std::move(config)) {}

SwarmStateCacheObservation SwarmStateCache::observe_packet(const EdgePeerPacket& packet,
                                                           uint64_t now_ms) {
    SwarmStateCacheObservation result;
    std::lock_guard lock(mutex_);

    const auto existing = peers_.find(packet.sender_id);
    const auto validation = validate_edge_packet(
        packet, {
                    now_ms,
                    existing != peers_.end() ? existing->second.last_sequence_number : 0u,
                    existing != peers_.end(),
                    1400u,
                });
    if (!validation.ok) {
        result.reason = validation.error;
        return result;
    }

    auto& peer = peers_[packet.sender_id];
    peer.peer_id = packet.sender_id;
    peer.last_packet_timestamp_ms = packet.timestamp_ms;
    peer.last_sequence_number = packet.sequence_number;
    peer.trust_epoch = packet.trust_epoch;
    peer.source = normalize_edge_source_tag(packet.source);
    peer.stale = false;
    peer.split_swarm_isolated = false;

    if (packet.heartbeat.has_value()) {
        const auto& payload = *packet.heartbeat;
        peer.last_heartbeat_timestamp_ms = packet.timestamp_ms;
        peer.peer_count = payload.peer_count;
        peer.stale_peer_count = payload.stale_peer_count;
        peer.edge_health_status = payload.edge_health_status;
        peer.autonomy_state = payload.autonomy_state;
        peer.emergency_fault = payload.emergency_fault;
    }
    if (packet.pose_state.has_value()) {
        const auto& payload = *packet.pose_state;
        peer.last_pose_timestamp_ms = packet.timestamp_ms;
        peer.position = payload.position;
        peer.velocity = payload.velocity;
        peer.localization_confidence = payload.localization_confidence;
    }
    if (packet.edge_health.has_value()) {
        const auto& payload = *packet.edge_health;
        peer.edge_health_status = payload.edge_health_status;
        peer.autonomy_state = payload.autonomy_state;
        peer.consensus_state = payload.consensus_state;
        peer.mesh_bandwidth_kbps = payload.mesh_bandwidth_kbps;
        peer.disconnected_operation = payload.disconnected_operation;
    }
    if (packet.obstacle_digest.has_value()) {
        const auto& payload = *packet.obstacle_digest;
        peer.last_obstacle_digest_timestamp_ms = packet.timestamp_ms;
        peer.local_obstacle_count = payload.local_obstacle_count;
        peer.shared_obstacle_count = payload.shared_obstacle_count;
        peer.obstacle_digest_freshness_ms = payload.freshness_ms;
    }
    if (packet.threat_digest.has_value()) {
        peer.threat_level = packet.threat_digest->threat_level;
    }
    if (packet.consensus_state.has_value()) {
        peer.consensus_state = packet.consensus_state->consensus_state;
    }
    if (packet.packet_type == EdgePacketType::PEER_GOODBYE) {
        peer.stale = true;
        peer.split_swarm_isolated = true;
    }

    trim_to_limit_locked();
    result.accepted = true;
    return result;
}

void SwarmStateCache::expire(uint64_t now_ms) {
    std::lock_guard lock(mutex_);
    for (auto it = peers_.begin(); it != peers_.end();) {
        const auto age_ms = now_ms > it->second.last_packet_timestamp_ms
                                ? (now_ms - it->second.last_packet_timestamp_ms)
                                : 0u;
        if (age_ms > config_.entry_expiry_ms) {
            it = peers_.erase(it);
            continue;
        }
        it->second.stale = age_ms > config_.stale_peer_timeout_ms;
        it->second.split_swarm_isolated = it->second.stale || it->second.disconnected_operation;
        ++it;
    }
}

void SwarmStateCache::clear() {
    std::lock_guard lock(mutex_);
    peers_.clear();
}

size_t SwarmStateCache::peer_count() const {
    std::lock_guard lock(mutex_);
    return peers_.size();
}

size_t SwarmStateCache::stale_peer_count() const {
    std::lock_guard lock(mutex_);
    return std::count_if(peers_.begin(), peers_.end(),
                         [](const auto& item) { return item.second.stale; });
}

size_t SwarmStateCache::safety_eligible_peer_count() const {
    std::lock_guard lock(mutex_);
    return std::count_if(peers_.begin(), peers_.end(), [](const auto& item) {
        return !item.second.stale && !item.second.disconnected_operation &&
               item.second.edge_health_status != "fault" && item.second.trust_epoch > 0;
    });
}

bool SwarmStateCache::disconnected_operation() const {
    std::lock_guard lock(mutex_);
    const auto eligible = std::count_if(peers_.begin(), peers_.end(), [](const auto& item) {
        return !item.second.stale && !item.second.disconnected_operation &&
               item.second.edge_health_status != "fault" && item.second.trust_epoch > 0;
    });
    return peers_.empty() || eligible == 0;
}

bool SwarmStateCache::split_swarm_isolated() const {
    std::lock_guard lock(mutex_);
    return !peers_.empty() && std::all_of(peers_.begin(), peers_.end(), [](const auto& item) {
        return item.second.stale || item.second.disconnected_operation;
    });
}

std::optional<uint32_t> SwarmStateCache::last_sequence_number(uint32_t peer_id) const {
    std::lock_guard lock(mutex_);
    const auto it = peers_.find(peer_id);
    if (it == peers_.end()) {
        return std::nullopt;
    }
    return it->second.last_sequence_number;
}

std::optional<CachedPeerState> SwarmStateCache::peer_state(uint32_t peer_id) const {
    std::lock_guard lock(mutex_);
    const auto it = peers_.find(peer_id);
    if (it == peers_.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::vector<CachedPeerState> SwarmStateCache::snapshot() const {
    std::lock_guard lock(mutex_);
    std::vector<CachedPeerState> out;
    out.reserve(peers_.size());
    for (const auto& [peer_id, state] : peers_) {
        (void)peer_id;
        out.push_back(state);
    }
    std::sort(out.begin(), out.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.peer_id < rhs.peer_id; });
    return out;
}

void SwarmStateCache::trim_to_limit_locked() {
    if (peers_.size() <= config_.max_entries) {
        return;
    }
    auto oldest_it = peers_.begin();
    for (auto it = peers_.begin(); it != peers_.end(); ++it) {
        if (it->second.last_packet_timestamp_ms < oldest_it->second.last_packet_timestamp_ms) {
            oldest_it = it;
        }
    }
    peers_.erase(oldest_it);
}

} // namespace drone::swarm
