#include "swarm/EdgeConsensusManager.hpp"

#include <algorithm>

namespace drone::swarm {

EdgeConsensusManager::EdgeConsensusManager(uint32_t local_id, int default_quorum)
    : local_id_(local_id),
      default_quorum_(std::max(default_quorum, 1)) {}

void EdgeConsensusManager::set_local_safety_override(bool enabled) {
    std::lock_guard lock(mutex_);
    state_.local_safety_override = enabled;
}

void EdgeConsensusManager::propose(ConsensusProposalType proposal_type, uint64_t epoch, int quorum_count) {
    std::lock_guard lock(mutex_);
    state_.proposal_type = proposal_type;
    state_.consensus_epoch = epoch;
    state_.quorum_count = std::max(quorum_count, default_quorum_);
    supporters_.clear();
    supporters_.insert(local_id_);
    state_.supporting_peer_count = supporters_.size();
    state_.quorum_met = supporters_.size() >= static_cast<size_t>(state_.quorum_count);
    state_.state = state_.quorum_met ? "quorum_met" : "quorum_pending";
}

void EdgeConsensusManager::observe_packet(const EdgePeerPacket& packet, const SwarmStateCache& cache) {
    if (packet.packet_type != EdgePacketType::CONSENSUS_STATE || !packet.consensus_state.has_value()) {
        return;
    }
    const auto peer = cache.peer_state(packet.sender_id);
    if (!peer.has_value() || peer->stale) {
        return;
    }

    std::lock_guard lock(mutex_);
    const auto& payload = *packet.consensus_state;
    if (payload.consensus_epoch > state_.consensus_epoch) {
        state_.proposal_type = payload.proposal_type;
        state_.consensus_epoch = payload.consensus_epoch;
        state_.quorum_count = std::max(payload.quorum_count, default_quorum_);
        state_.state = payload.consensus_state;
        supporters_.clear();
    }
    if (payload.consensus_epoch == state_.consensus_epoch &&
        payload.proposal_type == state_.proposal_type) {
        supporters_.insert(packet.sender_id);
    }
    state_.supporting_peer_count = supporters_.size();
    state_.quorum_met = supporters_.size() >= static_cast<size_t>(std::max(state_.quorum_count, default_quorum_));
    if (!state_.quorum_met && cache.split_swarm_isolated()) {
        state_.state = "split_swarm_recovery";
    } else if (state_.quorum_met) {
        state_.state = "quorum_met";
    } else {
        state_.state = "quorum_pending";
    }
}

void EdgeConsensusManager::expire(const SwarmStateCache& cache) {
    std::lock_guard lock(mutex_);
    const auto states = cache.snapshot();
    for (auto it = supporters_.begin(); it != supporters_.end();) {
        const auto peer = std::find_if(states.begin(), states.end(), [&](const auto& item) {
            return item.peer_id == *it;
        });
        if (*it != local_id_ && (peer == states.end() || peer->stale)) {
            it = supporters_.erase(it);
            continue;
        }
        ++it;
    }
    state_.supporting_peer_count = supporters_.size();
    state_.quorum_met = supporters_.size() >= static_cast<size_t>(std::max(state_.quorum_count, default_quorum_));
    if (cache.split_swarm_isolated()) {
        state_.state = "split_swarm_recovery";
    } else if (cache.safety_eligible_peer_count() == 0) {
        state_.state = "isolated";
    } else if (state_.quorum_met) {
        state_.state = "quorum_met";
    } else {
        state_.state = "quorum_pending";
    }
}

bool EdgeConsensusManager::should_apply_collective_action() const {
    std::lock_guard lock(mutex_);
    return !state_.local_safety_override &&
           state_.proposal_type != ConsensusProposalType::NONE &&
           state_.quorum_met;
}

bool EdgeConsensusManager::should_accept_emergency_packet(const EdgePeerPacket& packet) const {
    return packet.packet_type == EdgePacketType::EMERGENCY_CORRIDOR;
}

EdgeConsensusSnapshot EdgeConsensusManager::snapshot(const SwarmStateCache& cache) const {
    std::lock_guard lock(mutex_);
    auto out = state_;
    if (cache.split_swarm_isolated()) {
        out.state = "split_swarm_recovery";
    } else if (cache.safety_eligible_peer_count() == 0) {
        out.state = "isolated";
    }
    out.supporting_peer_count = supporters_.size();
    return out;
}

} // namespace drone::swarm
