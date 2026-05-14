#pragma once

#include "swarm/EdgePeerProtocol.hpp"
#include "swarm/SwarmStateCache.hpp"

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_set>

namespace drone::swarm {

struct EdgeConsensusSnapshot {
    ConsensusProposalType proposal_type{ConsensusProposalType::NONE};
    uint64_t consensus_epoch{0};
    int quorum_count{0};
    size_t supporting_peer_count{0};
    bool quorum_met{false};
    bool local_safety_override{false};
    std::string state{"single_node"};
};

class EdgeConsensusManager {
public:
    explicit EdgeConsensusManager(uint32_t local_id, int default_quorum = 2);

    void set_local_safety_override(bool enabled);
    void propose(ConsensusProposalType proposal_type, uint64_t epoch, int quorum_count);
    void observe_packet(const EdgePeerPacket& packet, const SwarmStateCache& cache);
    void expire(const SwarmStateCache& cache);

    [[nodiscard]] bool should_apply_collective_action() const;
    [[nodiscard]] bool should_accept_emergency_packet(const EdgePeerPacket& packet) const;
    [[nodiscard]] EdgeConsensusSnapshot snapshot(const SwarmStateCache& cache) const;

private:
    uint32_t local_id_{0};
    int default_quorum_{2};
    mutable std::mutex mutex_{};
    EdgeConsensusSnapshot state_{};
    std::unordered_set<uint32_t> supporters_{};
};

} // namespace drone::swarm
