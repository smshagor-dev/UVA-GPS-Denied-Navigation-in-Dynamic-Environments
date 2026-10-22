// Replay the exact received-packet CSV through production C++ authentication
// and SwarmStateCache. Only sequence policy is compared: synthetic packets
// are freshly signed with a test key before delivery to the real cache.
#include "security/PeerPacketAuth.hpp"
#include "swarm/SwarmStateCache.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

uint64_t parse_unsigned(const std::string& value) {
    if (value.empty() || value.front() == '-') {
        throw std::invalid_argument("invalid unsigned CSV field");
    }
    size_t used = 0;
    const auto parsed = std::stoull(value, &used);
    if (used != value.size()) {
        throw std::invalid_argument("noninteger CSV field");
    }
    return parsed;
}

std::vector<uint64_t> parse_row(const std::string& row) {
    std::istringstream stream(row);
    std::string column;
    std::vector<uint64_t> values;
    while (std::getline(stream, column, ',')) {
        values.push_back(parse_unsigned(column));
    }
    if (values.size() != 4) {
        throw std::invalid_argument("expected four receiver CSV columns");
    }
    return values;
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: peer_replay_cache_cli receiver.csv\n";
        return 2;
    }
    try {
        std::ifstream input(argv[1]);
        if (!input) {
            throw std::runtime_error("cannot open receiver CSV");
        }
        std::string header;
        if (!std::getline(input, header)) {
            throw std::runtime_error("missing CSV header");
        }
        if (!header.empty() && header.back() == '\r') {
            header.pop_back();
        }
        if (header != "sender_id,epoch,sequence,rx_ms") {
            throw std::invalid_argument("unexpected receiver CSV header");
        }

        drone::swarm::SwarmStateCache cache({32, 900, 2500});
        drone::security::PacketAuthConfig auth;
        auth.mode = drone::security::AuthMode::HMAC_SHA256;
        auth.shared_secret = "benchmark-only-not-a-production-secret-2026";
        auth.runtime_profile = "edge_swarm";
        auth.trust_epoch = 1;

        std::cout << "sender_id,epoch,sequence,rx_ms,accepted\n";
        std::string row;
        size_t line = 1;
        while (std::getline(input, row)) {
            ++line;
            if (row.empty()) {
                continue;
            }
            if (!row.empty() && row.back() == '\r') {
                row.pop_back();
            }
            const auto fields = parse_row(row);
            if (fields[0] == 0 || fields[0] > std::numeric_limits<uint32_t>::max() ||
                fields[1] != 1 || fields[2] > std::numeric_limits<uint32_t>::max()) {
                throw std::invalid_argument("invalid peer, epoch or sequence at line " +
                                            std::to_string(line));
            }

            drone::swarm::EdgePeerPacket packet;
            packet.packet_type = drone::swarm::EdgePacketType::HEARTBEAT;
            packet.sender_id = static_cast<uint32_t>(fields[0]);
            packet.trust_epoch = fields[1];
            packet.sequence_number = static_cast<uint32_t>(fields[2]);
            packet.timestamp_ms = 1000; // fixed authenticated/fresh observation context
            packet.ttl_ms = 1000;
            packet.source = "simulation";
            packet.heartbeat = drone::swarm::HeartbeatPayload{};
            const auto signed_result = drone::security::signPacket(packet, auth);
            if (!signed_result.accepted()) {
                throw std::runtime_error("packet signing failed");
            }
            const auto verified = drone::security::verifyPacket(packet, auth, 1000);
            if (!verified.accepted()) {
                throw std::runtime_error("synthetic packet authentication failed");
            }
            const auto outcome = cache.observe_packet(packet, 1000);
            std::cout << fields[0] << ',' << fields[1] << ',' << fields[2] << ',' << fields[3]
                      << ',' << (outcome.accepted ? 1 : 0) << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << "replay cache input validation failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
