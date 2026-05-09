// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

 
// V2XMeshNetwork.cpp    Swarm comms, leader election, formation control
// Drone Swarm Sensor Fusion  |  Phase 3
 
#include "swarm/V2XMeshNetwork.hpp"
#include "swarm/SwarmSecurity.hpp"
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>

#ifdef _WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#elif defined(__linux__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace drone::swarm {

namespace {

constexpr double kAvoidanceEps = 1e-6;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kDegToRad = 0.01745329251994329577;
constexpr double kDeadlockEscapeSpeed = 0.8;
constexpr size_t kHeartbeatPayloadSize = 1 + (sizeof(float) * 6) + 1;

Eigen::Vector3d deterministic_separation_axis(uint32_t local_id, uint32_t peer_id) {
    const uint32_t mixed = (local_id * 2654435761u) ^ (peer_id * 2246822519u);
    const double angle = static_cast<double>(mixed)
        * (kTwoPi / static_cast<double>(std::numeric_limits<uint32_t>::max()));
    return {std::cos(angle), std::sin(angle), 0.15};
}

Eigen::Vector3d deterministic_lateral_axis(const Eigen::Vector3d& preferred_velocity) {
    Eigen::Vector3d axis = preferred_velocity.cross(Eigen::Vector3d::UnitZ());
    if (axis.norm() < kAvoidanceEps) {
        axis = Eigen::Vector3d::UnitY();
    }
    axis.z() = 0.0;
    return axis.normalized();
}

double closing_speed_along(const Eigen::Vector3d& relative_position,
                           const Eigen::Vector3d& relative_velocity) {
    const double distance = std::max(relative_position.norm(), kAvoidanceEps);
    return -relative_velocity.dot(relative_position / distance);
}

} // namespace

//  Wire format header (little-endian) 
// [4B magic][4B src_id][4B dst_id][4B seq][8B timestamp][1B type][1B hop]
// [1B ttl][1B pad][2B payload_len][payload...]
static constexpr uint32_t kMagic = 0x56325831; // "V2X1"
static constexpr size_t   kHdrSize = 30;

 
std::vector<uint8_t> SwarmMessage::serialize() const {
    std::vector<uint8_t> out;
    out.reserve(kHdrSize + payload.size());

    auto push32 = [&](uint32_t v) {
        out.push_back(v & 0xFF);
        out.push_back((v >> 8)  & 0xFF);
        out.push_back((v >> 16) & 0xFF);
        out.push_back((v >> 24) & 0xFF);
    };
    auto push64 = [&](uint64_t v) {
        for (int i = 0; i < 8; ++i)
            out.push_back((v >> (8*i)) & 0xFF);
    };
    auto push16 = [&](uint16_t v) {
        out.push_back(v & 0xFF);
        out.push_back((v >> 8) & 0xFF);
    };

    push32(kMagic);
    push32(src_id);
    push32(dst_id);
    push32(seq_num);

    // Timestamp as fixed-point (ns)
    push64(static_cast<uint64_t>(timestamp * 1e9));

    out.push_back(static_cast<uint8_t>(type));
    out.push_back(hop_count);
    out.push_back(ttl);
    out.push_back(0x00);  // pad

    push16(static_cast<uint16_t>(payload.size()));
    out.insert(out.end(), payload.begin(), payload.end());

    return out;
}

std::optional<SwarmMessage> SwarmMessage::deserialize(const uint8_t* data, size_t len) {
    if (!data || len < kHdrSize) return std::nullopt;

    auto r32 = [&](size_t off) -> uint32_t {
        return static_cast<uint32_t>(data[off])
             | (static_cast<uint32_t>(data[off+1]) << 8)
             | (static_cast<uint32_t>(data[off+2]) << 16)
             | (static_cast<uint32_t>(data[off+3]) << 24);
    };
    auto r64 = [&](size_t off) -> uint64_t {
        uint64_t v = 0;
        for (int i = 0; i < 8; ++i)
            v |= static_cast<uint64_t>(data[off+i]) << (8*i);
        return v;
    };
    auto r16 = [&](size_t off) -> uint16_t {
        return static_cast<uint16_t>(data[off]) | (static_cast<uint16_t>(data[off+1]) << 8);
    };

    if (r32(0) != kMagic) return std::nullopt;

    SwarmMessage msg;
    msg.src_id      = r32(4);
    msg.dst_id      = r32(8);
    msg.seq_num     = r32(12);
    msg.timestamp   = static_cast<double>(r64(16)) * 1e-9;
    msg.type        = static_cast<Type>(data[24]);
    msg.hop_count   = data[25];
    msg.ttl         = data[26];
    // data[27] = pad
    msg.payload_len = r16(28);

    if (kHdrSize + msg.payload_len > len) return std::nullopt;

    msg.payload.assign(data + kHdrSize, data + kHdrSize + msg.payload_len);
    return msg;
}

 
// PeerInfo
 
bool PeerInfo::is_stale(double timeout_s) const {
    const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    return (now - last_seen_ts) > timeout_s;
}

 
// V2XMeshNetwork
 
V2XMeshNetwork::V2XMeshNetwork(uint32_t local_id,
                                 std::string multicast_group,
                                 uint16_t port)
    : local_id_(local_id)
    , multicast_group_(std::move(multicast_group))
    , port_(port) {
    logger_ = spdlog::get("V2X");
    if (!logger_) logger_ = spdlog::stdout_color_mt("V2X");
}

V2XMeshNetwork::~V2XMeshNetwork() { stop(); }

bool V2XMeshNetwork::start() {
    if (running_.exchange(true)) return true;

#ifdef _WIN32
    WSADATA wsa_data;
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
        running_.store(false);
        return false;
    }

    sock_ = static_cast<int>(::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
    if (sock_ < 0) {
        running_.store(false);
        return false;
    }

    BOOL yes = TRUE;
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        stop();
        return false;
    }

    u_long non_blocking = 1;
    ::ioctlsocket(static_cast<SOCKET>(sock_), FIONBIO, &non_blocking);
#elif defined(__linux__)
    // Create UDP socket
    sock_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ < 0) {
        logger_->error("V2X socket creation failed");
        running_.store(false);
        return false;
    }

    // Enable broadcast + multicast
    int yes = 1;
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));

    // Bind
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port_);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (::bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        logger_->error("V2X bind failed on port {}", port_);
    }

    // Join multicast group
    ip_mreq mreq{};
    mreq.imr_multiaddr.s_addr = ::inet_addr(multicast_group_.c_str());
    mreq.imr_interface.s_addr = INADDR_ANY;
    ::setsockopt(sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));

    // Set TTL for multicast
    int ttl = 16;
    ::setsockopt(sock_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
#endif

    recv_thread_      = std::thread([this] { recv_loop(); });
    heartbeat_thread_ = std::thread([this] { heartbeat_loop(); });

    logger_->info("V2X mesh started. ID={} group={} port={}",
                  local_id_, multicast_group_, port_);
    return true;
}

void V2XMeshNetwork::stop() {
    if (!running_.exchange(false)) return;

#ifdef _WIN32
    if (sock_ >= 0) { ::closesocket(static_cast<SOCKET>(sock_)); sock_ = -1; }
    WSACleanup();
#elif defined(__linux__)
    if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
#endif

    if (recv_thread_.joinable())      recv_thread_.join();
    if (heartbeat_thread_.joinable()) heartbeat_thread_.join();

    logger_->info("V2X mesh stopped. TX={} RX={} loss={:.1f}%",
                  tx_count_, rx_count_, packet_loss_pct_);
}

 
bool V2XMeshNetwork::broadcast(SwarmMessage::Type type,
                                std::vector<uint8_t> payload) {
    SwarmMessage msg;
    msg.src_id      = local_id_;
    msg.dst_id      = 0xFFFFFFFF;
    msg.seq_num     = ++seq_counter_;
    msg.timestamp   = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    msg.type        = type;
    msg.ttl         = 8;
    msg.payload     = std::move(payload);
    msg.payload_len = static_cast<uint16_t>(msg.payload.size());

    const auto bytes = (security_ && security_->enabled())
        ? security_->seal(msg)
        : msg.serialize();
    if (bytes.size() > kMaxPayload) return false;

#if defined(_WIN32) || defined(__linux__)
    sockaddr_in dest{};
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(port_);
    dest.sin_addr.s_addr = ::inet_addr(multicast_group_.c_str());

#ifdef _WIN32
    const int sent = ::sendto(
        static_cast<SOCKET>(sock_),
        reinterpret_cast<const char*>(bytes.data()),
        static_cast<int>(bytes.size()),
        0,
        reinterpret_cast<const sockaddr*>(&dest),
        sizeof(dest));
#else
    const ssize_t sent = ::sendto(
        sock_, bytes.data(), bytes.size(), 0,
        reinterpret_cast<sockaddr*>(&dest), sizeof(dest));
#endif

    if (sent < 0) {
        logger_->warn("V2X broadcast failed (type={})", static_cast<int>(type));
        return false;
    }
#endif

    ++tx_count_;
    return true;
}

bool V2XMeshNetwork::unicast(uint32_t dst,
                             SwarmMessage::Type type,
                             std::vector<uint8_t> payload) {
    SwarmMessage msg;
    msg.src_id      = local_id_;
    msg.dst_id      = dst;
    msg.seq_num     = ++seq_counter_;
    msg.timestamp   = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    msg.type        = type;
    msg.ttl         = 8;
    msg.payload     = std::move(payload);
    msg.payload_len = static_cast<uint16_t>(msg.payload.size());

    const auto bytes = (security_ && security_->enabled())
        ? security_->seal(msg)
        : msg.serialize();
    if (bytes.size() > kMaxPayload) return false;

#if defined(_WIN32) || defined(__linux__)
    sockaddr_in dest{};
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(port_);
    dest.sin_addr.s_addr = ::inet_addr(multicast_group_.c_str());

#ifdef _WIN32
    const int sent = ::sendto(
        static_cast<SOCKET>(sock_),
        reinterpret_cast<const char*>(bytes.data()),
        static_cast<int>(bytes.size()),
        0,
        reinterpret_cast<const sockaddr*>(&dest),
        sizeof(dest));
#else
    const ssize_t sent = ::sendto(
        sock_, bytes.data(), bytes.size(), 0,
        reinterpret_cast<sockaddr*>(&dest), sizeof(dest));
#endif

    if (sent < 0) {
        logger_->warn("V2X unicast failed (dst={} type={})", dst, static_cast<int>(type));
        return false;
    }
#endif

    ++tx_count_;
    return true;
}

bool V2XMeshNetwork::send_formation(const FormationCommand& cmd) {
    if (role_.load() != DroneRole::LEADER) {
        logger_->warn("V2X: node {} rejected formation broadcast because it is not leader", local_id_);
        return false;
    }

    std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float) * 2 + sizeof(double) * 3);
    size_t offset = 0;

    payload[offset++] = static_cast<uint8_t>(cmd.shape);

    std::memcpy(payload.data() + offset, &cmd.spacing_m, sizeof(cmd.spacing_m));
    offset += sizeof(cmd.spacing_m);

    std::memcpy(payload.data() + offset, &cmd.altitude_m, sizeof(cmd.altitude_m));
    offset += sizeof(cmd.altitude_m);

    std::memcpy(payload.data() + offset, cmd.leader_target.data(), sizeof(double) * 3);
    offset += sizeof(double) * 3;

    std::memcpy(payload.data() + offset, &cmd.velocity_mps, sizeof(cmd.velocity_mps));
    return broadcast(SwarmMessage::Type::FORMATION_CMD, std::move(payload));
}

 
void V2XMeshNetwork::recv_loop() {
    std::vector<uint8_t> buf(kMaxPayload);

    while (running_.load()) {
#if defined(_WIN32) || defined(__linux__)
        sockaddr_in sender{};
#ifdef _WIN32
        int sender_len = sizeof(sender);
#else
        socklen_t   sender_len = sizeof(sender);
#endif

#ifdef _WIN32
        const int n = ::recvfrom(
            static_cast<SOCKET>(sock_), reinterpret_cast<char*>(buf.data()), static_cast<int>(buf.size()), 0,
            reinterpret_cast<sockaddr*>(&sender), &sender_len);
#else
        const ssize_t n = ::recvfrom(
            sock_, buf.data(), buf.size(), MSG_DONTWAIT,
            reinterpret_cast<sockaddr*>(&sender), &sender_len);
#endif

        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }

        ++rx_count_;
        std::optional<SwarmMessage> msg;
        if (security_ && security_->enabled()) {
            msg = security_->open(buf.data(), static_cast<size_t>(n));
            if (!msg.has_value() && logger_) {
                logger_->warn("V2X secure frame rejected: {}", security_->last_error());
            }
        } else {
            msg = SwarmMessage::deserialize(buf.data(), static_cast<size_t>(n));
        }
        if (msg && msg->src_id != local_id_) {  // ignore own broadcasts
            handle_message(*msg);
            if (msg_cb_) msg_cb_(*msg);
        }
#else
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
#endif
    }
}

 
void V2XMeshNetwork::heartbeat_loop() {
    while (running_.load()) {
        broadcast(SwarmMessage::Type::HEARTBEAT, build_heartbeat_payload());
        expire_stale_peers();
        std::this_thread::sleep_for(
            std::chrono::duration<double>(kHeartbeatInterval_s));
    }
}

 
void V2XMeshNetwork::handle_heartbeat(const SwarmMessage& msg) {
    std::optional<PeerInfo> updated_peer;
    bool force_re_election = false;
    {
        std::lock_guard lock(peers_mutex_);
        auto& peer         = peers_[msg.src_id];
        peer.id            = msg.src_id;
        peer.last_seen_ts  = msg.timestamp;
        peer.seq_last      = msg.seq_num;
        peer.reachable     = true;

        if (msg.payload.size() >= kHeartbeatPayloadSize) {
            peer.role = static_cast<DroneRole>(msg.payload[0]);
            size_t offset = 1;
            auto read_metric = [&](float& value) {
                std::memcpy(&value, msg.payload.data() + offset, sizeof(float));
                offset += sizeof(float);
            };
            read_metric(peer.health.battery_pct);
            read_metric(peer.health.motor_health);
            read_metric(peer.health.link_quality);
            read_metric(peer.health.cpu_headroom);
            read_metric(peer.health.thermal_headroom);
            read_metric(peer.health.leadership_score);
            peer.health.emergency_fault = (msg.payload[offset] != 0);
            peer.battery_pct = peer.health.battery_pct;
        } else if (msg.payload.size() >= 5) {
            peer.role = static_cast<DroneRole>(msg.payload[0]);
            std::memcpy(&peer.battery_pct, &msg.payload[1], sizeof(float));
            peer.health.battery_pct = peer.battery_pct;
            peer.health.leadership_score = compute_leadership_score(peer.health);
        }

        if (peer.role == DroneRole::LEADER) {
            leader_id_.store(peer.id);
            force_re_election = should_force_re_election(peer);
        }
        updated_peer = peer;
    }

    if (updated_peer && peer_cb_) {
        peer_cb_(*updated_peer);
    }
    if (force_re_election) {
        logger_->warn("V2X: leader {} degraded, triggering MCSS re-election", msg.src_id);
        trigger_election();
    }
}

void V2XMeshNetwork::handle_message(const SwarmMessage& msg) {
    switch (msg.type) {
    case SwarmMessage::Type::HEARTBEAT:    handle_heartbeat(msg);    break;
    case SwarmMessage::Type::LEADER_ELECT: handle_election(msg);     break;
    case SwarmMessage::Type::POSE_UPDATE:  handle_pose_update(msg);  break;
    case SwarmMessage::Type::FORMATION_CMD: break;
    default: break;
    }
}

 
// MCSS-style leader election
 
void V2XMeshNetwork::trigger_election() {
    if (election_ongoing_.exchange(true)) return;
    logger_->info("V2X election triggered by node {}", local_id_);

    auto health = local_health();
    health.leadership_score = compute_leadership_score(health);
    set_local_health(health);

    std::vector<uint8_t> payload(sizeof(uint32_t) + sizeof(float));
    std::memcpy(payload.data(), &local_id_, 4);
    std::memcpy(payload.data() + sizeof(uint32_t), &health.leadership_score, sizeof(float));
    broadcast(SwarmMessage::Type::LEADER_ELECT, payload);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    uint32_t best_id = local_id_;
    float best_score = health.leadership_score;
    float best_battery = health.battery_pct;
    {
        std::lock_guard lock(peers_mutex_);
        for (const auto& [id, peer] : peers_) {
            if (!peer.reachable) continue;

            const bool better_score = peer.health.leadership_score > best_score;
            const bool better_battery = std::abs(peer.health.leadership_score - best_score) < 1e-4f
                && peer.health.battery_pct > best_battery;
            const bool tie_break = std::abs(peer.health.leadership_score - best_score) < 1e-4f
                && std::abs(peer.health.battery_pct - best_battery) < 1e-4f
                && id > best_id;
            if (better_score || better_battery || tie_break) {
                best_id = id;
                best_score = peer.health.leadership_score;
                best_battery = peer.health.battery_pct;
            }
        }
    }

    if (best_id == local_id_) {
        role_.store(DroneRole::LEADER);
        leader_id_.store(local_id_);
        logger_->info("V2X: node {} elected LEADER with score {:.3f}", local_id_, best_score);
    } else {
        role_.store(DroneRole::FOLLOWER);
        leader_id_.store(best_id);
        logger_->info("V2X: node {} is FOLLOWER (leader={} score {:.3f})",
                      local_id_, best_id, best_score);
    }
    election_ongoing_.store(false);
}

void V2XMeshNetwork::handle_election(const SwarmMessage& msg) {
    if (msg.payload.size() < sizeof(uint32_t)) return;
    uint32_t candidate_id;
    std::memcpy(&candidate_id, msg.payload.data(), sizeof(uint32_t));

    float candidate_score = 0.0f;
    if (msg.payload.size() >= sizeof(uint32_t) + sizeof(float)) {
        std::memcpy(&candidate_score, msg.payload.data() + sizeof(uint32_t), sizeof(float));
    }

    {
        std::lock_guard lock(peers_mutex_);
        auto& peer = peers_[candidate_id];
        peer.id = candidate_id;
        peer.reachable = true;
        peer.last_seen_ts = msg.timestamp;
        peer.health.leadership_score = candidate_score;
    }

    const auto local = local_health();
    const bool remote_wins = (candidate_score > local.leadership_score)
        || (std::abs(candidate_score - local.leadership_score) < 1e-4f
            && candidate_id > local_id_);
    if (remote_wins && role_.load() == DroneRole::LEADER) {
        role_.store(DroneRole::FOLLOWER);
        leader_id_.store(candidate_id);
        logger_->info("V2X: stepping down, new leader={} score={:.3f}",
                      candidate_id, candidate_score);
    }
}

void V2XMeshNetwork::handle_pose_update(const SwarmMessage& msg) {
    if (msg.payload.size() < 24) return;
    Eigen::Vector3d pos;
    std::memcpy(pos.data(), msg.payload.data(), 24);

    std::lock_guard lock(peers_mutex_);
    auto& peer    = peers_[msg.src_id];
    peer.id       = msg.src_id;
    peer.position = pos;
    if (msg.payload.size() >= 48) {
        std::memcpy(peer.velocity.data(), msg.payload.data() + 24, 24);
    }
    peer.last_seen_ts = msg.timestamp;
}

// peer liveness and leader-loss detection
void V2XMeshNetwork::expire_stale_peers() {
    bool leader_lost = false;
    {
        std::lock_guard lock(peers_mutex_);
        for (auto& [id, peer] : peers_) {
            if (peer.is_stale(kPeerTimeout_s)) {
                peer.reachable = false;
                logger_->warn("V2X: peer {} marked unreachable (last seen {:.1f}s ago)",
                              id, std::chrono::duration<double>(
                                  std::chrono::steady_clock::now().time_since_epoch()).count()
                              - peer.last_seen_ts);
                if (leader_id_.load() == id) {
                    leader_lost = true;
                }
            }
        }
    }

    if (leader_lost) {
        leader_id_.store(0);
        role_.store(DroneRole::CANDIDATE);
        logger_->warn("V2X: current leader lost, triggering decentralized re-election");
        trigger_election();
    }
}

// active peer snapshot
std::vector<PeerInfo> V2XMeshNetwork::active_peers() const {
    std::lock_guard lock(peers_mutex_);
    std::vector<PeerInfo> out;
    out.reserve(peers_.size());
    for (const auto& [id, p] : peers_)
        if (p.reachable) out.push_back(p);
    return out;
}

size_t V2XMeshNetwork::peer_count() const {
    std::lock_guard lock(peers_mutex_);
    return std::count_if(peers_.begin(), peers_.end(),
        [](const auto& kv) { return kv.second.reachable; });
}

std::vector<uint8_t> V2XMeshNetwork::build_heartbeat_payload() const {
    const auto health = local_health();
    std::vector<uint8_t> p(kHeartbeatPayloadSize);
    p[0] = static_cast<uint8_t>(role_.load());
    size_t offset = 1;
    auto write_metric = [&](float value) {
        std::memcpy(p.data() + offset, &value, sizeof(float));
        offset += sizeof(float);
    };
    write_metric(health.battery_pct);
    write_metric(health.motor_health);
    write_metric(health.link_quality);
    write_metric(health.cpu_headroom);
    write_metric(health.thermal_headroom);
    write_metric(health.leadership_score);
    p[offset] = static_cast<uint8_t>(health.emergency_fault ? 1 : 0);
    return p;
}
void V2XMeshNetwork::configure_security(SwarmSecurityConfig cfg) {
    security_ = std::make_unique<SwarmSecurityContext>(local_id_, std::move(cfg));
}
bool V2XMeshNetwork::security_enabled() const {
    return security_ && security_->enabled();
}
std::string V2XMeshNetwork::security_last_error() const {
    return security_ ? security_->last_error() : std::string{};
}
void V2XMeshNetwork::set_local_health(SwarmHealthMetrics health) {
    health.battery_pct = std::clamp(health.battery_pct, 0.0f, 100.0f);
    health.motor_health = std::clamp(health.motor_health, 0.0f, 1.0f);
    health.link_quality = std::clamp(health.link_quality, 0.0f, 1.0f);
    health.cpu_headroom = std::clamp(health.cpu_headroom, 0.0f, 1.0f);
    health.thermal_headroom = std::clamp(health.thermal_headroom, 0.0f, 1.0f);
    health.leadership_score = compute_leadership_score(health);
    std::lock_guard lock(health_mutex_);
    local_health_ = health;
}
SwarmHealthMetrics V2XMeshNetwork::local_health() const {
    std::lock_guard lock(health_mutex_);
    return local_health_;
}
float V2XMeshNetwork::compute_leadership_score(const SwarmHealthMetrics& health) {
    if (health.emergency_fault) {
        return 0.0f;
    }
    const float battery_norm = std::clamp(health.battery_pct / 100.0f, 0.0f, 1.0f);
    const float score =
        (0.32f * battery_norm) +
        (0.28f * std::clamp(health.motor_health, 0.0f, 1.0f)) +
        (0.18f * std::clamp(health.link_quality, 0.0f, 1.0f)) +
        (0.12f * std::clamp(health.cpu_headroom, 0.0f, 1.0f)) +
        (0.10f * std::clamp(health.thermal_headroom, 0.0f, 1.0f));
    return std::clamp(score, 0.0f, 1.0f);
}
bool V2XMeshNetwork::should_force_re_election(const PeerInfo& peer) const {
    return peer.health.emergency_fault
        || peer.health.motor_health < 0.35f
        || peer.health.link_quality < 0.25f
        || peer.health.battery_pct < 18.0f
        || peer.health.leadership_score < 0.40f;
}
// -----------------------------------------------------------------------------
// LeaderFollowerController
 
Eigen::Vector3d LeaderFollowerController::compute_target(
        const Eigen::Vector3d& leader_pos,
        const FormationCommand& cmd,
        uint32_t follower_index) const {

    const float s = cmd.spacing_m;
    Eigen::Vector3d offset;

    switch (cmd.shape) {
    case FormationCommand::Formation::DIAMOND: {
        // 4-drone diamond: [right, left, back-right, back-left]
        static const std::array<Eigen::Vector3f, 4> diamond = {{
            { s,  0,    0},
            {-s,  0,    0},
            { s*0.7f, -s, 0},
            {-s*0.7f, -s, 0},
        }};
        const auto& o = diamond[follower_index % 4];
        offset = Eigen::Vector3d(o.x(), o.y(), o.z());
        break;
    }
    case FormationCommand::Formation::VEE: {
        const float angle = static_cast<float>(30.0 * kDegToRad);
        const int   side  = (follower_index % 2 == 0) ? 1 : -1;
        const float row   = static_cast<float>((follower_index / 2) + 1);
        offset = Eigen::Vector3d(
            side * row * s * std::sin(angle),
            -row * s * std::cos(angle),
            0.0);
        break;
    }
    case FormationCommand::Formation::LINE: {
        offset = Eigen::Vector3d(0, -(follower_index + 1) * s, 0);
        break;
    }
    default:
        offset = Eigen::Vector3d::Zero();
    }

    return leader_pos + offset;
}

Eigen::Vector3d LeaderFollowerController::velocity_command(
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& current_velocity,
        const Eigen::Vector3d& target_pos,
        float kp, float max_speed_mps) const {
    const auto peers = net_ ? net_->active_peers() : std::vector<PeerInfo>{};
    return velocity_command(current_pos, current_velocity, target_pos, peers, {}, kp, max_speed_mps);
}

Eigen::Vector3d LeaderFollowerController::velocity_command(
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& current_velocity,
        const Eigen::Vector3d& target_pos,
        const std::vector<PeerInfo>& peers,
        const std::vector<AvoidanceObstacle>& obstacles,
        float kp, float max_speed_mps) const {
    Eigen::Vector3d error = target_pos - current_pos;
    Eigen::Vector3d preferred_vel = kp * error;

    // Blend goal tracking with VO-aware peer/static obstacle avoidance.
    Eigen::Vector3d vel = preferred_vel
        + compute_avoidance_velocity(current_pos, current_velocity, peers, obstacles);

    // Escape local minima by injecting a small tangential move when the
    // desired tracking velocity and avoidance field nearly cancel out.
    if (preferred_vel.norm() > 0.5 && vel.norm() < 0.25) {
        vel += deterministic_lateral_axis(preferred_vel) * kDeadlockEscapeSpeed;
    }

    // Clamp to max speed
    const double speed = vel.norm();
    if (speed > max_speed_mps)
        vel = vel * (max_speed_mps / speed);

    return vel;
}

Eigen::Vector3d LeaderFollowerController::compute_avoidance_velocity(
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& current_velocity,
        const std::vector<PeerInfo>& peers,
        const std::vector<AvoidanceObstacle>& obstacles,
        float min_separation_m,
        float influence_radius_m,
        float max_avoid_speed_mps,
        float prediction_horizon_s) const {
    if (peers.empty() && obstacles.empty()) {
        return Eigen::Vector3d::Zero();
    }
    if (influence_radius_m <= min_separation_m) {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d repulsion = Eigen::Vector3d::Zero();
    double strongest_hazard = 0.0;
    const double separation_band = std::max(
        static_cast<double>(influence_radius_m - min_separation_m),
        kAvoidanceEps);

    for (const auto& peer : peers) {
        if (!peer.reachable || peer.id == id_) {
            continue;
        }

        Eigen::Vector3d delta = current_pos - peer.position;
        double distance = delta.norm();
        if (distance > influence_radius_m) {
            continue;
        }

        if (distance < kAvoidanceEps) {
            delta = deterministic_separation_axis(id_, peer.id);
            distance = delta.norm();
        }

        Eigen::Vector3d direction = delta / std::max(distance, kAvoidanceEps);
        direction.z() *= 0.35;
        if (direction.norm() > kAvoidanceEps) {
            direction.normalize();
        }

        const double normalized_gap = std::clamp(
            (static_cast<double>(influence_radius_m) - distance) / separation_band,
            0.0, 1.0);

        double weight = normalized_gap * normalized_gap;
        if (distance < min_separation_m) {
            weight += 1.0 + ((static_cast<double>(min_separation_m) - distance)
                / std::max(static_cast<double>(min_separation_m), kAvoidanceEps));
        }

        const Eigen::Vector3d relative_velocity = peer.velocity - current_velocity;
        const double closing_speed = closing_speed_along(delta, relative_velocity);
        if (closing_speed > 0.0) {
            const double time_to_collision = distance / std::max(closing_speed, kAvoidanceEps);
            if (time_to_collision < prediction_horizon_s) {
                weight += (prediction_horizon_s - time_to_collision) / std::max(prediction_horizon_s, 0.1f);
            }
        }

        repulsion += direction * weight;
        strongest_hazard = std::max(strongest_hazard, weight);
    }

    for (const auto& obstacle : obstacles) {
        Eigen::Vector3d delta = current_pos - obstacle.position;
        double distance = delta.norm() - obstacle.radius_m;
        if (distance > influence_radius_m) {
            continue;
        }

        if (distance < kAvoidanceEps) {
            delta = deterministic_separation_axis(id_, static_cast<uint32_t>(obstacle.radius_m * 1000.0f + 1.0f));
            distance = delta.norm();
        }

        Eigen::Vector3d direction = delta / std::max(delta.norm(), kAvoidanceEps);
        direction.z() *= obstacle.dynamic ? 0.45 : 0.25;
        if (direction.norm() > kAvoidanceEps) {
            direction.normalize();
        }

        const double normalized_gap = std::clamp(
            (static_cast<double>(influence_radius_m) - distance) / separation_band,
            0.0, 1.0);

        double weight = 0.8 * normalized_gap * normalized_gap;
        if (distance < min_separation_m) {
            weight += 1.1 + ((static_cast<double>(min_separation_m) - distance)
                / std::max(static_cast<double>(min_separation_m), kAvoidanceEps));
        }

        const Eigen::Vector3d relative_velocity = obstacle.velocity - current_velocity;
        const double closing_speed = closing_speed_along(delta, relative_velocity);
        if (obstacle.dynamic && closing_speed > 0.0) {
            const double time_to_collision = std::max(distance, 0.0) / std::max(closing_speed, kAvoidanceEps);
            if (time_to_collision < prediction_horizon_s) {
                weight += 0.75 * (prediction_horizon_s - time_to_collision)
                    / std::max(prediction_horizon_s, 0.1f);
            }
        }

        repulsion += direction * weight;
        strongest_hazard = std::max(strongest_hazard, weight);
    }

    if (repulsion.norm() < kAvoidanceEps) {
        return Eigen::Vector3d::Zero();
    }

    repulsion.normalize();
    repulsion *= std::min<double>(max_avoid_speed_mps, max_avoid_speed_mps * strongest_hazard);
    return repulsion;
}

std::vector<AvoidanceObstacle> LeaderFollowerController::obstacles_from_lidar(
        const sensors::LidarMeasurement& scan,
        const Eigen::Vector3d& drone_position,
        size_t stride,
        float obstacle_radius_m) {
    std::vector<AvoidanceObstacle> out;
    if (!scan.cloud || scan.cloud->empty()) {
        return out;
    }

    const size_t step = std::max<size_t>(1, stride);
    out.reserve((scan.cloud->size() / step) + 1);

    for (size_t i = 0; i < scan.cloud->size(); i += step) {
        const auto& pt = scan.cloud->points[i];
        AvoidanceObstacle obstacle;
        obstacle.position = drone_position + Eigen::Vector3d(pt.x, pt.y, pt.z);
        obstacle.radius_m = obstacle_radius_m;
        obstacle.dynamic = false;
        out.push_back(obstacle);
    }

    return out;
}

} // namespace drone::swarm


// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
