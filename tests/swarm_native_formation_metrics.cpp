// Native closed-loop formation replay using the project's existing C++ controller.
// Synthetic trajectories only; NOT a UAV flight test or a collision guarantee.
#include "swarm/V2XMeshNetwork.hpp"

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using drone::swarm::FormationCommand;
using drone::swarm::LeaderFollowerController;
using drone::swarm::PeerInfo;

namespace {

struct Result {
    double rmse_m{0.0};
    double minimum_pair_separation_m{std::numeric_limits<double>::infinity()};
    int separation_pair_ticks_under_1m{0};
    double minimum_swept_separation_m{std::numeric_limits<double>::infinity()};
    int swept_pair_ticks_under_1m{0};
    int observations{0};
};

Result replay(bool peer_avoidance) {
    constexpr double dt = 0.1;
    constexpr int steps = 200;
    constexpr size_t count = 4;
    std::array<Eigen::Vector3d, count> pos{
        Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(5.0, 2.0, 0.0),
        Eigen::Vector3d(-5.0, -2.0, 0.0), Eigen::Vector3d(3.0, -6.0, 0.0)};
    std::array<Eigen::Vector3d, count> velocities{};
    for (auto& velocity : velocities) {
        velocity.setZero();
    }
    FormationCommand command;
    command.shape = FormationCommand::Formation::DIAMOND;
    command.spacing_m = 3.0f;
    LeaderFollowerController controller1(1, {});
    LeaderFollowerController controller2(2, {});
    LeaderFollowerController controller3(3, {});
    const std::array<LeaderFollowerController*, 3> controllers{
        &controller1, &controller2, &controller3};

    Result result;
    double squared_error_sum = 0.0;
    for (int step = 0; step < steps; ++step) {
        const double sim_time = step * dt;
        // The leader follows exactly the same synthetic reference in both arms.
        pos[0] = Eigen::Vector3d(0.4 * sim_time, 0.4 * std::sin(sim_time / 3.0), 0.0);
        std::array<Eigen::Vector3d, count> next_velocities = velocities;
        std::array<Eigen::Vector3d, count> next_positions = pos;
        for (size_t i = 1; i < count; ++i) {
            const auto target = controllers[i - 1]->compute_target(pos[0], command, i - 1);
            std::vector<PeerInfo> peers;
            if (peer_avoidance) {
                for (size_t j = 1; j < count; ++j) {
                    if (i == j) {
                        continue;
                    }
                    // Simulated loss of awareness of peer 1 after t=10 seconds.
                    if (sim_time >= 10.0 && j == 1) {
                        continue;
                    }
                    PeerInfo peer;
                    peer.id = static_cast<uint32_t>(j);
                    peer.reachable = true;
                    peer.position = pos[j];
                    peer.velocity = velocities[j];
                    peers.push_back(peer);
                }
            }
            const auto velocity = controllers[i - 1]->velocity_command(
                pos[i], velocities[i], target, peers, {}, 1.3f, 2.0f);
            if (!velocity.allFinite() || velocity.norm() > 2.00001) {
                throw std::runtime_error("non-finite or unbounded C++ controller command");
            }
            next_velocities[i] = velocity;
            next_positions[i] = pos[i] + velocity * dt;
            squared_error_sum += (next_positions[i] - target).squaredNorm();
            ++result.observations;
        }
        // Exact closest approach for linearly interpolated relative motion
        // within this discrete-time step; endpoint-only tests miss crossings.
        for (size_t i = 1; i < count; ++i) {
            for (size_t j = i + 1; j < count; ++j) {
                const Eigen::Vector3d relative_start = pos[i] - pos[j];
                const Eigen::Vector3d relative_delta =
                    (next_positions[i] - pos[i]) - (next_positions[j] - pos[j]);
                const double denom = relative_delta.squaredNorm();
                const double alpha =
                    denom > 1e-12
                        ? std::clamp(-relative_start.dot(relative_delta) / denom, 0.0, 1.0)
                        : 0.0;
                const double gap = (relative_start + alpha * relative_delta).norm();
                result.minimum_swept_separation_m =
                    std::min(result.minimum_swept_separation_m, gap);
                if (gap < 1.0) {
                    ++result.swept_pair_ticks_under_1m;
                }
            }
        }
        pos = next_positions;
        velocities = next_velocities;
        for (size_t i = 1; i < count; ++i) {
            for (size_t j = i + 1; j < count; ++j) {
                const double separation = (pos[i] - pos[j]).norm();
                result.minimum_pair_separation_m =
                    std::min(result.minimum_pair_separation_m, separation);
                if (separation < 1.0) {
                    ++result.separation_pair_ticks_under_1m;
                }
            }
        }
    }
    result.rmse_m = std::sqrt(squared_error_sum / result.observations);
    return result;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto no_avoidance = replay(false);
        const auto peer_avoidance = replay(true);
        const std::string json = [&]() {
            std::ostringstream output;
            output << std::fixed << std::setprecision(6)
                   << "{\n  \"scope\": \"synthetic_native_cpp_closed_loop_replay\",\n"
                   << "  \"radio_or_physical_flight\": false,\n"
                   << "  \"dt_s\": 0.1,\n"
                   << "  \"steps\": 200,\n"
                   << "  \"without_peer_avoidance\": {\n"
                   << "    \"formation_rmse_m\": " << no_avoidance.rmse_m << ",\n"
                   << "    \"min_pair_separation_m\": " << no_avoidance.minimum_pair_separation_m << ",\n"
                   << "    \"pair_ticks_under_1m\": "
                   << no_avoidance.separation_pair_ticks_under_1m << ",\n"
                   << "    \"min_swept_separation_m\": "
                   << no_avoidance.minimum_swept_separation_m << ",\n"
                   << "    \"swept_pair_ticks_under_1m\": "
                   << no_avoidance.swept_pair_ticks_under_1m << "\n  },\n"
                   << "  \"with_peer_avoidance\": {\n"
                   << "    \"formation_rmse_m\": " << peer_avoidance.rmse_m << ",\n"
                   << "    \"min_pair_separation_m\": " << peer_avoidance.minimum_pair_separation_m << ",\n"
                   << "    \"pair_ticks_under_1m\": "
                   << peer_avoidance.separation_pair_ticks_under_1m << ",\n"
                   << "    \"min_swept_separation_m\": "
                   << peer_avoidance.minimum_swept_separation_m << ",\n"
                   << "    \"swept_pair_ticks_under_1m\": "
                   << peer_avoidance.swept_pair_ticks_under_1m << "\n  }\n}\n";
            return output.str();
        }();
        if (argc > 1) {
            std::ofstream file(argv[1]);
            if (!file) {
                throw std::runtime_error("could not write native experiment artifact");
            }
            file << json;
        }
        std::cout << json;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
