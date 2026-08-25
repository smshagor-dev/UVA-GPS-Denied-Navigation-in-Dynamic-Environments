#pragma once

#include "vio/EKFEstimator.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone::vio {

struct VisualFeatureTrackConfig {
    double association_radius_px{3.0};
    double minimum_baseline_m{0.05};
    double minimum_parallax_deg{1.0};
    double maximum_ray_gap_m{0.25};
    double minimum_depth_m{0.10};
    double maximum_depth_m{80.0};
    std::size_t maximum_tracks{300};
};

struct TrackedVisualFeature {
    uint64_t track_id{0};
    Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
    Eigen::Vector3d world_point{Eigen::Vector3d::Zero()};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct VisualFeatureTrackBatch {
    std::vector<TrackedVisualFeature, Eigen::aligned_allocator<TrackedVisualFeature>> features{};
    uint64_t carried_tracks{0};
    uint64_t new_tracks{0};
    uint64_t initialized_tracks{0};
    uint64_t rejected_geometry{0};
};

class VisualFeatureTrackManager {
public:
    explicit VisualFeatureTrackManager(VisualFeatureTrackConfig config = {}) : config_(config) {}

    void reset() {
        tracks_.clear();
        next_track_id_ = 1;
    }

    [[nodiscard]] std::size_t track_count() const {
        return tracks_.size();
    }

    [[nodiscard]] uint64_t next_track_id() const {
        return next_track_id_;
    }

    [[nodiscard]] VisualFeatureTrackBatch update(
        const std::vector<Eigen::Vector2d>& previous_pixels,
        const std::vector<Eigen::Vector2d>& current_pixels,
        const PoseEstimate& previous_pose,
        const PoseEstimate& current_pose,
        const Eigen::Matrix3d& K) {
        VisualFeatureTrackBatch batch;
        if (!inputs_valid(previous_pixels, current_pixels, previous_pose, current_pose, K)) {
            tracks_.clear();
            return batch;
        }

        const std::size_t count = previous_pixels.size();
        std::vector<uint64_t> existing_ids;
        existing_ids.reserve(tracks_.size());
        for (const auto& [id, track] : tracks_) {
            (void)track;
            existing_ids.push_back(id);
        }
        std::sort(existing_ids.begin(), existing_ids.end());

        std::unordered_set<uint64_t> claimed;
        std::unordered_map<uint64_t, TrackState> next_tracks;
        next_tracks.reserve(std::min(count, config_.maximum_tracks));

        for (std::size_t i = 0; i < count && next_tracks.size() < config_.maximum_tracks; ++i) {
            if (!previous_pixels[i].array().isFinite().all() ||
                !current_pixels[i].array().isFinite().all()) {
                ++batch.rejected_geometry;
                continue;
            }

            const auto matched_id = match_track(previous_pixels[i], existing_ids, claimed);
            TrackState state;
            if (matched_id.has_value()) {
                state = tracks_.at(*matched_id);
                claimed.insert(*matched_id);
                ++batch.carried_tracks;
            } else {
                state.track_id = next_track_id_++;
                ++batch.new_tracks;
            }

            state.last_pixel = current_pixels[i];
            if (!state.world_point.has_value()) {
                const auto point = triangulate_two_view(previous_pixels[i], current_pixels[i],
                                                        previous_pose, current_pose, K);
                if (point.has_value()) {
                    state.world_point = *point;
                    ++batch.initialized_tracks;
                } else {
                    ++batch.rejected_geometry;
                }
            }

            if (state.world_point.has_value()) {
                batch.features.push_back(
                    TrackedVisualFeature{state.track_id, current_pixels[i], *state.world_point});
            }
            next_tracks.emplace(state.track_id, std::move(state));
        }

        tracks_ = std::move(next_tracks);
        std::sort(batch.features.begin(), batch.features.end(),
                  [](const TrackedVisualFeature& lhs, const TrackedVisualFeature& rhs) {
                      return lhs.track_id < rhs.track_id;
                  });
        return batch;
    }

private:
    struct TrackState {
        uint64_t track_id{0};
        Eigen::Vector2d last_pixel{Eigen::Vector2d::Zero()};
        std::optional<Eigen::Vector3d> world_point{};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    [[nodiscard]] bool inputs_valid(const std::vector<Eigen::Vector2d>& previous_pixels,
                                    const std::vector<Eigen::Vector2d>& current_pixels,
                                    const PoseEstimate& previous_pose,
                                    const PoseEstimate& current_pose,
                                    const Eigen::Matrix3d& K) const {
        if (previous_pixels.empty() || previous_pixels.size() != current_pixels.size()) {
            return false;
        }
        if (config_.association_radius_px <= 0.0 || config_.minimum_baseline_m < 0.0 ||
            config_.minimum_parallax_deg < 0.0 || config_.maximum_ray_gap_m <= 0.0 ||
            config_.minimum_depth_m <= 0.0 ||
            config_.maximum_depth_m <= config_.minimum_depth_m || config_.maximum_tracks == 0) {
            return false;
        }
        return previous_pose.position.array().isFinite().all() &&
               current_pose.position.array().isFinite().all() &&
               previous_pose.orientation.coeffs().array().isFinite().all() &&
               current_pose.orientation.coeffs().array().isFinite().all() &&
               K.array().isFinite().all() && std::abs(K.determinant()) > 1.0e-12;
    }

    [[nodiscard]] std::optional<uint64_t>
    match_track(const Eigen::Vector2d& previous_pixel, const std::vector<uint64_t>& ordered_ids,
                const std::unordered_set<uint64_t>& claimed) const {
        const double max_distance_sq = config_.association_radius_px * config_.association_radius_px;
        double best_distance_sq = max_distance_sq;
        std::optional<uint64_t> best_id;
        for (const uint64_t id : ordered_ids) {
            if (claimed.contains(id)) {
                continue;
            }
            const auto it = tracks_.find(id);
            if (it == tracks_.end()) {
                continue;
            }
            const double distance_sq = (it->second.last_pixel - previous_pixel).squaredNorm();
            if (distance_sq < best_distance_sq ||
                (std::abs(distance_sq - best_distance_sq) <= 1.0e-12 &&
                 (!best_id.has_value() || id < *best_id))) {
                best_distance_sq = distance_sq;
                best_id = id;
            }
        }
        return best_id;
    }

    [[nodiscard]] std::optional<Eigen::Vector3d>
    triangulate_two_view(const Eigen::Vector2d& previous_pixel,
                         const Eigen::Vector2d& current_pixel, const PoseEstimate& previous_pose,
                         const PoseEstimate& current_pose, const Eigen::Matrix3d& K) const {
        const Eigen::Vector3d baseline = current_pose.position - previous_pose.position;
        if (!baseline.array().isFinite().all() || baseline.norm() < config_.minimum_baseline_m) {
            return std::nullopt;
        }

        const Eigen::Matrix3d K_inv = K.inverse();
        Eigen::Vector3d ray_previous = previous_pose.R_wb() * K_inv *
                                       Eigen::Vector3d{previous_pixel.x(), previous_pixel.y(), 1.0};
        Eigen::Vector3d ray_current = current_pose.R_wb() * K_inv *
                                      Eigen::Vector3d{current_pixel.x(), current_pixel.y(), 1.0};
        if (!ray_previous.array().isFinite().all() || !ray_current.array().isFinite().all() ||
            ray_previous.norm() <= 1.0e-12 || ray_current.norm() <= 1.0e-12) {
            return std::nullopt;
        }
        ray_previous.normalize();
        ray_current.normalize();

        const double dot = std::clamp(ray_previous.dot(ray_current), -1.0, 1.0);
        const double parallax_deg = std::acos(dot) * (180.0 / 3.14159265358979323846);
        if (!std::isfinite(parallax_deg) || parallax_deg < config_.minimum_parallax_deg) {
            return std::nullopt;
        }

        Eigen::Matrix<double, 3, 2> A;
        A.col(0) = ray_previous;
        A.col(1) = -ray_current;
        const Eigen::Vector2d depths =
            A.colPivHouseholderQr().solve(current_pose.position - previous_pose.position);
        if (!depths.array().isFinite().all() || depths.x() < config_.minimum_depth_m ||
            depths.y() < config_.minimum_depth_m || depths.x() > config_.maximum_depth_m ||
            depths.y() > config_.maximum_depth_m) {
            return std::nullopt;
        }

        const Eigen::Vector3d p_previous = previous_pose.position + depths.x() * ray_previous;
        const Eigen::Vector3d p_current = current_pose.position + depths.y() * ray_current;
        const double ray_gap = (p_previous - p_current).norm();
        if (!std::isfinite(ray_gap) || ray_gap > config_.maximum_ray_gap_m) {
            return std::nullopt;
        }

        const Eigen::Vector3d point = 0.5 * (p_previous + p_current);
        if (!point.array().isFinite().all()) {
            return std::nullopt;
        }
        return point;
    }

    VisualFeatureTrackConfig config_{};
    std::unordered_map<uint64_t, TrackState> tracks_{};
    uint64_t next_track_id_{1};
};

} // namespace drone::vio
