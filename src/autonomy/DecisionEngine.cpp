#include "autonomy/DecisionEngine.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <sstream>

namespace drone::autonomy {

namespace {

constexpr double kEps = 1e-6;

Eigen::Vector3d body_forward(const vio::PoseEstimate& pose) {
    return pose.R_wb() * Eigen::Vector3d{0.0, 0.0, 1.0};
}

Eigen::Vector3d body_right(const vio::PoseEstimate& pose) {
    return pose.R_wb() * Eigen::Vector3d{1.0, 0.0, 0.0};
}

Eigen::Vector3d body_up(const vio::PoseEstimate& pose) {
    return pose.R_wb() * Eigen::Vector3d{0.0, 1.0, 0.0};
}

Eigen::Vector3d clamp_speed(Eigen::Vector3d velocity, double max_speed) {
    const double speed = velocity.norm();
    if (speed > max_speed && speed > kEps) {
        velocity *= (max_speed / speed);
    }
    return velocity;
}

std::string lowercase(std::string_view value) {
    std::string out(value.begin(), value.end());
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

} // namespace

DecisionEngine::DecisionEngine(DecisionConfig cfg)
    : cfg_(cfg) {}

DecisionCommand DecisionEngine::update(const DecisionContext& ctx) {
    if (!home_initialized_) {
        home_position_ = ctx.pose.position;
        home_initialized_ = true;
    }

    if (ctx.system.battery_pct > 0.0f && ctx.system.battery_pct <= cfg_.critical_battery_pct) {
        mode_ = BehaviorMode::EMERGENCY_LAND;
        return build_emergency_land(ctx);
    }

    if (ctx.system.battery_pct > 0.0f && ctx.system.battery_pct <= cfg_.low_battery_pct) {
        mode_ = BehaviorMode::RETURN_HOME;
        return build_return_home(ctx);
    }

    if (!ctx.inference_ready || !ctx.frame.has_value()) {
        mode_ = ctx.swarm_follower ? BehaviorMode::HOLD_POSITION : BehaviorMode::SEARCH;
        return ctx.swarm_follower
            ? build_hold(ctx, "Vision AI unavailable, follower holding formation slot")
            : build_search(ctx);
    }

    const auto focus = select_primary_detection(*ctx.frame);
    if (!focus.has_value()) {
        ++target_miss_count_;
        if (ctx.swarm_follower) {
            mode_ = BehaviorMode::HOLD_POSITION;
            return build_hold(ctx, "No actionable detection, follower preserving swarm stability");
        }

        mode_ = BehaviorMode::SEARCH;
        return build_search(ctx);
    }

    target_miss_count_ = 0;

    if (is_hazard_label(focus->detection.label) || focus->score >= cfg_.critical_obstacle_score) {
        mode_ = BehaviorMode::AVOID_OBSTACLE;
        return build_avoid(ctx, *focus);
    }

    if (is_target_label(focus->detection.label) &&
        focus->detection.confidence >= cfg_.target_track_confidence) {
        mode_ = BehaviorMode::TRACK_TARGET;
        return build_track(ctx, *focus);
    }

    mode_ = ctx.swarm_follower ? BehaviorMode::HOLD_POSITION : BehaviorMode::SEARCH;
    return ctx.swarm_follower
        ? build_hold(ctx, "Detection seen but not mission-critical, follower yielding to swarm plan")
        : build_search(ctx);
}

void DecisionEngine::reset() {
    mode_ = BehaviorMode::HOLD_POSITION;
    home_position_.setZero();
    home_initialized_ = false;
    target_miss_count_ = 0;
}

std::optional<PerceptionFocus> DecisionEngine::select_primary_detection(
        const sensors::CameraFrame& frame) const {
    std::optional<PerceptionFocus> best;

    for (const auto& detection : frame.detections) {
        if (detection.confidence < cfg_.min_detection_confidence) {
            continue;
        }

        const float cx = detection.bbox.x + (detection.bbox.width * 0.5f);
        const float cy = detection.bbox.y + (detection.bbox.height * 0.5f);
        const Eigen::Vector2f offset{cx - 0.5f, cy - 0.5f};
        const float area = std::clamp(detection.bbox.width * detection.bbox.height, 0.0f, 1.0f);
        const float centered = std::clamp(1.0f - (offset.norm() * 1.4f), 0.0f, 1.0f);
        const float class_bias = is_hazard_label(detection.label)
            ? 1.25f
            : (is_target_label(detection.label) ? 1.0f : 0.65f);

        PerceptionFocus candidate;
        candidate.detection = detection;
        candidate.normalized_area = area;
        candidate.image_offset = offset;
        candidate.estimated_distance_m = 1.0f / std::sqrt(std::max(area, 1.0e-3f));
        candidate.score = detection.confidence * (0.45f + 0.35f * centered + 0.20f * area) * class_bias;

        if (!best.has_value() || candidate.score > best->score) {
            best = candidate;
        }
    }

    return best;
}

DecisionCommand DecisionEngine::build_emergency_land(const DecisionContext& ctx) const {
    DecisionCommand command;
    command.mode = BehaviorMode::EMERGENCY_LAND;
    command.requires_operator_attention = true;
    command.desired_velocity = -body_up(ctx.pose) * 0.8;
    command.summary = "Critical battery, descending for emergency landing";
    return command;
}

DecisionCommand DecisionEngine::build_return_home(const DecisionContext& ctx) const {
    DecisionCommand command;
    command.mode = BehaviorMode::RETURN_HOME;

    Eigen::Vector3d to_home = home_position_ - ctx.pose.position;
    to_home.z() = std::clamp(static_cast<double>(cfg_.nominal_altitude_m) - ctx.pose.position.z(),
                             -0.8, 0.8);

    if (to_home.norm() > kEps) {
        command.desired_velocity = clamp_speed(to_home.normalized() * cfg_.max_return_speed_mps,
                                               cfg_.max_return_speed_mps);
    }

    command.summary = "Battery low, returning to launch anchor";
    return command;
}

DecisionCommand DecisionEngine::build_hold(const DecisionContext& ctx, std::string summary) const {
    DecisionCommand command;
    command.mode = BehaviorMode::HOLD_POSITION;
    command.desired_velocity = -ctx.pose.velocity * 0.35;
    command.summary = std::move(summary);
    return command;
}

DecisionCommand DecisionEngine::build_search(const DecisionContext& ctx) const {
    DecisionCommand command;
    command.mode = BehaviorMode::SEARCH;

    const double altitude_error = static_cast<double>(cfg_.nominal_altitude_m) - ctx.pose.position.z();
    command.desired_velocity =
        body_forward(ctx.pose) * cfg_.max_search_speed_mps +
        body_up(ctx.pose) * std::clamp(altitude_error * 0.18, -0.5, 0.5);

    command.desired_velocity = clamp_speed(command.desired_velocity, cfg_.max_search_speed_mps);
    command.desired_yaw_rate_rads = (target_miss_count_ % 2 == 0) ? 0.18 : -0.18;
    command.summary = "No validated target, continuing autonomous search sweep";
    return command;
}

DecisionCommand DecisionEngine::build_track(const DecisionContext& ctx,
                                            const PerceptionFocus& focus) const {
    DecisionCommand command;
    command.mode = BehaviorMode::TRACK_TARGET;
    command.focus = focus;
    command.target_confidence = focus.detection.confidence;

    const float desired_area = 0.12f;
    const float area_error = desired_area - focus.normalized_area;

    Eigen::Vector3d velocity =
        body_forward(ctx.pose) * std::clamp(area_error * 8.0f, -0.3f, cfg_.max_track_speed_mps) +
        body_right(ctx.pose) * std::clamp(focus.image_offset.x() * 2.2f, -1.0f, 1.0f) +
        body_up(ctx.pose) * std::clamp(-focus.image_offset.y() * 1.6f, -0.8f, 0.8f);

    command.desired_velocity = clamp_speed(velocity, cfg_.max_track_speed_mps);
    command.desired_yaw_rate_rads = std::clamp(static_cast<double>(focus.image_offset.x()) * 1.8, -0.9, 0.9);

    std::ostringstream oss;
    oss << "Tracking " << focus.detection.label
        << " conf=" << focus.detection.confidence
        << " est_dist=" << focus.estimated_distance_m << "m";
    command.summary = oss.str();
    return command;
}

DecisionCommand DecisionEngine::build_avoid(const DecisionContext& ctx,
                                            const PerceptionFocus& focus) const {
    DecisionCommand command;
    command.mode = BehaviorMode::AVOID_OBSTACLE;
    command.focus = focus;
    command.target_confidence = focus.detection.confidence;

    const double altitude_error = static_cast<double>(cfg_.safe_altitude_m) - ctx.pose.position.z();
    Eigen::Vector3d velocity =
        -body_forward(ctx.pose) * (0.9 + focus.normalized_area * 2.0) +
        -body_right(ctx.pose) * std::clamp(static_cast<double>(focus.image_offset.x()) * 3.0, -1.2, 1.2) +
        body_up(ctx.pose) * std::clamp(altitude_error * 0.15, 0.0, 0.9);

    command.desired_velocity = clamp_speed(velocity, cfg_.max_avoid_speed_mps);
    command.desired_yaw_rate_rads = -std::clamp(static_cast<double>(focus.image_offset.x()) * 1.6, -1.0, 1.0);
    command.requires_operator_attention = focus.normalized_area > 0.25f;

    std::ostringstream oss;
    oss << "Avoiding " << focus.detection.label
        << " score=" << focus.score;
    command.summary = oss.str();
    return command;
}

bool DecisionEngine::is_hazard_label(std::string_view label) {
    static const std::array<std::string_view, 11> hazards = {{
        "person", "car", "truck", "bus", "motorcycle", "bicycle",
        "bird", "drone", "dog", "animal", "tree"
    }};
    const auto lowered = lowercase(label);
    return std::find(hazards.begin(), hazards.end(), lowered) != hazards.end();
}

bool DecisionEngine::is_target_label(std::string_view label) {
    static const std::array<std::string_view, 6> targets = {{
        "person", "drone", "vehicle", "car", "truck", "boat"
    }};
    const auto lowered = lowercase(label);
    return std::find(targets.begin(), targets.end(), lowered) != targets.end();
}

} // namespace drone::autonomy
