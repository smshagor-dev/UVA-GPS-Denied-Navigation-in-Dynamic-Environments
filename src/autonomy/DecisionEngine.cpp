// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "autonomy/DecisionEngine.hpp"
#include "utils/RuntimeLogging.hpp"

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

double caution_scale(const DecisionContext& ctx) {
    if (!ctx.memory_prior.has_value() || !ctx.memory_prior->recommend_caution) {
        return 1.0;
    }
    return std::clamp(1.0 - (ctx.memory_prior->risk_score * 0.18), 0.55, 0.90);
}

double localization_scale(const DecisionContext& ctx) {
    return std::clamp(ctx.localization_confidence, 0.35, 1.0);
}

} // namespace

DecisionEngine::DecisionEngine(DecisionConfig cfg)
    : cfg_(cfg) {
    drone::utils::get_or_create_logger("AI")->info(
        "DecisionEngine initialized min_conf={} track_conf={} low_bat={} critical_bat={}",
        cfg_.min_detection_confidence,
        cfg_.target_track_confidence,
        cfg_.low_battery_pct,
        cfg_.critical_battery_pct);
}

DecisionCommand DecisionEngine::update(const DecisionContext& ctx) {
    auto logger = drone::utils::get_or_create_logger("AI");
    logger->debug("DecisionEngine::update battery={} inference_ready={} follower={} detections={} peers={}",
                  ctx.system.battery_pct,
                  ctx.inference_ready,
                  ctx.swarm_follower,
                  ctx.frame.has_value() ? ctx.frame->detections.size() : 0,
                  ctx.swarm_peer_count);
    if (!home_initialized_) {
        home_position_ = ctx.pose.position;
        home_initialized_ = true;
        logger->info("DecisionEngine home anchor initialized [{:.2f},{:.2f},{:.2f}]",
                     home_position_.x(), home_position_.y(), home_position_.z());
    }

    if (ctx.system.battery_pct > 0.0f && ctx.system.battery_pct <= cfg_.critical_battery_pct) {
        mode_ = BehaviorMode::EMERGENCY_LAND;
        return build_emergency_land(ctx);
    }

    if (ctx.localization_lost || ctx.localization_confidence <= cfg_.lost_localization_threshold) {
        if (ctx.tdoa_position.has_value() &&
            ctx.tdoa_confidence >= cfg_.tdoa_recovery_confidence &&
            ctx.visible_anchor_count > 0) {
            mode_ = BehaviorMode::SAFE_RETURN_BY_ANCHOR;
            return build_safe_return_by_anchor(ctx);
        }

        if (!ctx.camera_tracking_nominal || ctx.sync_confidence < 0.45 || ctx.relocalization_count == 0) {
            mode_ = BehaviorMode::HOVER_AND_SCAN;
            return build_hover_and_scan(ctx);
        }

        mode_ = BehaviorMode::LOCALIZATION_LOST;
        return build_localization_lost(ctx);
    }

    if (ctx.system.battery_pct > 0.0f && ctx.system.battery_pct <= cfg_.low_battery_pct) {
        mode_ = BehaviorMode::RETURN_HOME;
        return build_return_home(ctx);
    }

    if (ctx.localization_degraded || ctx.localization_confidence <= cfg_.degraded_localization_threshold) {
        if (!ctx.camera_tracking_nominal || ctx.sync_confidence < 0.55) {
            mode_ = BehaviorMode::HOVER_AND_SCAN;
            return build_hover_and_scan(ctx);
        }
        mode_ = BehaviorMode::LOCALIZATION_DEGRADED;
        return build_localization_degraded(ctx);
    }

    if (ctx.memory_prior.has_value() &&
        ctx.memory_prior->recommend_caution &&
        ctx.swarm_follower &&
        (!ctx.frame.has_value() || ctx.frame->detections.empty())) {
        mode_ = BehaviorMode::HOLD_POSITION;
        return build_hold(ctx, "Fleet memory marked this sector as risky, follower holding slot");
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
    drone::utils::get_or_create_logger("AI")->info("DecisionEngine reset");
    mode_ = BehaviorMode::HOLD_POSITION;
    home_position_.setZero();
    home_initialized_ = false;
    target_miss_count_ = 0;
}

std::optional<PerceptionFocus> DecisionEngine::select_primary_detection(
        const sensors::CameraFrame& frame) const {
    auto logger = drone::utils::get_or_create_logger("AI");
    logger->debug("DecisionEngine::select_primary_detection detections={}", frame.detections.size());
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

    if (best.has_value()) {
        logger->debug("DecisionEngine focus label={} score={:.3f} area={:.3f}",
                      best->detection.label, best->score, best->normalized_area);
    } else {
        logger->debug("DecisionEngine no detection passed selection gate");
    }
    return best;
}

DecisionCommand DecisionEngine::build_emergency_land(const DecisionContext& ctx) const {
    drone::utils::get_or_create_logger("AI")->warn("DecisionEngine emergency land battery={}", ctx.system.battery_pct);
    DecisionCommand command;
    command.mode = BehaviorMode::EMERGENCY_LAND;
    command.requires_operator_attention = true;
    command.desired_velocity = -body_up(ctx.pose) * 0.8;
    command.summary = "Critical battery, descending for emergency landing";
    return command;
}

DecisionCommand DecisionEngine::build_return_home(const DecisionContext& ctx) const {
    drone::utils::get_or_create_logger("AI")->info("DecisionEngine return-home battery={} tdoa_conf={:.2f}",
                                                   ctx.system.battery_pct, ctx.tdoa_confidence);
    DecisionCommand command;
    command.mode = BehaviorMode::RETURN_HOME;

    Eigen::Vector3d to_home = home_position_ - ctx.pose.position;
    if (ctx.tdoa_position.has_value() && ctx.tdoa_confidence >= 0.45) {
        to_home = home_position_ - ((ctx.pose.position * 0.45) + (*ctx.tdoa_position * 0.55));
    }
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
    drone::utils::get_or_create_logger("AI")->debug("DecisionEngine hold velocity=[{:.2f},{:.2f},{:.2f}] reason={}",
                                                    ctx.pose.velocity.x(), ctx.pose.velocity.y(), ctx.pose.velocity.z(),
                                                    summary);
    DecisionCommand command;
    command.mode = BehaviorMode::HOLD_POSITION;
    command.desired_velocity = -ctx.pose.velocity * 0.35;
    command.summary = std::move(summary);
    return command;
}

DecisionCommand DecisionEngine::build_search(const DecisionContext& ctx) const {
    drone::utils::get_or_create_logger("AI")->debug("DecisionEngine search miss_count={} caution={}",
                                                    target_miss_count_, caution_scale(ctx));
    DecisionCommand command;
    command.mode = BehaviorMode::SEARCH;
    const double speed_scale = caution_scale(ctx) * localization_scale(ctx);

    const double altitude_error = static_cast<double>(cfg_.nominal_altitude_m) - ctx.pose.position.z();
    command.desired_velocity =
        body_forward(ctx.pose) * (cfg_.max_search_speed_mps * speed_scale) +
        body_up(ctx.pose) * std::clamp(altitude_error * 0.18, -0.5, 0.5);

    command.desired_velocity = clamp_speed(command.desired_velocity, cfg_.max_search_speed_mps * speed_scale);
    command.desired_yaw_rate_rads = (target_miss_count_ % 2 == 0) ? 0.18 : -0.18;
    command.summary = ctx.memory_prior.has_value() && ctx.memory_prior->recommend_caution
        ? "No validated target, searching cautiously using learned fleet risk prior"
        : "No validated target, continuing autonomous search sweep";
    return command;
}

DecisionCommand DecisionEngine::build_hover_and_scan(const DecisionContext& ctx) const {
    drone::utils::get_or_create_logger("AI")->warn(
        "DecisionEngine hover-scan loc_conf={:.2f} sync={:.2f} camera_nominal={}",
        ctx.localization_confidence,
        ctx.sync_confidence,
        ctx.camera_tracking_nominal);
    DecisionCommand command;
    command.mode = BehaviorMode::HOVER_AND_SCAN;
    command.requires_operator_attention = ctx.localization_confidence < 0.3;
    command.desired_velocity =
        (-ctx.pose.velocity * 0.55) +
        body_up(ctx.pose) * std::clamp(
            static_cast<double>(cfg_.safe_altitude_m - 1.0f) - ctx.pose.position.z(),
            -0.15,
            0.25);
    command.desired_velocity = clamp_speed(command.desired_velocity, cfg_.max_recovery_speed_mps * 0.65);
    command.desired_yaw_rate_rads = ctx.swarm_follower ? 0.08 : 0.22;
    command.summary = "Localization unstable, hovering and scanning for relocalization";
    return command;
}

DecisionCommand DecisionEngine::build_safe_return_by_anchor(const DecisionContext& ctx) const {
    drone::utils::get_or_create_logger("AI")->warn(
        "DecisionEngine safe-return anchor_count={} tdoa_conf={:.2f}",
        ctx.visible_anchor_count,
        ctx.tdoa_confidence);
    DecisionCommand command;
    command.mode = BehaviorMode::SAFE_RETURN_BY_ANCHOR;
    command.requires_operator_attention = true;

    Eigen::Vector3d recovery = *ctx.tdoa_position - ctx.pose.position;
    recovery.z() = std::clamp(
        static_cast<double>(cfg_.safe_altitude_m) - ctx.pose.position.z(),
        -0.25,
        0.45);
    command.desired_velocity = clamp_speed(recovery, cfg_.max_recovery_speed_mps);
    command.summary = "Localization lost, returning via visible anchor geometry";
    return command;
}

DecisionCommand DecisionEngine::build_localization_degraded(const DecisionContext& ctx) const {
    drone::utils::get_or_create_logger("AI")->warn(
        "DecisionEngine degraded localization conf={:.2f} source={}",
        ctx.localization_confidence,
        ctx.localization_source);
    DecisionCommand command;
    command.mode = BehaviorMode::LOCALIZATION_DEGRADED;
    command.requires_operator_attention = ctx.localization_confidence < 0.45;

    const double altitude_error = static_cast<double>(cfg_.nominal_altitude_m) - ctx.pose.position.z();
    command.desired_velocity =
        (-ctx.pose.velocity * 0.45) +
        body_up(ctx.pose) * std::clamp(altitude_error * 0.15, -0.25, 0.25);
    command.desired_velocity = clamp_speed(command.desired_velocity, cfg_.max_recovery_speed_mps);
    command.desired_yaw_rate_rads = ctx.swarm_follower ? 0.0 : 0.12;
    command.summary = ctx.tdoa_confidence < 0.2
        ? "Localization degraded, cautious dead-reckoning while anchor aid is unavailable"
        : "Localization degraded, slowing down and stabilizing for recovery using " + ctx.localization_source;
    return command;
}

DecisionCommand DecisionEngine::build_localization_lost(const DecisionContext& ctx) const {
    drone::utils::get_or_create_logger("AI")->error(
        "DecisionEngine localization lost conf={:.2f} tdoa_conf={:.2f}",
        ctx.localization_confidence,
        ctx.tdoa_confidence);
    DecisionCommand command;
    command.mode = BehaviorMode::LOCALIZATION_LOST;
    command.requires_operator_attention = true;

    if (ctx.tdoa_position.has_value() && ctx.tdoa_confidence >= cfg_.tdoa_recovery_confidence) {
        Eigen::Vector3d recovery = *ctx.tdoa_position - ctx.pose.position;
        recovery.z() = std::clamp(
            static_cast<double>(cfg_.safe_altitude_m) - ctx.pose.position.z(),
            -0.3,
            0.5);
        command.desired_velocity = clamp_speed(recovery, cfg_.max_recovery_speed_mps);
        command.summary = "Localization lost, using TDOA recovery anchor";
        return command;
    }

    command.desired_velocity =
        (-ctx.pose.velocity * 0.6) +
        body_up(ctx.pose) * std::clamp(
            static_cast<double>(cfg_.safe_altitude_m - 1.5f) - ctx.pose.position.z(),
            -0.2,
            0.35);
    command.desired_velocity = clamp_speed(command.desired_velocity, cfg_.max_recovery_speed_mps * 0.8);
    command.summary = "Localization lost, holding for relocalization and operator intervention";
    return command;
}

DecisionCommand DecisionEngine::build_track(const DecisionContext& ctx,
                                            const PerceptionFocus& focus) const {
    drone::utils::get_or_create_logger("AI")->info("DecisionEngine track label={} conf={:.3f} area={:.3f}",
                                                   focus.detection.label,
                                                   focus.detection.confidence,
                                                   focus.normalized_area);
    DecisionCommand command;
    command.mode = BehaviorMode::TRACK_TARGET;
    command.focus = focus;
    command.target_confidence = focus.detection.confidence;

    const float desired_area = 0.12f;
    const float area_error = desired_area - focus.normalized_area;
    const double speed_scale = caution_scale(ctx) * localization_scale(ctx);

    Eigen::Vector3d velocity =
        body_forward(ctx.pose) * std::clamp(area_error * 8.0f,
                                            -0.3f,
                                            cfg_.max_track_speed_mps * static_cast<float>(speed_scale)) +
        body_right(ctx.pose) * std::clamp(focus.image_offset.x() * 2.2f, -1.0f, 1.0f) +
        body_up(ctx.pose) * std::clamp(-focus.image_offset.y() * 1.6f, -0.8f, 0.8f);

    command.desired_velocity = clamp_speed(velocity, cfg_.max_track_speed_mps * speed_scale);
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
    drone::utils::get_or_create_logger("AI")->warn("DecisionEngine avoid label={} score={:.3f} area={:.3f}",
                                                   focus.detection.label, focus.score, focus.normalized_area);
    DecisionCommand command;
    command.mode = BehaviorMode::AVOID_OBSTACLE;
    command.focus = focus;
    command.target_confidence = focus.detection.confidence;

    const double altitude_error = static_cast<double>(cfg_.safe_altitude_m) - ctx.pose.position.z();
    Eigen::Vector3d velocity =
        -body_forward(ctx.pose) * (0.9 + focus.normalized_area * 2.0) +
        -body_right(ctx.pose) * std::clamp(static_cast<double>(focus.image_offset.x()) * 3.0, -1.2, 1.2) +
        body_up(ctx.pose) * std::clamp(altitude_error * 0.15, 0.0, 0.9);

    command.desired_velocity =
        clamp_speed(velocity, cfg_.max_avoid_speed_mps * caution_scale(ctx) * localization_scale(ctx));
    command.desired_yaw_rate_rads = -std::clamp(static_cast<double>(focus.image_offset.x()) * 1.6, -1.0, 1.0);
    command.requires_operator_attention =
        is_hazard_label(focus.detection.label) || focus.normalized_area >= 0.20f;

    std::ostringstream oss;
    oss << "Avoiding " << focus.detection.label
        << " score=" << focus.score;
    command.summary = oss.str();
    return command;
}

bool DecisionEngine::is_hazard_label(std::string_view label) {
    static const std::array<std::string_view, 10> hazards = {{
        "person", "car", "truck", "bus", "motorcycle", "bicycle",
        "bird", "dog", "animal", "tree"
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
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
