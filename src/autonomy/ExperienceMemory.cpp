// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "autonomy/ExperienceMemory.hpp"
#include "utils/RuntimeLogging.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <unordered_map>

namespace drone::autonomy {

namespace {

std::string lowercase(std::string_view value) {
    std::string out(value.begin(), value.end());
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

double slope_per_minute(double x0, double y0, double x1, double y1) {
    const double dt = std::max(x1 - x0, 1.0e-6);
    return (y1 - y0) * (60.0 / dt);
}

} // namespace

ExperienceMemory::ExperienceMemory(Config cfg)
    : cfg_(cfg) {
    drone::utils::get_or_create_logger("MEMORY")->info(
        "ExperienceMemory initialized max_obs={} caution_threshold={}",
        cfg_.max_observations_per_drone,
        cfg_.caution_risk_threshold);
}

void ExperienceMemory::observe(uint32_t drone_id,
                               const vio::PoseEstimate& pose,
                               const hal::SystemStats& system,
                               const std::optional<sensors::CameraFrame>& frame,
                               size_t swarm_peer_count,
                               double localization_confidence,
                               std::string_view localization_source,
                               bool localization_lost,
                               double now_s) {
    auto logger = drone::utils::get_or_create_logger("MEMORY");
    Observation obs;
    obs.now_s = now_s;
    obs.drift_m = pose.pos_std.norm();
    obs.battery_pct = system.battery_pct;
    obs.swarm_peer_count = swarm_peer_count;
    obs.localization_confidence = localization_confidence;
    obs.localization_lost = localization_lost;
    obs.localization_source = lowercase(localization_source);

    if (frame.has_value() && !frame->detections.empty()) {
        const auto& detections = frame->detections;
        obs.detection_count = detections.size();
        const auto primary = std::max_element(detections.begin(), detections.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.confidence < rhs.confidence; });
        if (primary != detections.end()) {
            obs.primary_label = lowercase(primary->label);
        }
        for (const auto& detection : detections) {
            if (is_hazard_label(detection.label) || is_unknown_label(detection.label)) {
                ++obs.hazard_count;
            }
            if (is_target_label(detection.label)) {
                ++obs.target_count;
            }
        }
    }

    auto& queue = history_[drone_id];
    queue.push_back(std::move(obs));
    while (queue.size() > cfg_.max_observations_per_drone) {
        queue.pop_front();
    }
    logger->debug("ExperienceMemory observe drone={} queue={} drift={:.3f} battery={:.1f} hazards={} targets={}",
                  drone_id, queue.size(), obs.drift_m, obs.battery_pct, obs.hazard_count, obs.target_count);
}

MemoryPrior ExperienceMemory::summarize(uint32_t drone_id) const {
    auto logger = drone::utils::get_or_create_logger("MEMORY");
    auto it = history_.find(drone_id);
    if (it == history_.end()) {
        logger->debug("ExperienceMemory summarize drone={} empty-history", drone_id);
        return {};
    }
    auto prior = summarize_queue(it->second);
    logger->debug("ExperienceMemory summarize drone={} risk={:.3f} caution={} label={}",
                  drone_id, prior.risk_score, prior.recommend_caution, prior.dominant_label);
    return prior;
}

MemoryPrior ExperienceMemory::summarize_fleet() const {
    auto logger = drone::utils::get_or_create_logger("MEMORY");
    std::deque<Observation> combined;
    for (const auto& [drone_id, queue] : history_) {
        combined.insert(combined.end(), queue.begin(), queue.end());
    }
    auto prior = summarize_queue(combined);
    logger->info("ExperienceMemory fleet summary risk={:.3f} caution={} label={}",
                 prior.risk_score, prior.recommend_caution, prior.dominant_label);
    return prior;
}

void ExperienceMemory::reset() {
    drone::utils::get_or_create_logger("MEMORY")->info("ExperienceMemory reset");
    history_.clear();
}

MemoryPrior ExperienceMemory::summarize_queue(const std::deque<Observation>& queue) const {
    if (queue.empty()) {
        return {};
    }

    MemoryPrior prior;
    const auto& first = queue.front();
    const auto& last = queue.back();
    prior.drift_trend_m_per_min = slope_per_minute(first.now_s, first.drift_m, last.now_s, last.drift_m);
    prior.battery_burn_pct_per_min = -slope_per_minute(first.now_s, first.battery_pct, last.now_s, last.battery_pct);

    size_t hazards = 0;
    size_t targets = 0;
    size_t with_labels = 0;
    size_t localization_dropouts = 0;
    size_t low_feature_observations = 0;
    double localization_conf_sum = 0.0;
    std::unordered_map<std::string, size_t> label_histogram;
    std::unordered_map<std::string, size_t> localization_histogram;
    for (const auto& obs : queue) {
        hazards += obs.hazard_count;
        targets += obs.target_count;
        localization_conf_sum += obs.localization_confidence;
        if (obs.localization_lost || obs.localization_confidence < 0.35) {
            ++localization_dropouts;
        }
        if (obs.detection_count == 0) {
            ++low_feature_observations;
        }
        if (!obs.primary_label.empty()) {
            ++label_histogram[obs.primary_label];
            ++with_labels;
        }
        if (!obs.localization_source.empty()) {
            ++localization_histogram[obs.localization_source];
        }
    }

    const double denom = std::max(static_cast<double>(queue.size()), 1.0);
    prior.obstacle_frequency = static_cast<double>(hazards) / denom;
    prior.target_frequency = static_cast<double>(targets) / denom;
    prior.localization_confidence_avg = localization_conf_sum / denom;
    prior.localization_dropout_frequency = static_cast<double>(localization_dropouts) / denom;
    prior.low_feature_frequency = static_cast<double>(low_feature_observations) / denom;

    auto dominant = std::max_element(label_histogram.begin(), label_histogram.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });
    if (dominant != label_histogram.end()) {
        prior.dominant_label = dominant->first;
    }
    auto dominant_localization = std::max_element(
        localization_histogram.begin(), localization_histogram.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });
    if (dominant_localization != localization_histogram.end()) {
        prior.dominant_localization_source = dominant_localization->first;
    }

    prior.risk_score =
        std::clamp(prior.obstacle_frequency * 0.32
                 + std::max(0.0, prior.drift_trend_m_per_min) * 0.85
                 + std::max(0.0, prior.battery_burn_pct_per_min) * 0.06
                 + prior.localization_dropout_frequency * 0.70
                 + prior.low_feature_frequency * 0.22
                 + std::max(0.0, 0.70 - prior.localization_confidence_avg) * 0.55,
                   0.0, 1.5);
    prior.recommend_caution = prior.risk_score >= cfg_.caution_risk_threshold;
    return prior;
}

bool ExperienceMemory::is_hazard_label(std::string_view label) {
    static const std::array<std::string_view, 10> hazards = {{
        "person", "car", "truck", "bus", "motorcycle", "bicycle",
        "bird", "dog", "animal", "tree"
    }};
    const auto lowered = lowercase(label);
    return std::find(hazards.begin(), hazards.end(), lowered) != hazards.end();
}

bool ExperienceMemory::is_target_label(std::string_view label) {
    static const std::array<std::string_view, 6> targets = {{
        "person", "drone", "vehicle", "car", "truck", "boat"
    }};
    const auto lowered = lowercase(label);
    return std::find(targets.begin(), targets.end(), lowered) != targets.end();
}

bool ExperienceMemory::is_unknown_label(std::string_view label) {
    const auto lowered = lowercase(label);
    return lowered.rfind("unknown_class_", 0) == 0;
}

} // namespace drone::autonomy
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
