#include "runtime/RuntimeMode.hpp"
#include "utils/SimpleJson.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <regex>
#include <set>
#include <sstream>
#include <unordered_set>

namespace drone::runtime {

namespace {

std::string lowercase(std::string_view value) {
    std::string out(value.begin(), value.end());
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

bool finite_coord(double value) {
    return std::isfinite(value) && std::abs(value) < 1.0e6;
}

std::optional<uint32_t> parse_anchor_numeric_id(std::string_view id_text) {
    std::string trimmed(id_text.begin(), id_text.end());
    trimmed.erase(std::remove_if(trimmed.begin(), trimmed.end(),
                                 [](unsigned char c) { return std::isspace(c); }),
                  trimmed.end());
    if (trimmed.empty()) {
        return std::nullopt;
    }

    bool all_digits = std::all_of(trimmed.begin(), trimmed.end(),
                                  [](unsigned char c) { return std::isdigit(c); });
    try {
        if (all_digits) {
            return static_cast<uint32_t>(std::stoul(trimmed));
        }
        size_t digit_start = trimmed.find_first_of("0123456789");
        if (digit_start == std::string::npos) {
            return std::nullopt;
        }
        const std::string numeric = trimmed.substr(digit_start);
        if (!std::all_of(numeric.begin(), numeric.end(),
                         [](unsigned char c) { return std::isdigit(c); })) {
            return std::nullopt;
        }
        return static_cast<uint32_t>(std::stoul(numeric));
    } catch (...) {
        return std::nullopt;
    }
}

void assess_anchor_geometry(const std::vector<AnchorDefinition>& anchors,
                            std::vector<std::string>& warnings) {
    if (anchors.size() < 4) {
        return;
    }

    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < anchors.size(); ++i) {
        for (size_t j = i + 1; j < anchors.size(); ++j) {
            min_distance =
                std::min(min_distance, (anchors[i].position - anchors[j].position).norm());
        }
    }
    if (min_distance < 1.0) {
        warnings.push_back(
            "anchor geometry quality warning: at least two anchors are closer than 1 meter");
    }

    bool found_non_collinear = false;
    bool found_non_coplanar = false;
    for (size_t i = 0; i < anchors.size() && !found_non_coplanar; ++i) {
        for (size_t j = i + 1; j < anchors.size() && !found_non_coplanar; ++j) {
            for (size_t k = j + 1; k < anchors.size() && !found_non_coplanar; ++k) {
                const Eigen::Vector3d v1 = anchors[j].position - anchors[i].position;
                const Eigen::Vector3d v2 = anchors[k].position - anchors[i].position;
                const Eigen::Vector3d normal = v1.cross(v2);
                if (normal.norm() > 0.25) {
                    found_non_collinear = true;
                } else {
                    continue;
                }
                for (size_t m = k + 1; m < anchors.size(); ++m) {
                    const Eigen::Vector3d v3 = anchors[m].position - anchors[i].position;
                    const double tetra_measure = std::abs(normal.dot(v3));
                    if (tetra_measure > 0.25) {
                        found_non_coplanar = true;
                        break;
                    }
                }
            }
        }
    }

    if (!found_non_collinear) {
        warnings.push_back("anchor geometry quality warning: anchors are nearly collinear");
    } else if (!found_non_coplanar) {
        warnings.push_back("anchor geometry quality warning: anchors are nearly coplanar");
    }
}

} // namespace

std::string_view to_string(RuntimeMode mode) {
    switch (mode) {
    case RuntimeMode::SIMULATION:
        return "simulation";
    case RuntimeMode::BENCH:
        return "bench";
    case RuntimeMode::PRODUCTION:
        return "production";
    case RuntimeMode::EDGE_SWARM:
        return "edge_swarm";
    }
    return "simulation";
}

RuntimeMode parse_runtime_mode(std::string_view value) {
    const auto normalized = lowercase(value);
    if (normalized == "edge_swarm" || normalized == "edge-swarm" || normalized == "edge") {
        return RuntimeMode::EDGE_SWARM;
    }
    if (normalized == "bench") {
        return RuntimeMode::BENCH;
    }
    if (normalized == "production" || normalized == "prod") {
        return RuntimeMode::PRODUCTION;
    }
    return RuntimeMode::SIMULATION;
}

RuntimeFileConfig load_runtime_file(const std::string& path) {
    RuntimeFileConfig config;
    if (path.empty() || !std::filesystem::exists(path)) {
        return config;
    }

    std::ifstream input(path);
    if (!input.is_open()) {
        return config;
    }

    std::stringstream buffer;
    buffer << input.rdbuf();
    const std::string content = buffer.str();
    config.loaded = true;

    if (const auto mode = drone::utils::simple_json::extract_string(content, "runtime_mode")) {
        config.runtime_mode = parse_runtime_mode(*mode);
    }
    if (const auto anchor_path =
            drone::utils::simple_json::extract_string(content, "anchor_config_path")) {
        config.anchor_config_path = *anchor_path;
    }
    if (const auto lidar_path =
            drone::utils::simple_json::extract_string(content, "lidar_config_path")) {
        config.lidar_config_path = *lidar_path;
    }
    if (const auto detector_labels_path =
            drone::utils::simple_json::extract_string(content, "detector_labels_path")) {
        config.detector_labels_path = *detector_labels_path;
    }
    config.estimator_config_present = content.find("\"estimator\"") != std::string::npos;
    if (const auto mode = drone::utils::simple_json::extract_string(content, "mode")) {
        config.estimator_mode = *mode;
    }
    if (const auto enable =
            drone::utils::simple_json::extract_bool(content, "enable_experimental_hybrid")) {
        config.estimator_enable_experimental_hybrid = *enable;
    }
    if (const auto enable = drone::utils::simple_json::extract_bool(content, "enable_fej")) {
        config.estimator_enable_fej = *enable;
    }
    if (const auto enable = drone::utils::simple_json::extract_bool(content, "enable_msckf")) {
        config.estimator_enable_msckf = *enable;
    }
    if (const auto enable =
            drone::utils::simple_json::extract_bool(content, "enable_loop_closure_correction")) {
        config.estimator_enable_loop_closure_correction = *enable;
    }
    if (const auto enable =
            drone::utils::simple_json::extract_bool(content, "enable_automatic_zupt")) {
        config.estimator_enable_automatic_zupt = *enable;
    }
    if (const auto enable = drone::utils::simple_json::extract_bool(content, "enabled")) {
        config.estimator_shadow_requested = *enable;
        config.estimator_enable_shadow_estimator = *enable;
    }
    if (const auto enable =
            drone::utils::simple_json::extract_bool(content, "comparison_enabled")) {
        config.estimator_shadow_comparison_enabled = *enable;
    }
    if (const auto implementation =
            drone::utils::simple_json::extract_string(content, "implementation")) {
        config.estimator_shadow_implementation = *implementation;
    }
    if (const auto max_depth =
            drone::utils::simple_json::extract_number(content, "max_queue_depth")) {
        config.estimator_shadow_max_queue_depth = static_cast<size_t>(*max_depth);
    }
    if (const auto max_lag = drone::utils::simple_json::extract_number(content, "max_lag_ms")) {
        config.estimator_shadow_max_lag_ms = *max_lag;
    }
    if (const auto threshold =
            drone::utils::simple_json::extract_number(content, "position_divergence_m")) {
        config.estimator_shadow_position_divergence_m = *threshold;
    }
    if (const auto threshold =
            drone::utils::simple_json::extract_number(content, "velocity_divergence_mps")) {
        config.estimator_shadow_velocity_divergence_mps = *threshold;
    }
    if (const auto threshold =
            drone::utils::simple_json::extract_number(content, "orientation_divergence_deg")) {
        config.estimator_shadow_orientation_divergence_deg = *threshold;
    }
    if (const auto samples = drone::utils::simple_json::extract_u64(
            content, "required_consecutive_divergent_samples")) {
        config.estimator_shadow_required_consecutive_divergent_samples =
            static_cast<uint32_t>(*samples);
    }
    if (const auto enable =
            drone::utils::simple_json::extract_bool(content, "reject_non_finite_measurements")) {
        config.estimator_reject_non_finite_measurements = *enable;
    }
    if (const auto enable =
            drone::utils::simple_json::extract_bool(content, "require_monotonic_timestamps")) {
        config.estimator_require_monotonic_timestamps = *enable;
    }
    if (const auto dt = drone::utils::simple_json::extract_number(content, "max_imu_dt_s")) {
        config.estimator_max_imu_dt_s = *dt;
    }
    if (const auto enable =
            drone::utils::simple_json::extract_bool(content, "diagnostics_enabled")) {
        config.estimator_diagnostics_enabled = *enable;
    }
    if (!std::isfinite(config.estimator_max_imu_dt_s) || config.estimator_max_imu_dt_s <= 0.0 ||
        config.estimator_max_imu_dt_s > 0.5) {
        config.estimator_config_valid = false;
        config.estimator_errors.push_back(
            "estimator.max_imu_dt_s must be finite and within (0, 0.5]");
    }
    if (const auto estimator_mode = lowercase(config.estimator_mode);
        estimator_mode != "minimal" && estimator_mode != "hybrid_active") {
        config.estimator_config_valid = false;
        config.estimator_errors.push_back("unknown estimator.mode; only \"minimal\" is supported");
    }
    if (lowercase(config.estimator_mode) == "hybrid_active") {
        config.estimator_config_valid = false;
        config.estimator_errors.push_back(
            "estimator.mode=\"hybrid_active\" is unsupported and rejected in phase 16");
    }
    if (config.estimator_enable_experimental_hybrid || config.estimator_enable_fej ||
        config.estimator_enable_msckf || config.estimator_enable_loop_closure_correction) {
        config.estimator_config_valid = false;
        config.estimator_errors.push_back(
            "Phase 15 keeps experimental hybrid/FEJ/MSCKF/loop-closure estimator features "
            "disabled");
    }
    if (config.estimator_enable_automatic_zupt) {
        config.estimator_config_valid = false;
        config.estimator_errors.push_back("automatic ZUPT remains disabled in phase 16");
    }
    if (config.estimator_shadow_requested && !config.estimator_shadow_compile_time_supported) {
        config.estimator_config_valid = false;
        config.estimator_errors.push_back(
            "shadow estimator requested but compile-time shadow support is disabled");
    }
    if (config.estimator_shadow_implementation != "minimal_eskf_clone" &&
        config.estimator_shadow_implementation != "minimal_shadow_clone") {
        config.estimator_config_valid = false;
        config.estimator_errors.push_back(
            "shadow implementation is unsupported; expected minimal_eskf_clone");
    }
    if (config.estimator_shadow_max_queue_depth == 0 ||
        config.estimator_shadow_max_queue_depth > 4096) {
        config.estimator_config_valid = false;
        config.estimator_errors.push_back("shadow.max_queue_depth must be within [1, 4096]");
    }
    for (const auto value :
         {config.estimator_shadow_max_lag_ms, config.estimator_shadow_position_divergence_m,
          config.estimator_shadow_velocity_divergence_mps,
          config.estimator_shadow_orientation_divergence_deg}) {
        if (!std::isfinite(value) || value < 0.0) {
            config.estimator_config_valid = false;
            config.estimator_errors.push_back(
                "shadow thresholds and lag values must be finite and non-negative");
            break;
        }
    }
    if (config.estimator_shadow_required_consecutive_divergent_samples == 0) {
        config.estimator_config_valid = false;
        config.estimator_errors.push_back(
            "shadow.required_consecutive_divergent_samples must be positive");
    }
    config.estimator_shadow_effective = config.estimator_config_valid &&
                                        config.estimator_shadow_requested &&
                                        config.estimator_shadow_compile_time_supported;
    return config;
}

RuntimeValidationResult validate_runtime_configuration(const RuntimeValidationInputs& input) {
    RuntimeValidationResult result;
    const bool has_any_external_source =
        input.has_csv_source || input.has_udp_source || input.has_serial_source;
    const bool has_anchor_config = !input.anchor_config_path.empty();

    if (input.runtime_mode == RuntimeMode::SIMULATION) {
        return result;
    }

    if (!has_anchor_config) {
        result.ok = false;
        result.errors.push_back("anchor_config_path is required outside simulation mode");
    }

    if (!has_any_external_source) {
        result.ok = false;
        result.errors.push_back("an external TDOA source is required outside simulation mode");
    }

    if (input.runtime_mode == RuntimeMode::PRODUCTION ||
        input.runtime_mode == RuntimeMode::EDGE_SWARM) {
        if (input.has_csv_source) {
            result.ok = false;
            result.errors.push_back("CSV playback is not allowed in production or edge_swarm mode");
        }
        if (!input.has_udp_source && !input.has_serial_source) {
            result.ok = false;
            result.errors.push_back(
                "production and edge_swarm modes require live UDP or serial TDOA input");
        }
    }

    return result;
}

AnchorConfigLoadResult load_anchor_config_json(const std::string& path) {
    AnchorConfigLoadResult result;
    if (path.empty()) {
        result.errors.push_back("anchor config path is empty");
        return result;
    }
    if (!std::filesystem::exists(path)) {
        result.errors.push_back("anchor config file does not exist");
        return result;
    }

    std::ifstream input(path);
    if (!input.is_open()) {
        result.errors.push_back("anchor config file could not be opened");
        return result;
    }

    std::stringstream buffer;
    buffer << input.rdbuf();
    const std::string content = buffer.str();
    result.coordinate_frame =
        drone::utils::simple_json::extract_string(content, "coordinate_frame").value_or("unknown");
    result.units = drone::utils::simple_json::extract_string(content, "units").value_or("unknown");

    const std::regex anchor_pattern(
        "\\{[^\\{\\}]*\"id\"\\s*:\\s*\"([^\"]+)\"[^\\{\\}]*\"x\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:"
        "[eE][+-]?[0-9]+)?)\\s*,?[^\\{\\}]*\"y\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)"
        "?)\\s*,?[^\\{\\}]*\"z\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)",
        std::regex::icase);
    std::sregex_iterator it(content.begin(), content.end(), anchor_pattern);
    std::sregex_iterator end;

    std::unordered_set<std::string> seen_source_ids;
    std::unordered_set<uint32_t> seen_numeric_ids;
    for (; it != end; ++it) {
        const auto& match = *it;
        if (match.size() < 5) {
            continue;
        }

        const std::string source_id = match[1].str();
        const auto numeric_id = parse_anchor_numeric_id(source_id);
        if (!numeric_id.has_value()) {
            result.errors.push_back("anchor id \"" + source_id +
                                    "\" is invalid; expected digits or a string ending in digits");
            continue;
        }

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        try {
            x = std::stod(match[2].str());
            y = std::stod(match[3].str());
            z = std::stod(match[4].str());
        } catch (...) {
            result.errors.push_back("anchor \"" + source_id + "\" contains invalid coordinates");
            continue;
        }

        if (!finite_coord(x) || !finite_coord(y) || !finite_coord(z)) {
            result.errors.push_back("anchor \"" + source_id +
                                    "\" contains non-finite or out-of-range coordinates");
            continue;
        }
        if (!seen_source_ids.insert(source_id).second) {
            result.errors.push_back("duplicate anchor id \"" + source_id + "\"");
            continue;
        }
        if (!seen_numeric_ids.insert(*numeric_id).second) {
            result.errors.push_back("duplicate numeric anchor id derived from \"" + source_id +
                                    "\"");
            continue;
        }

        result.anchors.push_back({source_id, *numeric_id, Eigen::Vector3d{x, y, z}});
    }

    if (result.anchors.size() < 4) {
        result.errors.push_back("anchor config must contain at least 4 valid anchors");
    }

    assess_anchor_geometry(result.anchors, result.warnings);
    result.ok = result.errors.empty();
    return result;
}

LidarConfigLoadResult load_lidar_config_json(const std::string& path) {
    LidarConfigLoadResult result;
    if (path.empty()) {
        result.errors.push_back("lidar config path is empty");
        return result;
    }
    if (!std::filesystem::exists(path)) {
        result.errors.push_back("lidar config file does not exist");
        return result;
    }

    std::ifstream input(path);
    if (!input.is_open()) {
        result.errors.push_back("lidar config file could not be opened");
        return result;
    }

    std::stringstream buffer;
    buffer << input.rdbuf();
    const std::string content = buffer.str();

    result.host = drone::utils::simple_json::extract_string(content, "host").value_or(result.host);
    if (const auto port = drone::utils::simple_json::extract_number(content, "port")) {
        if (*port >= 1.0 && *port <= 65535.0) {
            result.port = static_cast<uint16_t>(*port);
        } else {
            result.errors.push_back("lidar port must be between 1 and 65535");
        }
    }
    result.model =
        drone::utils::simple_json::extract_string(content, "model").value_or(result.model);
    result.frame_id =
        drone::utils::simple_json::extract_string(content, "frame_id").value_or(result.frame_id);
    if (const auto min_range = drone::utils::simple_json::extract_number(content, "min_range_m")) {
        result.min_range_m = static_cast<float>(*min_range);
    }
    if (const auto max_range = drone::utils::simple_json::extract_number(content, "max_range_m")) {
        result.max_range_m = static_cast<float>(*max_range);
    }
    if (const auto required = drone::utils::simple_json::extract_bool(content, "required")) {
        result.required = *required;
    }

    if (result.host.empty()) {
        result.errors.push_back("lidar host is required");
    }
    if (result.model.empty()) {
        result.errors.push_back("lidar model is required");
    }
    if (result.frame_id.empty()) {
        result.errors.push_back("lidar frame_id is required");
    }
    if (!(std::isfinite(result.min_range_m) && std::isfinite(result.max_range_m))) {
        result.errors.push_back("lidar min/max range must be finite");
    } else if (result.min_range_m < 0.0f || result.max_range_m <= result.min_range_m) {
        result.errors.push_back("lidar range configuration is invalid");
    }

    result.ok = result.errors.empty();
    return result;
}

RuntimeValidationResult
validate_lidar_runtime_configuration(const LidarRuntimeValidationInputs& input) {
    RuntimeValidationResult result;
    if (!input.lidar_enabled || !input.lidar_required) {
        return result;
    }
    if (!input.lidar_initialized) {
        result.ok = false;
        result.errors.push_back("required LiDAR is unavailable; bench/production/edge_swarm mode "
                                "cannot continue without live LiDAR initialization");
    }
    return result;
}

std::string determine_localization_data_source(RuntimeMode mode, bool used_synthetic,
                                               bool used_csv_playback, bool used_live_external,
                                               bool has_measurements) {
    if (used_synthetic) {
        return "simulation";
    }
    if (used_live_external) {
        return "real";
    }
    if (used_csv_playback) {
        return "playback";
    }
    if (!has_measurements) {
        return "unavailable";
    }
    return mode == RuntimeMode::SIMULATION ? "simulation" : "unavailable";
}

} // namespace drone::runtime
