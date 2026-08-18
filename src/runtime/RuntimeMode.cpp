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

std::optional<std::string> extract_object_section(const std::string& content,
                                                  std::string_view key) {
    const std::string needle = "\"" + std::string(key) + "\"";
    const size_t key_pos = content.find(needle);
    if (key_pos == std::string::npos) {
        return std::nullopt;
    }
    const size_t colon_pos = content.find(':', key_pos + needle.size());
    if (colon_pos == std::string::npos) {
        return std::nullopt;
    }
    const size_t brace_pos = content.find('{', colon_pos + 1);
    if (brace_pos == std::string::npos) {
        return std::nullopt;
    }

    int depth = 0;
    bool in_string = false;
    bool escaped = false;
    for (size_t i = brace_pos; i < content.size(); ++i) {
        const char c = content[i];
        if (in_string) {
            if (escaped) {
                escaped = false;
            } else if (c == '\\') {
                escaped = true;
            } else if (c == '"') {
                in_string = false;
            }
            continue;
        }
        if (c == '"') {
            in_string = true;
            continue;
        }
        if (c == '{') {
            ++depth;
        } else if (c == '}') {
            --depth;
            if (depth == 0) {
                return content.substr(brace_pos, i - brace_pos + 1);
            }
        }
    }
    return std::nullopt;
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
    if (const auto value = drone::utils::simple_json::extract_string(content, "estimator_mode")) {
        config.estimator_mode = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_bool(content, "enable_experimental_hybrid")) {
        config.estimator_enable_experimental_hybrid = *value;
    }
    if (const auto value = drone::utils::simple_json::extract_bool(content, "enable_fej")) {
        config.estimator_enable_fej = *value;
        config.estimator_fej_enabled = *value;
    }
    if (const auto value = drone::utils::simple_json::extract_bool(content, "enable_msckf")) {
        config.estimator_enable_msckf = *value;
        config.estimator_msckf_enabled = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_bool(content, "enable_loop_closure_correction")) {
        config.estimator_enable_loop_closure_correction = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_bool(content, "enable_automatic_zupt")) {
        config.estimator_enable_automatic_zupt = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_bool(content, "enable_shadow_estimator")) {
        config.estimator_enable_shadow_estimator = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_bool(content, "shadow_comparison_enabled")) {
        config.estimator_shadow_comparison_enabled = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_u64(content, "shadow_max_queue_depth")) {
        config.estimator_shadow_max_queue_depth = static_cast<uint32_t>(*value);
    }
    if (const auto value =
            drone::utils::simple_json::extract_number(content, "shadow_max_lag_ms")) {
        config.estimator_shadow_max_lag_ms = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_number(content, "shadow_position_divergence_m")) {
        config.estimator_shadow_position_divergence_m = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_number(content, "shadow_velocity_divergence_mps")) {
        config.estimator_shadow_velocity_divergence_mps = *value;
    }
    if (const auto value = drone::utils::simple_json::extract_number(
            content, "shadow_orientation_divergence_deg")) {
        config.estimator_shadow_orientation_divergence_deg = *value;
    }
    if (const auto value = drone::utils::simple_json::extract_u64(
            content, "shadow_required_consecutive_divergent_samples")) {
        config.estimator_shadow_required_consecutive_divergent_samples =
            static_cast<uint32_t>(*value);
    }
    if (const auto value =
            drone::utils::simple_json::extract_bool(content, "reject_non_finite_measurements")) {
        config.estimator_reject_non_finite_measurements = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_bool(content, "require_monotonic_timestamps")) {
        config.estimator_require_monotonic_timestamps = *value;
    }
    if (const auto value = drone::utils::simple_json::extract_number(content, "max_imu_dt_s")) {
        config.estimator_max_imu_dt_s = *value;
    }
    if (const auto value = drone::utils::simple_json::extract_number(content, "min_imu_dt_s")) {
        config.estimator_min_imu_dt_s = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_number(content, "covariance_symmetry_tolerance")) {
        config.estimator_covariance_symmetry_tolerance = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_number(content, "variance_negativity_tolerance")) {
        config.estimator_variance_negativity_tolerance = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_number(content, "quaternion_min_norm")) {
        config.estimator_quaternion_min_norm = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_number(content, "zupt_sigma_velocity_mps")) {
        config.estimator_zupt_sigma_velocity_mps = *value;
    }
    if (const auto stationary_section = extract_object_section(content, "stationary_detector")) {
        if (const auto value =
                drone::utils::simple_json::extract_bool(*stationary_section, "enabled")) {
            config.estimator_stationary_detector_enabled = *value;
        }
        if (const auto value =
                drone::utils::simple_json::extract_number(*stationary_section, "accel_threshold")) {
            config.estimator_stationary_detector_accel_threshold = *value;
        }
        if (const auto value =
                drone::utils::simple_json::extract_number(*stationary_section, "gyro_threshold")) {
            config.estimator_stationary_detector_gyro_threshold = *value;
        }
        if (const auto value =
                drone::utils::simple_json::extract_u64(*stationary_section, "window_size")) {
            config.estimator_stationary_detector_window_size = static_cast<uint32_t>(*value);
        }
        if (const auto value =
                drone::utils::simple_json::extract_u64(*stationary_section, "enter_count")) {
            config.estimator_stationary_detector_enter_count = static_cast<uint32_t>(*value);
        }
        if (const auto value =
                drone::utils::simple_json::extract_u64(*stationary_section, "exit_count")) {
            config.estimator_stationary_detector_exit_count = static_cast<uint32_t>(*value);
        }
        if (const auto value = drone::utils::simple_json::extract_number(
                *stationary_section, "minimum_stationary_time")) {
            config.estimator_stationary_detector_minimum_stationary_time = *value;
        }
        if (const auto value = drone::utils::simple_json::extract_number(*stationary_section,
                                                                         "accel_exit_threshold")) {
            config.estimator_stationary_detector_accel_exit_threshold = *value;
        }
        if (const auto value = drone::utils::simple_json::extract_number(*stationary_section,
                                                                         "gyro_exit_threshold")) {
            config.estimator_stationary_detector_gyro_exit_threshold = *value;
        }
    }
    if (const auto zupt_section = extract_object_section(content, "zupt")) {
        if (const auto value = drone::utils::simple_json::extract_bool(*zupt_section, "enabled")) {
            config.estimator_zupt_enabled = *value;
        }
        if (const auto value =
                drone::utils::simple_json::extract_number(*zupt_section, "velocity_noise")) {
            config.estimator_zupt_velocity_noise_mps = *value;
        }
        if (const auto value =
                drone::utils::simple_json::extract_number(*zupt_section, "max_update_rate")) {
            config.estimator_zupt_max_update_rate_hz = *value;
        }
    }
    if (const auto fej_section = extract_object_section(content, "fej")) {
        if (const auto value = drone::utils::simple_json::extract_bool(*fej_section, "enabled")) {
            config.estimator_fej_enabled = *value;
        }
        if (const auto value =
                drone::utils::simple_json::extract_bool(*fej_section, "validation_checks")) {
            config.estimator_fej_validation_checks = *value;
        }
        if (const auto value =
                drone::utils::simple_json::extract_bool(*fej_section, "diagnostics_enabled")) {
            config.estimator_fej_diagnostics_enabled = *value;
        }
    }
    if (const auto msckf_section = extract_object_section(content, "msckf")) {
        if (const auto value = drone::utils::simple_json::extract_bool(*msckf_section, "enabled")) {
            config.estimator_msckf_enabled = *value;
        }
        if (const auto value =
                drone::utils::simple_json::extract_u64(*msckf_section, "max_camera_states")) {
            config.estimator_msckf_max_camera_states = static_cast<uint32_t>(*value);
        }
        if (const auto value =
                drone::utils::simple_json::extract_string(*msckf_section, "eviction_policy")) {
            config.estimator_msckf_eviction_policy = *value;
        }
        if (const auto value =
                drone::utils::simple_json::extract_bool(*msckf_section, "diagnostics_enabled")) {
            config.estimator_msckf_diagnostics_enabled = *value;
        }
        if (const auto triangulation_section =
                extract_object_section(*msckf_section, "triangulation")) {
            if (const auto value =
                    drone::utils::simple_json::extract_bool(*triangulation_section, "enabled")) {
                config.estimator_msckf_triangulation_enabled = *value;
            }
            if (const auto value = drone::utils::simple_json::extract_u64(*triangulation_section,
                                                                          "minimum_observations")) {
                config.estimator_msckf_triangulation_minimum_observations =
                    static_cast<uint32_t>(*value);
            }
            if (const auto value = drone::utils::simple_json::extract_number(*triangulation_section,
                                                                             "minimum_baseline")) {
                config.estimator_msckf_triangulation_minimum_baseline = *value;
            }
            if (const auto value = drone::utils::simple_json::extract_number(
                    *triangulation_section, "maximum_reprojection_error")) {
                config.estimator_msckf_triangulation_maximum_reprojection_error = *value;
            }
            if (const auto value = drone::utils::simple_json::extract_number(*triangulation_section,
                                                                             "minimum_depth")) {
                config.estimator_msckf_triangulation_minimum_depth = *value;
            }
            if (const auto value = drone::utils::simple_json::extract_number(*triangulation_section,
                                                                             "maximum_depth")) {
                config.estimator_msckf_triangulation_maximum_depth = *value;
            }
        }
        if (const auto update_section = extract_object_section(*msckf_section, "msckf_update")) {
            if (const auto value =
                    drone::utils::simple_json::extract_bool(*update_section, "enabled")) {
                config.estimator_msckf_update_enabled = *value;
            }
            if (const auto value = drone::utils::simple_json::extract_number(
                    *update_section, "chi_square_probability")) {
                config.estimator_msckf_update_chi_square_probability = *value;
            }
            if (const auto value = drone::utils::simple_json::extract_u64(*update_section,
                                                                          "minimum_track_length")) {
                config.estimator_msckf_update_minimum_track_length = static_cast<uint32_t>(*value);
            }
            if (const auto value = drone::utils::simple_json::extract_u64(*update_section,
                                                                          "maximum_track_length")) {
                config.estimator_msckf_update_maximum_track_length = static_cast<uint32_t>(*value);
            }
            if (const auto value = drone::utils::simple_json::extract_number(*update_section,
                                                                             "maximum_residual")) {
                config.estimator_msckf_update_maximum_residual = *value;
            }
            if (const auto value =
                    drone::utils::simple_json::extract_bool(*update_section, "validation_checks")) {
                config.estimator_msckf_update_validation_checks = *value;
            }
            if (const auto value = drone::utils::simple_json::extract_bool(*update_section,
                                                                           "diagnostics_enabled")) {
                config.estimator_msckf_update_diagnostics_enabled = *value;
            }
        }
    }
    if (const auto value =
            drone::utils::simple_json::extract_bool(content, "lidar_depth_correction_enabled")) {
        config.estimator_lidar_depth_correction_enabled = *value;
    }
    if (const auto value =
            drone::utils::simple_json::extract_bool(content, "diagnostics_enabled")) {
        config.estimator_diagnostics_enabled = *value;
    }

    auto note_estimator_error = [&](const std::string& error) {
        config.estimator_config_valid = false;
        config.estimator_errors.push_back(error);
    };
    if (!std::isfinite(config.estimator_max_imu_dt_s) || config.estimator_max_imu_dt_s <= 0.0) {
        note_estimator_error("estimator max_imu_dt_s must be finite and positive");
    }
    if (!std::isfinite(config.estimator_min_imu_dt_s) || config.estimator_min_imu_dt_s < 0.0) {
        note_estimator_error("estimator min_imu_dt_s must be finite and non-negative");
    }
    if (config.estimator_max_imu_dt_s < config.estimator_min_imu_dt_s) {
        note_estimator_error(
            "estimator max_imu_dt_s must be greater than or equal to min_imu_dt_s");
    }
    if (!std::isfinite(config.estimator_covariance_symmetry_tolerance) ||
        config.estimator_covariance_symmetry_tolerance < 0.0) {
        note_estimator_error(
            "estimator covariance_symmetry_tolerance must be finite and non-negative");
    }
    if (!std::isfinite(config.estimator_variance_negativity_tolerance) ||
        config.estimator_variance_negativity_tolerance < 0.0) {
        note_estimator_error(
            "estimator variance_negativity_tolerance must be finite and non-negative");
    }
    if (!std::isfinite(config.estimator_quaternion_min_norm) ||
        config.estimator_quaternion_min_norm <= 0.0) {
        note_estimator_error("estimator quaternion_min_norm must be finite and positive");
    }
    if (!std::isfinite(config.estimator_zupt_sigma_velocity_mps) ||
        config.estimator_zupt_sigma_velocity_mps <= 0.0) {
        note_estimator_error("estimator zupt_sigma_velocity_mps must be finite and positive");
    }
    if (!std::isfinite(config.estimator_stationary_detector_accel_threshold) ||
        config.estimator_stationary_detector_accel_threshold < 0.0) {
        note_estimator_error(
            "estimator stationary_detector.accel_threshold must be finite and non-negative");
    }
    if (!std::isfinite(config.estimator_stationary_detector_gyro_threshold) ||
        config.estimator_stationary_detector_gyro_threshold < 0.0) {
        note_estimator_error(
            "estimator stationary_detector.gyro_threshold must be finite and non-negative");
    }
    if (config.estimator_stationary_detector_window_size == 0u) {
        note_estimator_error("estimator stationary_detector.window_size must be greater than zero");
    }
    if (config.estimator_stationary_detector_enter_count == 0u) {
        note_estimator_error("estimator stationary_detector.enter_count must be greater than zero");
    }
    if (config.estimator_stationary_detector_exit_count == 0u) {
        note_estimator_error("estimator stationary_detector.exit_count must be greater than zero");
    }
    if (config.estimator_stationary_detector_enter_count >
        config.estimator_stationary_detector_window_size) {
        note_estimator_error(
            "estimator stationary_detector.enter_count must be less than or equal to window_size");
    }
    if (config.estimator_stationary_detector_exit_count >
        config.estimator_stationary_detector_window_size) {
        note_estimator_error(
            "estimator stationary_detector.exit_count must be less than or equal to window_size");
    }
    if (!std::isfinite(config.estimator_stationary_detector_minimum_stationary_time) ||
        config.estimator_stationary_detector_minimum_stationary_time < 0.0) {
        note_estimator_error("estimator stationary_detector.minimum_stationary_time must be "
                             "finite and non-negative");
    }
    if (!std::isfinite(config.estimator_stationary_detector_accel_exit_threshold) ||
        config.estimator_stationary_detector_accel_exit_threshold <
            config.estimator_stationary_detector_accel_threshold) {
        note_estimator_error("estimator stationary_detector.accel_exit_threshold must be finite "
                             "and greater than or equal to accel_threshold");
    }
    if (!std::isfinite(config.estimator_stationary_detector_gyro_exit_threshold) ||
        config.estimator_stationary_detector_gyro_exit_threshold <
            config.estimator_stationary_detector_gyro_threshold) {
        note_estimator_error("estimator stationary_detector.gyro_exit_threshold must be finite "
                             "and greater than or equal to gyro_threshold");
    }
    if (!std::isfinite(config.estimator_zupt_velocity_noise_mps) ||
        config.estimator_zupt_velocity_noise_mps <= 0.0) {
        note_estimator_error("estimator zupt.velocity_noise must be finite and positive");
    }
    if (!std::isfinite(config.estimator_zupt_max_update_rate_hz) ||
        config.estimator_zupt_max_update_rate_hz <= 0.0) {
        note_estimator_error("estimator zupt.max_update_rate must be finite and positive");
    }
    if (config.estimator_shadow_max_queue_depth == 0u) {
        note_estimator_error("estimator shadow_max_queue_depth must be greater than zero");
    }
    if (config.estimator_msckf_max_camera_states == 0u) {
        note_estimator_error("estimator msckf.max_camera_states must be greater than zero");
    }
    if (config.estimator_msckf_eviction_policy != "oldest_first") {
        note_estimator_error("estimator msckf.eviction_policy must be 'oldest_first'");
    }
    if (config.estimator_msckf_triangulation_minimum_observations < 2u) {
        note_estimator_error(
            "estimator msckf.triangulation.minimum_observations must be at least two");
    }
    if (!std::isfinite(config.estimator_msckf_triangulation_minimum_baseline) ||
        config.estimator_msckf_triangulation_minimum_baseline < 0.0) {
        note_estimator_error(
            "estimator msckf.triangulation.minimum_baseline must be finite and non-negative");
    }
    if (!std::isfinite(config.estimator_msckf_triangulation_maximum_reprojection_error) ||
        config.estimator_msckf_triangulation_maximum_reprojection_error < 0.0) {
        note_estimator_error("estimator msckf.triangulation.maximum_reprojection_error must be "
                             "finite and non-negative");
    }
    if (!std::isfinite(config.estimator_msckf_triangulation_minimum_depth) ||
        config.estimator_msckf_triangulation_minimum_depth <= 0.0) {
        note_estimator_error(
            "estimator msckf.triangulation.minimum_depth must be finite and positive");
    }
    if (!std::isfinite(config.estimator_msckf_triangulation_maximum_depth) ||
        config.estimator_msckf_triangulation_maximum_depth <
            config.estimator_msckf_triangulation_minimum_depth) {
        note_estimator_error("estimator msckf.triangulation.maximum_depth must be finite and "
                             "greater than or equal to minimum_depth");
    }
    if (config.estimator_msckf_triangulation_enabled && !config.estimator_msckf_enabled &&
        !config.estimator_enable_msckf) {
        note_estimator_error("estimator msckf.triangulation.enabled requires msckf.enabled");
    }
    if (!std::isfinite(config.estimator_msckf_update_chi_square_probability) ||
        config.estimator_msckf_update_chi_square_probability <= 0.0 ||
        config.estimator_msckf_update_chi_square_probability >= 1.0) {
        note_estimator_error(
            "estimator msckf.msckf_update.chi_square_probability must be finite and in (0, 1)");
    }
    if (config.estimator_msckf_update_minimum_track_length < 2u) {
        note_estimator_error(
            "estimator msckf.msckf_update.minimum_track_length must be at least two");
    }
    if (config.estimator_msckf_update_maximum_track_length <
        config.estimator_msckf_update_minimum_track_length) {
        note_estimator_error("estimator msckf.msckf_update.maximum_track_length must be greater "
                             "than or equal to minimum_track_length");
    }
    if (!std::isfinite(config.estimator_msckf_update_maximum_residual) ||
        config.estimator_msckf_update_maximum_residual < 0.0) {
        note_estimator_error(
            "estimator msckf.msckf_update.maximum_residual must be finite and non-negative");
    }
    if (config.estimator_msckf_update_enabled &&
        (!config.estimator_msckf_triangulation_enabled || !config.estimator_msckf_enabled)) {
        note_estimator_error("estimator msckf.msckf_update.enabled requires msckf.enabled and "
                             "triangulation.enabled");
    }
    if (!std::isfinite(config.estimator_shadow_max_lag_ms) ||
        config.estimator_shadow_max_lag_ms < 0.0) {
        note_estimator_error("estimator shadow_max_lag_ms must be finite and non-negative");
    }
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
