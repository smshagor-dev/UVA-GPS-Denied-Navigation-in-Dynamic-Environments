#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace drone::runtime {

enum class RuntimeMode : uint8_t {
    SIMULATION = 0,
    BENCH,
    PRODUCTION,
};

struct RuntimeFileConfig {
    bool loaded{false};
    std::optional<RuntimeMode> runtime_mode;
    std::string anchor_config_path;
    std::string lidar_config_path;
    std::string detector_labels_path;
};

struct RuntimeValidationInputs {
    RuntimeMode runtime_mode{RuntimeMode::SIMULATION};
    std::string anchor_config_path;
    bool has_csv_source{false};
    bool has_udp_source{false};
    bool has_serial_source{false};
};

struct RuntimeValidationResult {
    bool ok{true};
    std::vector<std::string> errors{};
};

struct AnchorDefinition {
    std::string source_id;
    uint32_t id{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

struct AnchorConfigLoadResult {
    bool ok{false};
    std::string coordinate_frame{"unknown"};
    std::string units{"unknown"};
    std::vector<AnchorDefinition> anchors{};
    std::vector<std::string> warnings{};
    std::vector<std::string> errors{};
};

struct LidarConfigLoadResult {
    bool ok{false};
    std::string host{"0.0.0.0"};
    uint16_t port{2368};
    std::string model{"generic_udp_cartesian_v1"};
    std::string frame_id{"lidar"};
    float min_range_m{0.3f};
    float max_range_m{80.0f};
    bool required{false};
    std::vector<std::string> errors{};
};

struct LidarRuntimeValidationInputs {
    RuntimeMode runtime_mode{RuntimeMode::SIMULATION};
    bool lidar_enabled{false};
    bool lidar_required{false};
    bool lidar_initialized{false};
};

[[nodiscard]] std::string_view to_string(RuntimeMode mode);
[[nodiscard]] RuntimeMode parse_runtime_mode(std::string_view value);
[[nodiscard]] RuntimeFileConfig load_runtime_file(const std::string& path);
[[nodiscard]] RuntimeValidationResult validate_runtime_configuration(const RuntimeValidationInputs& input);
[[nodiscard]] AnchorConfigLoadResult load_anchor_config_json(const std::string& path);
[[nodiscard]] LidarConfigLoadResult load_lidar_config_json(const std::string& path);
[[nodiscard]] RuntimeValidationResult validate_lidar_runtime_configuration(const LidarRuntimeValidationInputs& input);
[[nodiscard]] std::string determine_localization_data_source(
    RuntimeMode mode,
    bool used_synthetic,
    bool used_csv_playback,
    bool used_live_external,
    bool has_measurements);

} // namespace drone::runtime
