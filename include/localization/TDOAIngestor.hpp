#pragma once

#include "localization/TDOALocalizer.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace drone::localization {

class TDOAIngestor {
public:
    enum class Mode : uint8_t {
        DISABLED = 0,
        CSV_FILE,
        UDP_TEXT,
        SERIAL_TEXT,
    };

    struct Config {
        Mode mode{Mode::DISABLED};
        std::string csv_path;
        uint16_t udp_port{0};
        size_t max_batch_size{32};
    };

    explicit TDOAIngestor(Config cfg = {});
    ~TDOAIngestor();

    bool start();
    void stop();
    [[nodiscard]] bool running() const { return running_; }

    [[nodiscard]] std::optional<std::vector<TDOALocalizer::Measurement>> poll();
    [[nodiscard]] size_t visible_anchor_count() const;
    [[nodiscard]] double visibility_ratio(size_t total_anchor_count) const;

private:
    bool open_udp_socket();
    std::optional<std::vector<TDOALocalizer::Measurement>> poll_csv();
    std::optional<std::vector<TDOALocalizer::Measurement>> poll_udp();
    void mark_visible(uint32_t anchor_id);

    Config cfg_;
    bool running_{false};
    int udp_socket_{-1};
    size_t csv_cursor_{0};
    std::unordered_map<uint32_t, size_t> anchor_visibility_;
};

} // namespace drone::localization
