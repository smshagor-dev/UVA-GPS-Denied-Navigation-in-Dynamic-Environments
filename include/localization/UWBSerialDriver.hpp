#pragma once

#include "localization/TDOALocalizer.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace drone::localization {

class UWBSerialDriver {
public:
    struct Config {
        std::string device_path;
        uint32_t baud_rate{115200};
        size_t max_batch_size{32};
    };

    explicit UWBSerialDriver(Config cfg = {});
    ~UWBSerialDriver();

    bool start();
    void stop();
    [[nodiscard]] bool running() const { return running_; }
    [[nodiscard]] std::optional<std::vector<TDOALocalizer::Measurement>> poll();
    [[nodiscard]] double last_batch_timestamp_s() const { return last_batch_timestamp_s_; }

private:
    Config cfg_;
    bool running_{false};
    int handle_{-1};
    std::string buffered_text_;
    double last_batch_timestamp_s_{0.0};
};

} // namespace drone::localization
