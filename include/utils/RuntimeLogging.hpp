// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace drone::utils {

inline std::shared_ptr<spdlog::logger> get_or_create_logger(const std::string& name) {
    if (auto existing = spdlog::get(name)) {
        return existing;
    }

    auto base = spdlog::default_logger();
    if (!base) {
        return spdlog::default_logger();
    }

    auto logger =
        std::make_shared<spdlog::logger>(name, base->sinks().begin(), base->sinks().end());
    logger->set_level(base->level());
    spdlog::register_logger(logger);
    return logger;
}

} // namespace drone::utils
