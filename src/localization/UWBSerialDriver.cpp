#include "localization/UWBSerialDriver.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace drone::localization {

namespace {

std::optional<TDOALocalizer::Measurement> parse_serial_measurement(std::string line) {
    std::replace(line.begin(), line.end(), ';', ',');
    std::replace(line.begin(), line.end(), ':', '=');

    std::stringstream ss(line);
    std::string token;
    std::optional<uint32_t> anchor_id;
    std::optional<double> arrival_time_s;

    while (std::getline(ss, token, ',')) {
        const auto eq = token.find('=');
        if (eq == std::string::npos) {
            continue;
        }
        auto key = token.substr(0, eq);
        auto value = token.substr(eq + 1);
        key.erase(std::remove_if(key.begin(), key.end(), ::isspace), key.end());
        value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());
        std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });

        try {
            if (key == "anchor" || key == "anchor_id" || key == "anchorid" || key == "id") {
                anchor_id = static_cast<uint32_t>(std::stoul(value));
            } else if (key == "arrival_time_s" || key == "toa" || key == "timestamp" || key == "rx_time") {
                arrival_time_s = std::stod(value);
            }
        } catch (...) {
            return std::nullopt;
        }
    }

    if (!anchor_id.has_value() || !arrival_time_s.has_value()) {
        return std::nullopt;
    }
    return TDOALocalizer::Measurement{*anchor_id, *arrival_time_s};
}

} // namespace

UWBSerialDriver::UWBSerialDriver(Config cfg)
    : cfg_(std::move(cfg)) {}

UWBSerialDriver::~UWBSerialDriver() {
    stop();
}

bool UWBSerialDriver::start() {
    if (cfg_.device_path.empty()) {
        return false;
    }

#ifdef _WIN32
    const auto wide_path = std::wstring(cfg_.device_path.begin(), cfg_.device_path.end());
    HANDLE handle = CreateFileW(
        wide_path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
        return false;
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle, &dcb)) {
        CloseHandle(handle);
        return false;
    }
    dcb.BaudRate = cfg_.baud_rate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    if (!SetCommState(handle, &dcb)) {
        CloseHandle(handle);
        return false;
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = 10;
    timeouts.ReadTotalTimeoutConstant = 10;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    SetCommTimeouts(handle, &timeouts);

    handle_ = static_cast<int>(reinterpret_cast<intptr_t>(handle));
#else
    handle_ = ::open(cfg_.device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (handle_ < 0) {
        return false;
    }

    termios tty{};
    if (tcgetattr(handle_, &tty) != 0) {
        stop();
        return false;
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tcsetattr(handle_, TCSANOW, &tty);
#endif

    running_ = true;
    return true;
}

void UWBSerialDriver::stop() {
    running_ = false;
#ifdef _WIN32
    if (handle_ >= 0) {
        CloseHandle(reinterpret_cast<HANDLE>(static_cast<intptr_t>(handle_)));
    }
#else
    if (handle_ >= 0) {
        ::close(handle_);
    }
#endif
    handle_ = -1;
    buffered_text_.clear();
}

std::optional<std::vector<TDOALocalizer::Measurement>> UWBSerialDriver::poll() {
    if (!running_ || handle_ < 0) {
        return std::nullopt;
    }

    char buffer[512];
    int bytes_read = 0;
#ifdef _WIN32
    DWORD win_read = 0;
    if (!ReadFile(reinterpret_cast<HANDLE>(static_cast<intptr_t>(handle_)), buffer, sizeof(buffer) - 1, &win_read, nullptr)) {
        return std::nullopt;
    }
    bytes_read = static_cast<int>(win_read);
#else
    bytes_read = static_cast<int>(::read(handle_, buffer, sizeof(buffer) - 1));
    if (bytes_read < 0) {
        return std::nullopt;
    }
#endif

    if (bytes_read <= 0) {
        return std::nullopt;
    }

    buffer[bytes_read] = '\0';
    buffered_text_.append(buffer, static_cast<size_t>(bytes_read));

    std::vector<TDOALocalizer::Measurement> batch;
    size_t newline_pos = std::string::npos;
    while ((newline_pos = buffered_text_.find('\n')) != std::string::npos &&
           batch.size() < cfg_.max_batch_size) {
        auto line = buffered_text_.substr(0, newline_pos);
        buffered_text_.erase(0, newline_pos + 1);
        auto measurement = parse_serial_measurement(line);
        if (measurement.has_value()) {
            batch.push_back(*measurement);
        }
    }

    if (batch.size() < 4) {
        return std::nullopt;
    }
    return batch;
}

} // namespace drone::localization
