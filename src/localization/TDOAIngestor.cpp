#include "localization/TDOAIngestor.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <limits>
#include <sstream>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace drone::localization {

namespace {

std::string normalize_key(std::string key) {
    key.erase(std::remove_if(key.begin(), key.end(), [](unsigned char c) {
        return std::isspace(c) || c == '"' || c == '\'' || c == '{' || c == '}' || c == '[' || c == ']';
    }), key.end());
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return key;
}

std::string trim_token(std::string value) {
    value.erase(std::remove_if(value.begin(), value.end(), [](unsigned char c) {
        return c == '"' || c == '\'' || c == '{' || c == '}' || c == '[' || c == ']';
    }), value.end());

    const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) {
        return std::isspace(c);
    });
    const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) {
        return std::isspace(c);
    }).base();
    if (first >= last) {
        return {};
    }
    return std::string(first, last);
}

std::optional<uint32_t> parse_uint_token(const std::string& value) {
    try {
        return static_cast<uint32_t>(std::stoul(trim_token(value)));
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<double> parse_double_token(const std::string& value) {
    try {
        return std::stod(trim_token(value));
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<TDOALocalizer::Measurement> parse_key_value_measurement(std::string line) {
    std::replace(line.begin(), line.end(), ';', ',');
    std::replace(line.begin(), line.end(), '|', ',');
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

        const auto key = normalize_key(token.substr(0, eq));
        const auto value = trim_token(token.substr(eq + 1));
        if (key == "anchor" || key == "anchorid" || key == "anchor_id" || key == "id") {
            anchor_id = parse_uint_token(value);
        } else if (key == "time" || key == "timestamp" || key == "toa" ||
                   key == "rx" || key == "rx_time" || key == "arrival" ||
                   key == "arrivaltime" || key == "arrival_time" || key == "arrival_time_s") {
            arrival_time_s = parse_double_token(value);
        }
    }

    if (!anchor_id.has_value() || !arrival_time_s.has_value()) {
        return std::nullopt;
    }

    return TDOALocalizer::Measurement{*anchor_id, *arrival_time_s};
}

std::optional<TDOALocalizer::Measurement> parse_csv_measurement(std::string line) {
    if (line.empty() || line[0] == '#') {
        return std::nullopt;
    }
    std::replace(line.begin(), line.end(), ';', ',');
    std::stringstream ss(line);
    std::string anchor_text;
    std::string time_text;
    if (!std::getline(ss, anchor_text, ',') || !std::getline(ss, time_text, ',')) {
        return std::nullopt;
    }

    try {
        return TDOALocalizer::Measurement{
            static_cast<uint32_t>(std::stoul(anchor_text)),
            std::stod(time_text),
        };
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<TDOALocalizer::Measurement> parse_measurement_line(std::string line) {
    if (line.empty() || line[0] == '#') {
        return std::nullopt;
    }

    if (line.find('=') != std::string::npos ||
        line.find('{') != std::string::npos ||
        line.find(':') != std::string::npos) {
        if (auto parsed = parse_key_value_measurement(line)) {
            return parsed;
        }
    }

    return parse_csv_measurement(std::move(line));
}

} // namespace

TDOAIngestor::TDOAIngestor()
    : TDOAIngestor(Config{}) {}

TDOAIngestor::TDOAIngestor(Config cfg)
    : cfg_(std::move(cfg)) {}

TDOAIngestor::~TDOAIngestor() {
    stop();
}

bool TDOAIngestor::start() {
    if (cfg_.mode == Mode::DISABLED) {
        running_ = false;
        return false;
    }
    if (cfg_.mode == Mode::UDP_TEXT) {
        running_ = open_udp_socket();
        return running_;
    }
    running_ = true;
    return true;
}

void TDOAIngestor::stop() {
    running_ = false;
#ifdef _WIN32
    if (udp_socket_ >= 0) {
        closesocket(static_cast<SOCKET>(udp_socket_));
    }
#else
    if (udp_socket_ >= 0) {
        close(udp_socket_);
    }
#endif
    udp_socket_ = -1;
}

std::optional<std::vector<TDOALocalizer::Measurement>> TDOAIngestor::poll() {
    if (!running_) {
        return std::nullopt;
    }
    switch (cfg_.mode) {
    case Mode::CSV_FILE:
        return poll_csv();
    case Mode::UDP_TEXT:
        return poll_udp();
    case Mode::DISABLED:
    default:
        return std::nullopt;
    }
}

size_t TDOAIngestor::visible_anchor_count() const {
    return anchor_visibility_.size();
}

double TDOAIngestor::visibility_ratio(size_t total_anchor_count) const {
    if (total_anchor_count == 0) {
        return 0.0;
    }
    return static_cast<double>(visible_anchor_count()) / static_cast<double>(total_anchor_count);
}

std::vector<uint32_t> TDOAIngestor::visible_anchor_ids() const {
    std::vector<uint32_t> ids;
    ids.reserve(anchor_visibility_.size());
    for (const auto& [anchor_id, _count] : anchor_visibility_) {
        ids.push_back(anchor_id);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

bool TDOAIngestor::open_udp_socket() {
#ifdef _WIN32
    WSADATA wsa_data;
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
        return false;
    }
#endif
    udp_socket_ = static_cast<int>(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
    if (udp_socket_ < 0) {
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(cfg_.udp_port);
    if (bind(udp_socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        stop();
        return false;
    }

#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(static_cast<SOCKET>(udp_socket_), FIONBIO, &mode);
#else
    fcntl(udp_socket_, F_SETFL, O_NONBLOCK);
#endif
    return true;
}

std::optional<std::vector<TDOALocalizer::Measurement>> TDOAIngestor::poll_csv() {
    std::ifstream input(cfg_.csv_path);
    if (!input.is_open()) {
        return std::nullopt;
    }

    std::vector<TDOALocalizer::Measurement> batch;
    std::string line;
    size_t line_index = 0;
    while (std::getline(input, line)) {
        if (line_index++ < csv_cursor_) {
            continue;
        }
        auto measurement = parse_measurement_line(line);
        ++csv_cursor_;
        if (!measurement.has_value()) {
            continue;
        }
        mark_visible(measurement->anchor_id);
        batch.push_back(*measurement);
        if (batch.size() >= cfg_.max_batch_size) {
            break;
        }
    }

    if (batch.size() < 4) {
        return std::nullopt;
    }
    last_batch_timestamp_s_ = batch.back().arrival_time_s;
    return batch;
}

std::optional<std::vector<TDOALocalizer::Measurement>> TDOAIngestor::poll_udp() {
    std::vector<TDOALocalizer::Measurement> batch;
    char buffer[2048];
    while (batch.size() < cfg_.max_batch_size) {
        const int received = static_cast<int>(recv(udp_socket_, buffer, sizeof(buffer) - 1, 0));
        if (received <= 0) {
            break;
        }
        buffer[received] = '\0';
        std::stringstream ss(buffer);
        std::string line;
        while (std::getline(ss, line) && batch.size() < cfg_.max_batch_size) {
            auto measurement = parse_measurement_line(line);
            if (!measurement.has_value()) {
                const auto parsed_inline = parse_key_value_measurement(line);
                if (parsed_inline.has_value()) {
                    measurement = parsed_inline;
                }
            }
            if (!measurement.has_value()) {
                continue;
            }
            mark_visible(measurement->anchor_id);
            batch.push_back(*measurement);
        }
    }

    if (batch.size() < 4) {
        return std::nullopt;
    }
    last_batch_timestamp_s_ = batch.back().arrival_time_s;
    return batch;
}

void TDOAIngestor::mark_visible(uint32_t anchor_id) {
    anchor_visibility_[anchor_id] += 1;
}

} // namespace drone::localization
