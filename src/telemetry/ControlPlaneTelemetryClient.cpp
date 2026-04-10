// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "telemetry/ControlPlaneTelemetryClient.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string_view>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <winhttp.h>
#pragma comment(lib, "winhttp.lib")
#endif

namespace drone::telemetry {

namespace {

std::wstring widen(std::string_view value) {
    if (value.empty()) {
        return {};
    }
    const int size = MultiByteToWideChar(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), nullptr, 0);
    if (size <= 0) {
        return {};
    }
    std::wstring out(static_cast<size_t>(size), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), out.data(), size);
    return out;
}

std::string json_escape(std::string_view value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (const unsigned char c : value) {
        switch (c) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (c < 0x20) {
                std::ostringstream hex;
                hex << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
                out += hex.str();
            } else {
                out.push_back(static_cast<char>(c));
            }
            break;
        }
    }
    return out;
}

std::string format_double(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4) << value;
    return oss.str();
}

std::string format_timestamp_utc() {
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
#ifdef _WIN32
    gmtime_s(&utc_tm, &time);
#else
    gmtime_r(&time, &utc_tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

template <typename Vector3>
std::string json_vec3(const Vector3& value) {
    return "[" + format_double(value.x()) + "," + format_double(value.y()) + "," + format_double(value.z()) + "]";
}

std::string json_string_array(const std::vector<std::string>& values) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            oss << ",";
        }
        oss << "\"" << json_escape(values[i]) << "\"";
    }
    oss << "]";
    return oss.str();
}

} // namespace

ControlPlaneTelemetryClient::ControlPlaneTelemetryClient(std::string backend_url, int interval_ms)
    : endpoint_(parse_backend_url(backend_url)),
      interval_(std::chrono::milliseconds(std::max(interval_ms, 200))) {
    enabled_ = !backend_url.empty();
    if (enabled_) {
        last_status_ = "ready";
    }
}

bool ControlPlaneTelemetryClient::enabled() const {
    return enabled_;
}

bool ControlPlaneTelemetryClient::should_publish(std::chrono::steady_clock::time_point now) const {
    if (!enabled_) {
        return false;
    }
    if (last_attempt_ == std::chrono::steady_clock::time_point{}) {
        return true;
    }
    return (now - last_attempt_) >= interval_;
}

bool ControlPlaneTelemetryClient::publish(const TelemetrySnapshot& snapshot,
                                          std::chrono::steady_clock::time_point now) {
    if (!enabled_) {
        return false;
    }

    const std::string body = build_payload(snapshot);

#ifdef _WIN32
    auto host = widen(endpoint_.host);
    auto path = widen(endpoint_.path);
    HINTERNET session = WinHttpOpen(L"drone_swarm/telemetry-client",
                                    WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
                                    WINHTTP_NO_PROXY_NAME,
                                    WINHTTP_NO_PROXY_BYPASS,
                                    0);
    if (!session) {
        mark_result(false, "WinHttpOpen failed", now);
        return false;
    }

    HINTERNET connect = WinHttpConnect(session, host.c_str(), endpoint_.port, 0);
    if (!connect) {
        WinHttpCloseHandle(session);
        mark_result(false, "WinHttpConnect failed", now);
        return false;
    }

    DWORD flags = endpoint_.https ? WINHTTP_FLAG_SECURE : 0;
    HINTERNET request = WinHttpOpenRequest(connect,
                                           L"POST",
                                           path.c_str(),
                                           nullptr,
                                           WINHTTP_NO_REFERER,
                                           WINHTTP_DEFAULT_ACCEPT_TYPES,
                                           flags);
    if (!request) {
        WinHttpCloseHandle(connect);
        WinHttpCloseHandle(session);
        mark_result(false, "WinHttpOpenRequest failed", now);
        return false;
    }

    static constexpr wchar_t kHeaders[] = L"Content-Type: application/json\r\n";
    const BOOL sent = WinHttpSendRequest(request,
                                         kHeaders,
                                         static_cast<DWORD>(std::wcslen(kHeaders)),
                                         const_cast<char*>(body.data()),
                                         static_cast<DWORD>(body.size()),
                                         static_cast<DWORD>(body.size()),
                                         0);
    if (!sent || !WinHttpReceiveResponse(request, nullptr)) {
        WinHttpCloseHandle(request);
        WinHttpCloseHandle(connect);
        WinHttpCloseHandle(session);
        mark_result(false, "WinHTTP send/receive failed", now);
        return false;
    }

    DWORD status_code = 0;
    DWORD size = sizeof(status_code);
    const BOOL queried = WinHttpQueryHeaders(request,
                                             WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
                                             WINHTTP_HEADER_NAME_BY_INDEX,
                                             &status_code,
                                             &size,
                                             WINHTTP_NO_HEADER_INDEX);

    WinHttpCloseHandle(request);
    WinHttpCloseHandle(connect);
    WinHttpCloseHandle(session);

    if (!queried) {
        mark_result(false, "WinHTTP status query failed", now);
        return false;
    }
    const bool ok = status_code >= 200 && status_code < 300;
    mark_result(ok, ok ? ("telemetry accepted status=" + std::to_string(status_code))
                       : ("telemetry rejected status=" + std::to_string(status_code)),
                now);
    return ok;
#else
    (void) snapshot;
    mark_result(false, "telemetry client unsupported on this platform build", now);
    return false;
#endif
}

ControlPlaneTelemetryClient::ParsedEndpoint ControlPlaneTelemetryClient::parse_backend_url(const std::string& backend_url) {
    ParsedEndpoint endpoint;
    if (backend_url.empty()) {
        return endpoint;
    }

    std::string url = backend_url;
    if (const auto pos = url.find("://"); pos != std::string::npos) {
        endpoint.https = url.substr(0, pos) == "https";
        url.erase(0, pos + 3);
    }
    const auto slash = url.find('/');
    if (slash != std::string::npos) {
        url.erase(slash);
    }
    const auto colon = url.rfind(':');
    if (colon != std::string::npos && colon + 1 < url.size()) {
        endpoint.host = url.substr(0, colon);
        try {
            endpoint.port = static_cast<uint16_t>(std::stoul(url.substr(colon + 1)));
        } catch (const std::exception&) {
            endpoint.port = endpoint.https ? 443 : 80;
        }
    } else if (!url.empty()) {
        endpoint.host = url;
        endpoint.port = endpoint.https ? 443 : 80;
    }
    return endpoint;
}

std::string ControlPlaneTelemetryClient::build_payload(const TelemetrySnapshot& snapshot) {
    std::ostringstream oss;
    oss << "{"
        << "\"drone_id\":" << snapshot.drone_id << ","
        << "\"cluster_id\":\"" << json_escape(snapshot.cluster_id) << "\","
        << "\"role\":\"" << json_escape(snapshot.role) << "\","
        << "\"connectivity\":\"" << json_escape(snapshot.connectivity) << "\","
        << "\"reachable\":" << (snapshot.reachable ? "true" : "false") << ","
        << "\"position\":" << json_vec3(snapshot.position) << ","
        << "\"velocity\":" << json_vec3(snapshot.velocity) << ","
        << "\"attitude_rpy\":" << json_vec3(snapshot.attitude_rpy) << ","
        << "\"thrust_vector\":" << json_vec3(snapshot.thrust_vector) << ","
        << "\"commanded_altitude_m\":" << format_double(snapshot.commanded_altitude_m) << ","
        << "\"commanded_speed_mps\":" << format_double(snapshot.commanded_speed_mps) << ","
        << "\"drift_m\":" << format_double(snapshot.drift_m) << ","
        << "\"battery_pct\":" << format_double(snapshot.battery_pct) << ","
        << "\"rssi_dbm\":" << format_double(snapshot.rssi_dbm) << ","
        << "\"cpu_temp_c\":" << format_double(snapshot.cpu_temp_c) << ","
        << "\"gpu_load_pct\":" << format_double(snapshot.gpu_load_pct) << ","
        << "\"mission_state\":\"" << json_escape(snapshot.mission_state) << "\","
        << "\"localization_source\":\"" << json_escape(snapshot.localization_source) << "\","
        << "\"localization_state\":\"" << json_escape(snapshot.localization_state) << "\","
        << "\"localization_confidence\":" << format_double(snapshot.localization_confidence) << ","
        << "\"tdoa_confidence\":" << format_double(snapshot.tdoa_confidence) << ","
        << "\"confidence_trend\":" << format_double(snapshot.confidence_trend) << ","
        << "\"relocalization_count\":" << snapshot.relocalization_count << ","
        << "\"visible_anchor_count\":" << snapshot.visible_anchor_count << ","
        << "\"occupancy_ratio\":" << format_double(snapshot.occupancy_ratio) << ","
        << "\"sync_confidence\":" << format_double(snapshot.sync_confidence) << ","
        << "\"imu_camera_offset_ms\":" << format_double(snapshot.imu_camera_offset_ms) << ","
        << "\"security_state\":\"" << json_escape(snapshot.security_state) << "\","
        << "\"security_summary\":\"" << json_escape(snapshot.security_summary) << "\","
        << "\"remote_command_allowed\":" << (snapshot.remote_command_allowed ? "true" : "false") << ","
        << "\"telemetry_uplink_allowed\":" << (snapshot.telemetry_uplink_allowed ? "true" : "false") << ","
        << "\"link_integrity_score\":" << format_double(snapshot.link_integrity_score) << ","
        << "\"health_flags\":" << json_string_array(snapshot.health_flags) << ","
        << "\"timestamp\":\"" << format_timestamp_utc() << "\""
        << "}";
    return oss.str();
}

void ControlPlaneTelemetryClient::mark_result(bool ok,
                                              std::string status,
                                              std::chrono::steady_clock::time_point now) {
    last_attempt_ = now;
    last_status_ = ok ? std::move(status) : ("error: " + status);
}

} // namespace drone::telemetry
