// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "telemetry/ControlPlaneTelemetryClient.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string_view>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <winhttp.h>
#include <wincrypt.h>
#include <ncrypt.h>
#pragma comment(lib, "winhttp.lib")
#pragma comment(lib, "crypt32.lib")
#pragma comment(lib, "ncrypt.lib")
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#endif

namespace drone::telemetry {

namespace {

#ifdef _WIN32
std::wstring widen(std::string_view value) {
    if (value.empty()) {
        return {};
    }
    const int size =
        MultiByteToWideChar(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), nullptr, 0);
    if (size <= 0) {
        return {};
    }
    std::wstring out(static_cast<size_t>(size), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), out.data(), size);
    return out;
}
#endif

std::string json_escape(std::string_view value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (const unsigned char c : value) {
        switch (c) {
        case '\\':
            out += "\\\\";
            break;
        case '"':
            out += "\\\"";
            break;
        case '\b':
            out += "\\b";
            break;
        case '\f':
            out += "\\f";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (c < 0x20) {
                std::ostringstream hex;
                hex << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                    << static_cast<int>(c);
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

std::string read_env(const char* key) {
    if (key == nullptr) {
        return {};
    }
#ifdef _WIN32
    char* buffer = nullptr;
    size_t size = 0;
    if (_dupenv_s(&buffer, &size, key) == 0 && buffer != nullptr) {
        std::string value(buffer);
        free(buffer);
        return value;
    }
#else
    if (const char* value = std::getenv(key)) {
        return value;
    }
#endif
    return {};
}

bool parse_bool_env(const char* key, bool fallback) {
    std::string value = read_env(key);
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "1" || value == "true" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "off") {
        return false;
    }
    return fallback;
}

template <typename Vector3> std::string json_vec3(const Vector3& value) {
    return "[" + format_double(value.x()) + "," + format_double(value.y()) + "," +
           format_double(value.z()) + "]";
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

#ifdef _WIN32
struct CertStoreCloser {
    void operator()(HCERTSTORE store) const noexcept {
        if (store != nullptr) {
            CertCloseStore(store, 0);
        }
    }
};

struct CertContextDeleter {
    void operator()(PCCERT_CONTEXT ctx) const noexcept {
        if (ctx != nullptr) {
            CertFreeCertificateContext(ctx);
        }
    }
};

struct ChainEngineCloser {
    void operator()(HCERTCHAINENGINE engine) const noexcept {
        if (engine != nullptr) {
            CertFreeCertificateChainEngine(engine);
        }
    }
};

struct ChainContextCloser {
    void operator()(PCCERT_CHAIN_CONTEXT chain) const noexcept {
        if (chain != nullptr) {
            CertFreeCertificateChain(chain);
        }
    }
};

using unique_cert_store = std::unique_ptr<void, CertStoreCloser>;
using unique_cert_context = std::unique_ptr<const CERT_CONTEXT, CertContextDeleter>;
using unique_chain_engine = std::unique_ptr<void, ChainEngineCloser>;
using unique_chain_context = std::unique_ptr<const CERT_CHAIN_CONTEXT, ChainContextCloser>;

std::string win32_error_message(DWORD code) {
    LPSTR buffer = nullptr;
    const DWORD flags =
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS;
    const DWORD size =
        FormatMessageA(flags, nullptr, code, 0, reinterpret_cast<LPSTR>(&buffer), 0, nullptr);
    if (size == 0 || buffer == nullptr) {
        return "error code " + std::to_string(code);
    }
    std::string out(buffer, size);
    LocalFree(buffer);
    while (!out.empty() && (out.back() == '\r' || out.back() == '\n' || out.back() == ' ')) {
        out.pop_back();
    }
    return out;
}

unique_cert_context load_client_certificate_from_pfx(const std::string& pfx_path,
                                                     const std::string& password,
                                                     std::string& error) {
    if (pfx_path.empty()) {
        error = "DRONE_TLS_CLIENT_PFX_FILE is not configured";
        return {};
    }

    HANDLE file = CreateFileW(widen(pfx_path).c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                              OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file == INVALID_HANDLE_VALUE) {
        error = "failed to open client PFX: " + win32_error_message(GetLastError());
        return {};
    }

    LARGE_INTEGER file_size{};
    if (!GetFileSizeEx(file, &file_size) || file_size.QuadPart <= 0) {
        CloseHandle(file);
        error = "failed to read client PFX size";
        return {};
    }

    std::vector<std::byte> buffer(static_cast<size_t>(file_size.QuadPart));
    DWORD bytes_read = 0;
    const BOOL read_ok =
        ReadFile(file, buffer.data(), static_cast<DWORD>(buffer.size()), &bytes_read, nullptr);
    CloseHandle(file);
    if (!read_ok || bytes_read != buffer.size()) {
        error = "failed to read client PFX contents";
        return {};
    }

    CRYPT_DATA_BLOB blob{};
    blob.pbData = reinterpret_cast<BYTE*>(buffer.data());
    blob.cbData = bytes_read;

    unique_cert_store store(PFXImportCertStore(&blob, widen(password).c_str(),
                                               CRYPT_USER_KEYSET | PKCS12_ALLOW_OVERWRITE_KEY),
                            CertStoreCloser{});
    if (!store) {
        error = "failed to import client PFX: " + win32_error_message(GetLastError());
        return {};
    }

    PCCERT_CONTEXT found = nullptr;
    while ((found = CertFindCertificateInStore(static_cast<HCERTSTORE>(store.get()),
                                               X509_ASN_ENCODING | PKCS_7_ASN_ENCODING, 0,
                                               CERT_FIND_ANY, nullptr, found)) != nullptr) {
        DWORD key_spec = 0;
        BOOL must_free = FALSE;
        HCRYPTPROV_OR_NCRYPT_KEY_HANDLE key_handle = 0;
        if (CryptAcquireCertificatePrivateKey(found, CRYPT_ACQUIRE_SILENT_FLAG, nullptr,
                                              &key_handle, &key_spec, &must_free)) {
            if (must_free) {
                if (key_spec == CERT_NCRYPT_KEY_SPEC) {
                    NCryptFreeObject(key_handle);
                } else {
                    CryptReleaseContext(static_cast<HCRYPTPROV>(key_handle), 0);
                }
            }
            return unique_cert_context(CertDuplicateCertificateContext(found));
        }
    }

    error = "no client certificate with private key found in PFX";
    return {};
}

unique_cert_store load_ca_store_from_file(const std::string& ca_file, std::string& error) {
    if (ca_file.empty()) {
        error = "DRONE_TLS_CA_FILE is not configured";
        return unique_cert_store(nullptr, CertStoreCloser{});
    }

    HCERTSTORE store = nullptr;
    PCCERT_CONTEXT cert_context = nullptr;
    DWORD encoding = 0;
    DWORD content_type = 0;
    DWORD format_type = 0;
    if (!CryptQueryObject(CERT_QUERY_OBJECT_FILE, widen(ca_file).c_str(),
                          CERT_QUERY_CONTENT_FLAG_CERT, CERT_QUERY_FORMAT_FLAG_ALL, 0, &encoding,
                          &content_type, &format_type, &store, nullptr,
                          reinterpret_cast<const void**>(&cert_context))) {
        error = "failed to parse DRONE_TLS_CA_FILE: " + win32_error_message(GetLastError());
        return unique_cert_store(nullptr, CertStoreCloser{});
    }
    if (cert_context != nullptr) {
        CertFreeCertificateContext(cert_context);
    }
    return unique_cert_store(store, CertStoreCloser{});
}

bool validate_server_certificate(
    HINTERNET request, const std::string& host,
    const drone::telemetry::ControlPlaneTelemetryClient::TLSRuntimeConfig& tls,
    std::string& error) {
    if (tls.skip_verify) {
        return true;
    }

    DWORD size = 0;
    WinHttpQueryOption(request, WINHTTP_OPTION_SERVER_CERT_CONTEXT, nullptr, &size);
    if (size == 0) {
        error = "failed to query server certificate size";
        return false;
    }

    std::vector<std::byte> cert_buffer(size);
    if (!WinHttpQueryOption(request, WINHTTP_OPTION_SERVER_CERT_CONTEXT, cert_buffer.data(),
                            &size)) {
        error = "failed to query server certificate: " + win32_error_message(GetLastError());
        return false;
    }
    PCCERT_CONTEXT raw_server_cert = *reinterpret_cast<PCCERT_CONTEXT*>(cert_buffer.data());
    unique_cert_context server_cert(raw_server_cert);
    if (!server_cert) {
        error = "server certificate context was empty";
        return false;
    }

    std::string ca_error;
    unique_cert_store ca_store = load_ca_store_from_file(tls.ca_file, ca_error);
    if (!ca_store) {
        error = ca_error;
        return false;
    }

    CERT_CHAIN_ENGINE_CONFIG engine_cfg{};
    engine_cfg.cbSize = sizeof(engine_cfg);
    engine_cfg.hExclusiveRoot = static_cast<HCERTSTORE>(ca_store.get());

    HCERTCHAINENGINE raw_engine = nullptr;
    if (!CertCreateCertificateChainEngine(&engine_cfg, &raw_engine)) {
        error = "failed to create certificate chain engine: " + win32_error_message(GetLastError());
        return false;
    }
    unique_chain_engine chain_engine(raw_engine, ChainEngineCloser{});

    CERT_CHAIN_PARA chain_para{};
    chain_para.cbSize = sizeof(chain_para);
    PCCERT_CHAIN_CONTEXT raw_chain = nullptr;
    if (!CertGetCertificateChain(static_cast<HCERTCHAINENGINE>(chain_engine.get()),
                                 server_cert.get(), nullptr, server_cert->hCertStore, &chain_para,
                                 0, nullptr, &raw_chain)) {
        error = "failed to build server certificate chain: " + win32_error_message(GetLastError());
        return false;
    }
    unique_chain_context chain(raw_chain, ChainContextCloser{});

    HTTPSPolicyCallbackData ssl_policy{};
    ssl_policy.cbStruct = sizeof(ssl_policy);
    ssl_policy.dwAuthType = AUTHTYPE_SERVER;
    const auto wide_host = widen(host);
    ssl_policy.pwszServerName = const_cast<LPWSTR>(wide_host.c_str());

    CERT_CHAIN_POLICY_PARA policy_para{};
    policy_para.cbSize = sizeof(policy_para);
    policy_para.pvExtraPolicyPara = &ssl_policy;

    CERT_CHAIN_POLICY_STATUS policy_status{};
    policy_status.cbSize = sizeof(policy_status);
    if (!CertVerifyCertificateChainPolicy(CERT_CHAIN_POLICY_SSL, chain.get(), &policy_para,
                                          &policy_status)) {
        error =
            "failed to verify server certificate policy: " + win32_error_message(GetLastError());
        return false;
    }
    if (policy_status.dwError != 0) {
        error = "server certificate policy rejected backend: " +
                win32_error_message(policy_status.dwError);
        return false;
    }
    return true;
}
#endif

std::string json_double_array(const std::vector<double>& values) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            oss << ",";
        }
        oss << format_double(values[i]);
    }
    oss << "]";
    return oss.str();
}

std::string json_sensor_vec3(const SensorVector3& value) {
    return std::string("{") + "\"x\":" + format_double(value.x) + "," +
           "\"y\":" + format_double(value.y) + "," + "\"z\":" + format_double(value.z) + "}";
}

std::string json_camera_payload(const CameraTelemetryPayload& camera) {
    std::ostringstream oss;
    oss << "{"
        << "\"status\":\"" << json_escape(camera.status) << "\","
        << "\"fps\":" << format_double(camera.fps) << ","
        << "\"frame_age_ms\":" << format_double(camera.frame_age_ms) << ","
        << "\"resolution\":\"" << json_escape(camera.resolution) << "\","
        << "\"dropped_frames\":" << camera.dropped_frames << ","
        << "\"source\":\"" << json_escape(camera.source) << "\","
        << "\"preview_url\":\"" << json_escape(camera.preview_url) << "\","
        << "\"latest_frame_ref\":\"" << json_escape(camera.latest_frame_ref) << "\""
        << "}";
    return oss.str();
}

std::string json_imu_payload(const IMUTelemetryPayload& imu) {
    std::ostringstream oss;
    oss << "{"
        << "\"status\":\"" << json_escape(imu.status) << "\","
        << "\"sample_rate_hz\":" << format_double(imu.sample_rate_hz) << ","
        << "\"last_sample_age_ms\":" << format_double(imu.last_sample_age_ms) << ","
        << "\"accel\":" << json_sensor_vec3(imu.accel) << ","
        << "\"gyro\":" << json_sensor_vec3(imu.gyro) << ","
        << "\"health\":\"" << json_escape(imu.health) << "\","
        << "\"source\":\"" << json_escape(imu.source) << "\""
        << "}";
    return oss.str();
}

std::string json_lidar_payload(const LidarTelemetryPayload& lidar) {
    std::ostringstream points;
    points << "[";
    for (size_t i = 0; i < lidar.points_2d.size(); ++i) {
        if (i > 0) {
            points << ",";
        }
        const auto& point = lidar.points_2d[i];
        points << "{"
               << "\"x\":" << format_double(point.x) << ","
               << "\"y\":" << format_double(point.y) << ","
               << "\"intensity\":" << format_double(point.intensity) << "}";
    }
    points << "]";
    std::ostringstream oss;
    oss << "{"
        << "\"status\":\"" << json_escape(lidar.status) << "\","
        << "\"packet_rate_hz\":" << format_double(lidar.packet_rate_hz) << ","
        << "\"scan_age_ms\":" << format_double(lidar.scan_age_ms) << ","
        << "\"point_count\":" << lidar.point_count << ","
        << "\"points_2d\":" << points.str() << ","
        << "\"min_range_m\":" << format_double(lidar.min_range_m) << ","
        << "\"max_range_m\":" << format_double(lidar.max_range_m) << ","
        << "\"source\":\"" << json_escape(lidar.source) << "\""
        << "}";
    return oss.str();
}

std::string json_tdoa_payload(const TDOATelemetryPayload& tdoa) {
    std::ostringstream anchors;
    anchors << "[";
    for (size_t i = 0; i < tdoa.anchors.size(); ++i) {
        if (i > 0) {
            anchors << ",";
        }
        const auto& anchor = tdoa.anchors[i];
        anchors << "{"
                << "\"id\":\"" << json_escape(anchor.id) << "\","
                << "\"x\":" << format_double(anchor.x) << ","
                << "\"y\":" << format_double(anchor.y) << ","
                << "\"z\":" << format_double(anchor.z) << ","
                << "\"visible\":" << (anchor.visible ? "true" : "false") << ","
                << "\"last_seen_ms\":" << format_double(anchor.last_seen_ms) << "}";
    }
    anchors << "]";
    std::ostringstream oss;
    oss << "{"
        << "\"status\":\"" << json_escape(tdoa.status) << "\","
        << "\"source\":\"" << json_escape(tdoa.source) << "\","
        << "\"visible_anchor_count\":" << tdoa.visible_anchor_count << ","
        << "\"anchors\":" << anchors.str() << ","
        << "\"estimated_position\":" << json_sensor_vec3(tdoa.estimated_position) << ","
        << "\"calibration_warning\":\"" << json_escape(tdoa.calibration_warning) << "\""
        << "}";
    return oss.str();
}

std::string json_replay_payload(const ReplayTelemetryPayload& replay) {
    std::ostringstream oss;
    oss << "{"
        << "\"status\":\"" << json_escape(replay.status) << "\","
        << "\"active\":" << (replay.active ? "true" : "false") << ","
        << "\"file_name\":\"" << json_escape(replay.file_name) << "\","
        << "\"progress\":" << format_double(replay.progress) << ","
        << "\"current_time\":" << format_double(replay.current_time) << ","
        << "\"confidence_series\":" << json_double_array(replay.confidence_series) << ","
        << "\"source\":\"" << json_escape(replay.source) << "\""
        << "}";
    return oss.str();
}

#ifndef _WIN32
struct SocketGuard {
    int fd{-1};
    ~SocketGuard() {
        if (fd >= 0) {
            close(fd);
        }
    }
};

bool set_nonblocking(int fd, bool enabled) {
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    const int updated = enabled ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
    return fcntl(fd, F_SETFL, updated) == 0;
}

bool wait_for_socket(int fd, bool write_ready, int timeout_ms) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    timeval timeout{};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    const int ready = select(fd + 1, write_ready ? nullptr : &fds, write_ready ? &fds : nullptr,
                             nullptr, &timeout);
    return ready > 0;
}

ControlPlaneTelemetryClient::HttpResponse
send_http_posix(const ControlPlaneTelemetryClient::ParsedEndpoint& endpoint, std::string_view body,
                const ControlPlaneTelemetryClient::HeaderList& headers, int timeout_ms) {
    if (endpoint.https) {
        return {false, 0, "https unsupported in minimal posix telemetry client"};
    }

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    addrinfo* result = nullptr;
    const auto port_string = std::to_string(endpoint.port);
    if (getaddrinfo(endpoint.host.c_str(), port_string.c_str(), &hints, &result) != 0 || !result) {
        return {false, 0, "dns resolution failed"};
    }

    std::string request;
    request.reserve(body.size() + 512);
    request += "POST " + endpoint.path + " HTTP/1.1\r\n";
    request += "Host: " + endpoint.host + "\r\n";
    for (const auto& [name, value] : headers) {
        request += name + ": " + value + "\r\n";
    }
    request += "Content-Length: " + std::to_string(body.size()) + "\r\n";
    request += "Connection: close\r\n\r\n";
    request.append(body.begin(), body.end());

    ControlPlaneTelemetryClient::HttpResponse response{false, 0, "socket connect failed"};

    for (addrinfo* ai = result; ai != nullptr; ai = ai->ai_next) {
        SocketGuard socket_guard;
        socket_guard.fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (socket_guard.fd < 0) {
            continue;
        }

        set_nonblocking(socket_guard.fd, true);
        const int connect_result =
            connect(socket_guard.fd, ai->ai_addr, static_cast<socklen_t>(ai->ai_addrlen));
        if (connect_result != 0 && errno != EINPROGRESS) {
            response.status_text = "connect failed";
            continue;
        }
        if (!wait_for_socket(socket_guard.fd, true, timeout_ms)) {
            response.status_text = "connect timeout";
            continue;
        }

        int socket_error = 0;
        socklen_t socket_error_size = sizeof(socket_error);
        if (getsockopt(socket_guard.fd, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_size) !=
                0 ||
            socket_error != 0) {
            response.status_text = "connect error";
            continue;
        }

        set_nonblocking(socket_guard.fd, false);
        timeval timeout{};
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;
        setsockopt(socket_guard.fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        setsockopt(socket_guard.fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        size_t sent = 0;
        while (sent < request.size()) {
            const ssize_t wrote =
                send(socket_guard.fd, request.data() + sent, request.size() - sent, 0);
            if (wrote <= 0) {
                response.status_text = "send failed";
                break;
            }
            sent += static_cast<size_t>(wrote);
        }
        if (sent < request.size()) {
            continue;
        }

        std::string raw_response;
        char buffer[2048];
        while (true) {
            const ssize_t received = recv(socket_guard.fd, buffer, sizeof(buffer), 0);
            if (received <= 0) {
                break;
            }
            raw_response.append(buffer, buffer + received);
            if (raw_response.find("\r\n") != std::string::npos &&
                raw_response.find("\r\n\r\n") != std::string::npos) {
                break;
            }
        }
        if (raw_response.empty()) {
            response.status_text = "empty response";
            continue;
        }

        const auto line_end = raw_response.find("\r\n");
        const std::string status_line = raw_response.substr(0, line_end);
        std::istringstream status_stream(status_line);
        std::string http_version;
        status_stream >> http_version >> response.status_code;
        std::getline(status_stream, response.status_text);
        if (!response.status_text.empty() && response.status_text.front() == ' ') {
            response.status_text.erase(response.status_text.begin());
        }
        response.transport_ok = response.status_code > 0;
        break;
    }

    freeaddrinfo(result);
    return response;
}
#endif

} // namespace

ControlPlaneTelemetryClient::ControlPlaneTelemetryClient(std::string backend_url,
                                                         std::string auth_token, int interval_ms,
                                                         int timeout_ms, TransportFn transport)
    : endpoint_(parse_backend_url(backend_url)), tls_(load_tls_runtime_config()),
      interval_(std::chrono::milliseconds(std::max(interval_ms, 200))),
      timeout_(std::chrono::milliseconds(std::max(timeout_ms, 200))),
      auth_token_(std::move(auth_token)),
      transport_(transport ? std::move(transport)
                           : &ControlPlaneTelemetryClient::default_transport) {
    enabled_ = !backend_url.empty();
    if (enabled_) {
        last_status_ = "ready";
    }
}

ControlPlaneTelemetryClient::TLSRuntimeConfig
ControlPlaneTelemetryClient::load_tls_runtime_config() {
    TLSRuntimeConfig out;
    out.skip_verify = parse_bool_env("DRONE_TLS_SKIP_VERIFY", false);
    out.ca_file = read_env("DRONE_TLS_CA_FILE");
    out.client_pfx_file = read_env("DRONE_TLS_CLIENT_PFX_FILE");
    out.client_pfx_password = read_env("DRONE_TLS_CLIENT_PFX_PASSWORD");
    return out;
}

bool ControlPlaneTelemetryClient::enabled() const {
    return enabled_;
}

bool ControlPlaneTelemetryClient::should_publish(std::chrono::steady_clock::time_point now) const {
    std::lock_guard lock(state_mutex_);
    if (!enabled_) {
        return false;
    }
    if (now < next_retry_not_before_) {
        return false;
    }
    if (consecutive_failures_ > 0) {
        return true;
    }
    if (last_attempt_ == std::chrono::steady_clock::time_point{}) {
        return true;
    }
    return (now - last_attempt_) >= interval_;
}

bool ControlPlaneTelemetryClient::publish(const TelemetrySnapshot& snapshot,
                                          std::chrono::steady_clock::time_point now) {
    {
        std::lock_guard lock(state_mutex_);
        if (!enabled_) {
            return false;
        }
    }

    const std::string body = serialize_payload(snapshot);
    const auto headers = build_headers(auth_token_, snapshot.drone_id);
    const auto response = transport_(endpoint_, body, headers, static_cast<int>(timeout_.count()));
    const bool ok =
        response.transport_ok && response.status_code >= 200 && response.status_code < 300;
    mark_result(ok,
                ok ? ("telemetry accepted status=" + std::to_string(response.status_code))
                   : (response.status_code > 0
                          ? ("telemetry rejected status=" + std::to_string(response.status_code) +
                             " " + response.status_text)
                          : ("telemetry transport failed: " + response.status_text)),
                now);
    return ok;
}

std::string ControlPlaneTelemetryClient::last_status() const {
    std::lock_guard lock(state_mutex_);
    return last_status_;
}

int ControlPlaneTelemetryClient::consecutive_failures() const {
    std::lock_guard lock(state_mutex_);
    return consecutive_failures_;
}

ControlPlaneTelemetryClient::ParsedEndpoint
ControlPlaneTelemetryClient::parse_backend_url(const std::string& backend_url) {
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
        endpoint.path = url.substr(slash);
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
    if (endpoint.path.empty()) {
        endpoint.path = "/api/v1/telemetry";
    }
    return endpoint;
}

ControlPlaneTelemetryClient::HeaderList
ControlPlaneTelemetryClient::build_headers(std::string_view auth_token, uint32_t drone_id) {
    HeaderList headers;
    headers.emplace_back("Content-Type", "application/json");
    headers.emplace_back("X-Drone-Id", std::to_string(drone_id));
    if (!auth_token.empty()) {
        headers.emplace_back("Authorization", "Bearer " + std::string(auth_token));
        headers.emplace_back("X-Drone-Token", std::string(auth_token));
    }
    return headers;
}

std::string ControlPlaneTelemetryClient::serialize_payload(const TelemetrySnapshot& snapshot) {
    std::ostringstream oss;
    oss << "{"
        << "\"drone_id\":" << snapshot.drone_id << ","
        << "\"source\":\"" << json_escape(snapshot.source) << "\","
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
        << "\"localization_data_source\":\"" << json_escape(snapshot.localization_data_source)
        << "\","
        << "\"localization_state\":\"" << json_escape(snapshot.localization_state) << "\","
        << "\"localization_confidence\":" << format_double(snapshot.localization_confidence) << ","
        << "\"tdoa_confidence\":" << format_double(snapshot.tdoa_confidence) << ","
        << "\"confidence_trend\":" << format_double(snapshot.confidence_trend) << ","
        << "\"relocalization_count\":" << snapshot.relocalization_count << ","
        << "\"visible_anchor_count\":" << snapshot.visible_anchor_count << ","
        << "\"occupancy_ratio\":" << format_double(snapshot.occupancy_ratio) << ","
        << "\"sync_confidence\":" << format_double(snapshot.sync_confidence) << ","
        << "\"imu_camera_offset_ms\":" << format_double(snapshot.imu_camera_offset_ms) << ","
        << "\"peer_count\":" << snapshot.peer_count << ","
        << "\"stale_peer_count\":" << snapshot.stale_peer_count << ","
        << "\"mesh_topology_mode\":\"" << json_escape(snapshot.mesh_topology_mode) << "\","
        << "\"local_consensus_state\":\"" << json_escape(snapshot.local_consensus_state) << "\","
        << "\"local_consensus_epoch\":" << snapshot.local_consensus_epoch << ","
        << "\"peer_latency_ms\":" << format_double(snapshot.peer_latency_ms) << ","
        << "\"mesh_bandwidth_kbps\":" << format_double(snapshot.mesh_bandwidth_kbps) << ","
        << "\"edge_serialization_mode\":\"" << json_escape(snapshot.edge_serialization_mode)
        << "\","
        << "\"edge_average_packet_size_bytes\":"
        << format_double(snapshot.edge_average_packet_size_bytes) << ","
        << "\"edge_bandwidth_savings_estimate_pct\":"
        << format_double(snapshot.edge_bandwidth_savings_estimate_pct) << ","
        << "\"edge_packet_encode_latency_us\":"
        << format_double(snapshot.edge_packet_encode_latency_us) << ","
        << "\"auth_mode\":\"" << json_escape(snapshot.auth_mode) << "\","
        << "\"auth_failures\":" << snapshot.auth_failures << ","
        << "\"unsigned_packets\":" << snapshot.unsigned_packets << ","
        << "\"last_auth_result\":\"" << json_escape(snapshot.last_auth_result) << "\","
        << "\"pqc_ready_status\":\"" << json_escape(snapshot.pqc_ready_status) << "\","
        << "\"disconnected_operation\":" << (snapshot.disconnected_operation ? "true" : "false")
        << ","
        << "\"edge_health_status\":\"" << json_escape(snapshot.edge_health_status) << "\","
        << "\"edge_autonomy_state\":\"" << json_escape(snapshot.edge_autonomy_state) << "\","
        << "\"edge_inference_status\":\"" << json_escape(snapshot.edge_inference_status) << "\","
        << "\"edge_inference_fps\":" << format_double(snapshot.edge_inference_fps) << ","
        << "\"edge_inference_confidence\":" << format_double(snapshot.edge_inference_confidence)
        << ","
        << "\"local_obstacle_count\":" << snapshot.local_obstacle_count << ","
        << "\"shared_obstacle_count\":" << snapshot.shared_obstacle_count << ","
        << "\"security_state\":\"" << json_escape(snapshot.security_state) << "\","
        << "\"security_summary\":\"" << json_escape(snapshot.security_summary) << "\","
        << "\"security_transition_reason\":\"" << json_escape(snapshot.security_transition_reason)
        << "\","
        << "\"remote_command_allowed\":" << (snapshot.remote_command_allowed ? "true" : "false")
        << ","
        << "\"telemetry_uplink_allowed\":" << (snapshot.telemetry_uplink_allowed ? "true" : "false")
        << ","
        << "\"link_integrity_score\":" << format_double(snapshot.link_integrity_score) << ","
        << "\"trust_epoch\":" << snapshot.trust_epoch << ","
        << "\"last_auth_failure_at_s\":" << format_double(snapshot.last_auth_failure_at_s) << ","
        << "\"tamper_score\":" << format_double(snapshot.tamper_score) << ","
        << "\"firmware_measurement\":\"" << json_escape(snapshot.firmware_measurement) << "\","
        << "\"firmware_version\":\"" << json_escape(snapshot.firmware_version) << "\","
        << "\"secure_boot_state\":\"" << json_escape(snapshot.secure_boot_state) << "\","
        << "\"boot_trust_summary\":\"" << json_escape(snapshot.boot_trust_summary) << "\","
        << "\"rollback_counter\":" << snapshot.rollback_counter << ","
        << "\"maintenance_mode\":" << (snapshot.maintenance_mode ? "true" : "false") << ","
        << "\"update_channel_state\":\"" << json_escape(snapshot.update_channel_state) << "\","
        << "\"safety_state\":\"" << json_escape(snapshot.safety_state) << "\","
        << "\"safety_summary\":\"" << json_escape(snapshot.safety_summary) << "\","
        << "\"health_flags\":" << json_string_array(snapshot.health_flags) << ","
        << "\"camera\":" << json_camera_payload(snapshot.camera) << ","
        << "\"imu\":" << json_imu_payload(snapshot.imu) << ","
        << "\"lidar\":" << json_lidar_payload(snapshot.lidar) << ","
        << "\"tdoa\":" << json_tdoa_payload(snapshot.tdoa) << ","
        << "\"replay\":" << json_replay_payload(snapshot.replay) << ","
        << "\"timestamp\":\"" << format_timestamp_utc() << "\""
        << "}";
    return oss.str();
}

ControlPlaneTelemetryClient::HttpResponse
ControlPlaneTelemetryClient::default_transport(const ParsedEndpoint& endpoint,
                                               std::string_view body, const HeaderList& headers,
                                               int timeout_ms) {
#ifdef _WIN32
    auto host = widen(endpoint.host);
    auto path = widen(endpoint.path);
    HINTERNET session =
        WinHttpOpen(L"drone_swarm/telemetry-client", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
                    WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
    if (!session) {
        return {false, 0, "WinHttpOpen failed"};
    }

    HINTERNET connect = WinHttpConnect(session, host.c_str(), endpoint.port, 0);
    if (!connect) {
        WinHttpCloseHandle(session);
        return {false, 0, "WinHttpConnect failed"};
    }

    DWORD flags = endpoint.https ? WINHTTP_FLAG_SECURE : 0;
    HINTERNET request = WinHttpOpenRequest(connect, L"POST", path.c_str(), nullptr,
                                           WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, flags);
    if (!request) {
        WinHttpCloseHandle(connect);
        WinHttpCloseHandle(session);
        return {false, 0, "WinHttpOpenRequest failed"};
    }

    std::wstring header_block;
    for (const auto& [name, value] : headers) {
        header_block += widen(name + ": " + value + "\r\n");
    }

    DWORD timeout = static_cast<DWORD>(std::max(timeout_ms, 200));
    WinHttpSetTimeouts(session, timeout, timeout, timeout, timeout);

    const BOOL sent = WinHttpSendRequest(
        request, header_block.empty() ? WINHTTP_NO_ADDITIONAL_HEADERS : header_block.c_str(),
        static_cast<DWORD>(header_block.size()), const_cast<char*>(body.data()),
        static_cast<DWORD>(body.size()), static_cast<DWORD>(body.size()), 0);
    if (!sent || !WinHttpReceiveResponse(request, nullptr)) {
        WinHttpCloseHandle(request);
        WinHttpCloseHandle(connect);
        WinHttpCloseHandle(session);
        return {false, 0, "WinHTTP send/receive failed"};
    }

    DWORD status_code = 0;
    DWORD size = sizeof(status_code);
    const BOOL queried = WinHttpQueryHeaders(
        request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
        WINHTTP_HEADER_NAME_BY_INDEX, &status_code, &size, WINHTTP_NO_HEADER_INDEX);

    WinHttpCloseHandle(request);
    WinHttpCloseHandle(connect);
    WinHttpCloseHandle(session);

    if (!queried) {
        return {false, 0, "WinHTTP status query failed"};
    }
    return {true, static_cast<int>(status_code), "ok"};
#else
    return send_http_posix(endpoint, body, headers, timeout_ms);
#endif
}

std::chrono::milliseconds ControlPlaneTelemetryClient::current_backoff() const {
    const int exponent = std::clamp(consecutive_failures_ - 1, 0, 5);
    const auto base = std::chrono::milliseconds(300);
    return base * (1 << exponent);
}

void ControlPlaneTelemetryClient::mark_result(bool ok, std::string status,
                                              std::chrono::steady_clock::time_point now) {
    std::lock_guard lock(state_mutex_);
    last_attempt_ = now;
    if (ok) {
        consecutive_failures_ = 0;
        next_retry_not_before_ = now + interval_;
        last_status_ = std::move(status);
        return;
    }

    ++consecutive_failures_;
    next_retry_not_before_ = now + current_backoff();
    last_status_ =
        "error: " + status + " retry_in_ms=" +
        std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(current_backoff()).count());
}

} // namespace drone::telemetry
