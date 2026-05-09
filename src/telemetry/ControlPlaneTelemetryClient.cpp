// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "telemetry/ControlPlaneTelemetryClient.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
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

std::string read_env(const char* key) {
    if (key == nullptr) {
        return {};
    }
    if (const char* value = std::getenv(key)) {
        return value;
    }
    return {};
}

bool parse_bool_env(const char* key, bool fallback) {
    std::string value = read_env(key);
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (value == "1" || value == "true" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "off") {
        return false;
    }
    return fallback;
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
    const DWORD flags = FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS;
    const DWORD size = FormatMessageA(flags, nullptr, code, 0, reinterpret_cast<LPSTR>(&buffer), 0, nullptr);
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

    HANDLE file = CreateFileW(
        widen(pfx_path).c_str(),
        GENERIC_READ,
        FILE_SHARE_READ,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);
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
    const BOOL read_ok = ReadFile(file, buffer.data(), static_cast<DWORD>(buffer.size()), &bytes_read, nullptr);
    CloseHandle(file);
    if (!read_ok || bytes_read != buffer.size()) {
        error = "failed to read client PFX contents";
        return {};
    }

    CRYPT_DATA_BLOB blob{};
    blob.pbData = reinterpret_cast<BYTE*>(buffer.data());
    blob.cbData = bytes_read;

    unique_cert_store store(
        PFXImportCertStore(&blob, widen(password).c_str(), CRYPT_USER_KEYSET | PKCS12_ALLOW_OVERWRITE_KEY),
        CertStoreCloser{});
    if (!store) {
        error = "failed to import client PFX: " + win32_error_message(GetLastError());
        return {};
    }

    PCCERT_CONTEXT found = nullptr;
    while ((found = CertFindCertificateInStore(
                static_cast<HCERTSTORE>(store.get()),
                X509_ASN_ENCODING | PKCS_7_ASN_ENCODING,
                0,
                CERT_FIND_ANY,
                nullptr,
                found)) != nullptr) {
        DWORD key_spec = 0;
        BOOL must_free = FALSE;
        HCRYPTPROV_OR_NCRYPT_KEY_HANDLE key_handle = 0;
        if (CryptAcquireCertificatePrivateKey(found, CRYPT_ACQUIRE_SILENT_FLAG, nullptr, &key_handle, &key_spec, &must_free)) {
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
    if (!CryptQueryObject(
            CERT_QUERY_OBJECT_FILE,
            widen(ca_file).c_str(),
            CERT_QUERY_CONTENT_FLAG_CERT,
            CERT_QUERY_FORMAT_FLAG_ALL,
            0,
            &encoding,
            &content_type,
            &format_type,
            &store,
            nullptr,
            reinterpret_cast<const void**>(&cert_context))) {
        error = "failed to parse DRONE_TLS_CA_FILE: " + win32_error_message(GetLastError());
        return unique_cert_store(nullptr, CertStoreCloser{});
    }
    if (cert_context != nullptr) {
        CertFreeCertificateContext(cert_context);
    }
    return unique_cert_store(store, CertStoreCloser{});
}

bool validate_server_certificate(HINTERNET request,
                                 const std::string& host,
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
    if (!WinHttpQueryOption(request, WINHTTP_OPTION_SERVER_CERT_CONTEXT, cert_buffer.data(), &size)) {
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
    if (!CertGetCertificateChain(
            static_cast<HCERTCHAINENGINE>(chain_engine.get()),
            server_cert.get(),
            nullptr,
            server_cert->hCertStore,
            &chain_para,
            0,
            nullptr,
            &raw_chain)) {
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
    if (!CertVerifyCertificateChainPolicy(CERT_CHAIN_POLICY_SSL, chain.get(), &policy_para, &policy_status)) {
        error = "failed to verify server certificate policy: " + win32_error_message(GetLastError());
        return false;
    }
    if (policy_status.dwError != 0) {
        error = "server certificate policy rejected backend: " + win32_error_message(policy_status.dwError);
        return false;
    }
    return true;
}
#endif

} // namespace

ControlPlaneTelemetryClient::ControlPlaneTelemetryClient(std::string backend_url, int interval_ms)
    : endpoint_(parse_backend_url(backend_url)),
      tls_(load_tls_runtime_config()),
      interval_(std::chrono::milliseconds(std::max(interval_ms, 200))) {
    enabled_ = !backend_url.empty();
    if (enabled_) {
        last_status_ = "ready";
    }
}

ControlPlaneTelemetryClient::TLSRuntimeConfig ControlPlaneTelemetryClient::load_tls_runtime_config() {
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

    if (endpoint_.https) {
        DWORD security_flags = 0;
        if (tls_.skip_verify) {
            security_flags = SECURITY_FLAG_IGNORE_UNKNOWN_CA |
                             SECURITY_FLAG_IGNORE_CERT_CN_INVALID |
                             SECURITY_FLAG_IGNORE_CERT_DATE_INVALID |
                             SECURITY_FLAG_IGNORE_CERT_WRONG_USAGE;
        } else if (!tls_.ca_file.empty()) {
            security_flags = SECURITY_FLAG_IGNORE_UNKNOWN_CA;
        }
        if (security_flags != 0) {
            WinHttpSetOption(request, WINHTTP_OPTION_SECURITY_FLAGS, &security_flags, sizeof(security_flags));
        }
        if (!tls_.client_pfx_file.empty()) {
            std::string cert_error;
            const auto client_cert = load_client_certificate_from_pfx(tls_.client_pfx_file, tls_.client_pfx_password, cert_error);
            if (!client_cert) {
                WinHttpCloseHandle(request);
                WinHttpCloseHandle(connect);
                WinHttpCloseHandle(session);
                mark_result(false, cert_error, now);
                return false;
            }
            if (!WinHttpSetOption(request,
                                  WINHTTP_OPTION_CLIENT_CERT_CONTEXT,
                                  const_cast<CERT_CONTEXT*>(client_cert.get()),
                                  sizeof(CERT_CONTEXT))) {
                WinHttpCloseHandle(request);
                WinHttpCloseHandle(connect);
                WinHttpCloseHandle(session);
                mark_result(false, "failed to attach client certificate: " + win32_error_message(GetLastError()), now);
                return false;
            }
        }
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
        const DWORD error_code = GetLastError();
        WinHttpCloseHandle(request);
        WinHttpCloseHandle(connect);
        WinHttpCloseHandle(session);
        mark_result(false, "WinHTTP send/receive failed: " + win32_error_message(error_code), now);
        return false;
    }

    if (endpoint_.https) {
        std::string cert_error;
        if (!validate_server_certificate(request, endpoint_.host, tls_, cert_error)) {
            WinHttpCloseHandle(request);
            WinHttpCloseHandle(connect);
            WinHttpCloseHandle(session);
            mark_result(false, cert_error, now);
            return false;
        }
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
        << "\"security_transition_reason\":\"" << json_escape(snapshot.security_transition_reason) << "\","
        << "\"remote_command_allowed\":" << (snapshot.remote_command_allowed ? "true" : "false") << ","
        << "\"telemetry_uplink_allowed\":" << (snapshot.telemetry_uplink_allowed ? "true" : "false") << ","
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
