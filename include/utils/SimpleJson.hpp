#pragma once

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>
#include <string_view>

namespace drone::utils::simple_json {

namespace detail {

inline size_t skip_ws(std::string_view content, size_t pos) {
    while (pos < content.size() &&
           std::isspace(static_cast<unsigned char>(content[pos])) != 0) {
        ++pos;
    }
    return pos;
}

inline std::optional<std::pair<std::string, size_t>>
read_quoted_string(std::string_view content, size_t pos) {
    if (pos >= content.size() || content[pos] != '"') {
        return std::nullopt;
    }

    std::string value;
    value.reserve(32);
    bool escaped = false;
    for (size_t i = pos + 1; i < content.size(); ++i) {
        const char ch = content[i];
        if (escaped) {
            switch (ch) {
            case '"':
            case '\\':
            case '/':
                value.push_back(ch);
                break;
            case 'b':
                value.push_back('\b');
                break;
            case 'f':
                value.push_back('\f');
                break;
            case 'n':
                value.push_back('\n');
                break;
            case 'r':
                value.push_back('\r');
                break;
            case 't':
                value.push_back('\t');
                break;
            default:
                value.push_back(ch);
                break;
            }
            escaped = false;
            continue;
        }
        if (ch == '\\') {
            escaped = true;
            continue;
        }
        if (ch == '"') {
            return std::pair<std::string, size_t>{std::move(value), i + 1};
        }
        value.push_back(ch);
    }
    return std::nullopt;
}

inline std::optional<size_t> find_value_offset(std::string_view content, std::string_view key) {
    size_t pos = 0;
    while (pos < content.size()) {
        pos = content.find('"', pos);
        if (pos == std::string_view::npos) {
            return std::nullopt;
        }
        const auto parsed = read_quoted_string(content, pos);
        if (!parsed.has_value()) {
            return std::nullopt;
        }
        const auto after_key = skip_ws(content, parsed->second);
        if (parsed->first == key && after_key < content.size() && content[after_key] == ':') {
            return skip_ws(content, after_key + 1);
        }
        pos = parsed->second;
    }
    return std::nullopt;
}

inline std::optional<std::string> extract_number_token(std::string_view content, size_t pos) {
    const size_t begin = pos;
    while (pos < content.size()) {
        const char ch = content[pos];
        if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+' || ch == '.' || ch == 'e' ||
            ch == 'E') {
            ++pos;
            continue;
        }
        break;
    }
    if (pos == begin) {
        return std::nullopt;
    }
    return std::string(content.substr(begin, pos - begin));
}

} // namespace detail

inline std::optional<std::string> extract_string(std::string_view content, std::string_view key) {
    const auto value_offset = detail::find_value_offset(content, key);
    if (!value_offset.has_value()) {
        return std::nullopt;
    }
    const auto parsed = detail::read_quoted_string(content, *value_offset);
    if (!parsed.has_value()) {
        return std::nullopt;
    }
    return parsed->first;
}

inline std::optional<bool> extract_bool(std::string_view content, std::string_view key) {
    const auto value_offset = detail::find_value_offset(content, key);
    if (!value_offset.has_value()) {
        return std::nullopt;
    }
    if (content.substr(*value_offset, 4) == "true") {
        return true;
    }
    if (content.substr(*value_offset, 5) == "false") {
        return false;
    }
    return std::nullopt;
}

inline std::optional<double> extract_number(std::string_view content, std::string_view key) {
    const auto value_offset = detail::find_value_offset(content, key);
    if (!value_offset.has_value()) {
        return std::nullopt;
    }
    const auto token = detail::extract_number_token(content, *value_offset);
    if (!token.has_value()) {
        return std::nullopt;
    }
    try {
        return std::stod(*token);
    } catch (...) {
        return std::nullopt;
    }
}

inline std::optional<uint64_t> extract_u64(std::string_view content, std::string_view key) {
    const auto value_offset = detail::find_value_offset(content, key);
    if (!value_offset.has_value()) {
        return std::nullopt;
    }
    const auto token = detail::extract_number_token(content, *value_offset);
    if (!token.has_value() || token->find_first_of(".eE-+") != std::string::npos) {
        return std::nullopt;
    }
    try {
        return static_cast<uint64_t>(std::stoull(*token));
    } catch (...) {
        return std::nullopt;
    }
}

} // namespace drone::utils::simple_json
