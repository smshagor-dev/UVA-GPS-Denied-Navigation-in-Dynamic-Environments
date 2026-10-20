#pragma once

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

namespace drone::swarm {

struct ReplayLedgerRecord {
    uint32_t peer_id{0};
    uint64_t epoch{0};
    uint32_t highest_sequence{0};
    uint64_t bitmap{1u};
};

// A single-writer, append-only, durable replay journal on trusted local storage.
// All callers synchronize access externally. Malformed/torn writes fail closed.
class ReplayJournal {
public:
    [[nodiscard]] bool configure(const std::string& path, bool initialize_new) {
        enabled_ = true;
        healthy_ = false;
        if (path.empty() || !path_.empty()) {
            return false;
        }
        path_ = path;
        std::error_code ec;
        const bool exists = std::filesystem::exists(path, ec);
        if (ec || (!exists && !initialize_new)) {
            return false;
        }
        if (!exists && !create_empty(path)) {
            return false;
        }
        std::unordered_map<uint32_t, ReplayLedgerRecord> restored;
        if (!load(path, restored)) {
            return false;
        }
        records_ = std::move(restored);
        healthy_ = true;
        return true;
    }

    [[nodiscard]] bool enabled() const { return enabled_; }
    [[nodiscard]] bool healthy() const { return healthy_; }
    [[nodiscard]] size_t peer_count() const { return records_.size(); }

    [[nodiscard]] const ReplayLedgerRecord* find(uint32_t peer_id) const {
        const auto it = records_.find(peer_id);
        return it == records_.end() ? nullptr : &it->second;
    }

    [[nodiscard]] bool persist(const ReplayLedgerRecord& record) {
        if (!enabled_) {
            return true;
        }
        if (!healthy_) {
            return false;
        }
        std::FILE* file = std::fopen(path_.c_str(), "rb+");
        if (file == nullptr) {
            healthy_ = false;
            return false;
        }
        const std::string row = "R " + std::to_string(record.peer_id) + " " +
                                std::to_string(record.epoch) + " " +
                                std::to_string(record.highest_sequence) + " " +
                                std::to_string(record.bitmap) + "\n";
        const bool positioned = std::fseek(file, 0, SEEK_END) == 0;
        const bool written =
            positioned && std::fwrite(row.data(), 1, row.size(), file) == row.size();
        const bool synced = written && flush_durable(file);
        const bool closed = std::fclose(file) == 0;
        if (!synced || !closed) {
            healthy_ = false;
            return false;
        }
        records_[record.peer_id] = record;
        return true;
    }

private:
    static constexpr const char* kVersion = "UAV_REPLAY_LEDGER_V1";

    static bool flush_durable(std::FILE* file) {
        if (std::fflush(file) != 0) {
            return false;
        }
#ifdef _WIN32
        return _commit(_fileno(file)) == 0;
#else
        return fsync(fileno(file)) == 0;
#endif
    }

    static bool create_empty(const std::string& path) {
        // Only use a private directory owned by this UAV's service account.
        std::FILE* file = std::fopen(path.c_str(), "wb");
        if (file == nullptr) {
            return false;
        }
        const std::string header = std::string(kVersion) + "\n";
        const bool written = std::fwrite(header.data(), 1, header.size(), file) == header.size();
        const bool synced = written && flush_durable(file);
        const bool closed = std::fclose(file) == 0;
        return synced && closed;
    }

    static bool parse_record(const std::string& row, ReplayLedgerRecord& record) {
        std::istringstream stream(row);
        std::string marker;
        if (!(stream >> marker >> record.peer_id >> record.epoch >>
              record.highest_sequence >> record.bitmap)) {
            return false;
        }
        std::string trailing;
        return marker == "R" && record.peer_id != 0 && (record.bitmap & 1u) != 0u &&
               !(stream >> trailing);
    }

    static bool load(const std::string& path,
                     std::unordered_map<uint32_t, ReplayLedgerRecord>& records) {
        std::ifstream file(path, std::ios::binary);
        std::string row;
        if (!std::getline(file, row) || file.eof() || row != kVersion) {
            return false;
        }
        while (std::getline(file, row)) {
            if (file.eof()) {
                return false; // Missing final newline means a possibly torn write.
            }
            ReplayLedgerRecord record;
            if (!parse_record(row, record)) {
                return false;
            }
            const auto previous = records.find(record.peer_id);
            if (previous != records.end()) {
                const auto& old = previous->second;
                if (record.epoch < old.epoch) {
                    return false;
                }
                if (record.epoch == old.epoch) {
                    if (record.highest_sequence < old.highest_sequence) {
                        return false;
                    }
                    if (record.highest_sequence == old.highest_sequence) {
                        if ((record.bitmap & old.bitmap) != old.bitmap) {
                            return false;
                        }
                    } else {
                        const uint32_t delta = record.highest_sequence - old.highest_sequence;
                        const uint64_t expected =
                            delta >= 64u ? uint64_t{1} : (old.bitmap << delta) | uint64_t{1};
                        if (record.bitmap != expected) {
                            return false;
                        }
                    }
                } else if (record.bitmap != 1u) {
                    return false;
                }
            }
            records[record.peer_id] = record;
        }
        return !file.bad();
    }

    std::string path_{};
    bool enabled_{false};
    bool healthy_{true};
    std::unordered_map<uint32_t, ReplayLedgerRecord> records_{};
};

} // namespace drone::swarm
