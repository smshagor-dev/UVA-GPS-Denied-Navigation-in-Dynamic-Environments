// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "slam/KeyframeManager.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <utility>

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace drone::slam {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool thread_sanitizer_enabled() {
#if defined(__has_feature)
#if __has_feature(thread_sanitizer)
    return true;
#endif
#endif
#if defined(__SANITIZE_THREAD__)
    return true;
#endif
    return false;
}

void configure_opencv_threads_for_tsan() {
    static std::once_flag once;
    if (!thread_sanitizer_enabled()) {
        return;
    }
    std::call_once(once, [] { cv::setNumThreads(1); });
}

uint64_t make_global_keyframe_id(uint32_t drone_id, uint64_t local_id) {
    return (static_cast<uint64_t>(drone_id) << 32) | (local_id & 0xFFFFFFFFull);
}

template <typename T> void append_bytes(std::vector<uint8_t>& out, const T& value) {
    const auto* begin = reinterpret_cast<const uint8_t*>(&value);
    out.insert(out.end(), begin, begin + sizeof(T));
}

template <typename T> bool read_bytes(const uint8_t*& ptr, size_t& remaining, T& value) {
    if (remaining < sizeof(T)) {
        return false;
    }
    std::memcpy(&value, ptr, sizeof(T));
    ptr += sizeof(T);
    remaining -= sizeof(T);
    return true;
}

void append_keypoint(std::vector<uint8_t>& out, const cv::KeyPoint& kp) {
    append_bytes(out, kp.pt.x);
    append_bytes(out, kp.pt.y);
    append_bytes(out, kp.size);
    append_bytes(out, kp.angle);
    append_bytes(out, kp.response);
    append_bytes(out, kp.octave);
    append_bytes(out, kp.class_id);
}

bool read_keypoint(const uint8_t*& ptr, size_t& remaining, cv::KeyPoint& kp) {
    float x = 0.0f;
    float y = 0.0f;
    if (!read_bytes(ptr, remaining, x) || !read_bytes(ptr, remaining, y) ||
        !read_bytes(ptr, remaining, kp.size) || !read_bytes(ptr, remaining, kp.angle) ||
        !read_bytes(ptr, remaining, kp.response) || !read_bytes(ptr, remaining, kp.octave) ||
        !read_bytes(ptr, remaining, kp.class_id)) {
        return false;
    }
    kp.pt = cv::Point2f{x, y};
    return true;
}

size_t descriptor_match_score(const cv::Ptr<cv::BFMatcher>& matcher, const cv::Mat& lhs,
                              const cv::Mat& rhs) {
    if (!matcher || lhs.empty() || rhs.empty()) {
        return 0;
    }

    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher->knnMatch(lhs, rhs, knn_matches, 2);

    size_t good = 0;
    for (const auto& pair : knn_matches) {
        if (pair.empty()) {
            continue;
        }
        if (pair.size() == 1) {
            if (pair[0].distance <= 40.0f) {
                ++good;
            }
            continue;
        }
        if (pair[0].distance < 0.75f * pair[1].distance && pair[0].distance <= 50.0f) {
            ++good;
        }
    }
    return good;
}

double orientation_delta_deg(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
    const Eigen::Quaterniond delta = a.conjugate() * b;
    const double w = std::clamp(std::abs(delta.w()), 0.0, 1.0);
    return 2.0 * std::acos(w) * 180.0 / kPi;
}

Eigen::Vector3d keypoint_to_bearing(const cv::KeyPoint& kp, const cv::Size& size) {
    const double fx = std::max(size.width, 1);
    const double fy = std::max(size.height, 1);
    const double cx = static_cast<double>(size.width) * 0.5;
    const double cy = static_cast<double>(size.height) * 0.5;
    Eigen::Vector3d ray{(kp.pt.x - cx) / fx, (kp.pt.y - cy) / fy, 1.0};
    return ray.normalized();
}

} // namespace

KeyframeManager::KeyframeManager(uint32_t drone_id, std::shared_ptr<swarm::V2XMeshNetwork> net,
                                 KeyframeSelectionPolicy policy)
    : drone_id_(drone_id), net_(std::move(net)), policy_(std::move(policy)),
      orb_(cv::ORB::create(512)), matcher_(cv::BFMatcher::create(cv::NORM_HAMMING, false)) {
    configure_opencv_threads_for_tsan();
    logger_ = spdlog::get("SLAM");
    if (!logger_) {
        logger_ = spdlog::stdout_color_mt("SLAM");
    }
}

std::optional<uint64_t> KeyframeManager::try_add_frame(const cv::Mat& image,
                                                       const Eigen::Vector3d& pos,
                                                       const Eigen::Quaterniond& ori,
                                                       double timestamp) {
    if (image.empty()) {
        return std::nullopt;
    }

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    std::vector<cv::KeyPoint> preview_keypoints;
    cv::Mat preview_descriptors;
    orb_->detectAndCompute(gray, cv::noArray(), preview_keypoints, preview_descriptors);

    if (!should_create_keyframe(pos, ori, timestamp, preview_keypoints.size())) {
        return std::nullopt;
    }

    Keyframe kf;
    kf.id = next_kf_id();
    kf.drone_id = drone_id_;
    kf.timestamp = timestamp;
    kf.position = pos;
    kf.orientation = ori.normalized();
    kf.keypoints = std::move(preview_keypoints);
    kf.descriptors = preview_descriptors.clone();

    {
        std::lock_guard lock(map_mutex_);

        if (!kf.descriptors.empty()) {
            kf.map_point_ids.reserve(static_cast<size_t>(kf.descriptors.rows));
            kf.bearings.reserve(static_cast<size_t>(kf.descriptors.rows));

            for (int row = 0; row < kf.descriptors.rows; ++row) {
                const cv::KeyPoint& kp = kf.keypoints[static_cast<size_t>(row)];
                const Eigen::Vector3d bearing_cam = keypoint_to_bearing(kp, gray.size());
                const Eigen::Vector3d bearing_world = (kf.orientation * bearing_cam).normalized();

                kf.bearings.push_back(bearing_world);

                const cv::Mat descriptor_row = kf.descriptors.row(row);
                const Eigen::Vector3d projected = kf.position + bearing_world * 5.0;

                uint64_t best_id = 0;
                int best_distance = std::numeric_limits<int>::max();
                for (auto& [mp_id, mp] : map_points_) {
                    if (mp.descriptor.empty()) {
                        continue;
                    }
                    const int distance = cv::norm(descriptor_row, mp.descriptor, cv::NORM_HAMMING);
                    const double geometry_distance = (mp.pos_world - projected).norm();
                    if (distance < best_distance && distance <= 24 && geometry_distance <= 2.5) {
                        best_distance = distance;
                        best_id = mp_id;
                    }
                }

                if (best_id != 0) {
                    auto& mp = map_points_.at(best_id);
                    mp.obs_count++;
                    mp.last_seen_ts = timestamp;
                    mp.confidence = std::min(1.0f, mp.confidence + 0.05f);
                    mp.pos_world = 0.7 * mp.pos_world + 0.3 * projected;
                    kf.map_point_ids.push_back(best_id);
                } else {
                    MapPoint mp;
                    mp.id = next_mp_id();
                    mp.pos_world = projected;
                    mp.descriptor = descriptor_row.clone();
                    mp.owner_id = drone_id_;
                    mp.obs_count = 1;
                    mp.last_seen_ts = timestamp;
                    mp.confidence = 0.55f;
                    map_points_.emplace(mp.id, mp);
                    kf.map_point_ids.push_back(mp.id);
                }
            }
        }

        keyframes_.push_back(kf);
        while (keyframes_.size() > kMaxLocalKeyframes) {
            keyframes_.pop_front();
        }
    }

    last_kf_pos_ = pos;
    last_kf_ori_ = ori.normalized();
    last_kf_ts_ = timestamp;

    const auto loop_candidates = find_loop_closure_candidates(kf.descriptors, 3);
    last_loop_candidate_count_.store(loop_candidates.size());
    if (!loop_candidates.empty() && logger_) {
        relocalization_count_.fetch_add(1);
        logger_->info("SLAM keyframe {} has {} loop-closure candidates", kf.id,
                      loop_candidates.size());
    }

    if (net_) {
        share_latest_keyframe();
    }

    return kf.id;
}

std::vector<MapPoint> KeyframeManager::get_local_map_points() const {
    std::lock_guard lock(map_mutex_);
    std::vector<MapPoint> out;
    out.reserve(map_points_.size());
    for (const auto& [id, mp] : map_points_) {
        MapPoint copy = mp;
        copy.descriptor = mp.descriptor.clone();
        out.push_back(std::move(copy));
    }
    return out;
}

std::vector<Keyframe> KeyframeManager::get_recent_keyframes(size_t n) const {
    std::lock_guard lock(map_mutex_);
    const size_t count = std::min(n, keyframes_.size());
    std::vector<Keyframe> out;
    out.reserve(count);
    const auto begin = keyframes_.end() - static_cast<std::ptrdiff_t>(count);
    for (auto it = begin; it != keyframes_.end(); ++it) {
        Keyframe copy = *it;
        copy.descriptors = it->descriptors.clone();
        out.push_back(std::move(copy));
    }
    return out;
}

size_t KeyframeManager::keyframe_count() const {
    std::lock_guard lock(map_mutex_);
    return keyframes_.size();
}

size_t KeyframeManager::map_point_count() const {
    std::lock_guard lock(map_mutex_);
    return map_points_.size();
}

KeyframeManager::Status KeyframeManager::status() const {
    std::lock_guard lock(map_mutex_);
    return Status{
        keyframes_.size(),
        remote_keyframes_.size(),
        map_points_.size(),
        last_loop_candidate_count_.load(),
        relocalization_count_.load(),
        last_relocalization_confidence_.load(),
        last_relocalized_keyframe_.load(),
    };
}

void KeyframeManager::share_latest_keyframe() {
    std::vector<uint8_t> payload;
    uint64_t shared_id = 0;

    {
        std::lock_guard lock(map_mutex_);
        if (!net_ || keyframes_.empty()) {
            return;
        }

        Keyframe shareable = keyframes_.back();
        payload = encode_keyframe(shareable);
        if (payload.empty()) {
            return;
        }
        keyframes_.back().shared = true;
        shared_id = shareable.id;
    }

    if (!net_->broadcast(swarm::SwarmMessage::Type::KEYFRAME_SHARE, std::move(payload)) &&
        logger_) {
        logger_->warn("SLAM failed to broadcast keyframe {}", shared_id);
    }
}

void KeyframeManager::on_remote_keyframe(const swarm::SwarmMessage& msg) {
    if (msg.type != swarm::SwarmMessage::Type::KEYFRAME_SHARE || msg.payload.empty()) {
        return;
    }

    auto decoded = decode_keyframe(msg.payload.data(), msg.payload.size());
    if (!decoded.has_value()) {
        if (logger_) {
            logger_->warn("SLAM rejected malformed remote keyframe from drone {}", msg.src_id);
        }
        return;
    }

    Keyframe remote = std::move(decoded.value());
    remote.drone_id = msg.src_id;
    remote.shared = true;

    std::lock_guard lock(map_mutex_);

    const auto remote_gid = make_global_keyframe_id(remote.drone_id, remote.id);
    const auto exists = std::any_of(
        remote_keyframes_.begin(), remote_keyframes_.end(), [&](const Keyframe& candidate) {
            return make_global_keyframe_id(candidate.drone_id, candidate.id) == remote_gid;
        });
    if (exists) {
        return;
    }

    for (size_t i = 0;
         i < remote.bearings.size() && i < static_cast<size_t>(remote.descriptors.rows); ++i) {
        const Eigen::Vector3d projected = remote.position + remote.bearings[i].normalized() * 5.0;
        const cv::Mat descriptor_row = remote.descriptors.row(static_cast<int>(i));

        uint64_t matched_id = 0;
        int best_distance = std::numeric_limits<int>::max();
        for (auto& [mp_id, mp] : map_points_) {
            if (mp.descriptor.empty()) {
                continue;
            }
            const int distance = cv::norm(descriptor_row, mp.descriptor, cv::NORM_HAMMING);
            const double geometry_distance = (mp.pos_world - projected).norm();
            if (distance < best_distance && distance <= 28 && geometry_distance <= 3.0) {
                best_distance = distance;
                matched_id = mp_id;
            }
        }

        if (matched_id != 0) {
            auto& mp = map_points_.at(matched_id);
            mp.obs_count++;
            mp.owner_id = std::min(mp.owner_id, remote.drone_id);
            mp.last_seen_ts = remote.timestamp;
            mp.confidence = std::min(1.0f, mp.confidence + 0.08f);
            mp.pos_world = 0.5 * mp.pos_world + 0.5 * projected;
            if (i < remote.map_point_ids.size()) {
                remote.map_point_ids[i] = matched_id;
            }
        } else {
            MapPoint mp;
            mp.id = next_mp_id();
            mp.pos_world = projected;
            mp.descriptor = descriptor_row.clone();
            mp.owner_id = remote.drone_id;
            mp.obs_count = 1;
            mp.last_seen_ts = remote.timestamp;
            mp.confidence = 0.65f;
            map_points_.emplace(mp.id, mp);
            if (i < remote.map_point_ids.size()) {
                remote.map_point_ids[i] = mp.id;
            } else {
                remote.map_point_ids.push_back(mp.id);
            }
        }
    }

    remote_keyframes_.push_back(std::move(remote));
    if (remote_keyframes_.size() > kMaxLocalKeyframes) {
        remote_keyframes_.erase(
            remote_keyframes_.begin(),
            remote_keyframes_.begin() +
                static_cast<std::ptrdiff_t>(remote_keyframes_.size() - kMaxLocalKeyframes));
    }
}

std::vector<uint64_t> KeyframeManager::find_loop_closure_candidates(const cv::Mat& query_desc,
                                                                    size_t top_k) const {
    struct CandidateScore {
        uint64_t id{0};
        size_t score{0};
    };

    std::vector<CandidateScore> scored;
    std::lock_guard lock(map_mutex_);

    auto score_set = [&](const auto& keyframes) {
        for (const auto& kf : keyframes) {
            const size_t score = descriptor_match_score(matcher_, query_desc, kf.descriptors);
            if (score >= 15) {
                scored.push_back({make_global_keyframe_id(kf.drone_id, kf.id), score});
            }
        }
    };

    score_set(keyframes_);
    score_set(remote_keyframes_);

    std::sort(scored.begin(), scored.end(), [](const CandidateScore& a, const CandidateScore& b) {
        if (a.score != b.score) {
            return a.score > b.score;
        }
        return a.id < b.id;
    });

    if (scored.size() > top_k) {
        scored.resize(top_k);
    }

    std::vector<uint64_t> out;
    out.reserve(scored.size());
    for (const auto& candidate : scored) {
        out.push_back(candidate.id);
    }
    return out;
}

std::optional<KeyframeManager::RelocalizationResult>
KeyframeManager::attempt_relocalization(const cv::Mat& image, const Eigen::Vector3d& pose_guess,
                                        const Eigen::Quaterniond& orientation_guess) const {
    if (image.empty()) {
        return std::nullopt;
    }

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb_->detectAndCompute(gray, cv::noArray(), keypoints, descriptors);
    if (descriptors.empty()) {
        return std::nullopt;
    }

    struct BestMatch {
        uint64_t keyframe_id{0};
        uint32_t drone_id{0};
        size_t score{0};
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    } best;

    {
        std::lock_guard lock(map_mutex_);
        auto consider = [&](const auto& frames) {
            for (const auto& frame : frames) {
                const size_t score =
                    descriptor_match_score(matcher_, descriptors, frame.descriptors);
                if (score > best.score) {
                    best.keyframe_id = frame.id;
                    best.drone_id = frame.drone_id;
                    best.score = score;
                    best.position = frame.position;
                    best.orientation = frame.orientation;
                }
            }
        };
        consider(keyframes_);
        consider(remote_keyframes_);
    }

    if (best.score < 18) {
        return std::nullopt;
    }

    RelocalizationResult out;
    out.matched_keyframe_id = make_global_keyframe_id(best.drone_id, best.keyframe_id);
    out.confidence = std::clamp(static_cast<double>(best.score) / 80.0, 0.0, 1.0);
    out.corrected_position = (best.position * 0.7) + (pose_guess * 0.3);
    out.corrected_orientation = best.orientation.slerp(0.3, orientation_guess).normalized();

    last_relocalization_confidence_.store(out.confidence);
    last_relocalized_keyframe_.store(out.matched_keyframe_id);
    if (out.confidence >= 0.45) {
        relocalization_count_.fetch_add(1);
    }

    return out;
}

bool KeyframeManager::save_map(const std::string& filepath) const {
    std::lock_guard lock(map_mutex_);

    std::ofstream out(filepath, std::ios::binary);
    if (!out) {
        return false;
    }

    const uint32_t magic = 0x4B46534D; // KFSM
    const uint32_t version = 1;
    out.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    out.write(reinterpret_cast<const char*>(&version), sizeof(version));

    const uint32_t keyframe_count = static_cast<uint32_t>(keyframes_.size());
    const uint32_t remote_count = static_cast<uint32_t>(remote_keyframes_.size());
    const uint32_t map_point_count = static_cast<uint32_t>(map_points_.size());
    out.write(reinterpret_cast<const char*>(&keyframe_count), sizeof(keyframe_count));
    out.write(reinterpret_cast<const char*>(&remote_count), sizeof(remote_count));
    out.write(reinterpret_cast<const char*>(&map_point_count), sizeof(map_point_count));

    auto write_keyframe = [&](const Keyframe& kf) {
        const auto encoded = encode_keyframe(kf);
        const uint32_t payload_size = static_cast<uint32_t>(encoded.size());
        out.write(reinterpret_cast<const char*>(&payload_size), sizeof(payload_size));
        out.write(reinterpret_cast<const char*>(encoded.data()),
                  static_cast<std::streamsize>(encoded.size()));
    };

    for (const auto& kf : keyframes_) {
        write_keyframe(kf);
    }
    for (const auto& kf : remote_keyframes_) {
        write_keyframe(kf);
    }

    for (const auto& [id, mp] : map_points_) {
        out.write(reinterpret_cast<const char*>(&mp.id), sizeof(mp.id));
        out.write(reinterpret_cast<const char*>(&mp.owner_id), sizeof(mp.owner_id));
        out.write(reinterpret_cast<const char*>(&mp.obs_count), sizeof(mp.obs_count));
        out.write(reinterpret_cast<const char*>(&mp.last_seen_ts), sizeof(mp.last_seen_ts));
        out.write(reinterpret_cast<const char*>(&mp.confidence), sizeof(mp.confidence));
        out.write(reinterpret_cast<const char*>(mp.pos_world.data()), sizeof(double) * 3);
        const int rows = mp.descriptor.rows;
        const int cols = mp.descriptor.cols;
        const int type = mp.descriptor.type();
        out.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
        out.write(reinterpret_cast<const char*>(&cols), sizeof(cols));
        out.write(reinterpret_cast<const char*>(&type), sizeof(type));
        if (!mp.descriptor.empty()) {
            out.write(
                reinterpret_cast<const char*>(mp.descriptor.data),
                static_cast<std::streamsize>(mp.descriptor.total() * mp.descriptor.elemSize()));
        }
    }

    return static_cast<bool>(out);
}

bool KeyframeManager::load_map(const std::string& filepath) {
    std::ifstream in(filepath, std::ios::binary);
    if (!in) {
        return false;
    }

    uint32_t magic = 0;
    uint32_t version = 0;
    uint32_t local_count = 0;
    uint32_t remote_count = 0;
    uint32_t point_count = 0;

    in.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    in.read(reinterpret_cast<char*>(&version), sizeof(version));
    in.read(reinterpret_cast<char*>(&local_count), sizeof(local_count));
    in.read(reinterpret_cast<char*>(&remote_count), sizeof(remote_count));
    in.read(reinterpret_cast<char*>(&point_count), sizeof(point_count));

    if (!in || magic != 0x4B46534D || version != 1) {
        return false;
    }

    std::deque<Keyframe> local_keyframes;
    std::vector<Keyframe> remote_keyframes;
    std::map<uint64_t, MapPoint> map_points;
    uint64_t max_kf_id = 0;
    uint64_t max_mp_id = 0;

    auto read_keyframe_payload = [&](Keyframe& out_kf) -> bool {
        uint32_t payload_size = 0;
        in.read(reinterpret_cast<char*>(&payload_size), sizeof(payload_size));
        if (!in) {
            return false;
        }
        std::vector<uint8_t> payload(payload_size);
        if (payload_size > 0) {
            in.read(reinterpret_cast<char*>(payload.data()), payload_size);
        }
        if (!in) {
            return false;
        }
        auto decoded = decode_keyframe(payload.data(), payload.size());
        if (!decoded.has_value()) {
            return false;
        }
        out_kf = std::move(decoded.value());
        max_kf_id = std::max(max_kf_id, out_kf.id);
        return true;
    };

    for (uint32_t i = 0; i < local_count; ++i) {
        Keyframe kf;
        if (!read_keyframe_payload(kf)) {
            return false;
        }
        local_keyframes.push_back(std::move(kf));
    }
    for (uint32_t i = 0; i < remote_count; ++i) {
        Keyframe kf;
        if (!read_keyframe_payload(kf)) {
            return false;
        }
        remote_keyframes.push_back(std::move(kf));
    }

    for (uint32_t i = 0; i < point_count; ++i) {
        MapPoint mp;
        in.read(reinterpret_cast<char*>(&mp.id), sizeof(mp.id));
        in.read(reinterpret_cast<char*>(&mp.owner_id), sizeof(mp.owner_id));
        in.read(reinterpret_cast<char*>(&mp.obs_count), sizeof(mp.obs_count));
        in.read(reinterpret_cast<char*>(&mp.last_seen_ts), sizeof(mp.last_seen_ts));
        in.read(reinterpret_cast<char*>(&mp.confidence), sizeof(mp.confidence));
        in.read(reinterpret_cast<char*>(mp.pos_world.data()), sizeof(double) * 3);
        int rows = 0;
        int cols = 0;
        int type = 0;
        in.read(reinterpret_cast<char*>(&rows), sizeof(rows));
        in.read(reinterpret_cast<char*>(&cols), sizeof(cols));
        in.read(reinterpret_cast<char*>(&type), sizeof(type));
        if (!in) {
            return false;
        }
        if (rows > 0 && cols > 0) {
            mp.descriptor.create(rows, cols, type);
            in.read(reinterpret_cast<char*>(mp.descriptor.data),
                    static_cast<std::streamsize>(mp.descriptor.total() * mp.descriptor.elemSize()));
        }
        if (!in) {
            return false;
        }
        max_mp_id = std::max(max_mp_id, mp.id);
        map_points.emplace(mp.id, std::move(mp));
    }

    {
        std::lock_guard lock(map_mutex_);
        keyframes_ = std::move(local_keyframes);
        remote_keyframes_ = std::move(remote_keyframes);
        map_points_ = std::move(map_points);
    }

    kf_counter_.store(max_kf_id + 1);
    mp_counter_.store(max_mp_id + 1);

    if (!local_keyframes.empty()) {
        last_kf_pos_ = local_keyframes.back().position;
        last_kf_ori_ = local_keyframes.back().orientation;
        last_kf_ts_ = local_keyframes.back().timestamp;
    } else {
        last_kf_pos_.setZero();
        last_kf_ori_.setIdentity();
        last_kf_ts_ = -1.0;
    }

    return true;
}

bool KeyframeManager::should_create_keyframe(const Eigen::Vector3d& pos,
                                             const Eigen::Quaterniond& ori, double ts,
                                             size_t tracked_features) const {
    if (tracked_features < policy_.min_tracked_features) {
        return false;
    }
    if (last_kf_ts_ < 0.0) {
        return true;
    }
    if ((ts - last_kf_ts_) < policy_.min_time_s) {
        return false;
    }

    const double translation = (pos - last_kf_pos_).norm();
    const double rotation_deg = orientation_delta_deg(last_kf_ori_, ori.normalized());

    if (translation >= policy_.min_translation_m || rotation_deg >= policy_.min_rotation_deg) {
        return true;
    }

    std::lock_guard lock(map_mutex_);
    if (!keyframes_.empty() && tracked_features > 0) {
        const auto& prev = keyframes_.back();
        const size_t comparable = std::min(prev.map_point_ids.size(), tracked_features);
        if (comparable == 0) {
            return true;
        }
        const double overlap =
            static_cast<double>(comparable) /
            static_cast<double>(std::max(prev.map_point_ids.size(), tracked_features));
        if (overlap < policy_.max_point_overlap) {
            return true;
        }
    }

    return false;
}

std::vector<uint64_t> KeyframeManager::extract_and_triangulate(const cv::Mat& image, Keyframe& kf) {
    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    orb_->detectAndCompute(gray, cv::noArray(), kf.keypoints, kf.descriptors);
    kf.map_point_ids.clear();
    kf.bearings.clear();
    if (kf.descriptors.empty()) {
        return {};
    }

    kf.map_point_ids.reserve(static_cast<size_t>(kf.descriptors.rows));
    kf.bearings.reserve(static_cast<size_t>(kf.descriptors.rows));

    for (int row = 0; row < kf.descriptors.rows; ++row) {
        const auto bearing =
            (kf.orientation *
             keypoint_to_bearing(kf.keypoints[static_cast<size_t>(row)], gray.size()))
                .normalized();
        kf.bearings.push_back(bearing);

        MapPoint mp;
        mp.id = next_mp_id();
        mp.pos_world = kf.position + bearing * 5.0;
        mp.descriptor = kf.descriptors.row(row).clone();
        mp.owner_id = kf.drone_id;
        mp.last_seen_ts = kf.timestamp;
        map_points_[mp.id] = mp;
        kf.map_point_ids.push_back(mp.id);
    }

    return kf.map_point_ids;
}

std::vector<uint8_t> KeyframeManager::encode_keyframe(const Keyframe& kf) const {
    std::vector<uint8_t> out;

    const int desc_rows = kf.descriptors.empty() ? 0 : kf.descriptors.rows;
    const int desc_cols = kf.descriptors.empty() ? 0 : kf.descriptors.cols;
    const int bytes_per_row =
        (desc_rows > 0) ? desc_cols * static_cast<int>(kf.descriptors.elemSize1()) : 0;

    size_t keep_rows = static_cast<size_t>(std::max(desc_rows, 0));
    if (bytes_per_row > 0) {
        keep_rows =
            std::min<size_t>(keep_rows, kMaxSharedDescSize / static_cast<size_t>(bytes_per_row));
    }
    keep_rows = std::min(keep_rows, kf.keypoints.size());
    keep_rows = std::min(keep_rows, kf.bearings.size());
    keep_rows = std::min(keep_rows, kf.map_point_ids.size());

    const uint32_t magic = 0x4B465248; // KFRH
    const uint32_t version = 1;
    append_bytes(out, magic);
    append_bytes(out, version);
    append_bytes(out, kf.id);
    append_bytes(out, kf.drone_id);
    append_bytes(out, kf.timestamp);
    append_bytes(out, kf.position.x());
    append_bytes(out, kf.position.y());
    append_bytes(out, kf.position.z());
    append_bytes(out, kf.orientation.w());
    append_bytes(out, kf.orientation.x());
    append_bytes(out, kf.orientation.y());
    append_bytes(out, kf.orientation.z());

    const uint32_t row_count = static_cast<uint32_t>(keep_rows);
    append_bytes(out, row_count);
    append_bytes(out, desc_cols);

    for (size_t i = 0; i < keep_rows; ++i) {
        append_keypoint(out, kf.keypoints[i]);
        append_bytes(out, kf.map_point_ids[i]);
        append_bytes(out, kf.bearings[i].x());
        append_bytes(out, kf.bearings[i].y());
        append_bytes(out, kf.bearings[i].z());
        const cv::Mat row = kf.descriptors.row(static_cast<int>(i));
        out.insert(out.end(), row.datastart, row.dataend);
    }

    return out;
}

std::optional<Keyframe> KeyframeManager::decode_keyframe(const uint8_t* data, size_t len) const {
    if (!data || len == 0) {
        return std::nullopt;
    }

    const uint8_t* ptr = data;
    size_t remaining = len;

    uint32_t magic = 0;
    uint32_t version = 0;
    Keyframe kf;
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    double qw = 1.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    uint32_t row_count = 0;
    int desc_cols = 0;

    if (!read_bytes(ptr, remaining, magic) || !read_bytes(ptr, remaining, version) ||
        !read_bytes(ptr, remaining, kf.id) || !read_bytes(ptr, remaining, kf.drone_id) ||
        !read_bytes(ptr, remaining, kf.timestamp) || !read_bytes(ptr, remaining, px) ||
        !read_bytes(ptr, remaining, py) || !read_bytes(ptr, remaining, pz) ||
        !read_bytes(ptr, remaining, qw) || !read_bytes(ptr, remaining, qx) ||
        !read_bytes(ptr, remaining, qy) || !read_bytes(ptr, remaining, qz) ||
        !read_bytes(ptr, remaining, row_count) || !read_bytes(ptr, remaining, desc_cols)) {
        return std::nullopt;
    }

    if (magic != 0x4B465248 || version != 1 || desc_cols < 0) {
        return std::nullopt;
    }

    kf.position = Eigen::Vector3d{px, py, pz};
    kf.orientation = Eigen::Quaterniond{qw, qx, qy, qz}.normalized();
    kf.keypoints.reserve(row_count);
    kf.map_point_ids.reserve(row_count);
    kf.bearings.reserve(row_count);
    if (row_count > 0 && desc_cols > 0) {
        kf.descriptors.create(static_cast<int>(row_count), desc_cols, CV_8U);
    }

    for (uint32_t i = 0; i < row_count; ++i) {
        cv::KeyPoint kp;
        uint64_t map_point_id = 0;
        double bx = 0.0;
        double by = 0.0;
        double bz = 1.0;
        if (!read_keypoint(ptr, remaining, kp) || !read_bytes(ptr, remaining, map_point_id) ||
            !read_bytes(ptr, remaining, bx) || !read_bytes(ptr, remaining, by) ||
            !read_bytes(ptr, remaining, bz)) {
            return std::nullopt;
        }

        kf.keypoints.push_back(kp);
        kf.map_point_ids.push_back(map_point_id);
        kf.bearings.emplace_back(bx, by, bz);

        if (desc_cols > 0) {
            if (remaining < static_cast<size_t>(desc_cols)) {
                return std::nullopt;
            }
            std::memcpy(kf.descriptors.ptr(static_cast<int>(i)), ptr,
                        static_cast<size_t>(desc_cols));
            ptr += desc_cols;
            remaining -= static_cast<size_t>(desc_cols);
        }
    }

    return kf;
}

} // namespace drone::slam
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
