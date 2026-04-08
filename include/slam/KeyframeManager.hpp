// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once
 
// KeyframeManager.hpp    Keyframe-based SLAM + swarm map sharing
// Only feature descriptors & 3D points are shared; raw frames stay local.
// Drone Swarm Sensor Fusion  |  Phase 3  SLAM
 
#include "swarm/V2XMeshNetwork.hpp"
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace drone::slam {

//  Map point 
struct MapPoint {
    uint64_t        id;
    Eigen::Vector3d pos_world;  // 3D world position
    cv::Mat         descriptor; // 32-byte ORB descriptor
    uint32_t        owner_id;   // drone that first observed this point
    uint32_t        obs_count{1};
    double          last_seen_ts{0.0};
    float           confidence{1.0f};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//  Keyframe â”€
struct Keyframe {
    uint64_t   id;
    uint32_t   drone_id;
    double     timestamp;

    Eigen::Vector3d    position;
    Eigen::Quaterniond orientation;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat                   descriptors;        // Nx32 ORB
    std::vector<uint64_t>     map_point_ids;      // indices into local/shared map
    std::vector<Eigen::Vector3d> bearings;        // unit bearing vectors

    // True if this keyframe has been shared with peers
    bool shared{false};

    // Approximate bandwidth cost estimate (bytes)
    [[nodiscard]] size_t serialized_size() const {
        return descriptors.total() * descriptors.elemSize()
             + keypoints.size() * sizeof(cv::KeyPoint)
             + sizeof(Keyframe);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

 
// KeyframeSelector    decides when a new keyframe should be created
 
struct KeyframeSelectionPolicy {
    float   min_translation_m{0.4f};   // min travel before new KF
    float   min_rotation_deg{10.0f};   // min rotation before new KF
    float   max_point_overlap{0.9f};   // max covisibility with last KF
    double  min_time_s{0.5};           // minimum time interval
    size_t  min_tracked_features{20};  // must track â‰¥ N features
};

 
class KeyframeManager {
public:
    struct RelocalizationResult {
        Eigen::Vector3d corrected_position{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond corrected_orientation{Eigen::Quaterniond::Identity()};
        double confidence{0.0};
        uint64_t matched_keyframe_id{0};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct Status {
        size_t local_keyframes{0};
        size_t remote_keyframes{0};
        size_t map_points{0};
        size_t loop_candidates{0};
        uint32_t relocalization_count{0};
        double last_relocalization_confidence{0.0};
        uint64_t last_relocalized_keyframe{0};
    };

    explicit KeyframeManager(uint32_t drone_id,
                              std::shared_ptr<swarm::V2XMeshNetwork> net,
                              KeyframeSelectionPolicy policy = {});

    //  Add a new observation 
    //   Returns new Keyframe ID if one was created, else std::nullopt
    std::optional<uint64_t> try_add_frame(
        const cv::Mat&                image,
        const Eigen::Vector3d&        pos,
        const Eigen::Quaterniond&     ori,
        double                        timestamp);

    //  Local map â”€
    [[nodiscard]] std::vector<MapPoint>  get_local_map_points() const;
    [[nodiscard]] std::vector<Keyframe>  get_recent_keyframes(size_t n = 10) const;
    [[nodiscard]] size_t                 keyframe_count() const;
    [[nodiscard]] size_t                 map_point_count() const;
    [[nodiscard]] Status                 status() const;

    //  Swarm sharing 
    void share_latest_keyframe();   // encode + broadcast via V2X
    void on_remote_keyframe(const swarm::SwarmMessage& msg);  // ingest

    //  Loop closure hint â”€
    //   Returns candidate Keyframe IDs that may close a loop
    [[nodiscard]] std::vector<uint64_t> find_loop_closure_candidates(
        const cv::Mat& query_desc, size_t top_k = 5) const;
    [[nodiscard]] std::optional<RelocalizationResult> attempt_relocalization(
        const cv::Mat& image,
        const Eigen::Vector3d& pose_guess,
        const Eigen::Quaterniond& orientation_guess) const;

    //  Persistence â”€
    bool save_map(const std::string& filepath) const;
    bool load_map(const std::string& filepath);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    bool should_create_keyframe(const Eigen::Vector3d& pos,
                                 const Eigen::Quaterniond& ori,
                                 double ts,
                                 size_t tracked_features) const;

    uint64_t next_kf_id() { return kf_counter_++; }
    uint64_t next_mp_id() { return mp_counter_++; }

    std::vector<uint64_t> extract_and_triangulate(
        const cv::Mat& image, Keyframe& kf);

    // Serialization helpers
    std::vector<uint8_t> encode_keyframe(const Keyframe& kf) const;
    std::optional<Keyframe> decode_keyframe(const uint8_t* data, size_t len) const;

    uint32_t drone_id_;
    std::shared_ptr<swarm::V2XMeshNetwork> net_;
    KeyframeSelectionPolicy policy_;

    mutable std::mutex                  map_mutex_;
    std::deque<Keyframe>                keyframes_;       // local keyframes
    std::map<uint64_t, MapPoint>        map_points_;      // local + merged remote

    // Remote keyframes from peers (for loop closure)
    std::vector<Keyframe>               remote_keyframes_;

    std::atomic<uint64_t> kf_counter_{0};
    std::atomic<uint64_t> mp_counter_{0};
    mutable std::atomic<uint32_t> relocalization_count_{0};
    mutable std::atomic<size_t>   last_loop_candidate_count_{0};
    mutable std::atomic<double>   last_relocalization_confidence_{0.0};
    mutable std::atomic<uint64_t> last_relocalized_keyframe_{0};

    // Feature detector/descriptor
    cv::Ptr<cv::ORB>         orb_;
    cv::Ptr<cv::BFMatcher>   matcher_;

    // Last keyframe state (for selection policy)
    Eigen::Vector3d      last_kf_pos_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond   last_kf_ori_{Eigen::Quaterniond::Identity()};
    double               last_kf_ts_{-1.0};

    std::shared_ptr<spdlog::logger> logger_{spdlog::get("SLAM")};

    static constexpr size_t kMaxLocalKeyframes{500};
    static constexpr size_t kMaxSharedDescSize{4096}; // bytes per KF share
};

} // namespace drone::slam
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
