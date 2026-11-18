// Native EuRoC sensor fusion replay: real IMU CSV and independently produced
// camera-derived visual odometry CSV, NEVER EuRoC publisher ground truth.
// Visual position/velocity and quaternion attitude updates share the VO frame.
// Angular uncertainty is provisional; this is NOT a validated 6-DoF benchmark.
#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/Phase17ESKFEstimator.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace {
namespace fs = std::filesystem;

using drone::vio::EKFStateEstimatorAdapter;
using drone::vio::EstimatorOperationResult;
using drone::vio::ImuMeasurementPayload;
using drone::vio::MeasurementEnvelope;
using drone::vio::MeasurementFrame;
using drone::vio::MeasurementType;
using drone::vio::Phase17StateEstimatorAdapter;
using drone::vio::VisualAttitudeMeasurementPayload;
using drone::vio::VisualPoseMeasurementPayload;

struct ImuSample {
    uint64_t timestamp_ns{0};
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
};

struct VisualSample {
    uint64_t timestamp_ns{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    double sigma_position_m{0.5};
    double sigma_velocity_mps{0.75};
    size_t inliers{0};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    double sigma_orientation_rad{0.25};
};

struct Result {
    std::string active_csv{};
    std::string shadow_csv{};
    size_t real_imu_count{0};
    size_t interpolated_imu_count{0};
    size_t visual_update_count{0};
};

std::vector<std::string> split_csv(const std::string& line) {
    std::istringstream stream(line);
    std::vector<std::string> columns;
    std::string token;
    while (std::getline(stream, token, ',')) {
        columns.push_back(token);
    }
    return columns;
}

std::string trimmed(std::string s) {
    const auto non_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), non_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), non_space).base(), s.end());
    return s;
}

uint64_t decimal_timestamp(const std::string& text) {
    const std::string value = trimmed(text);
    if (value.empty() || !std::all_of(value.begin(), value.end(),
                                      [](unsigned char c) { return c >= '0' && c <= '9'; })) {
        throw std::runtime_error("timestamp_ns must be an unsigned decimal integer");
    }
    size_t used = 0;
    const auto v = std::stoull(value, &used, 10);
    if (used != value.size()) {
        throw std::runtime_error("invalid timestamp_ns");
    }
    return static_cast<uint64_t>(v);
}

double finite_scalar(const std::string& field) {
    size_t used = 0;
    const double value = std::stod(trimmed(field), &used);
    if (used != trimmed(field).size() || !std::isfinite(value)) {
        throw std::runtime_error("nonfinite or invalid source numeric value");
    }
    return value;
}

void check_regular_input(const fs::path& path) {
    if (!fs::is_regular_file(path) || fs::is_symlink(path)) {
        throw std::runtime_error("source file must be a regular non-symlink CSV");
    }
}

std::vector<ImuSample> load_imu(const fs::path& path) {
    check_regular_input(path);
    if (path.filename() != "data.csv" || path.parent_path().filename() != "imu0") {
        throw std::runtime_error("expected original imu0/data.csv");
    }
    std::ifstream in(path);
    std::string line;
    if (!std::getline(in, line)) {
        throw std::runtime_error("empty IMU source");
    }
    const auto headers = split_csv(line);
    const std::array<std::string, 6> columns{"w_RS_S_x", "w_RS_S_y", "w_RS_S_z",
                                             "a_RS_S_x", "a_RS_S_y", "a_RS_S_z"};
    if (headers.size() != 7 || trimmed(headers.front()).rfind("#timestamp", 0) != 0) {
        throw std::runtime_error("unexpected IMU source header");
    }
    for (size_t i = 0; i < columns.size(); ++i) {
        if (trimmed(headers[i + 1]).rfind(columns[i], 0) != 0) {
            throw std::runtime_error("unexpected EuRoC IMU column ordering");
        }
    }
    std::vector<ImuSample> samples;
    while (std::getline(in, line)) {
        const auto cells = split_csv(line);
        if (cells.size() != 7) {
            throw std::runtime_error("invalid IMU sample column count");
        }
        ImuSample s;
        s.timestamp_ns = decimal_timestamp(cells[0]);
        if (!samples.empty() && s.timestamp_ns <= samples.back().timestamp_ns) {
            throw std::runtime_error("nonmonotonic IMU sample timestamp");
        }
        s.gyro = {finite_scalar(cells[1]), finite_scalar(cells[2]), finite_scalar(cells[3])};
        s.accel = {finite_scalar(cells[4]), finite_scalar(cells[5]), finite_scalar(cells[6])};
        samples.push_back(s);
        if (samples.size() > 2000000) {
            throw std::runtime_error("IMU input size cap exceeded");
        }
    }
    if (in.bad() || samples.size() < 2) {
        throw std::runtime_error("IMU input truncated or insufficient");
    }
    return samples;
}

std::vector<VisualSample> load_visual(const fs::path& path) {
    check_regular_input(path);
    std::ifstream in(path);
    std::string line;
    if (!std::getline(in, line)) {
        throw std::runtime_error("empty visual source");
    }
    const auto columns = split_csv(line);
    const std::array<std::string, 16> expected{"timestamp_ns",
                                               "x_m",
                                               "y_m",
                                               "z_m",
                                               "vx_mps",
                                               "vy_mps",
                                               "vz_mps",
                                               "sigma_position_m",
                                               "sigma_velocity_mps",
                                               "pnp_inliers",
                                               "pnp_reprojection_rmse_px",
                                               "qw",
                                               "qx",
                                               "qy",
                                               "qz",
                                               "sigma_orientation_rad"};
    if (columns.size() != expected.size()) {
        throw std::runtime_error("unexpected visual input schema");
    }
    for (size_t j = 0; j < columns.size(); ++j) {
        if (trimmed(columns[j]) != expected[j]) {
            throw std::runtime_error("unsupported visual column or order");
        }
    }
    std::vector<VisualSample> result;
    while (std::getline(in, line)) {
        const auto cells = split_csv(line);
        if (cells.size() != expected.size()) {
            throw std::runtime_error("invalid visual row column count");
        }
        VisualSample s;
        s.timestamp_ns = decimal_timestamp(cells[0]);
        if (!result.empty() && s.timestamp_ns <= result.back().timestamp_ns) {
            throw std::runtime_error("nonmonotonic visual timestamps");
        }
        s.position = {finite_scalar(cells[1]), finite_scalar(cells[2]), finite_scalar(cells[3])};
        s.velocity = {finite_scalar(cells[4]), finite_scalar(cells[5]), finite_scalar(cells[6])};
        s.sigma_position_m = finite_scalar(cells[7]);
        s.sigma_velocity_mps = finite_scalar(cells[8]);
        const uint64_t n_inliers = decimal_timestamp(cells[9]);
        const double reprojection_rmse_px = finite_scalar(cells[10]);
        if (n_inliers < 8 || n_inliers > 100000 || s.sigma_position_m <= 0.0 ||
            s.sigma_velocity_mps <= 0.0 || s.sigma_position_m > 100 || s.sigma_velocity_mps > 100 ||
            reprojection_rmse_px < 0.0 || reprojection_rmse_px > 3.0 || s.velocity.norm() > 30.0) {
            throw std::runtime_error("unsupported visual quality/sigma values");
        }
        s.inliers = static_cast<size_t>(n_inliers);
        s.orientation = Eigen::Quaterniond(finite_scalar(cells[11]), finite_scalar(cells[12]),
                                           finite_scalar(cells[13]), finite_scalar(cells[14]));
        s.sigma_orientation_rad = finite_scalar(cells[15]);
        if (!s.orientation.coeffs().array().isFinite().all() ||
            std::abs(s.orientation.squaredNorm() - 1.0) > 0.02 || s.sigma_orientation_rad <= 0.0 ||
            s.sigma_orientation_rad > 1.0) {
            throw std::runtime_error("invalid visual orientation or attitude uncertainty");
        }
        s.orientation.normalize();
        result.push_back(s);
        if (result.size() > 500000) {
            throw std::runtime_error("visual source size cap exceeded");
        }
    }
    if (in.bad() || result.empty()) {
        throw std::runtime_error("visual input truncated or empty");
    }
    return result;
}

Result replay(const std::vector<ImuSample>& imu, const std::vector<VisualSample>& vision) {
    if (imu.size() < 2 || vision.empty()) {
        throw std::runtime_error("IMU and visual observations required");
    }
    if (vision.front().timestamp_ns <= imu.front().timestamp_ns ||
        vision.back().timestamp_ns > imu.back().timestamp_ns) {
        throw std::runtime_error("visual timestamps outside original IMU support");
    }
    EKFStateEstimatorAdapter active;
    Phase17StateEstimatorAdapter shadow;
    // Identity is a declared prior, not a GT-derived initial orientation.
    active.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    shadow.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    Result result;
    std::ostringstream active_csv;
    std::ostringstream shadow_csv;
    active_csv << std::setprecision(17);
    shadow_csv << std::setprecision(17);
    const std::string header = "timestamp_ns,x_m,y_m,z_m,qw,qx,qy,qz\n";
    active_csv << header;
    shadow_csv << header;
    const uint64_t origin = imu.front().timestamp_ns;
    uint64_t last_submitted = 0;
    uint64_t seq = 0;
    size_t visual_index = 0;

    const auto timestamp_s = [origin](uint64_t ns) {
        if (ns < origin) {
            throw std::runtime_error("timestamp before IMU origin");
        }
        return 1.0 + static_cast<double>(ns - origin) * 1.0e-9;
    };
    const auto emit_imu = [&](const ImuSample& sample) {
        if (last_submitted && sample.timestamp_ns <= last_submitted) {
            throw std::runtime_error("nonmonotonic merged IMU event");
        }
        MeasurementEnvelope e;
        e.type = MeasurementType::Imu;
        e.source_id = "euroc_imu0";
        e.frame = MeasurementFrame::Body;
        e.timestamp_s = timestamp_s(sample.timestamp_ns);
        e.sequence_id = ++seq;
        ImuMeasurementPayload payload;
        payload.accel_mps2 = sample.accel;
        payload.gyro_rads = sample.gyro;
        e.payload = payload;
        if (active.process_measurement(e) != EstimatorOperationResult::Accepted ||
            shadow.process_measurement(e) != EstimatorOperationResult::Accepted) {
            throw std::runtime_error("native IMU propagation rejected at " +
                                     std::to_string(sample.timestamp_ns));
        }
        last_submitted = sample.timestamp_ns;
    };
    const auto emit_vision = [&](const VisualSample& s) {
        MeasurementEnvelope e;
        e.type = MeasurementType::VisualPose;
        e.source_id = "euroc_stereo_pnp_vo";
        e.frame = MeasurementFrame::World;
        e.timestamp_s = timestamp_s(s.timestamp_ns);
        e.sequence_id = ++seq;
        VisualPoseMeasurementPayload payload;
        payload.position_m = s.position;
        payload.velocity_mps = s.velocity;
        payload.sigma_position_m = s.sigma_position_m;
        payload.sigma_velocity_mps = s.sigma_velocity_mps;
        e.payload = payload;
        if (active.process_measurement(e) != EstimatorOperationResult::Accepted ||
            shadow.process_measurement(e) != EstimatorOperationResult::Accepted) {
            throw std::runtime_error("native visual pose update rejected at " +
                                     std::to_string(s.timestamp_ns));
        }
        e.type = MeasurementType::VisualAttitude;
        e.source_id = "euroc_stereo_pnp_attitude";
        e.sequence_id = ++seq;
        VisualAttitudeMeasurementPayload attitude;
        attitude.world_from_imu = s.orientation;
        attitude.sigma_orientation_rad = s.sigma_orientation_rad;
        e.payload = attitude;
        if (active.process_measurement(e) != EstimatorOperationResult::Accepted ||
            shadow.process_measurement(e) != EstimatorOperationResult::Accepted) {
            throw std::runtime_error("native visual attitude update rejected at " +
                                     std::to_string(s.timestamp_ns));
        }
        const auto a = active.snapshot();
        const auto b = shadow.snapshot();
        if (!a.initialized || !b.initialized || !a.position_m.allFinite() ||
            !b.position_m.allFinite() || !a.orientation.coeffs().allFinite() ||
            !b.orientation.coeffs().allFinite() ||
            std::abs(a.orientation.squaredNorm() - 1.0) > 0.01 ||
            std::abs(b.orientation.squaredNorm() - 1.0) > 0.01) {
            throw std::runtime_error("invalid fused estimator state or quaternion");
        }
        active_csv << s.timestamp_ns << "," << a.position_m.x() << "," << a.position_m.y() << ","
                   << a.position_m.z() << "," << a.orientation.w() << "," << a.orientation.x()
                   << "," << a.orientation.y() << "," << a.orientation.z() << "\n";
        shadow_csv << s.timestamp_ns << "," << b.position_m.x() << "," << b.position_m.y() << ","
                   << b.position_m.z() << "," << b.orientation.w() << "," << b.orientation.x()
                   << "," << b.orientation.y() << "," << b.orientation.z() << "\n";
        ++result.visual_update_count;
    };

    // Align each visual observation between surrounding original IMU samples.
    // Intermediate IMU values are explicitly LINEARLY INTERPOLATED at the
    // camera timestamp; never silently round a camera timestamp to its neighbor.
    for (size_t i = 0; i < imu.size(); ++i) {
        const auto& next = imu[i];
        if (i > 0 && next.timestamp_ns <= imu[i - 1].timestamp_ns) {
            throw std::runtime_error("nonmonotonic IMU source");
        }
        while (visual_index < vision.size() &&
               vision[visual_index].timestamp_ns < next.timestamp_ns) {
            if (i == 0) {
                throw std::runtime_error("visual timestamp precedes IMU");
            }
            const auto& prev = imu[i - 1];
            const uint64_t visual_ns = vision[visual_index].timestamp_ns;
            if (visual_ns < prev.timestamp_ns ||
                next.timestamp_ns - prev.timestamp_ns > 25000000ULL) {
                throw std::runtime_error("unsupported camera/IMU bracket gap");
            }
            if (visual_ns > prev.timestamp_ns) {
                const double fraction = static_cast<double>(visual_ns - prev.timestamp_ns) /
                                        static_cast<double>(next.timestamp_ns - prev.timestamp_ns);
                ImuSample estimated;
                estimated.timestamp_ns = visual_ns;
                estimated.gyro = (1.0 - fraction) * prev.gyro + fraction * next.gyro;
                estimated.accel = (1.0 - fraction) * prev.accel + fraction * next.accel;
                emit_imu(estimated);
                ++result.interpolated_imu_count;
            }
            emit_vision(vision[visual_index]);
            ++visual_index;
        }
        if (next.timestamp_ns > last_submitted) {
            emit_imu(next);
            ++result.real_imu_count;
        }
        while (visual_index < vision.size() &&
               vision[visual_index].timestamp_ns == next.timestamp_ns) {
            emit_vision(vision[visual_index]);
            ++visual_index;
        }
    }
    if (visual_index != vision.size()) {
        throw std::runtime_error("unprocessed visual measurements remain");
    }
    result.active_csv = active_csv.str();
    result.shadow_csv = shadow_csv.str();
    return result;
}

void self_test() {
    const uint64_t start = 1403636579763555584ULL;
    std::vector<ImuSample> imu;
    for (uint64_t n = 0; n <= 15; ++n) {
        imu.push_back({
            start + n * 5000000ULL,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d{0.0, 0.0, 9.81},
        });
    }
    std::vector<VisualSample> visual{
        {start + 22000000ULL, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.5, 0.75, 36},
        {start + 50000000ULL, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.5, 0.75, 32},
    };
    visual[1].orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitZ()));
    const auto good = replay(imu, visual);
    if (good.visual_update_count != 2 || good.interpolated_imu_count != 1 ||
        std::count(good.active_csv.begin(), good.active_csv.end(), '\n') != 3 ||
        std::count(good.shadow_csv.begin(), good.shadow_csv.end(), '\n') != 3) {
        throw std::runtime_error("synthetic fused replay self-test failed");
    }
    std::istringstream trajectory(good.active_csv);
    std::string current_line;
    std::getline(trajectory, current_line); // header
    std::getline(trajectory, current_line); // first visual pose
    std::getline(trajectory, current_line); // second visual pose
    const auto columns = split_csv(current_line);
    if (columns.size() != 8 || std::stod(columns[7]) < 0.001) {
        throw std::runtime_error("native fused visual orientation did not update");
    }
    auto invalid = visual;
    invalid[1].timestamp_ns = invalid[0].timestamp_ns - 1;
    try {
        (void)replay(imu, invalid);
        throw std::runtime_error("bad visual chronology accepted");
    } catch (const std::runtime_error& e) {
        if (std::string(e.what()).find("bad visual chronology accepted") != std::string::npos) {
            throw;
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--self-test") {
            self_test();
            std::cout << "PASS: synthetic fused IMU and visual pose self-test\n";
            return 0;
        }
        if (argc != 10 || std::string(argv[1]) != "--imu-csv" ||
            std::string(argv[3]) != "--visual-csv" || std::string(argv[5]) != "--active-output" ||
            std::string(argv[7]) != "--shadow-output" ||
            std::string(argv[9]) != "--assume-initial-imu-identity") {
            std::cerr << "Usage: euroc_stereo_imu_native_fusion --imu-csv <imu0/data.csv> "
                         "--visual-csv <visual.csv> --active-output <active.csv> "
                         "--shadow-output <shadow.csv> --assume-initial-imu-identity\n";
            return 2;
        }
        const fs::path imu_path = argv[2];
        const fs::path visual_path = argv[4];
        const fs::path active_path = argv[6];
        const fs::path shadow_path = argv[8];
        if (active_path == shadow_path || fs::exists(active_path) || fs::exists(shadow_path) ||
            fs::is_symlink(active_path) || fs::is_symlink(shadow_path) ||
            !fs::is_directory(active_path.parent_path()) ||
            !fs::is_directory(shadow_path.parent_path())) {
            throw std::runtime_error("output paths must differ, not exist, and have parents");
        }
        // The source data tree is immutable; resolve output parents and any
        // symlink ancestors before creating a new trajectory file.
        const fs::path source_root =
            fs::weakly_canonical(imu_path.parent_path().parent_path().parent_path());
        const auto output_within_source = [&source_root](const fs::path& path) {
            const fs::path resolved = fs::weakly_canonical(path);
            auto actual = resolved.begin();
            for (auto source = source_root.begin(); source != source_root.end(); ++source) {
                if (actual == resolved.end() || *actual != *source) {
                    return false;
                }
                ++actual;
            }
            return true;
        };
        if (output_within_source(active_path) || output_within_source(shadow_path)) {
            throw std::runtime_error("output trajectories must remain outside source sequence");
        }
        const auto imu = load_imu(imu_path);
        const auto visual = load_visual(visual_path);
        const auto output = replay(imu, visual);
        try {
            {
                std::ofstream a(active_path, std::ios::out | std::ios::trunc);
                if (!a || !(a << output.active_csv)) {
                    throw std::runtime_error("could not write active trajectory");
                }
                a.close();
                if (!a) {
                    throw std::runtime_error("could not finalize active trajectory");
                }
            }
            {
                std::ofstream b(shadow_path, std::ios::out | std::ios::trunc);
                if (!b || !(b << output.shadow_csv)) {
                    throw std::runtime_error("could not write shadow trajectory");
                }
                b.close();
                if (!b) {
                    throw std::runtime_error("could not finalize shadow trajectory");
                }
            }
        } catch (...) {
            std::error_code cleanup_error;
            fs::remove(active_path, cleanup_error);
            fs::remove(shadow_path, cleanup_error);
            throw;
        }
        std::cout << "IMU: " << output.real_imu_count << " original rows; "
                  << output.interpolated_imu_count << " interpolated camera-time rows; "
                  << output.visual_update_count
                  << " visual updates accepted by both filters. "
                     "No ground truth used. No physical UAV evidence.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "EuRoC visual-inertial replay failed closed: " << e.what() << "\n";
        return 1;
    }
}
