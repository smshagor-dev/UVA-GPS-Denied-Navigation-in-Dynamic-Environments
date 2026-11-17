// Research utility: raw EuRoC IMU-only replay to the existing active EKF and shadow ESKF.
// It does not decode cameras, ingest ground truth, or produce a VIO benchmark.
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

struct Sample {
    uint64_t timestamp_ns{0};
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
};

std::vector<std::string> split_csv(const std::string& line) {
    std::istringstream stream(line);
    std::vector<std::string> fields;
    std::string value;
    while (std::getline(stream, value, ',')) {
        fields.push_back(value);
    }
    return fields;
}

std::string trimmed(std::string value) {
    const auto non_space = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), non_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), non_space).base(), value.end());
    return value;
}

uint64_t parse_timestamp(const std::string& value) {
    const std::string token = trimmed(value);
    if (token.empty() ||
        !std::all_of(token.begin(), token.end(),
                     [](unsigned char c) { return c >= '0' && c <= '9'; })) {
        throw std::runtime_error("timestamp is not an unsigned decimal nanosecond integer");
    }
    size_t consumed = 0;
    const auto result = std::stoull(token, &consumed, 10);
    if (consumed != token.size()) {
        throw std::runtime_error("invalid timestamp");
    }
    return static_cast<uint64_t>(result);
}

double parse_scalar(const std::string& value) {
    const auto token = trimmed(value);
    size_t consumed = 0;
    const double result = std::stod(token, &consumed);
    if (consumed != token.size() || !std::isfinite(result)) {
        throw std::runtime_error("invalid or nonfinite IMU value");
    }
    return result;
}

std::vector<Sample> load_imu(const fs::path& input) {
    if (!fs::is_regular_file(input) || fs::is_symlink(input)) {
        throw std::runtime_error("input must be a regular non-symlink imu0/data.csv");
    }
    if (input.filename() != "data.csv" || input.parent_path().filename() != "imu0") {
        throw std::runtime_error("expected genuine-looking imu0/data.csv path");
    }
    std::ifstream stream(input);
    if (!stream) {
        throw std::runtime_error("could not read imu0/data.csv");
    }
    std::string line;
    if (!std::getline(stream, line)) {
        throw std::runtime_error("missing IMU header");
    }
    const auto header = split_csv(line);
    const std::array<std::string, 6> names{"w_RS_S_x", "w_RS_S_y", "w_RS_S_z",
                                           "a_RS_S_x", "a_RS_S_y", "a_RS_S_z"};
    if (header.size() != 7 || trimmed(header[0]).rfind("#timestamp", 0) != 0) {
        throw std::runtime_error("invalid EuRoC IMU header");
    }
    for (size_t j = 0; j < names.size(); ++j) {
        if (trimmed(header[j + 1]).rfind(names[j], 0) != 0) {
            throw std::runtime_error("unexpected IMU column ordering");
        }
    }
    std::vector<Sample> samples;
    uint64_t previous_ns = 0;
    while (std::getline(stream, line)) {
        const auto fields = split_csv(line);
        if (fields.size() != 7) {
            throw std::runtime_error("malformed IMU sample row");
        }
        Sample sample;
        sample.timestamp_ns = parse_timestamp(fields[0]);
        if (!samples.empty() && sample.timestamp_ns <= previous_ns) {
            throw std::runtime_error("non-monotonic IMU nanosecond timestamp");
        }
        previous_ns = sample.timestamp_ns;
        sample.gyro = {parse_scalar(fields[1]), parse_scalar(fields[2]),
                       parse_scalar(fields[3])};
        sample.accel = {parse_scalar(fields[4]), parse_scalar(fields[5]),
                        parse_scalar(fields[6])};
        samples.push_back(sample);
        if (samples.size() > 2000000) {
            throw std::runtime_error("IMU sample limit exceeded");
        }
    }
    if (stream.bad() || samples.size() < 2) {
        throw std::runtime_error("IMU stream unreadable or too short");
    }
    return samples;
}

std::string replay(const std::vector<Sample>& samples) {
    if (samples.size() < 2) {
        throw std::runtime_error("at least two IMU samples required");
    }
    EKFStateEstimatorAdapter active;
    Phase17StateEstimatorAdapter shadow;
    // Deliberate non-GT initialization, shared by both estimators.
    active.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                 Eigen::Vector3d::Zero());
    shadow.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                 Eigen::Vector3d::Zero());

    std::ostringstream output;
    output << std::setprecision(17);
    output << "timestamp_ns,active_x_m,active_y_m,active_z_m,"
              "shadow_x_m,shadow_y_m,shadow_z_m\n";
    const uint64_t first = samples.front().timestamp_ns;
    for (size_t i = 0; i < samples.size(); ++i) {
        const auto& s = samples[i];
        if (i > 0 && s.timestamp_ns <= samples[i - 1].timestamp_ns) {
            throw std::runtime_error("non-monotonic input sequence");
        }
        if (!s.accel.array().isFinite().all() || !s.gyro.array().isFinite().all()) {
            throw std::runtime_error("nonfinite IMU input");
        }
        MeasurementEnvelope envelope;
        envelope.type = MeasurementType::Imu;
        envelope.frame = MeasurementFrame::Body;
        envelope.source_id = "euroc_imu0";
        envelope.sequence_id = static_cast<uint64_t>(i);
        // Use a relative clock to avoid precision loss from 2014 epoch ns->double.
        envelope.timestamp_s = 1.0 + static_cast<double>(s.timestamp_ns - first) * 1.0e-9;
        ImuMeasurementPayload payload;
        payload.gyro_rads = s.gyro;
        payload.accel_mps2 = s.accel;
        envelope.payload = payload;

        if (active.process_measurement(envelope) != EstimatorOperationResult::Accepted ||
            shadow.process_measurement(envelope) != EstimatorOperationResult::Accepted) {
            throw std::runtime_error("native estimator rejected IMU sample " + std::to_string(i));
        }
        const auto a = active.snapshot();
        const auto b = shadow.snapshot();
        if (!a.initialized || !b.initialized || !a.position_m.array().isFinite().all() ||
            !b.position_m.array().isFinite().all()) {
            throw std::runtime_error("invalid estimator snapshot");
        }
        output << s.timestamp_ns << "," << a.position_m.x() << "," << a.position_m.y()
               << "," << a.position_m.z() << "," << b.position_m.x() << ","
               << b.position_m.y() << "," << b.position_m.z() << "\n";
    }
    return output.str();
}

void self_test() {
    const uint64_t t = 1403636579763555584ULL;
    const std::vector<Sample> samples{
        {t, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.81)},
        {t + 5000000, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.81)},
        {t + 10000000, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.81)},
    };
    const auto text = replay(samples);
    if (std::count(text.begin(), text.end(), '\n') != 4) {
        throw std::runtime_error("self test expected header and three samples");
    }
    auto invalid = samples;
    invalid[2].timestamp_ns = invalid[1].timestamp_ns;
    try {
        (void)replay(invalid);
        throw std::runtime_error("non-monotonic sample not rejected");
    } catch (const std::runtime_error& ex) {
        if (std::string(ex.what()).find("non-monotonic") == std::string::npos) {
            throw;
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--self-test") {
            self_test();
            std::cout << "PASS: synthetic IMU-to-active/shadow estimator replay\n";
            return 0;
        }
        if (argc != 6 || std::string(argv[1]) != "--imu-csv" ||
            std::string(argv[3]) != "--output" ||
            std::string(argv[5]) != "--assume-imu-frame-identity") {
            std::cerr << "Usage: euroc_imu_native_replay --imu-csv <imu0/data.csv> "
                         "--output <raw_baseline.csv> --assume-imu-frame-identity\n";
            return 2;
        }
        const fs::path input = argv[2];
        const fs::path output = argv[4];
        if (fs::exists(output)) {
            throw std::runtime_error("refusing to overwrite existing output");
        }
        if (!fs::is_directory(output.parent_path())) {
            throw std::runtime_error("output parent directory must already exist");
        }
        const auto samples = load_imu(input);
        const auto contents = replay(samples);
        std::ofstream stream(output, std::ios::binary | std::ios::out | std::ios::trunc);
        if (!stream) {
            throw std::runtime_error("could not create output");
        }
        stream << contents;
        stream.close();
        if (!stream) {
            fs::remove(output);
            throw std::runtime_error("failed to write output completely");
        }
        std::cout << "Processed " << samples.size()
                  << " actual IMU rows, identity-init dead reckoning ONLY; "
                     "no camera, GT, VIO or physical swarm evidence.\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "EuRoC replay rejected: " << ex.what() << "\n";
        return 1;
    }
}
