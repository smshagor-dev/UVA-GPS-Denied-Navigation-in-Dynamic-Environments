#include "estimation/ReplayRunner.hpp"

#include "security/FirmwareTrust.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace drone::estimation {

namespace {

using json = nlohmann::json;

template <typename T, size_t N> std::array<T, N> require_array(const json& node, const char* key) {
    if (!node.contains(key) || !node.at(key).is_array() || node.at(key).size() != N) {
        throw std::runtime_error(std::string(key) + " must be an array of length " +
                                 std::to_string(N));
    }
    std::array<T, N> out{};
    for (size_t i = 0; i < N; ++i) {
        out[i] = node.at(key).at(i).get<T>();
        if constexpr (std::is_floating_point_v<T>) {
            if (!std::isfinite(out[i])) {
                throw std::runtime_error(std::string(key) + " must contain finite values");
            }
        }
    }
    return out;
}

Eigen::Vector3d require_vec3(const json& node, const char* key) {
    const auto values = require_array<double, 3>(node, key);
    return Eigen::Vector3d{values[0], values[1], values[2]};
}

double require_finite_number(const json& node, const char* key) {
    if (!node.contains(key) || !node.at(key).is_number()) {
        throw std::runtime_error(std::string(key) + " must be a number");
    }
    const double value = node.at(key).get<double>();
    if (!std::isfinite(value)) {
        throw std::runtime_error(std::string(key) + " must be finite");
    }
    return value;
}

std::string require_string_bounded(const json& node, const char* key, size_t max_length) {
    if (!node.contains(key) || !node.at(key).is_string()) {
        throw std::runtime_error(std::string(key) + " must be a string");
    }
    const auto value = node.at(key).get<std::string>();
    if (value.empty() || value.size() > max_length) {
        throw std::runtime_error(std::string(key) + " length is invalid");
    }
    return value;
}

CoordinateFrame parse_coordinate_frame(std::string_view frame) {
    if (frame == "world" || frame == "local_enu") {
        return CoordinateFrame::WORLD;
    }
    if (frame == "body") {
        return CoordinateFrame::BODY;
    }
    throw std::runtime_error("unsupported coordinate frame");
}

std::string health_to_string(drone::vio::EstimatorHealthState state) {
    switch (state) {
    case drone::vio::EstimatorHealthState::INITIALIZING:
        return "initializing";
    case drone::vio::EstimatorHealthState::NOMINAL:
        return "nominal";
    case drone::vio::EstimatorHealthState::DEGRADED:
        return "degraded";
    case drone::vio::EstimatorHealthState::REJECTING_MEASUREMENTS:
        return "rejecting_measurements";
    case drone::vio::EstimatorHealthState::NUMERICAL_WARNING:
        return "numerical_warning";
    case drone::vio::EstimatorHealthState::INVALID:
        return "invalid";
    }
    return "unknown";
}

std::string shadow_to_string(ShadowHealthState state) {
    switch (state) {
    case ShadowHealthState::DISABLED:
        return "disabled";
    case ShadowHealthState::STARTING:
        return "starting";
    case ShadowHealthState::SYNCHRONIZED:
        return "synchronized";
    case ShadowHealthState::LAGGING:
        return "lagging";
    case ShadowHealthState::STALE:
        return "stale";
    case ShadowHealthState::DIVERGED:
        return "diverged";
    case ShadowHealthState::FAILED:
        return "failed";
    case ShadowHealthState::STOPPED:
        return "stopped";
    }
    return "unknown";
}

std::string fixed_scalar(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(12) << value;
    return oss.str();
}

std::string canonical_hash_payload(const ReplayReport& report) {
    std::ostringstream oss;
    oss << report.report_schema_version << '\n'
        << report.replay_input_schema_version << '\n'
        << report.replay_mode << '\n'
        << report.active_estimator_name << '\n'
        << report.shadow_estimator_name << '\n'
        << report.coordinate_frame << '\n'
        << report.input_record_count << '\n'
        << report.processed_record_count << '\n'
        << report.accepted_update_count << '\n'
        << report.rejected_update_count << '\n'
        << report.invalid_record_count << '\n'
        << report.timestamp_violation_count << '\n'
        << report.unsupported_measurement_count << '\n'
        << fixed_scalar(report.final_pose.position.x()) << '\n'
        << fixed_scalar(report.final_pose.position.y()) << '\n'
        << fixed_scalar(report.final_pose.position.z()) << '\n'
        << fixed_scalar(report.final_pose.velocity.x()) << '\n'
        << fixed_scalar(report.final_pose.velocity.y()) << '\n'
        << fixed_scalar(report.final_pose.velocity.z()) << '\n'
        << fixed_scalar(report.final_pose.orientation.w()) << '\n'
        << fixed_scalar(report.final_pose.orientation.x()) << '\n'
        << fixed_scalar(report.final_pose.orientation.y()) << '\n'
        << fixed_scalar(report.final_pose.orientation.z()) << '\n'
        << fixed_scalar(report.final_pose.accel_bias.x()) << '\n'
        << fixed_scalar(report.final_pose.accel_bias.y()) << '\n'
        << fixed_scalar(report.final_pose.accel_bias.z()) << '\n'
        << fixed_scalar(report.final_pose.gyro_bias.x()) << '\n'
        << fixed_scalar(report.final_pose.gyro_bias.y()) << '\n'
        << fixed_scalar(report.final_pose.gyro_bias.z()) << '\n'
        << fixed_scalar(report.final_position_uncertainty_m) << '\n'
        << report.active_estimator_health << '\n'
        << report.shadow_estimator_health << '\n'
        << fixed_scalar(report.active_shadow_position_delta_m) << '\n'
        << fixed_scalar(report.active_shadow_velocity_delta_mps) << '\n'
        << fixed_scalar(report.active_shadow_orientation_delta_deg) << '\n'
        << report.comparable_snapshot_count << '\n'
        << report.skipped_comparison_count << '\n'
        << report.shadow_dropped_event_count << '\n'
        << report.shadow_queue_high_water_mark << '\n'
        << report.shadow_synchronization_state << '\n'
        << (report.success ? "1" : "0") << '\n'
        << report.failure_reason << '\n';
    return oss.str();
}

EstimatorMeasurement make_measurement_from_record(const json& record,
                                                  CoordinateFrame coordinate_frame,
                                                  uint64_t sequence_id, size_t max_string_length,
                                                  uint64_t& unsupported_count) {
    const auto type = require_string_bounded(record, "type", max_string_length);
    const double timestamp_s = require_finite_number(record, "timestamp_s");

    if (type == "imu") {
        EstimatorMeasurement measurement;
        measurement.header = {timestamp_s, sequence_id, SensorSource::IMU, CoordinateFrame::BODY,
                              MeasurementType::IMU};
        measurement.data = ImuMeasurementData{require_vec3(record, "accel_mps2"),
                                              require_vec3(record, "gyro_rads")};
        return measurement;
    }
    if (type == "visual_pose") {
        EstimatorMeasurement measurement;
        measurement.header = {timestamp_s, sequence_id, SensorSource::VISUAL_FRONTEND,
                              coordinate_frame, MeasurementType::VISUAL_POSE};
        VisualPoseMeasurementData visual;
        visual.position = require_vec3(record, "position_m");
        visual.velocity = require_vec3(record, "velocity_mps");
        visual.quality = 1.0;
        if (record.contains("quality")) {
            visual.quality = require_finite_number(record, "quality");
            if (visual.quality < 0.0 || visual.quality > 1.0) {
                throw std::runtime_error("quality must be within [0, 1]");
            }
        }
        const double sigma_position_m = require_finite_number(record, "sigma_position_m");
        const double sigma_velocity_mps = require_finite_number(record, "sigma_velocity_mps");
        if (sigma_position_m <= 0.0 || sigma_velocity_mps <= 0.0) {
            throw std::runtime_error("visual pose sigma must be positive");
        }
        visual.covariance.dimension = 6;
        visual.covariance.matrix.setZero();
        visual.covariance.matrix.block<3, 3>(0, 0) =
            Eigen::Matrix3d::Identity() * sigma_position_m * sigma_position_m;
        visual.covariance.matrix.block<3, 3>(3, 3) =
            Eigen::Matrix3d::Identity() * sigma_velocity_mps * sigma_velocity_mps;
        if (record.contains("orientation_wxyz")) {
            const auto q = require_array<double, 4>(record, "orientation_wxyz");
            Eigen::Quaterniond orientation(q[0], q[1], q[2], q[3]);
            if (!std::isfinite(orientation.norm()) || orientation.norm() < 1.0e-12) {
                throw std::runtime_error("orientation_wxyz is invalid");
            }
            visual.orientation = orientation.normalized();
        }
        measurement.data = std::move(visual);
        return measurement;
    }
    if (type == "depth" || type == "altitude") {
        EstimatorMeasurement measurement;
        measurement.header = {timestamp_s, sequence_id, SensorSource::DEPTH_SENSOR,
                              coordinate_frame, MeasurementType::ALTITUDE};
        const double altitude = record.contains("z_world_m")
                                    ? require_finite_number(record, "z_world_m")
                                    : require_finite_number(record, "altitude_m");
        const double sigma_m = require_finite_number(record, "sigma_m");
        if (sigma_m <= 0.0) {
            throw std::runtime_error("sigma_m must be positive");
        }
        measurement.data = AltitudeMeasurementData{altitude, sigma_m};
        return measurement;
    }
    if (type == "zupt") {
        EstimatorMeasurement measurement;
        measurement.header = {timestamp_s, sequence_id, SensorSource::MANUAL_ZUPT,
                              CoordinateFrame::BODY, MeasurementType::ZERO_VELOCITY};
        measurement.data = ZeroVelocityMeasurementData{};
        return measurement;
    }
    if (type == "tdoa_candidate") {
        EstimatorMeasurement measurement;
        measurement.header = {timestamp_s, sequence_id, SensorSource::TDOA_HEURISTIC,
                              coordinate_frame, MeasurementType::TDOA_POSITION_CANDIDATE};
        const double confidence = require_finite_number(record, "confidence");
        if (confidence < 0.0 || confidence > 1.0) {
            throw std::runtime_error("confidence must be within [0, 1]");
        }
        measurement.data =
            TdoaPositionMeasurementData{require_vec3(record, "position_m"), confidence, true};
        return measurement;
    }
    if (type == "ground_truth") {
        unsupported_count += 1;
        throw std::runtime_error("ground_truth records are metadata and not estimator inputs");
    }

    unsupported_count += 1;
    throw std::runtime_error("unsupported measurement type");
}

} // namespace

std::string to_string(ReplayMode mode) {
    switch (mode) {
    case ReplayMode::ACTIVE_ONLY:
        return "active_only";
    case ReplayMode::ACTIVE_WITH_IDENTICAL_SHADOW:
        return "active_with_identical_shadow";
    }
    return "active_only";
}

ReplayRunResult run_replay_file(const std::filesystem::path& input_path,
                                const ReplayRunConfig& config) {
    ReplayRunResult result;
    auto& report = result.report;
    report.replay_mode = to_string(config.mode);

    try {
        if (input_path.empty() || !std::filesystem::exists(input_path)) {
            throw std::runtime_error("replay input file not found");
        }
        const auto file_size = std::filesystem::file_size(input_path);
        if (file_size > config.limits.max_file_bytes) {
            throw std::runtime_error("replay input exceeds size limit");
        }

        std::ifstream input(input_path, std::ios::binary);
        if (!input.is_open()) {
            throw std::runtime_error("replay input file could not be opened");
        }
        json payload = json::parse(input, nullptr, true, true);
        report.replay_input_schema_version =
            payload.value("schema_version", payload.value("version", 0));
        if (report.replay_input_schema_version != 1) {
            throw std::runtime_error("unsupported replay schema version");
        }

        const auto coordinate_frame_text =
            payload.value("coordinate_frame", std::string("local_enu"));
        if (coordinate_frame_text.size() > config.limits.max_string_length) {
            throw std::runtime_error("coordinate_frame length is invalid");
        }
        report.coordinate_frame = coordinate_frame_text;
        const auto coordinate_frame = parse_coordinate_frame(coordinate_frame_text);

        if (!payload.contains("records") || !payload.at("records").is_array()) {
            throw std::runtime_error("records must be an array");
        }
        report.input_record_count = payload.at("records").size();
        if (report.input_record_count > config.limits.max_record_count) {
            throw std::runtime_error("replay record count exceeds limit");
        }

        EstimatorInitialState initial{};
        if (payload.contains("initial_state") && payload.at("initial_state").is_object()) {
            const auto& initial_state = payload.at("initial_state");
            initial.position = require_vec3(initial_state, "position");
            initial.velocity = require_vec3(initial_state, "velocity");
            const double yaw_rad = require_finite_number(initial_state, "yaw_rad");
            initial.orientation =
                Eigen::Quaterniond(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
        }

        auto active = std::make_shared<MinimalEskfAdapter>(config.ekf_config);
        active->set_validation_config(config.validation);
        EstimatorCoordinator coordinator(std::static_pointer_cast<StateEstimator>(active));
        coordinator.reset(initial);
        if (config.mode == ReplayMode::ACTIVE_WITH_IDENTICAL_SHADOW) {
            ShadowEstimatorConfig shadow_config;
            shadow_config.enabled = true;
            shadow_config.max_queue_depth =
                std::max<size_t>(8, config.validation.shadow_max_queue_depth);
            shadow_config.max_lag_ms = std::max(1.0, config.validation.shadow_max_lag_ms);
            auto shadow = std::make_unique<MinimalEskfAdapter>(config.ekf_config);
            shadow->set_validation_config(config.validation);
            coordinator.configure_shadow(std::move(shadow_config), std::move(shadow));
            report.shadow_estimator_name = "minimal_eskf";
        }

        double last_timestamp_s = -1.0;
        uint64_t sequence_id = 0;
        for (const auto& record : payload.at("records")) {
            if (!record.is_object()) {
                throw std::runtime_error("record must be an object");
            }
            const auto type = record.value("type", std::string());
            if (type == "ground_truth") {
                if (!record.contains("timestamp_s")) {
                    throw std::runtime_error("ground_truth record missing timestamp");
                }
                require_finite_number(record, "timestamp_s");
                continue;
            }
            EstimatorMeasurement measurement = make_measurement_from_record(
                record, coordinate_frame, ++sequence_id, config.limits.max_string_length,
                report.unsupported_measurement_count);
            if (last_timestamp_s >= 0.0 && measurement.header.timestamp_s <= last_timestamp_s) {
                report.timestamp_violation_count += 1;
                throw std::runtime_error("timestamps must be strictly monotonic");
            }
            last_timestamp_s = measurement.header.timestamp_s;
            const auto update = coordinator.process(measurement);
            report.processed_record_count += 1;
            if (update.status == EstimatorUpdateStatus::ACCEPTED) {
                report.accepted_update_count += 1;
            } else {
                report.rejected_update_count += 1;
            }
            if (update.validation.status == MeasurementValidationStatus::REJECTED_UNSUPPORTED) {
                report.unsupported_measurement_count += 1;
            }
        }

        if (config.mode == ReplayMode::ACTIVE_WITH_IDENTICAL_SHADOW) {
            if (!coordinator.wait_for_shadow_idle(std::chrono::milliseconds(5000))) {
                throw std::runtime_error("shadow replay did not drain within timeout");
            }
        }

        const auto active_snapshot = coordinator.active_snapshot();
        const auto telemetry = coordinator.telemetry();
        const auto shadow_snapshot = coordinator.shadow_snapshot();
        report.final_pose = active_snapshot.pose;
        report.final_position_uncertainty_m = active_snapshot.pose.pos_std.norm();
        report.active_estimator_health = health_to_string(active_snapshot.diagnostics.health_state);
        report.shadow_estimator_health = shadow_to_string(telemetry.shadow_health);
        if (shadow_snapshot.has_value()) {
            report.active_shadow_position_delta_m =
                (active_snapshot.pose.position - shadow_snapshot->pose.position).norm();
            report.active_shadow_velocity_delta_mps =
                (active_snapshot.pose.velocity - shadow_snapshot->pose.velocity).norm();
            report.active_shadow_orientation_delta_deg =
                Eigen::AngleAxisd(active_snapshot.pose.orientation.conjugate() *
                                  shadow_snapshot->pose.orientation)
                    .angle() *
                (180.0 / std::numbers::pi_v<double>);
        } else {
            report.active_shadow_position_delta_m = telemetry.position_delta_m;
            report.active_shadow_velocity_delta_mps = telemetry.velocity_delta_mps;
            report.active_shadow_orientation_delta_deg = telemetry.orientation_delta_deg;
        }
        report.comparable_snapshot_count = telemetry.comparable_snapshot_count;
        report.skipped_comparison_count = telemetry.skipped_comparison_count;
        report.shadow_dropped_event_count = telemetry.dropped_events;
        report.shadow_queue_high_water_mark = telemetry.queue_high_water_mark;
        report.shadow_synchronization_state = shadow_to_string(telemetry.shadow_health);
        report.average_propagation_latency_us =
            active_snapshot.diagnostics.average_propagation_latency_us;
        report.maximum_propagation_latency_us = active_snapshot.diagnostics.max_update_latency_us;
        report.average_measurement_update_latency_us =
            active_snapshot.diagnostics.average_measurement_latency_us;
        report.maximum_measurement_update_latency_us =
            active_snapshot.diagnostics.max_update_latency_us;
        report.success = true;
        report.failure_reason = "none";
        report.deterministic_result_hash = security::sha3_hex(canonical_hash_payload(report));
        result.exit_code = 0;
        coordinator.stop_shadow();
    } catch (const std::exception& ex) {
        report.success = false;
        report.failure_reason = ex.what();
        report.deterministic_result_hash = security::sha3_hex(canonical_hash_payload(report));
        result.exit_code = 1;
    }

    return result;
}

std::string replay_report_json(const ReplayReport& report) {
    json output;
    output["report_schema_version"] = report.report_schema_version;
    output["replay_input_schema_version"] = report.replay_input_schema_version;
    output["replay_mode"] = report.replay_mode;
    output["active_estimator_name"] = report.active_estimator_name;
    output["shadow_estimator_name"] = report.shadow_estimator_name;
    output["coordinate_frame"] = report.coordinate_frame;
    output["input_record_count"] = report.input_record_count;
    output["processed_record_count"] = report.processed_record_count;
    output["accepted_update_count"] = report.accepted_update_count;
    output["rejected_update_count"] = report.rejected_update_count;
    output["invalid_record_count"] = report.invalid_record_count;
    output["timestamp_violation_count"] = report.timestamp_violation_count;
    output["unsupported_measurement_count"] = report.unsupported_measurement_count;
    output["final_position_m"] = {report.final_pose.position.x(), report.final_pose.position.y(),
                                  report.final_pose.position.z()};
    output["final_velocity_mps"] = {report.final_pose.velocity.x(), report.final_pose.velocity.y(),
                                    report.final_pose.velocity.z()};
    output["final_orientation_wxyz"] = {
        report.final_pose.orientation.w(), report.final_pose.orientation.x(),
        report.final_pose.orientation.y(), report.final_pose.orientation.z()};
    output["final_accel_bias_mps2"] = {report.final_pose.accel_bias.x(),
                                       report.final_pose.accel_bias.y(),
                                       report.final_pose.accel_bias.z()};
    output["final_gyro_bias_rads"] = {report.final_pose.gyro_bias.x(),
                                      report.final_pose.gyro_bias.y(),
                                      report.final_pose.gyro_bias.z()};
    output["final_estimated_position_uncertainty_m"] = report.final_position_uncertainty_m;
    output["active_estimator_health"] = report.active_estimator_health;
    output["shadow_estimator_health"] = report.shadow_estimator_health;
    output["active_shadow_position_delta_m"] = report.active_shadow_position_delta_m;
    output["active_shadow_velocity_delta_mps"] = report.active_shadow_velocity_delta_mps;
    output["active_shadow_orientation_delta_deg"] = report.active_shadow_orientation_delta_deg;
    output["comparable_snapshot_count"] = report.comparable_snapshot_count;
    output["skipped_comparison_count"] = report.skipped_comparison_count;
    output["shadow_dropped_event_count"] = report.shadow_dropped_event_count;
    output["shadow_queue_high_water_mark"] = report.shadow_queue_high_water_mark;
    output["shadow_synchronization_state"] = report.shadow_synchronization_state;
    output["deterministic_result_hash"] = report.deterministic_result_hash;
    output["average_propagation_latency_us"] = report.average_propagation_latency_us;
    output["maximum_propagation_latency_us"] = report.maximum_propagation_latency_us;
    output["average_measurement_update_latency_us"] = report.average_measurement_update_latency_us;
    output["maximum_measurement_update_latency_us"] = report.maximum_measurement_update_latency_us;
    output["success"] = report.success;
    output["failure_reason"] = report.failure_reason;
    return output.dump(2);
}

} // namespace drone::estimation
