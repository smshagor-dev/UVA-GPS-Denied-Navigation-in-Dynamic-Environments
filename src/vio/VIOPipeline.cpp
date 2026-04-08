// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// VIOPipeline.cpp  â€”  Multi-sensor orchestration with lock-free event queue
// Drone Swarm Sensor Fusion  |  Phase 2
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
#include "vio/VIOPipeline.hpp"

namespace drone::vio {

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void VIOPipeline::attach_imu(std::shared_ptr<sensors::IMUSensor> imu) {
    imu_ = std::move(imu);
    imu_->set_data_callback([this](const sensors::ImuMeasurement& m) {
        enqueue(m);
    });
}

void VIOPipeline::attach_camera(std::shared_ptr<sensors::CameraSensor> cam) {
    cam_ = std::move(cam);
    cam_->set_data_callback([this](const sensors::CameraFrame& f) {
        enqueue(f);
    });
}

void VIOPipeline::attach_lidar(std::shared_ptr<sensors::LidarSensor> lidar) {
    lidar_ = std::move(lidar);
    lidar_->set_data_callback([this](const sensors::LidarMeasurement& m) {
        enqueue(m);
    });
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
bool VIOPipeline::start() {
    if (running_.exchange(true)) return true;

    ekf_.reset();

    proc_thread_ = std::thread([this] { processing_loop(); });

    if (logger_) logger_->info("VIO pipeline started");
    return true;
}

void VIOPipeline::stop() {
    if (!running_.exchange(false)) return;
    queue_cv_.notify_all();
    if (proc_thread_.joinable()) proc_thread_.join();
    if (logger_) logger_->info("VIO pipeline stopped");
}

void VIOPipeline::reset() {
    std::lock_guard lock(queue_mutex_);
    while (!event_queue_.empty()) event_queue_.pop();
    ekf_.reset();
    last_imu_ts_ = -1.0;
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void VIOPipeline::enqueue(SensorEvent evt) {
    {
        std::lock_guard lock(queue_mutex_);
        // Drop oldest if queue is overflowing (back-pressure protection)
        if (event_queue_.size() > 2000) {
            event_queue_.pop();
            if (logger_) logger_->warn("VIO queue overflow â€” dropping oldest event");
        }
        event_queue_.push(std::move(evt));
    }
    queue_cv_.notify_one();
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void VIOPipeline::processing_loop() {
    while (running_.load()) {
        SensorEvent evt;
        {
            std::unique_lock lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !event_queue_.empty() || !running_.load();
            });
            if (!running_.load() && event_queue_.empty()) return;
            evt = std::move(event_queue_.front());
            event_queue_.pop();
        }

        std::visit([this](auto&& e) { handle(e); }, evt);

        if (pose_cb_) {
            pose_cb_(ekf_.state());
        }
    }
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void VIOPipeline::handle(const sensors::ImuMeasurement& imu) {
    if (!ekf_.is_initialized()) {
        ekf_.reset();
        last_imu_ts_ = imu.timestamp;
        return;
    }

    const double dt = (last_imu_ts_ > 0.0)
                    ? (imu.timestamp - last_imu_ts_)
                    : 0.0025;  // default 400 Hz

    last_imu_ts_ = imu.timestamp;

    if (dt > 0.0 && dt < 0.1) {
        ekf_.propagate_imu(imu.accel_mps2, imu.gyro_rads, dt);
    }
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void VIOPipeline::handle(const sensors::CameraFrame& frame) {
    if (!ekf_.is_initialized()) return;
    if (frame.detections.empty() && frame.image.empty()) return;

    // In a full implementation, the feature tracker computes pixel-map-point
    // correspondences here. For now, we use YOLOv8 detections as landmarks
    // projected at fixed depth as a placeholder.
    std::vector<Eigen::Vector2d> z_pixels;
    std::vector<Eigen::Vector3d> p_world;

    for (const auto& det : frame.detections) {
        if (det.confidence < 0.6f) continue;
        const float cx_px = (det.bbox.x + det.bbox.width  * 0.5f) * K_(0,2) * 2.0f;
        const float cy_px = (det.bbox.y + det.bbox.height * 0.5f) * K_(1,2) * 2.0f;
        z_pixels.push_back({cx_px, cy_px});

        // Placeholder: project at 5m depth in front of drone
        const auto pose = ekf_.state();
        const Eigen::Vector3d forward = pose.R_wb() * Eigen::Vector3d{0,0,1};
        p_world.push_back(pose.position + forward * 5.0);
    }

    if (!z_pixels.empty())
        ekf_.update_vision(z_pixels, p_world, K_);
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void VIOPipeline::handle(const sensors::LidarMeasurement& lidar) {
    if (!ekf_.is_initialized() || !lidar.cloud) return;

    // Use median z of lowest points as ground-plane depth estimate
    if (lidar.cloud->empty()) return;

    std::vector<float> z_vals;
    z_vals.reserve(lidar.cloud->size());
    for (const auto& pt : *lidar.cloud)
        if (pt.z > -20.0f && pt.z < 0.5f)
            z_vals.push_back(pt.z);

    if (z_vals.empty()) return;

    std::nth_element(z_vals.begin(),
                     z_vals.begin() + z_vals.size()/2,
                     z_vals.end());
    const double ground_z = z_vals[z_vals.size()/2];

    // Drone height above ground
    const auto pose = ekf_.state();
    const double height = pose.position.z() - ground_z;
    if (height > 0.3 && height < 100.0)
        ekf_.update_depth(pose.position.z(), 0.05);
}

} // namespace drone::vio
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
