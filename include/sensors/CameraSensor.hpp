#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// CameraSensor.hpp  —  ESP32-CAM via RTSP/UDP + YOLOv8n TRT inference
// Drone Swarm Sensor Fusion  |  Phase 2
// ─────────────────────────────────────────────────────────────────────────────
#include "sensors/SensorBase.hpp"
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

#ifdef DRONE_HAS_TENSORRT
#include <NvInfer.h>
#include <NvOnnxParser.h>
#endif

namespace drone::sensors {

// ─── Detection result ────────────────────────────────────────────────────────
struct Detection {
    int   class_id{-1};
    float confidence{0.0f};
    cv::Rect2f bbox;        // normalized [0,1]
    std::string label;
};

// ─── Camera frame ────────────────────────────────────────────────────────────
struct CameraFrame : SensorMeasurement {
    cv::Mat              image;         // BGR, undistorted
    cv::Mat              depth_map;     // optional monocular depth estimate
    std::vector<Detection> detections; // YOLOv8n results
    uint32_t             frame_id{0};
    double               exposure_ms{0.0};
};

// ─── Camera intrinsics ───────────────────────────────────────────────────────
struct CameraIntrinsics {
    double fx{800}, fy{800};  // focal lengths (px)
    double cx{320}, cy{240};  // principal point
    std::array<double,5> dist_coeffs{0,0,0,0,0};  // k1,k2,p1,p2,k3
    int    width{640}, height{480};
};

// ─────────────────────────────────────────────────────────────────────────────
class CameraSensor : public SensorBase {
public:
    explicit CameraSensor(std::string id, std::string stream_url,
                          CameraIntrinsics intrinsics = {})
        : SensorBase(std::move(id), "Camera")
        , stream_url_(std::move(stream_url))
        , intrinsics_(std::move(intrinsics)) {}

    bool initialize() override;
    bool reconfigure(const std::string& config_json) override;
    void poll() override;

    [[nodiscard]] std::optional<CameraFrame> latest() const {
        std::lock_guard lock(data_mutex_);
        return latest_;
    }

    void set_data_callback(DataCallback<CameraFrame> cb) {
        std::lock_guard lock(cb_mutex_);
        data_cb_ = std::move(cb);
    }

    // ── TensorRT YOLOv8n ──────────────────────────────────────────────────
    bool load_yolo_model(const std::string& engine_path,
                         float conf_thresh = 0.45f,
                         float nms_thresh  = 0.5f);

    [[nodiscard]] bool inference_enabled() const { return inference_ready_; }

    // ── Undistortion ──────────────────────────────────────────────────────
    void precompute_undistort_maps();

private:
    cv::Mat undistort(const cv::Mat& raw) const;
    std::vector<Detection> run_inference(const cv::Mat& frame);
    std::vector<Detection> run_inference_dnn_fallback(const cv::Mat& frame);

    std::string       stream_url_;
    cv::VideoCapture  cap_;
    CameraIntrinsics  intrinsics_;
    cv::Mat           map1_, map2_;   // undistortion maps

    std::optional<CameraFrame>  latest_;
    DataCallback<CameraFrame>   data_cb_;
    uint32_t                    frame_counter_{0};

    // YOLOv8n inference
    bool    inference_ready_{false};
    float   conf_thresh_{0.45f};
    float   nms_thresh_{0.50f};

#ifdef DRONE_HAS_TENSORRT
    // TensorRT engine (Jetson Nano path)
    struct TRTDeleter {
        void operator()(nvinfer1::IRuntime*       p) const { p->destroy(); }
        void operator()(nvinfer1::ICudaEngine*    p) const { p->destroy(); }
        void operator()(nvinfer1::IExecutionContext* p) const { p->destroy(); }
    };
    std::unique_ptr<nvinfer1::IRuntime,          TRTDeleter> trt_runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine,       TRTDeleter> trt_engine_;
    std::unique_ptr<nvinfer1::IExecutionContext, TRTDeleter> trt_ctx_;
    void*   d_input_{nullptr};   // CUDA device buffers
    void*   d_output_{nullptr};
    static constexpr int kInputH{640}, kInputW{640};
#else
    // OpenCV DNN fallback (CPU/GPU)
    cv::dnn::Net dnn_net_;
    std::vector<std::string> class_names_;
#endif
};

} // namespace drone::sensors
