#pragma once

#include "vio/Phase17ESKFEstimator.hpp"

namespace drone::vio {

class Phase17ESKFEstimatorTestAccess {
public:
    [[nodiscard]] static uint64_t capture_msckf_camera_state(Phase17ESKFEstimator& ekf) {
        return ekf.capture_msckf_camera_state_for_test();
    }

    [[nodiscard]] static bool process_msckf_observations(
        Phase17ESKFEstimator& ekf, uint64_t state_id, const std::vector<Eigen::Vector2d>& z_pixels,
        const std::vector<Eigen::Vector3d>& p_world, const Eigen::Matrix3d& K) {
        return ekf.process_msckf_observations_for_test(state_id, z_pixels, p_world, K);
    }

    static void corrupt_first_msckf_fej_clone(Phase17ESKFEstimator& ekf) {
        ekf.corrupt_first_msckf_fej_clone_for_test();
    }
};

} // namespace drone::vio
