#include <gtest/gtest.h>

#include <limits>

#include "localization/LocalizationFusion.hpp"

using namespace drone;

namespace {

localization::LocalizationFusionInput make_nominal_input() {
    localization::LocalizationFusionInput input;
    input.vio_pose.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    input.vio_pose.drift_m = 0.2;
    input.vio_pose.localization_confidence = 0.90;
    input.camera_available = true;
    input.lidar_available = true;
    input.rangefinder_available = true;
    input.anchor_visibility_ratio = 1.0;
    input.time_sync.confidence = 1.0;
    input.time_sync.synchronized = true;
    return input;
}

void expect_metadata_fail_closed(const localization::LocalizationFusionOutput& output,
                                 const Eigen::Vector3d& expected_position) {
    EXPECT_TRUE(output.fused_position.isApprox(expected_position));
    EXPECT_DOUBLE_EQ(output.tdoa_weight, 0.0);
    EXPECT_DOUBLE_EQ(output.confidence, 0.0);
    EXPECT_TRUE(output.lost);
    EXPECT_TRUE(output.degraded);
    EXPECT_EQ(output.state, "lost");
    EXPECT_EQ(output.source, "invalid-localization-metadata");
}

} // namespace

TEST(LocalizationFusionMetadata, NonFiniteAnchorVisibilityFailsClosed) {
    localization::LocalizationFusion fusion;
    auto input = make_nominal_input();
    input.anchor_visibility_ratio = std::numeric_limits<double>::quiet_NaN();

    const auto output = fusion.update(input);

    expect_metadata_fail_closed(output, input.vio_pose.position);
}

TEST(LocalizationFusionMetadata, NonFiniteTimeSyncConfidenceFailsClosed) {
    localization::LocalizationFusion fusion;
    auto input = make_nominal_input();
    input.time_sync.confidence = std::numeric_limits<double>::infinity();
    input.time_sync.synchronized = false;

    const auto output = fusion.update(input);

    expect_metadata_fail_closed(output, input.vio_pose.position);
}
