#include "manager/dart_launch_sequence.hpp"

#include <gtest/gtest.h>

#include <eigen3/Eigen/Dense>

namespace rmcs_dart_guidance::manager::test {

TEST(DartLaunchSequenceTest, TracksDesiredTargetAcrossFiresAndReset) {
    DartLaunchSequence sequence;
    sequence.configure_from_parameter_values(DartLaunchSequenceRawConfig{
        .dart_count = 4,
        .aim_reference_pixel = {100.0, 200.0},
        .aim_dart_offsets_px = {0.0, 0.0, 2.0, -1.0, 4.0, -2.0, 6.0, -3.0},
    });

    EXPECT_EQ(sequence.current_dart_index(), 0U);
    EXPECT_TRUE(
        sequence.current_desired_target_px().isApprox(Eigen::Vector2d(100.0, 200.0), 1e-9));

    EXPECT_TRUE(sequence.advance_after_fire());
    EXPECT_EQ(sequence.current_dart_index(), 1U);
    EXPECT_TRUE(
        sequence.current_desired_target_px().isApprox(Eigen::Vector2d(102.0, 199.0), 1e-9));

    EXPECT_TRUE(sequence.advance_after_fire());
    EXPECT_TRUE(sequence.advance_after_fire());
    EXPECT_FALSE(sequence.advance_after_fire());
    EXPECT_EQ(sequence.current_dart_index(), 3U);
    EXPECT_TRUE(
        sequence.current_desired_target_px().isApprox(Eigen::Vector2d(106.0, 197.0), 1e-9));

    sequence.reset();
    EXPECT_EQ(sequence.current_dart_index(), 0U);
    EXPECT_TRUE(
        sequence.current_desired_target_px().isApprox(Eigen::Vector2d(100.0, 200.0), 1e-9));
}

} // namespace rmcs_dart_guidance::manager::test
