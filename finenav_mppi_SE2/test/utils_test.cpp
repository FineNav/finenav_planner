/* Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <gtest/gtest.h>

#include <cmath>

#include "finenav_mppi_SE2/models/eigen_types.hpp"
#include "finenav_mppi_SE2/tools/utils.hpp"

namespace mppi::utils {
namespace {

template <typename T>
constexpr T degreesToRadians(T degrees) {
    return degrees * static_cast<T>(M_PI) / static_cast<T>(180.0);
}

TEST(UtilsTest, NormalizeAnglesSupportsScalarFloat) {
    EXPECT_NEAR(normalize_angles(degreesToRadians(370.0F)), degreesToRadians(10.0F), 1.0e-6F);
}

TEST(UtilsTest, NormalizeAnglesSupportsScalarDouble) {
    EXPECT_NEAR(normalize_angles(degreesToRadians(-725.0)), degreesToRadians(-5.0), 1.0e-12);
}

TEST(UtilsTest, ShortestAngularDistanceWrapsScalarsAcrossPi) {
    EXPECT_NEAR(shortest_angular_distance(degreesToRadians(170.0F), degreesToRadians(-170.0F)), degreesToRadians(20.0F),
                1.0e-6F);
    EXPECT_NEAR(shortest_angular_distance(degreesToRadians(-170.0F), degreesToRadians(170.0F)),
                degreesToRadians(-20.0F), 1.0e-6F);
}

TEST(UtilsTest, NormalizeAnglesPreservesEigenBatchBehavior) {
    models::Vector angles(3);
    angles << degreesToRadians(370.0F), degreesToRadians(-370.0F), degreesToRadians(180.0F);

    const auto normalized = normalize_angles(angles);

    EXPECT_NEAR(normalized(0), degreesToRadians(10.0F), 1.0e-6F);
    EXPECT_NEAR(normalized(1), degreesToRadians(-10.0F), 1.0e-6F);
    EXPECT_NEAR(normalized(2), degreesToRadians(180.0F), 1.0e-6F);
}

TEST(UtilsTest, FindClosestPathPointClampsDistanceBeyondPathEnd) {
    const std::vector<float> integrated_distances{0.0F, 1.0F, 2.0F};

    EXPECT_EQ(findClosestPathPt(integrated_distances, 3.0F), 2U);
}

TEST(UtilsTest, FindClosestPathPointPreservesNonzeroSearchStart) {
    const std::vector<float> integrated_distances{0.0F, 1.0F, 2.0F, 3.0F};

    EXPECT_EQ(findClosestPathPt(integrated_distances, 1.9F, 2U), 2U);
}

TEST(UtilsTest, CalculatePathLengthAddsAllPlanSegments) {
    nav_msgs::msg::Path path;
    path.poses.resize(3);
    path.poses[1].pose.position.x = 3.0;
    path.poses[1].pose.position.y = 4.0;
    path.poses[2].pose.position.x = 3.0;
    path.poses[2].pose.position.y = 8.0;

    EXPECT_DOUBLE_EQ(calculatePathLength(path), 9.0);
}

TEST(UtilsTest, PathValidityUsesWorldViewCollisionPolicy) {
    struct CollisionOnlyWorldView {
        bool isCollision(float x, float y, float yaw) const {
            return x == 1.0F && y == 2.0F && yaw == 0.5F;
        }
    };
    models::State state;
    models::Trajectories trajectories;
    models::Path path;
    path.reset(4);
    path.x << 0.0F, 1.0F, 1.0F, 3.0F;
    path.y << 0.0F, 2.0F, 2.0F, 0.0F;
    path.yaws << 0.0F, 0.5F, 1.0F, 0.0F;
    models::Vector costs = models::Vector::Zero(1);
    float model_dt = 0.05F;
    CriticData data{state, trajectories, path, costs, model_dt, false, nullptr, std::nullopt, std::nullopt};

    findPathCosts(data, CollisionOnlyWorldView{});

    ASSERT_TRUE(data.path_pts_valid.has_value());
    ASSERT_EQ(data.path_pts_valid->size(), 3U);
    EXPECT_TRUE((*data.path_pts_valid)[0]);
    EXPECT_FALSE((*data.path_pts_valid)[1]);
    EXPECT_TRUE((*data.path_pts_valid)[2]);
}

}  // namespace
}  // namespace mppi::utils
