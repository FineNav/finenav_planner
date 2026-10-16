/* Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <thread>

#include "finenav_mppi_SE2/mppi_se2.hpp"
#include "finenav_visualizer/finenav_visualizer.hpp"

namespace finenav_mppi_SE2 {
namespace {

class TestWorldView final : public IWorldView {
  public:

    bool isCollision(float, float, float) const override { return false; }

    int getCost(float, float, float) const override {
        ++cost_queries_;
        return 0;
    }

    std::size_t costQueries() const { return cost_queries_.load(); }

  private:
    mutable std::atomic<std::size_t> cost_queries_{0};
};

class ConcurrentWorldView final : public IWorldView {
  public:

    bool isCollision(float, float, float) const override { return false; }

    int getCost(float, float, float) const override {
        const int active = active_cost_queries_.fetch_add(1) + 1;
        int observed = max_concurrent_cost_queries_.load();
        while (active > observed && !max_concurrent_cost_queries_.compare_exchange_weak(observed, active)) {}
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        active_cost_queries_.fetch_sub(1);
        return 0;
    }

    int maxConcurrentCostQueries() const { return max_concurrent_cost_queries_.load(); }

  private:
    mutable std::atomic<int> active_cost_queries_{0};
    mutable std::atomic<int> max_concurrent_cost_queries_{0};
};

finenav_mppi_SE2::Params makePlannerParams() {
    finenav_mppi_SE2::Params params;
    params.time_steps = 8;
    params.batch_size = 32;
    params.iteration_count = 1;
    params.model_dt = 0.1;
    params.regenerate_noises = false;
    params.vx_max = 0.5;
    params.vx_min = -0.2;
    params.vy_max = 0.0;
    params.wz_max = 1.0;
    params.ax_min = -0.2;
    params.ax_max = 0.2;
    params.ay_min = -0.3;
    params.ay_max = 0.3;
    params.az_max = 0.4;
    params.vx_std = 0.1;
    params.vy_std = 0.1;
    params.wz_std = 0.1;
    params.motion_model = "DiffDrive";
    params.PathAlignCritic.offset_from_furthest = 1;
    params.PathAlignCritic.trajectory_point_step = 1;
    params.PathFollowCritic.offset_from_furthest = 1;
    params.PathAngleCritic.offset_from_furthest = 1;
    params.TwirlingCritic.enabled = false;
    params.VelocityDeadbandCritic.enabled = false;
    return params;
}

struct TestContext {
    finenav::core::RobotState robot_state;
    finenav::core::InputTrajectoryT<MPPISE2> ref_traj;

    operator MPPISE2::Context() const { return {robot_state, ref_traj}; }
};

TestContext makeContext(std::size_t path_size = 24) {
    TestContext context;
    context.robot_state.pose.orientation.w = 1.0;
    context.ref_traj.poses.reserve(path_size);
    for (std::size_t i = 0; i < path_size; ++i) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = static_cast<double>(i) * 0.1;
        pose.orientation.w = 1.0;
        context.ref_traj.poses.push_back(pose);
    }
    return context;
}

void expectFiniteBoundedOutput(const finenav::core::OutputTrajectoryT<MPPISE2>& output,
                               const finenav_mppi_SE2::Params& params) {
    ASSERT_EQ(output.poses.size(), static_cast<std::size_t>(params.time_steps));
    ASSERT_EQ(output.twists.size(), static_cast<std::size_t>(params.time_steps));

    for (std::size_t i = 0; i < output.poses.size(); ++i) {
        const auto& pose = output.poses[i];
        const auto& twist = output.twists[i];
        EXPECT_TRUE(std::isfinite(pose.position.x));
        EXPECT_TRUE(std::isfinite(pose.position.y));
        EXPECT_NEAR(pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w, 1.0, 1e-5);
        EXPECT_TRUE(std::isfinite(twist.linear.x));
        EXPECT_GE(twist.linear.x, params.vx_min - 1e-5);
        EXPECT_LE(twist.linear.x, params.vx_max + 1e-5);
        EXPECT_NEAR(twist.linear.y, 0.0, 1e-6);
        EXPECT_LE(std::abs(twist.angular.z), params.wz_max + 1e-5);
    }
}

TEST(MPPISE2PlanTest, AcceptsFrameworkLoggerInjection) {
    MPPISE2 planner;

    EXPECT_NO_THROW(planner.setLogger(rclcpp::get_logger("injected_mppi_test")));
}

TEST(MPPISE2PlanTest, PlanReturnsFiniteBoundedTrajectory) {
    const auto params = makePlannerParams();
    MPPISE2 planner;
    planner.configure(params);
    TestWorldView world_view;

    const auto result = planner.plan(makeContext(), world_view);

    ASSERT_TRUE(result.has_value());
    expectFiniteBoundedOutput(*result, params);
    EXPECT_GT(world_view.costQueries(), 0u);
}

TEST(MPPISE2PlanTest, ResetAllowsPlanningAgain) {
    const auto params = makePlannerParams();
    MPPISE2 planner;
    planner.configure(params);
    TestWorldView world_view;

    ASSERT_TRUE(planner.plan(makeContext(), world_view).has_value());
    planner.reset();
    const auto second = planner.plan(makeContext(), world_view);

    ASSERT_TRUE(second.has_value());
    expectFiniteBoundedOutput(*second, params);
}

TEST(MPPISE2PlanTest, VisualizedPlanResetStressWithPracticalDimensions) {
    if (!rclcpp::ok()) {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }

    auto params = makePlannerParams();
    params.visualize = true;
    params.batch_size = 3000;
    params.time_steps = 30;
    params.CostCritic.num_threads = 8;
    params.CostCritic.chunk_size = 1;

    {
        auto visualizer_node = std::make_shared<rclcpp::Node>("mppi_se2_plan_visualizer_test_node");
        MPPISE2 planner;
        planner.configure(params);
        {
            finenav::viz::FineNavVisualizer framework_visualizer(visualizer_node);
            planner.setVisualizer(framework_visualizer);
        }
        TestWorldView world_view;

        for (int episode = 0; episode < 5; ++episode) {
            for (int cycle = 0; cycle < 3; ++cycle) {
                ASSERT_TRUE(planner.plan(makeContext(), world_view).has_value());
            }
            planner.reset();
        }
    }

    rclcpp::shutdown();
}

TEST(MPPISE2PlanTest, PlanPreservesConfiguredHorizon) {
    auto params = makePlannerParams();
    params.time_steps = 5;
    MPPISE2 planner;
    planner.configure(params);
    TestWorldView world_view;

    const auto result = planner.plan(makeContext(), world_view);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->poses.size(), 5u);
    EXPECT_EQ(result->twists.size(), 5u);
}

TEST(MPPISE2PlanTest, PlanUsesConfiguredCostCriticWorkerCount) {
    auto params = makePlannerParams();
    params.CostCritic.num_threads = 4;
    params.CostCritic.chunk_size = 1;
    MPPISE2 planner;
    planner.configure(params);
    ConcurrentWorldView world_view;

    const auto result = planner.plan(makeContext(), world_view);

    ASSERT_TRUE(result.has_value());
    expectFiniteBoundedOutput(*result, params);
    EXPECT_GT(world_view.maxConcurrentCostQueries(), 1);
}

TEST(MPPISE2PlanTest, RejectsNegativeCostCriticWorkerCount) {
    auto params = makePlannerParams();
    params.CostCritic.num_threads = -1;
    MPPISE2 planner;

    EXPECT_THROW(planner.configure(params), std::invalid_argument);
}

TEST(MPPISE2PlanTest, RejectsNegativeCostCriticChunkSize) {
    auto params = makePlannerParams();
    params.CostCritic.chunk_size = -1;
    MPPISE2 planner;

    EXPECT_THROW(planner.configure(params), std::invalid_argument);
}

TEST(MPPISE2PlanTest, EmptyReferencePathReturnsNoPlan) {
    const auto params = makePlannerParams();
    MPPISE2 planner;
    planner.configure(params);
    TestWorldView world_view;
    const auto context = makeContext(0);

    const auto result = planner.plan(context, world_view);

    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(world_view.costQueries(), 0u);
}

TEST(MPPISE2PlanTest, FirstCommandRespectsFeedbackAccelerationBounds) {
    auto params = makePlannerParams();
    params.time_steps = 6;
    params.model_dt = 0.1;
    params.ax_min = -0.2;
    params.ax_max = 0.2;
    params.az_max = 0.4;

    MPPISE2 planner;
    planner.configure(params);
    TestWorldView world_view;
    auto context = makeContext();
    context.robot_state.twist.linear.x = 0.45;
    context.robot_state.twist.angular.z = 0.6;

    const auto result = planner.plan(context, world_view);

    ASSERT_TRUE(result.has_value());
    ASSERT_FALSE(result->twists.empty());
    EXPECT_GE(result->twists.front().linear.x, 0.45 - 0.2 * 0.1 - 1e-5);
    EXPECT_LE(result->twists.front().linear.x, 0.45 + 0.2 * 0.1 + 1e-5);
    EXPECT_GE(result->twists.front().angular.z, 0.6 - 0.4 * 0.1 - 1e-5);
    EXPECT_LE(result->twists.front().angular.z, 0.6 + 0.4 * 0.1 + 1e-5);
}

TEST(MPPISE2PlanTest, OutputSequenceRespectsAccelerationBetweenKnots) {
    auto params = makePlannerParams();
    params.time_steps = 8;
    params.model_dt = 0.1;
    params.ax_min = -0.2;
    params.ax_max = 0.2;
    params.az_max = 0.4;

    MPPISE2 planner;
    planner.configure(params);
    TestWorldView world_view;
    auto context = makeContext();
    context.robot_state.twist.linear.x = 0.0;
    context.robot_state.twist.angular.z = 0.0;

    const auto result = planner.plan(context, world_view);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->twists.size(), static_cast<std::size_t>(params.time_steps));
    for (std::size_t i = 1; i < result->twists.size(); ++i) {
        EXPECT_LE(result->twists[i].linear.x - result->twists[i - 1].linear.x, 0.2 * 0.1 + 1e-5);
        EXPECT_GE(result->twists[i].linear.x - result->twists[i - 1].linear.x, -0.2 * 0.1 - 1e-5);
        EXPECT_LE(result->twists[i].angular.z - result->twists[i - 1].angular.z, 0.4 * 0.1 + 1e-5);
        EXPECT_GE(result->twists[i].angular.z - result->twists[i - 1].angular.z, -0.4 * 0.1 - 1e-5);
    }
}

TEST(MPPISE2PlanTest, RejectsInvalidAccelerationBounds) {
    auto params = makePlannerParams();
    params.ax_min = 0.1;
    params.ax_max = 0.2;
    MPPISE2 planner;
    EXPECT_THROW(planner.configure(params), std::invalid_argument);

    params = makePlannerParams();
    params.ay_min = 0.2;
    params.ay_max = -0.2;
    EXPECT_THROW(planner.configure(params), std::invalid_argument);

    params = makePlannerParams();
    params.az_max = -0.1;
    EXPECT_THROW(planner.configure(params), std::invalid_argument);
}

}  // namespace
}  // namespace finenav_mppi_SE2
