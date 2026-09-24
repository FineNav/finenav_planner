/* Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <bit>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "finenav_mppi_controller/critic_manager.hpp"

namespace {

static_assert(
    std::is_same_v<typename decltype(mppi::CriticCostBreakdown::contributions)::value_type, std::vector<float>>,
    "Diagnostic critic contributions must use STL-owned float storage");

class EmptyWorldView : public nav2_mppi_controller::IWorldView {
  public:
    bool isTrackingUnknown() const override { return false; }

    bool isCollision(float, float, float) const override { return false; }

    int getCost(float, float, float) const override { return 0; }
};

class ScoringWorldView final : public EmptyWorldView {
  public:
    bool score(mppi::CriticData& data) const override {
        data.costs(0) += 3.0F;
        data.costs(1) += 0.5F;
        return true;
    }
};

class AddCritic final : public mppi::critics::CriticFunction {
  public:
    explicit AddCritic(const float amount) : amount_(amount) {}

    void initialize(const finenav_mppi_controller::Params&) override {}

    void score(mppi::CriticData& data, const nav2_mppi_controller::IWorldView&) override {
        data.costs += mppi::models::Vector::Constant(data.costs.size(), amount_);
    }

  private:
    float amount_;
};

class CollisionCritic final : public mppi::critics::CriticFunction {
  public:
    void initialize(const finenav_mppi_controller::Params&) override {}

    void score(mppi::CriticData& data, const nav2_mppi_controller::IWorldView&) override {
        data.costs(0) = std::numeric_limits<float>::infinity();
    }
};

TEST(CriticBreakdownTest, CapturesFiniteAndCollisionDeltas) {
    mppi::models::State state;
    mppi::models::Trajectories trajectories;
    mppi::models::Path path;
    mppi::models::Vector costs = mppi::models::Vector::Zero(2);
    float model_dt = 0.05F;
    mppi::CriticCostBreakdown breakdown;
    mppi::CriticData data{state, trajectories, path,         costs,        model_dt,
                          false, nullptr,      std::nullopt, std::nullopt, &breakdown};

    std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
    critics.push_back(std::make_unique<AddCritic>(2.5F));
    critics.push_back(std::make_unique<CollisionCritic>());

    finenav_mppi_controller::Params params;
    mppi::CriticManager manager;
    manager.on_configure(critics, params);
    EmptyWorldView world_view;
    manager.evalTrajectoriesScores(data, world_view);

    ASSERT_TRUE(breakdown.valid);
    ASSERT_EQ(breakdown.names.size(), 2U);
    ASSERT_EQ(breakdown.contributions.size(), 2U);
    EXPECT_EQ(breakdown.names[0], "UnknownCritic");
    EXPECT_EQ(breakdown.names[1], "UnknownCritic");
    EXPECT_FLOAT_EQ(breakdown.contributions[0][0], 2.5F);
    EXPECT_FLOAT_EQ(breakdown.contributions[0][1], 2.5F);
    EXPECT_EQ(std::bit_cast<std::uint32_t>(breakdown.contributions[1][0]), 0x7F800000U);
    EXPECT_FLOAT_EQ(breakdown.contributions[1][1], 0.0F);
    EXPECT_EQ(std::bit_cast<std::uint32_t>(data.costs(0)), 0x7F800000U);
    EXPECT_FLOAT_EQ(data.costs(1), 2.5F);
}

TEST(CriticBreakdownTest, NullBreakdownPreservesScoringPath) {
    mppi::models::State state;
    mppi::models::Trajectories trajectories;
    mppi::models::Path path;
    mppi::models::Vector costs = mppi::models::Vector::Zero(2);
    float model_dt = 0.05F;
    mppi::CriticData data{state, trajectories, path, costs, model_dt, false, nullptr, std::nullopt, std::nullopt};

    std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
    critics.push_back(std::make_unique<AddCritic>(1.0F));
    finenav_mppi_controller::Params params;
    mppi::CriticManager manager;
    manager.on_configure(critics, params);
    EmptyWorldView world_view;
    manager.evalTrajectoriesScores(data, world_view);

    EXPECT_FLOAT_EQ(data.costs(0), 1.0F);
    EXPECT_FLOAT_EQ(data.costs(1), 1.0F);
}

TEST(CriticBreakdownTest, DefaultWorldViewLeavesCostsAndBreakdownUnchanged) {
    mppi::models::State state;
    mppi::models::Trajectories trajectories;
    mppi::models::Path path;
    mppi::models::Vector costs = mppi::models::Vector::Zero(2);
    float model_dt = 0.05F;
    mppi::CriticCostBreakdown breakdown;
    mppi::CriticData data{state, trajectories, path,         costs,        model_dt,
                          false, nullptr,      std::nullopt, std::nullopt, &breakdown};

    std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
    finenav_mppi_controller::Params params;
    mppi::CriticManager manager;
    manager.on_configure(critics, params);
    EmptyWorldView world_view;

    manager.evalTrajectoriesScores(data, world_view);

    EXPECT_FLOAT_EQ(data.costs(0), 0.0F);
    EXPECT_FLOAT_EQ(data.costs(1), 0.0F);
    EXPECT_FALSE(breakdown.valid);
    EXPECT_TRUE(breakdown.names.empty());
}

TEST(CriticBreakdownTest, CapturesWorldViewScoreDelta) {
    mppi::models::State state;
    mppi::models::Trajectories trajectories;
    mppi::models::Path path;
    mppi::models::Vector costs = mppi::models::Vector::Zero(2);
    float model_dt = 0.05F;
    mppi::CriticCostBreakdown breakdown;
    mppi::CriticData data{state, trajectories, path,         costs,        model_dt,
                          false, nullptr,      std::nullopt, std::nullopt, &breakdown};

    std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
    finenav_mppi_controller::Params params;
    mppi::CriticManager manager;
    manager.on_configure(critics, params);
    ScoringWorldView world_view;

    manager.evalTrajectoriesScores(data, world_view);

    ASSERT_TRUE(breakdown.valid);
    ASSERT_EQ(breakdown.names.size(), 1U);
    ASSERT_EQ(breakdown.contributions.size(), 1U);
    EXPECT_EQ(breakdown.names.front(), "WorldViewScore");
    EXPECT_FLOAT_EQ(breakdown.contributions.front()[0], 3.0F);
    EXPECT_FLOAT_EQ(breakdown.contributions.front()[1], 0.5F);
    EXPECT_FLOAT_EQ(data.costs(0), 3.0F);
    EXPECT_FLOAT_EQ(data.costs(1), 0.5F);
}

}  // namespace
