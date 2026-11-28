/* Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "finenav_mppi_SE2/optimizer.hpp"

namespace mppi {
namespace {

class TestWorldView : public finenav_mppi_SE2::IWorldView {
  public:

    bool isCollision(float, float, float) const override { return false; }

    int getCost(float, float, float) const override { return 0; }
};

class TrackingCritic : public critics::CriticFunction {
  public:
    TrackingCritic(int& initialize_count, int& score_count)
        : initialize_count_(initialize_count), score_count_(score_count) {}

    void initialize(const finenav_mppi_SE2::Params&) override { ++initialize_count_; }

    void score(CriticData&, const finenav_mppi_SE2::IWorldView&) override { ++score_count_; }

  private:
    int& initialize_count_;
    int& score_count_;
};

class RetryOnceCritic : public critics::CriticFunction {
  public:
    void initialize(const finenav_mppi_SE2::Params&) override {}

    void score(CriticData& data, const finenav_mppi_SE2::IWorldView&) override {
        scored_steps.push_back(data.trajectories.x.cols());
        data.fail_flag = scored_steps.size() == 1;
    }

    std::vector<Eigen::Index> scored_steps;
};

class TestOptimizer : public Optimizer {
  public:
    class RecordingMotionModel : public DiffDriveMotionModel {
      public:
        void predict(models::State& state) override {
            ++predict_calls;
            DiffDriveMotionModel::predict(state);
        }

        void applyConstraints(models::ControlSequence& control_sequence) override {
            ++apply_constraints_calls;
            DiffDriveMotionModel::applyConstraints(control_sequence);
        }

        int predict_calls{0};
        int apply_constraints_calls{0};
    };

    class RecordingAckermannMotionModel : public AckermannMotionModel {
      public:
        void applyConstraints(models::ControlSequence& control_sequence) override {
            ++apply_constraints_calls;
            AckermannMotionModel::applyConstraints(control_sequence);
        }

        int apply_constraints_calls{0};
    };

    void setKnownControlSequence() {
        settings_.time_steps = 3;
        active_time_steps_ = settings_.time_steps;
        settings_.model_dt = 1.0f;
        settings_.constraints.vx_min = -10.0f;
        settings_.constraints.vx_max = 10.0f;
        settings_.constraints.vy = 10.0f;
        settings_.constraints.wz = 10.0f;
        motion_model_ = std::make_shared<DiffDriveMotionModel>();
        motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
        state_.pose.pose.orientation.w = 1.0;
        control_sequence_.vx.resize(3);
        control_sequence_.vy.resize(3);
        control_sequence_.wz.resize(3);
        control_sequence_.vx << 1.0f, 2.0f, 3.0f;
        control_sequence_.vy.setZero();
        control_sequence_.wz.setZero();
    }

    void setTurningControlSequence() {
        settings_.time_steps = 1;
        active_time_steps_ = settings_.time_steps;
        settings_.model_dt = 1.0f;
        settings_.constraints.vx_min = -10.0f;
        settings_.constraints.vx_max = 10.0f;
        settings_.constraints.vy = 10.0f;
        settings_.constraints.wz = 10.0f;
        motion_model_ = std::make_shared<DiffDriveMotionModel>();
        motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
        state_.pose.pose.orientation.w = 1.0;
        control_sequence_.vx.resize(1);
        control_sequence_.vy.resize(1);
        control_sequence_.wz.resize(1);
        control_sequence_.vx(0) = 1.0f;
        control_sequence_.vy(0) = 0.0f;
        control_sequence_.wz(0) = 1.0f;
    }

    void setBatchDiffDriveState() {
        settings_.batch_size = 2;
        settings_.time_steps = 2;
        settings_.model_dt = 1.0f;
        motion_model_ = std::make_shared<DiffDriveMotionModel>();
        motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
        state_.reset(settings_.batch_size, settings_.time_steps);
        state_.pose.pose.position.x = 10.0;
        state_.pose.pose.position.y = -2.0;
        state_.pose.pose.orientation.w = 1.0;
        state_.vx << 1.0f, 2.0f, 3.0f, 4.0f;
        state_.wz << 1.0f, -0.5f, 0.0f, 0.5f;
        state_.vy.setZero();
        generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);
    }

    void setBatchHolonomicState() {
        settings_.batch_size = 2;
        settings_.time_steps = 2;
        settings_.model_dt = 1.0f;
        motion_model_ = std::make_shared<OmniMotionModel>();
        motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
        state_.reset(settings_.batch_size, settings_.time_steps);
        state_.pose.pose.orientation.w = 1.0;
        state_.vx << 1.0f, 0.0f, 0.0f, 1.0f;
        state_.vy << 1.0f, 1.0f, 2.0f, 0.0f;
        state_.wz << 1.57079632679f, 0.0f, 0.0f, 0.0f;
        generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);
    }

    void integrateBatchState() { integrateBatchStateVelocities(generated_trajectories_, state_); }

    OptimizationResult finalize() { return finalizeControlSequence(); }

    const models::ControlSequence& internalControlSequence() const { return control_sequence_; }

    void seedWarmStart() {
        control_sequence_.vx.setLinSpaced(1.0F, static_cast<float>(settings_.time_steps));
        control_sequence_.vy = -control_sequence_.vx;
        control_sequence_.wz = 2.0F * control_sequence_.vx;
    }

    using Optimizer::prepare;

    void configureConstraints() {
        finenav_mppi_SE2::Params params;
        params.batch_size = 1;
        params.time_steps = 3;
        params.regenerate_noises = false;
        params.vx_max = 0.5;
        params.vx_min = -0.2;
        params.vy_max = 0.3;
        params.wz_max = 0.8;
        getParams(params);
    }

    void setActiveConstraints(const models::ControlConstraints& constraints) { settings_.constraints = constraints; }

    std::shared_ptr<RecordingMotionModel> setRecordingMotionModel() {
        auto motion_model = std::make_shared<RecordingMotionModel>();
        motion_model->setConstraints(settings_.constraints, settings_.model_dt);
        motion_model_ = motion_model;
        return motion_model;
    }

    void setFeedbackSpeed(float vx, float vy, float wz) {
        state_.speed.linear.x = vx;
        state_.speed.linear.y = vy;
        state_.speed.angular.z = wz;
    }

    void setReverseCandidate() {
        settings_.batch_size = 1;
        settings_.time_steps = 2;
        settings_.model_dt = 0.1F;
        settings_.constraints = {2.0F, -2.0F, 0.0F, 2.0F, -1.0F, 3.0F, -1.0F, 1.0F, 4.0F};
        motion_model_ = std::make_shared<DiffDriveMotionModel>();
        motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
        state_.reset(settings_.batch_size, settings_.time_steps);
        state_.speed.linear.x = -0.5F;
        state_.cvx << -1.0F, -1.0F;
        state_.cwz.setZero();
    }

    OptimizationResult projectKnownSequence() {
        applyControlSequenceConstraints();
        const auto controls = getControlSequence();
        return {controls, getOptimizedTrajectory(controls)};
    }

    void setInfeasibleCandidateBatch() {
        settings_.batch_size = 2;
        settings_.time_steps = 4;
        settings_.model_dt = 0.1F;
        settings_.constraints = {10.0F, -10.0F, 10.0F, 10.0F, -0.5F, 0.5F, -1.0F, 1.0F, 0.25F};
        motion_model_ = std::make_shared<DiffDriveMotionModel>();
        motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
        state_.reset(settings_.batch_size, settings_.time_steps);
        state_.speed.linear.x = 0.0;
        state_.speed.angular.z = 0.0;
        state_.cvx.setConstant(5.0F);
        state_.cwz.setConstant(2.0F);
        generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);
    }

    void setOmniLateralControlSequence() {
        settings_.time_steps = 3;
        active_time_steps_ = settings_.time_steps;
        settings_.model_dt = 0.1F;
        settings_.constraints = {10.0F, -10.0F, 10.0F, 10.0F, -10.0F, 10.0F, -1.0F, 1.0F, 10.0F};
        motion_model_ = std::make_shared<OmniMotionModel>();
        motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
        state_.speed.linear.y = 0.0;
        control_sequence_.vx.resize(3);
        control_sequence_.vy.resize(3);
        control_sequence_.wz.resize(3);
        control_sequence_.vx.setZero();
        control_sequence_.vy.setConstant(5.0F);
        control_sequence_.wz.setZero();
    }

    void setAckermannControl(float vx, float requested_wz, float previous_wz) {
        settings_.time_steps = 1;
        active_time_steps_ = settings_.time_steps;
        settings_.model_dt = 0.1F;
        settings_.constraints = {2.0F, -2.0F, 0.0F, 2.0F, -10.0F, 10.0F, -1.0F, 1.0F, 1.0F};
        motion_model_ = std::make_shared<AckermannMotionModel>();
        motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
        state_.speed.linear.x = vx;
        state_.speed.angular.z = previous_wz;
        control_sequence_.vx.resize(1);
        control_sequence_.vy.resize(1);
        control_sequence_.wz.resize(1);
        control_sequence_.vx(0) = vx;
        control_sequence_.vy(0) = 0.0F;
        control_sequence_.wz(0) = requested_wz;
    }

    std::shared_ptr<RecordingAckermannMotionModel> setRecordingAckermannControl(float vx, float requested_wz,
                                                                                float previous_wz) {
        settings_.time_steps = 1;
        active_time_steps_ = settings_.time_steps;
        settings_.model_dt = 0.1F;
        settings_.constraints = {2.0F, -2.0F, 0.0F, 2.0F, -10.0F, 10.0F, -1.0F, 1.0F, 1.0F};
        auto motion_model = std::make_shared<RecordingAckermannMotionModel>();
        motion_model->setConstraints(settings_.constraints, settings_.model_dt);
        motion_model_ = motion_model;
        state_.speed.linear.x = vx;
        state_.speed.angular.z = previous_wz;
        control_sequence_.vx.resize(1);
        control_sequence_.vy.resize(1);
        control_sequence_.wz.resize(1);
        control_sequence_.vx(0) = vx;
        control_sequence_.vy(0) = 0.0F;
        control_sequence_.wz(0) = requested_wz;
        return motion_model;
    }

    void projectCandidateBatch() {
        updateInitialStateVelocities(state_);
        propagateStateVelocitiesFromInitials(state_);
    }

    const models::State& candidateState() const { return state_; }

    float prepareAndGetLocalPathLength(const nav_msgs::msg::Path& plan) {
        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::Twist speed;
        prepare(pose, speed, plan);
        return state_.local_path_length;
    }

    models::ControlConstraints constraints() const { return settings_.constraints; }

    void resetOptimizer() { reset(); }

    void scoreCritics(const finenav_mppi_SE2::IWorldView& world_view) {
        critic_manager_.evalTrajectoriesScores(critics_data_, world_view);
    }
};

TEST(OptimizerResultTest, ReturnsCurrentResultAndShiftsWarmStart) {
    TestOptimizer optimizer;
    optimizer.setKnownControlSequence();

    const auto result = optimizer.finalize();

    EXPECT_FLOAT_EQ(result.controls(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(result.controls(1, 0), 2.0f);
    EXPECT_FLOAT_EQ(result.controls(2, 0), 3.0f);
    EXPECT_FLOAT_EQ(result.trajectory(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(result.trajectory(1, 0), 3.0f);
    EXPECT_FLOAT_EQ(result.trajectory(2, 0), 6.0f);

    const auto& warm_start = optimizer.internalControlSequence();
    EXPECT_FLOAT_EQ(warm_start.vx(0), 2.0f);
    EXPECT_FLOAT_EQ(warm_start.vx(1), 3.0f);
    EXPECT_FLOAT_EQ(warm_start.vx(2), 3.0f);
}

TEST(OptimizerResultTest, PrepareCalculatesLocalPathLengthAlongAllSegments) {
    nav_msgs::msg::Path path;
    path.poses.resize(3);
    path.poses[1].pose.position.x = 3.0;
    path.poses[1].pose.position.y = 4.0;
    path.poses[2].pose.position.x = 3.0;
    path.poses[2].pose.position.y = 8.0;
    TestOptimizer optimizer;

    const float length = optimizer.prepareAndGetLocalPathLength(path);

    EXPECT_FLOAT_EQ(length, 9.0F);
}

geometry_msgs::msg::PoseStamped makePose(double x, double yaw = 0.0) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    return pose;
}

finenav_mppi_SE2::Params makeHorizonParams() {
    finenav_mppi_SE2::Params params;
    params.model_dt = 0.125;
    params.min_time_steps = 2;
    params.batch_size = 16;
    params.regenerate_noises = false;
    params.vx_min = -0.6;
    params.ax_min = -0.5;
    return params;
}

void expectResultHorizon(const OptimizationResult& result, Eigen::Index steps) {
    EXPECT_EQ(result.controls.rows(), steps);
    EXPECT_EQ(result.trajectory.rows(), steps);
    EXPECT_EQ(result.controls.cols(), 3);
    EXPECT_EQ(result.trajectory.cols(), 3);
    // These tests inherit -ffast-math, so verify finiteness through the exponent bits.
    for (const auto* matrix : {&result.controls, &result.trajectory}) {
        for (Eigen::Index i = 0; i < matrix->size(); ++i) {
            EXPECT_NE(std::bit_cast<std::uint32_t>(matrix->data()[i]) & 0x7F800000U, 0x7F800000U);
        }
    }
}

TEST(OptimizerResultTest, AdaptiveHorizonUsesPathLengthAndGoalDistance) {
    const auto params = makeHorizonParams();
    Optimizer optimizer;
    std::vector<std::unique_ptr<critics::CriticFunction>> critics;
    optimizer.initialize("test", critics, params);
    const auto pose = makePose(0.0);
    const geometry_msgs::msg::Twist speed;
    TestWorldView world_view;
    nav_msgs::msg::Path path;

    expectResultHorizon(optimizer.evalControl(pose, speed, path, world_view), params.time_steps);

    path.poses = {makePose(0.02)};
    expectResultHorizon(optimizer.evalControl(pose, speed, path, world_view),
                        params.min_time_steps);

    path.poses = {makePose(0.2)};
    expectResultHorizon(optimizer.evalControl(pose, speed, path, world_view), 7);

    path.poses = {pose, makePose(0.25), pose};
    expectResultHorizon(optimizer.evalControl(pose, speed, path, world_view), 16);

    path.poses = {pose, makePose(0.51)};
    expectResultHorizon(optimizer.evalControl(pose, speed, path, world_view), params.time_steps);
}

TEST(OptimizerResultTest, AdaptiveHorizonUsesShortestHeadingChangeAndBrakingTime) {
    auto params = makeHorizonParams();
    Optimizer optimizer;
    std::vector<std::unique_ptr<critics::CriticFunction>> critics;
    optimizer.initialize("test", critics, params);
    TestWorldView world_view;
    nav_msgs::msg::Path path;
    geometry_msgs::msg::Twist speed;

    path.poses = {makePose(0.02, -M_PI + 0.4)};
    const auto angled_pose = makePose(0.0, M_PI - 0.4);
    expectResultHorizon(optimizer.evalControl(angled_pose, speed, path, world_view), 13);

    path.poses = {makePose(0.02)};
    speed.linear.x = -0.45;
    expectResultHorizon(optimizer.evalControl(makePose(0.0), speed, path, world_view), 8);

    params.ax_min = 0.0;
    optimizer.initialize("test", critics, params);
    expectResultHorizon(optimizer.evalControl(makePose(0.0), speed, path, world_view),
                        params.min_time_steps);
}

TEST(OptimizerResultTest, DisabledGoalAngleCriticDoesNotExtendAdaptiveHorizon) {
    auto params = makeHorizonParams();
    params.GoalAngleCritic.enabled = false;
    Optimizer optimizer;
    std::vector<std::unique_ptr<critics::CriticFunction>> critics;
    optimizer.initialize("test", critics, params);
    TestWorldView world_view;
    nav_msgs::msg::Path path;
    path.poses = {makePose(0.02, M_PI / 2.0)};
    const geometry_msgs::msg::Twist speed;

    const auto result = optimizer.evalControl(makePose(0.0), speed, path, world_view);

    expectResultHorizon(result, params.min_time_steps);
}

TEST(OptimizerResultTest, NonPositiveMinimumHorizonIsClampedDuringInitialize) {
    TestWorldView world_view;
    nav_msgs::msg::Path path;
    path.poses = {makePose(0.02)};
    const geometry_msgs::msg::Twist speed;

    for (const std::int64_t min_time_steps : {0, -5}) {
        auto params = makeHorizonParams();
        params.min_time_steps = min_time_steps;
        Optimizer optimizer;
        std::vector<std::unique_ptr<critics::CriticFunction>> critics;
        optimizer.initialize("test", critics, params);

        const auto result = optimizer.evalControl(makePose(0.0), speed, path, world_view);

        expectResultHorizon(result, 1);
    }
}

TEST(OptimizerResultTest, AdaptiveHorizonShiftsActiveControlsAndClearsTail) {
    auto params = makeHorizonParams();
    params.motion_model = "Omni";
    params.vy_max = 0.5;
    params.regenerate_noises = true;
    TestOptimizer optimizer;
    std::vector<std::unique_ptr<critics::CriticFunction>> critics;
    optimizer.initialize("test", critics, params);
    TestWorldView world_view;
    const auto pose = makePose(0.0);
    const geometry_msgs::msg::Twist speed;
    nav_msgs::msg::Path near_path;
    near_path.poses = {makePose(0.02)};
    nav_msgs::msg::Path far_path;
    far_path.poses = {pose, makePose(1.0)};

    expectResultHorizon(optimizer.evalControl(pose, speed, far_path, world_view),
                        params.time_steps);
    optimizer.seedWarmStart();
    optimizer.prepare(pose, speed, near_path);
    const auto result = optimizer.finalize();
    expectResultHorizon(result, 2);
    EXPECT_EQ(optimizer.getGeneratedTrajectories().x.cols(), 2);

    const auto& warm_start = optimizer.internalControlSequence();
    ASSERT_EQ(warm_start.vx.size(), params.time_steps);
    ASSERT_EQ(warm_start.vy.size(), params.time_steps);
    ASSERT_EQ(warm_start.wz.size(), params.time_steps);
    EXPECT_TRUE(warm_start.vx.head(2).isConstant(2.0F));
    EXPECT_TRUE(warm_start.vy.head(2).isConstant(-2.0F));
    EXPECT_TRUE(warm_start.wz.head(2).isConstant(4.0F));
    EXPECT_TRUE(warm_start.vx.tail(params.time_steps - 2).isZero());
    EXPECT_TRUE(warm_start.vy.tail(params.time_steps - 2).isZero());
    EXPECT_TRUE(warm_start.wz.tail(params.time_steps - 2).isZero());

    expectResultHorizon(optimizer.evalControl(pose, speed, near_path, world_view), 2);
    optimizer.prepare(pose, speed, far_path);
    EXPECT_TRUE(optimizer.getControlSequence().bottomRows(params.time_steps - 2).isZero());
    expectResultHorizon(optimizer.evalControl(pose, speed, far_path, world_view),
                        params.time_steps);
    EXPECT_EQ(optimizer.getGeneratedTrajectories().x.cols(), params.time_steps);

    expectResultHorizon(optimizer.evalControl(pose, speed, near_path, world_view), 2);
    optimizer.reset();
    EXPECT_EQ(optimizer.getControlSequence().rows(), params.time_steps);
    EXPECT_TRUE(optimizer.getControlSequence().isZero());
    EXPECT_EQ(optimizer.getGeneratedTrajectories().x.cols(), params.time_steps);
    expectResultHorizon(optimizer.evalControl(pose, speed, near_path, world_view), 2);
}

TEST(OptimizerResultTest, RetryKeepsAdaptiveHorizon) {
    const auto params = makeHorizonParams();
    Optimizer optimizer;
    auto critic = std::make_unique<RetryOnceCritic>();
    const auto* const observed = critic.get();
    std::vector<std::unique_ptr<critics::CriticFunction>> critics;
    critics.push_back(std::move(critic));
    optimizer.initialize("test", critics, params);
    TestWorldView world_view;
    nav_msgs::msg::Path path;
    path.poses = {makePose(0.02)};

    const geometry_msgs::msg::Twist speed;
    const auto result = optimizer.evalControl(makePose(0.0), speed, path, world_view);

    expectResultHorizon(result, 2);
    EXPECT_EQ(observed->scored_steps, (std::vector<Eigen::Index>{2, 2}));
}

TEST(OptimizerResultTest, VelocityLimitStopReturnsOnlyActiveZeroControls) {
    const auto params = makeHorizonParams();
    TestOptimizer optimizer;
    std::vector<std::unique_ptr<critics::CriticFunction>> critics;
    optimizer.initialize("test", critics, params);
    optimizer.seedWarmStart();
    TestWorldView world_view;
    nav_msgs::msg::Path path;
    path.poses = {makePose(0.02)};
    geometry_msgs::msg::Twist speed;
    speed.angular.z = params.wz_max + params.az_max * params.model_dt + 0.1;

    const auto result = optimizer.evalControl(makePose(0.0), speed, path, world_view);

    expectResultHorizon(result, 2);
    EXPECT_TRUE(result.controls.isZero());
    EXPECT_TRUE(result.trajectory.isZero());
    EXPECT_TRUE(optimizer.internalControlSequence().vx.isZero());
    EXPECT_TRUE(optimizer.internalControlSequence().vy.isZero());
    EXPECT_TRUE(optimizer.internalControlSequence().wz.isZero());
}

TEST(OptimizerResultTest, CandidatePropagationUsesMotionModelPredict) {
    TestOptimizer optimizer;
    optimizer.setBatchDiffDriveState();
    const auto motion_model = optimizer.setRecordingMotionModel();

    optimizer.projectCandidateBatch();

    EXPECT_EQ(motion_model->predict_calls, 1);
}

TEST(OptimizerResultTest, NominalProjectionUsesMotionModelConstraintsBeforeAndAfterDynamics) {
    TestOptimizer optimizer;
    optimizer.setKnownControlSequence();
    const auto motion_model = optimizer.setRecordingMotionModel();

    optimizer.projectKnownSequence();

    EXPECT_EQ(motion_model->apply_constraints_calls, 2);
}

TEST(OptimizerResultTest, ReverseAccelerationUsesTravelDirection) {
    TestOptimizer optimizer;
    optimizer.setReverseCandidate();

    optimizer.projectCandidateBatch();

    // With vx=-0.5, ax_max=3, ax_min=-1 and dt=0.1, the reachable
    // reverse-speed interval is [-0.8, -0.4].
    EXPECT_FLOAT_EQ(optimizer.candidateState().vx(0, 0), -0.8F);
    EXPECT_FLOAT_EQ(optimizer.candidateState().vx(0, 1), -1.0F);
}

TEST(OptimizerResultTest, FeedbackAboveVelocityLimitStopsRegardlessOfRequest) {
    for (const float request : {0.0F, 0.5F, 2.0F}) {
        const float projected = MotionModel::projectVelocity(request, 0.8F, -0.5F, 0.5F, -0.2F, 0.2F, 0.1F);
        EXPECT_NEAR(projected, 0.0F, 1e-6F);
    }
}

TEST(OptimizerResultTest, FeedbackBelowVelocityLimitStopsRegardlessOfRequest) {
    for (const float request : {-2.0F, -0.5F, 0.0F}) {
        const float projected = MotionModel::projectVelocity(request, -0.8F, -0.5F, 0.5F, -0.2F, 0.2F, 0.1F);
        EXPECT_NEAR(projected, 0.0F, 1e-6F);
    }
}

TEST(OptimizerResultTest, FeedbackWithinVelocityLimitKeepsNormalProjection) {
    const float projected = MotionModel::projectVelocity(2.0F, 0.3F, 0.0F, 0.4F, -3.0F, 3.0F, 0.05F);
    EXPECT_NEAR(projected, 0.4F, 1e-6F);
}

TEST(OptimizerResultTest, StopCheckUsesApplicableVelocityAxes) {
    models::ControlConstraints constraints;
    constraints.vx_min = -0.4F;
    constraints.vx_max = 0.4F;
    constraints.vy = 0.4F;
    constraints.wz = 0.4F;
    constraints.ax_min = -3.0F;
    constraints.ax_max = 3.0F;
    constraints.ay_min = -3.0F;
    constraints.ay_max = 3.0F;
    constraints.az_max = 3.0F;

    DiffDriveMotionModel diff_drive;
    diff_drive.setConstraints(constraints, 0.05F);
    EXPECT_FALSE(diff_drive.shouldStopForVelocityLimits(0.0F, 0.58F, 0.0F));
    EXPECT_TRUE(diff_drive.shouldStopForVelocityLimits(0.58F, 0.0F, 0.0F));
    EXPECT_TRUE(diff_drive.shouldStopForVelocityLimits(-0.58F, 0.0F, 0.0F));
    EXPECT_TRUE(diff_drive.shouldStopForVelocityLimits(0.0F, 0.0F, 0.58F));
    EXPECT_TRUE(diff_drive.shouldStopForVelocityLimits(0.0F, 0.0F, -0.58F));
    EXPECT_FALSE(diff_drive.shouldStopForVelocityLimits(0.55F, 0.0F, 0.55F));

    OmniMotionModel omni;
    omni.setConstraints(constraints, 0.05F);
    EXPECT_TRUE(omni.shouldStopForVelocityLimits(0.0F, 0.58F, 0.0F));
    EXPECT_TRUE(omni.shouldStopForVelocityLimits(0.0F, -0.58F, 0.0F));
}

TEST(OptimizerResultTest, DisjointFeedbackVelocityReturnsZeroControlSequence) {
    TestWorldView world_view;
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 1.0;
    pose.pose.position.y = 2.0;
    pose.pose.orientation.w = 1.0;
    nav_msgs::msg::Path path;

    for (const auto& [motion_model, vx, vy, wz] : {
             std::tuple{"DiffDrive", 0.58, 0.0, 0.0},
             std::tuple{"Omni", 0.0, 0.58, 0.0},
             std::tuple{"DiffDrive", 0.0, 0.0, 0.58}}) {
        finenav_mppi_SE2::Params params;
        params.motion_model = motion_model;
        params.vx_min = -0.4;
        params.vx_max = 0.4;
        params.vy_max = 0.4;
        params.wz_max = 0.4;
        params.ax_min = -3.0;
        params.ax_max = 3.0;
        params.ay_min = -3.0;
        params.ay_max = 3.0;
        params.az_max = 3.0;
        params.time_steps = 3;
        params.batch_size = 2;
        params.regenerate_noises = false;

        Optimizer optimizer;
        std::vector<std::unique_ptr<critics::CriticFunction>> critics;
        optimizer.initialize("test", critics, params);
        geometry_msgs::msg::Twist speed;
        speed.linear.x = vx;
        speed.linear.y = vy;
        speed.angular.z = wz;

        const auto result = optimizer.evalControl(pose, speed, path, world_view);
        EXPECT_EQ(result.controls.rows(), 3);
        EXPECT_TRUE(result.controls.isZero());
        for (Eigen::Index i = 0; i < result.trajectory.rows(); ++i) {
            EXPECT_FLOAT_EQ(result.trajectory(i, 0), 1.0F);
            EXPECT_FLOAT_EQ(result.trajectory(i, 1), 2.0F);
            EXPECT_FLOAT_EQ(result.trajectory(i, 2), 0.0F);
        }
    }
}

TEST(OptimizerResultTest, AngularFeedbackAboveVelocityLimitKeepsAccelerationBound) {
    DiffDriveMotionModel model;
    models::ControlConstraints constraints;
    constraints.vx_min = -1.0F;
    constraints.vx_max = 1.0F;
    constraints.wz = 0.5F;
    constraints.az_max = 0.2F;
    model.setConstraints(constraints, 0.1F);

    const float projected = model.projectAngularVelocity(0.0F, 0.8F, 0.0F, 0.1F);

    EXPECT_NEAR(projected, 0.78F, 1e-6F);
}

TEST(OptimizerResultTest, AckermannAngularFeedbackAboveVelocityLimitKeepsAccelerationBound) {
    AckermannMotionModel model;
    models::ControlConstraints constraints;
    constraints.vx_min = -2.0F;
    constraints.vx_max = 2.0F;
    constraints.wz = 0.5F;
    constraints.az_max = 0.2F;
    model.setConstraints(constraints, 0.1F);

    const float projected = model.projectAngularVelocity(0.0F, 0.8F, 1.0F, 0.1F);

    EXPECT_NEAR(projected, 0.78F, 1e-6F);
}

TEST(OptimizerResultTest, ProjectsNominalSequenceFromFeedbackSpeed) {
    TestOptimizer optimizer;
    optimizer.setKnownControlSequence();
    optimizer.setFeedbackSpeed(0.0, 0.0, 0.0);
    optimizer.setActiveConstraints({10.0f, -10.0f, 10.0f, 10.0f, -0.5f, 0.5f, -1.0f, 1.0f, 0.25f});

    const auto result = optimizer.projectKnownSequence();

    EXPECT_FLOAT_EQ(result.controls(0, 0), 0.5f);
    EXPECT_FLOAT_EQ(result.controls(1, 0), 1.0f);
    EXPECT_FLOAT_EQ(result.controls(2, 0), 1.5f);
}

TEST(OptimizerResultTest, CandidateRolloutVelocitiesRespectAccelerationLimits) {
    TestOptimizer optimizer;
    optimizer.setInfeasibleCandidateBatch();
    optimizer.projectCandidateBatch();

    const auto& state = optimizer.candidateState();
    for (Eigen::Index row = 0; row < state.vx.rows(); ++row) {
        // The first rollout interval is driven by c[0], projected from feedback.
        EXPECT_FLOAT_EQ(state.vx(row, 0), 0.05F);
        EXPECT_FLOAT_EQ(state.wz(row, 0), 0.025F);
        EXPECT_LE(state.vx(row, 0) - state.speed.linear.x, 0.05F + 1e-6F);
        EXPECT_GE(state.vx(row, 0) - state.speed.linear.x, -0.05F - 1e-6F);
        EXPECT_LE(state.wz(row, 0) - state.speed.angular.z, 0.025F + 1e-6F);
        EXPECT_GE(state.wz(row, 0) - state.speed.angular.z, -0.025F - 1e-6F);
        for (Eigen::Index t = 1; t < state.vx.cols(); ++t) {
            EXPECT_LE(state.vx(row, t) - state.vx(row, t - 1), 0.05F + 1e-6F);
            EXPECT_GE(state.vx(row, t) - state.vx(row, t - 1), -0.05F - 1e-6F);
            EXPECT_LE(state.wz(row, t) - state.wz(row, t - 1), 0.025F + 1e-6F);
            EXPECT_GE(state.wz(row, t) - state.wz(row, t - 1), -0.025F - 1e-6F);
        }
    }
}

TEST(OptimizerResultTest, CandidateFirstTrajectoryIntervalUsesFirstProjectedControl) {
    TestOptimizer optimizer;
    optimizer.setInfeasibleCandidateBatch();
    optimizer.projectCandidateBatch();
    optimizer.integrateBatchState();

    const auto& state = optimizer.candidateState();
    const auto& trajectories = optimizer.getGeneratedTrajectories();
    EXPECT_NEAR(state.vx(0, 0), 0.05F, 1e-6F);
    EXPECT_NEAR(trajectories.x(0, 0), 0.005F, 1e-6F);
    EXPECT_NEAR(trajectories.y(0, 0), 0.0F, 1e-6F);
}

TEST(OptimizerResultTest, OmniLateralControlsRespectAccelerationBetweenAdjacentOutputs) {
    TestOptimizer optimizer;
    optimizer.setOmniLateralControlSequence();

    const auto result = optimizer.projectKnownSequence();

    EXPECT_NEAR(result.controls(0, 2), 0.1F, 1e-6F);
    EXPECT_NEAR(result.controls(1, 2), 0.2F, 1e-6F);
    EXPECT_NEAR(result.controls(2, 2), 0.3F, 1e-6F);
    for (Eigen::Index t = 1; t < result.controls.rows(); ++t) {
        EXPECT_LE(result.controls(t, 2) - result.controls(t - 1, 2), 0.1F + 1e-6F);
        EXPECT_GE(result.controls(t, 2) - result.controls(t - 1, 2), -0.1F - 1e-6F);
    }
}

TEST(OptimizerResultTest, AckermannAngularProjectionMeetsOverlappingConstraints) {
    TestOptimizer optimizer;
    optimizer.setAckermannControl(0.1F, 1.0F, 0.4F);

    const auto result = optimizer.projectKnownSequence();
    const float vx = result.controls(0, 0);
    const float wz = result.controls(0, 1);

    EXPECT_NEAR(wz, 0.5F, 1e-6F);
    EXPECT_LE(std::abs(wz - 0.4F), 0.1F + 1e-6F);
    EXPECT_LE(std::abs(wz), std::abs(vx) / 0.2F + 1e-6F);
}

TEST(OptimizerResultTest, AckermannEmptyIntersectionPrioritizesTurningRadius) {
    TestOptimizer optimizer;
    optimizer.setAckermannControl(0.0F, -1.0F, 0.8F);

    const auto result = optimizer.projectKnownSequence();

    EXPECT_FLOAT_EQ(result.controls(0, 0), 0.0F);
    EXPECT_FLOAT_EQ(result.controls(0, 1), 0.0F);
}

TEST(OptimizerResultTest, AckermannProjectionAppliesSpecialtyConstraintsBeforeAndAfterDynamics) {
    TestOptimizer optimizer;
    const auto motion_model = optimizer.setRecordingAckermannControl(0.1F, 1.0F, 0.4F);

    const auto result = optimizer.projectKnownSequence();

    EXPECT_NEAR(result.controls(0, 1), 0.5F, 1e-6F);
    EXPECT_EQ(motion_model->apply_constraints_calls, 2);
}

TEST(OptimizerResultTest, IntegratesPositionUsingYawBeforeCurrentTurn) {
    TestOptimizer optimizer;
    optimizer.setTurningControlSequence();

    const auto result = optimizer.finalize();

    EXPECT_FLOAT_EQ(result.trajectory(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(result.trajectory(0, 1), 0.0f);
    EXPECT_FLOAT_EQ(result.trajectory(0, 2), 1.0f);
}

TEST(OptimizerResultTest, IntegratesBatchTrajectoriesIndependently) {
    TestOptimizer optimizer;
    optimizer.setBatchDiffDriveState();

    optimizer.integrateBatchState();
    const auto& trajectories = optimizer.getGeneratedTrajectories();

    EXPECT_NEAR(trajectories.x(0, 0), 11.0f, 1e-5f);
    EXPECT_NEAR(trajectories.y(0, 0), -2.0f, 1e-5f);
    EXPECT_NEAR(trajectories.yaws(0, 0), 1.0f, 1e-5f);
    EXPECT_NEAR(trajectories.x(0, 1), 12.0806046f, 1e-5f);
    EXPECT_NEAR(trajectories.y(0, 1), -0.3170580f, 1e-5f);
    EXPECT_NEAR(trajectories.yaws(0, 1), 0.5f, 1e-5f);

    EXPECT_NEAR(trajectories.x(1, 0), 13.0f, 1e-5f);
    EXPECT_NEAR(trajectories.y(1, 0), -2.0f, 1e-5f);
    EXPECT_NEAR(trajectories.yaws(1, 0), 0.0f, 1e-5f);
    EXPECT_NEAR(trajectories.x(1, 1), 17.0f, 1e-5f);
    EXPECT_NEAR(trajectories.y(1, 1), -2.0f, 1e-5f);
    EXPECT_NEAR(trajectories.yaws(1, 1), 0.5f, 1e-5f);
}

TEST(OptimizerResultTest, IntegratesBatchHolonomicLateralVelocity) {
    TestOptimizer optimizer;
    optimizer.setBatchHolonomicState();

    optimizer.integrateBatchState();
    const auto& trajectories = optimizer.getGeneratedTrajectories();

    EXPECT_NEAR(trajectories.x(0, 0), 1.0f, 1e-5f);
    EXPECT_NEAR(trajectories.y(0, 0), 1.0f, 1e-5f);
    EXPECT_NEAR(trajectories.yaws(0, 0), 1.5707963f, 1e-5f);
    EXPECT_NEAR(trajectories.x(0, 1), 0.0f, 1e-5f);
    EXPECT_NEAR(trajectories.y(0, 1), 1.0f, 1e-5f);
    EXPECT_NEAR(trajectories.yaws(0, 1), 1.5707963f, 1e-5f);

    EXPECT_NEAR(trajectories.x(1, 0), 0.0f, 1e-5f);
    EXPECT_NEAR(trajectories.y(1, 0), 2.0f, 1e-5f);
    EXPECT_NEAR(trajectories.yaws(1, 0), 0.0f, 1e-5f);
    EXPECT_NEAR(trajectories.x(1, 1), 1.0f, 1e-5f);
    EXPECT_NEAR(trajectories.y(1, 1), 2.0f, 1e-5f);
    EXPECT_NEAR(trajectories.yaws(1, 1), 0.0f, 1e-5f);
}

TEST(OptimizerResultTest, ResetPreservesActiveConstraints) {
    TestOptimizer optimizer;
    optimizer.configureConstraints();
    optimizer.setActiveConstraints({1.2f, -0.7f, 0.6f, 1.5f});

    optimizer.resetOptimizer();

    const auto constraints = optimizer.constraints();
    EXPECT_FLOAT_EQ(constraints.vx_max, 1.2f);
    EXPECT_FLOAT_EQ(constraints.vx_min, -0.7f);
    EXPECT_FLOAT_EQ(constraints.vy, 0.6f);
    EXPECT_FLOAT_EQ(constraints.wz, 1.5f);
}

TEST(OptimizerResultTest, InitializeTransfersCriticsToManager) {
    int initialize_count = 0;
    int score_count = 0;
    std::vector<std::unique_ptr<critics::CriticFunction>> critics;
    critics.push_back(std::make_unique<TrackingCritic>(initialize_count, score_count));

    finenav_mppi_SE2::Params params;
    params.batch_size = 1;
    params.time_steps = 3;
    params.regenerate_noises = false;

    TestOptimizer optimizer;
    optimizer.initialize("MPPISE2", critics, params);

    EXPECT_TRUE(critics.empty());
    EXPECT_EQ(initialize_count, 1);

    TestWorldView world_view;
    optimizer.scoreCritics(world_view);
    EXPECT_EQ(score_count, 1);
}

}  // namespace
}  // namespace mppi
