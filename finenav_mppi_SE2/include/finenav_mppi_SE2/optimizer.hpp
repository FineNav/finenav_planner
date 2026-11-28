/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *   Apache License 2.0
 *
 *   Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 * Modifications Copyright (c) 2026 Chenyang Wang, IWIN-FINS Lab, Shanghai Jiao Tong University.
 * This file has been modified from the original Nav2 source.
 * Modifications: Adapted optimization to FineNav planning and map interfaces, migrated to Eigen, and added
 *   acceleration constraints, diagnostics, visualization, and reset handling.
 */

#ifndef NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_
#define NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "angles/angles.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

#include "finenav_mppi_SE2/critic_manager.hpp"
#include "finenav_mppi_SE2/models/optimizer_settings.hpp"
#include "finenav_mppi_SE2/models/path.hpp"
#include "finenav_mppi_SE2/models/state.hpp"
#include "finenav_mppi_SE2/models/trajectories.hpp"
#include "finenav_mppi_SE2/motion_models.hpp"
#include "finenav_mppi_SE2/tools/noise_generator.hpp"
#include "finenav_mppi_SE2/finenav_mppi_SE2_params.hpp"
#include "finenav_mppi_SE2/world_view_def.hpp"
#include "finenav_mppi_SE2/tools/utils.hpp"

namespace mppi {

struct OptimizationResult {
    models::BatchArray controls;
    models::BatchArray trajectory;
};

/**
 * @class mppi::Optimizer
 * @brief Main algorithm optimizer of the MPPI Controller
 */

class Optimizer {
  public:
    Optimizer() = default;

    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

    ~Optimizer() { shutdown(); }

    /**
     * @brief Initialize optimizer settings, critics, and sampling state.
     * @param name Plugin name.
     * @param critics Critic instances transferred to the optimizer's manager.
     * @param params Optimizer and critic parameters.
     */
    void initialize(const std::string& name, std::vector<std::unique_ptr<critics::CriticFunction>>& critics,
                    const finenav_mppi_SE2::Params& params) {
        name_ = name;
        getParams(params);

        critics_data_.critic_cost_breakdown = params.visualize ? &critic_cost_breakdown_ : nullptr;

        critic_manager_.on_configure(critics, params);
        noise_generator_.initialize(settings_, isHolonomic(), regenerate_noises_);

        reset();

        RCLCPP_INFO(logger_, "MPPI optimizer initialized");
    }

    /// Stop the noise generation worker.
    void shutdown() { noise_generator_.shutdown(); }

    void getParams(const finenav_mppi_SE2::Params& p) {
        auto& s = settings_;

        s.model_dt = static_cast<float>(p.model_dt);
        s.time_steps = static_cast<unsigned int>(p.time_steps);
        s.adaptive_horizon = p.adaptive_horizon;
        s.min_time_steps = static_cast<unsigned int>(std::max<std::int64_t>(1, p.min_time_steps));
        s.consider_goal_angle = p.GoalAngleCritic.enabled;
        s.adaptive_horizon_linear_vel = p.adaptive_horizon_linear_vel;
        s.adaptive_horizon_angular_vel = p.adaptive_horizon_angular_vel;
        s.threshold_to_consider = static_cast<float>(p.threshold_to_consider);
        s.batch_size = static_cast<unsigned int>(p.batch_size);
        s.iteration_count = static_cast<unsigned int>(p.iteration_count);
        s.temperature = static_cast<float>(p.temperature);
        s.gamma = static_cast<float>(p.gamma);

        s.constraints.vx_max = p.vx_max;
        s.constraints.vx_min = p.vx_min;
        s.constraints.vy = p.vy_max;
        s.constraints.wz = p.wz_max;
        s.constraints.ax_min = static_cast<float>(p.ax_min);
        s.constraints.ax_max = static_cast<float>(p.ax_max);
        s.constraints.ay_min = static_cast<float>(p.ay_min);
        s.constraints.ay_max = static_cast<float>(p.ay_max);
        s.constraints.az_max = static_cast<float>(p.az_max);

        s.sampling_std.vx = p.vx_std;
        s.sampling_std.vy = p.vy_std;
        s.sampling_std.wz = p.wz_std;
        regenerate_noises_ = p.regenerate_noises;

        s.retry_attempt_limit = 1;

        setMotionModel(p.motion_model);

        RCLCPP_INFO(logger_, "MPPI optimizer parameters prepared");
    }

    /**
     * @brief Optimize controls and roll out the resulting trajectory.
     * @param robot_pose Current robot pose.
     * @param robot_speed Measured robot velocity.
     * @param plan Reference path to track.
     * @param world_view Environment queries used for trajectory scoring.
     * @return Optimized control sequence and corresponding pose trajectory.
     */
    OptimizationResult evalControl(const geometry_msgs::msg::PoseStamped& robot_pose,
                                   const geometry_msgs::msg::Twist& robot_speed, const nav_msgs::msg::Path& plan,
                                   const finenav_mppi_SE2::IWorldView& world_view) {
        prepare(robot_pose, robot_speed, plan);

        if (motion_model_->shouldStopForVelocityLimits(robot_speed.linear.x, robot_speed.linear.y,
                                                        robot_speed.angular.z)) {
            control_sequence_.vx.head(active_time_steps_).setZero();
            control_sequence_.vy.head(active_time_steps_).setZero();
            control_sequence_.wz.head(active_time_steps_).setZero();
            return finalizeControlSequence();
        }

        do {
            optimize(world_view);
        } while (fallback(critics_data_.fail_flag));
        return finalizeControlSequence();
    }

    /// Sampled trajectories from the current optimization cycle.
    models::Trajectories& getGeneratedTrajectories() { return generated_trajectories_; }

    const CriticCostBreakdown& getCriticCostBreakdown() const { return critic_cost_breakdown_; }

    /**
     * @brief Return the optimized controls with shape [active_time_steps, 3].
     * @return Columns (vx, wz, vy); vy is zero for nonholonomic models.
     */
    models::BatchArray getControlSequence() const {
        models::BatchArray seq(active_time_steps_, 3);
        seq.col(0) = control_sequence_.vx.head(active_time_steps_);
        seq.col(1) = control_sequence_.wz.head(active_time_steps_);
        if (isHolonomic()) {
            seq.col(2) = control_sequence_.vy.head(active_time_steps_);
        } else {
            seq.col(2).setZero();
        }
        return seq;
    }

    models::BatchArray getOptimizedTrajectory(const models::BatchArray& sequence) const {
        models::BatchArray trajectories(active_time_steps_, 3);
        trajectories.setZero();
        const Eigen::Index steps = std::min<Eigen::Index>(sequence.rows(), active_time_steps_);
        integrateStateVelocities(trajectories, sequence.topRows(steps));
        return trajectories;
    }

    /// Clear optimization state and regenerate sampling noise.
    void reset() { resetOptimizationState(settings_.time_steps); }

  protected:
    void resetOptimizationState(unsigned int steps) {
        active_time_steps_ = steps;
        state_.reset(settings_.batch_size, active_time_steps_);
        control_sequence_.reset(settings_.time_steps);
        costs_.resize(settings_.batch_size);
        costs_.setZero();
        generated_trajectories_.reset(settings_.batch_size, active_time_steps_);
        critic_cost_breakdown_.clear();
        resetCriticData();
        noise_generator_.reset(settings_, isHolonomic());
    }

    OptimizationResult finalizeControlSequence() {
        auto controls = getControlSequence();
        auto trajectory = getOptimizedTrajectory(controls);

        shiftControlSequence();

        return {std::move(controls), std::move(trajectory)};
    }

    /// Sample trajectories, score them, and update the controls.
    void optimize(const finenav_mppi_SE2::IWorldView& world_view) {
        for (size_t i = 0; i < settings_.iteration_count; ++i) {
            generateNoisedTrajectories();
            critic_manager_.evalTrajectoriesScores(critics_data_, world_view);
            updateControlSequence();
        }
    }

    /// Set the measured state and reference path, and clear per-cycle scoring data.
    void prepare(const geometry_msgs::msg::PoseStamped& robot_pose, const geometry_msgs::msg::Twist& robot_speed,
                 const nav_msgs::msg::Path& plan) {
        state_.pose = robot_pose;
        state_.speed = robot_speed;
        state_.local_path_length = static_cast<float>(utils::calculatePathLength(plan));
        path_ = utils::toTensor(plan);
        updateActiveTimeSteps(computeActiveTimeSteps(plan));
        costs_.fill(0);
        resetCriticData();
    }

    void resetCriticData() {
        critics_data_.fail_flag = false;
        critics_data_.motion_model = motion_model_;
        critics_data_.furthest_reached_path_point.reset();
        critics_data_.path_pts_valid.reset();
    }

    unsigned int computeActiveTimeSteps(const nav_msgs::msg::Path& plan) const {
        const auto& s = settings_;
        if (!s.adaptive_horizon || state_.local_path_length > s.threshold_to_consider ||
            plan.poses.empty() || s.min_time_steps >= s.time_steps) {
            return s.time_steps;
        }

        const auto& robot = state_.pose.pose;
        const auto& goal = plan.poses.back().pose;
        const double d_goal = std::hypot(goal.position.x - robot.position.x,
                                         goal.position.y - robot.position.y);
        const double t_pos =
            std::max<double>(state_.local_path_length, d_goal) / s.adaptive_horizon_linear_vel;

        double t_yaw = 0.0;
        if (s.consider_goal_angle) {
            const double yaw_error = angles::shortest_angular_distance(
                tf2::getYaw(robot.orientation), tf2::getYaw(goal.orientation));
            t_yaw = std::abs(yaw_error) / s.adaptive_horizon_angular_vel;
        }

        const double deceleration = std::abs(s.constraints.ax_min);
        const double t_brake =
            deceleration > 1e-6 ? std::abs(state_.speed.linear.x) / deceleration : 0.0;
        const double steps = std::ceil(std::max({t_pos, t_yaw, t_brake}) / s.model_dt);
        return static_cast<unsigned int>(std::clamp<double>(steps, s.min_time_steps, s.time_steps));
    }

    void updateActiveTimeSteps(unsigned int steps) {
        active_time_steps_ = steps;
        if (state_.vx.cols() != active_time_steps_) {
            state_.reset(settings_.batch_size, active_time_steps_);
            generated_trajectories_.reset(settings_.batch_size, active_time_steps_);
        }
        clearInactiveControls();
    }

    void clearInactiveControls() {
        const Eigen::Index inactive_steps = settings_.time_steps - active_time_steps_;
        control_sequence_.vx.tail(inactive_steps).setZero();
        control_sequence_.vy.tail(inactive_steps).setZero();
        control_sequence_.wz.tail(inactive_steps).setZero();
    }

    /**
   * @brief Set the motion model of the vehicle platform
   * @param model Model string to use
   */
    void setMotionModel(const std::string& model) {
        if (model == "DiffDrive") {
            motion_model_ = std::make_shared<DiffDriveMotionModel>();
        } else if (model == "Omni") {
            motion_model_ = std::make_shared<OmniMotionModel>();
        } else if (model == "Ackermann") {
            motion_model_ = std::make_shared<AckermannMotionModel>();
        } else {
            throw std::runtime_error(std::string("Model " + model +
                                                 " is not valid! Valid options are DiffDrive, Omni, "
                                                 "or Ackermann"));
        }

        motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
    }

    /// Shift controls by one step to warm-start the next cycle.
    void shiftControlSequence() {
        const Eigen::Index size = active_time_steps_;
        if (size > 1) {
            const auto shift = [size](models::Vector& sequence) {
                std::copy(sequence.data() + 1, sequence.data() + size, sequence.data());
            };
            shift(control_sequence_.vx);
            shift(control_sequence_.wz);

            if (isHolonomic()) {
                shift(control_sequence_.vy);
            }
        }
        clearInactiveControls();
    }

    rclcpp::Logger logger_{rclcpp::get_logger("MPPISE2")};

    /// Add noise to the current controls and roll out the sampled trajectories.
    void generateNoisedTrajectories() {
        noise_generator_.setNoisedControls(state_, control_sequence_);
        noise_generator_.generateNextNoises();
        updateStateVelocities(state_);
        integrateStateVelocities(generated_trajectories_, state_);
    }

    /**
   * @brief Apply hard vehicle constraints on control sequence
   */
    void applyControlSequenceConstraints() {
        if (active_time_steps_ == settings_.time_steps) {
            projectControlSequence(control_sequence_);
            return;
        }

        // Vehicle-specific constraints take an owning control sequence.
        models::ControlSequence active_controls{control_sequence_.vx.head(active_time_steps_),
                                                control_sequence_.vy.head(active_time_steps_),
                                                control_sequence_.wz.head(active_time_steps_)};
        projectControlSequence(active_controls);
        control_sequence_.vx.head(active_time_steps_) = active_controls.vx;
        control_sequence_.vy.head(active_time_steps_) = active_controls.vy;
        control_sequence_.wz.head(active_time_steps_) = active_controls.wz;
    }

    void projectControlSequence(models::ControlSequence& controls) {
        // Apply vehicle-specific constraints before projecting velocity and acceleration limits.
        motion_model_->applyConstraints(controls);

        float previous_vx = state_.speed.linear.x;
        float previous_vy = state_.speed.linear.y;
        float previous_wz = state_.speed.angular.z;
        const auto& limits = settings_.constraints;
        const float dt = settings_.model_dt;
        for (Eigen::Index t = 0; t < controls.vx.size(); ++t) {
            const float vx = MotionModel::projectVelocity(
                controls.vx(t), previous_vx, limits.vx_min, limits.vx_max, limits.ax_min,
                limits.ax_max, dt);
            const float vy = isHolonomic() ? MotionModel::projectVelocity(
                                                controls.vy(t), previous_vy, -limits.vy, limits.vy,
                                                limits.ay_min, limits.ay_max, dt)
                                          : 0.0F;
            const float wz =
                motion_model_->projectAngularVelocity(controls.wz(t), previous_wz, vx, dt);

            controls.vx(t) = vx;
            controls.vy(t) = vy;
            controls.wz(t) = wz;
            previous_vx = vx;
            previous_vy = vy;
            previous_wz = wz;
        }

        motion_model_->applyConstraints(controls);
    }

    /**
   * @brief  Update velocities in state
   * @param state fill state with velocities on each step
   */
    void updateStateVelocities(models::State& state) const {
        updateInitialStateVelocities(state);
        propagateStateVelocitiesFromInitials(state);
    }

    /**
   * @brief  Update initial velocity in state
   * @param state fill state
   */
    void updateInitialStateVelocities(models::State& state) const {
        state.vx.col(0).setConstant(state.speed.linear.x);
        state.wz.col(0).setConstant(state.speed.angular.z);

        if (isHolonomic()) {
            state.vy.col(0).setConstant(state.speed.linear.y);
        } else {
            state.vy.col(0).setZero();
        }
    }

    /**
   * @brief Project each sampled control onto the reachable velocity interval.
   *
   * The measured velocity is the projection anchor before the horizon. It is
   * not stored as an additional rollout knot: state.v(:, t) is the velocity
   * driven by the sampled control at the same t.
   * @param state fill state
   */
    void propagateStateVelocitiesFromInitials(models::State& state) const { motion_model_->predict(state); }

    /**
     * @brief Integrate controls from the current robot pose.
     * @param trajectory Output poses with columns (x, y, yaw).
     * @param sequence Controls with columns (vx, wz, vy).
     */
    void integrateStateVelocities(models::BatchArray& trajectory,
                                  const Eigen::Ref<const models::BatchArray>& sequence) const {
        const Eigen::Index time_steps = sequence.rows();
        if (time_steps == 0) {
            return;
        }

        const float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);
        models::Vector yaws(time_steps);
        float last_yaw = initial_yaw;
        for (Eigen::Index t = 0; t < time_steps; ++t) {
            last_yaw += sequence(t, 1) * settings_.model_dt;
            yaws(t) = last_yaw;
        }

        models::Vector yaw_cos(time_steps);
        models::Vector yaw_sin(time_steps);
        yaw_cos(0) = std::cos(initial_yaw);
        yaw_sin(0) = std::sin(initial_yaw);
        if (time_steps > 1) {
            yaw_cos.tail(time_steps - 1) = yaws.head(time_steps - 1).cos();
            yaw_sin.tail(time_steps - 1) = yaws.head(time_steps - 1).sin();
        }

        models::Vector dx = sequence.col(0) * yaw_cos;
        models::Vector dy = sequence.col(0) * yaw_sin;
        if (isHolonomic()) {
            dx -= sequence.col(2) * yaw_sin;
            dy += sequence.col(2) * yaw_cos;
        }

        float x = static_cast<float>(state_.pose.pose.position.x);
        float y = static_cast<float>(state_.pose.pose.position.y);
        for (Eigen::Index t = 0; t < time_steps; ++t) {
            x += dx(t) * settings_.model_dt;
            y += dy(t) * settings_.model_dt;
            trajectory(t, 0) = x;
            trajectory(t, 1) = y;
            trajectory(t, 2) = yaws(t);
        }
    }

    void integrateStateVelocities(models::Trajectories& trajectories, const models::State& state) const {
        integrateBatchStateVelocities(trajectories, state);
    }

    void integrateBatchStateVelocities(models::Trajectories& trajectories, const models::State& state) const {
        const Eigen::Index batch_size = state.vx.rows();
        const Eigen::Index time_steps = state.vx.cols();
        if (batch_size == 0 || time_steps == 0) {
            return;
        }

        const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);
        models::Vector yaws(time_steps);
        models::Vector yaw_cos(time_steps);
        models::Vector yaw_sin(time_steps);
        models::Vector dx(time_steps);
        models::Vector dy(time_steps);

        // RowMajor storage makes each trajectory row contiguous for Eigen operations.
        for (Eigen::Index i = 0; i < batch_size; ++i) {
            const Eigen::Map<const models::Vector> vx_row(state.vx.row(i).data(), time_steps);
            const Eigen::Map<const models::Vector> wz_row(state.wz.row(i).data(), time_steps);

            float last_yaw = initial_yaw;
            for (Eigen::Index t = 0; t < time_steps; ++t) {
                yaw_cos(t) = std::cos(last_yaw);
                yaw_sin(t) = std::sin(last_yaw);
                last_yaw += wz_row(t) * settings_.model_dt;
                yaws(t) = last_yaw;
            }

            dx = vx_row * yaw_cos;
            dy = vx_row * yaw_sin;
            if (isHolonomic()) {
                const Eigen::Map<const models::Vector> vy_row(state.vy.row(i).data(), time_steps);
                dx -= vy_row * yaw_sin;
                dy += vy_row * yaw_cos;
            }

            float x = static_cast<float>(state.pose.pose.position.x);
            float y = static_cast<float>(state.pose.pose.position.y);
            for (Eigen::Index t = 0; t < time_steps; ++t) {
                x += dx(t) * settings_.model_dt;
                y += dy(t) * settings_.model_dt;
                trajectories.x(i, t) = x;
                trajectories.y(i, t) = y;
                trajectories.yaws(i, t) = yaws(t);
            }
        }
    }

    /**
   * @brief Update control sequence with state controls weighted by costs
   * using softmax function
   */
    void updateControlSequence() {
        auto& s = settings_;
        auto accumulate_control_cost = [this, &s](const models::BatchArray& controls,
                                                  const Eigen::Ref<const models::Vector>& sequence,
                                                  float standard_deviation) {
            models::BatchArray bounded_noises = controls;
            bounded_noises.rowwise() -= sequence.transpose();
            bounded_noises.rowwise() *= sequence.transpose();
            costs_ += s.gamma / std::pow(standard_deviation, 2.0F) * bounded_noises.rowwise().sum();
        };

        accumulate_control_cost(state_.cvx, control_sequence_.vx.head(active_time_steps_),
                                s.sampling_std.vx);
        accumulate_control_cost(state_.cwz, control_sequence_.wz.head(active_time_steps_),
                                s.sampling_std.wz);

        if (isHolonomic()) {
            accumulate_control_cost(state_.cvy, control_sequence_.vy.head(active_time_steps_),
                                    s.sampling_std.vy);
        }

        const models::Vector exponents = (-(costs_.array() - costs_.minCoeff()) / settings_.temperature).exp();
        const models::Vector softmaxes = exponents / exponents.sum();
        for (Eigen::Index t = 0; t < active_time_steps_; ++t) {
            control_sequence_.vx(t) = (state_.cvx.col(t) * softmaxes).sum();
            control_sequence_.wz(t) = (state_.cwz.col(t) * softmaxes).sum();
            if (isHolonomic()) {
                control_sequence_.vy(t) = (state_.cvy.col(t) * softmaxes).sum();
            }
        }

        applyControlSequenceConstraints();
    }

    /**
   * @brief Whether the motion model is holonomic
   * @return Bool if holonomic to populate `y` axis of state
   */
    bool isHolonomic() const { return motion_model_->isHolonomic(); }

    /**
     * @brief Reset and retry when trajectory scoring fails.
     * @param fail Whether the current trajectory batch failed.
     * @return True when optimization should retry; throws when retries are exhausted.
     */
    bool fallback(bool fail) {
        static size_t counter = 0;

        if (!fail) {
            counter = 0;
            return false;
        }

        resetOptimizationState(active_time_steps_);

        if (++counter > settings_.retry_attempt_limit) {
            counter = 0;
            throw std::runtime_error("Optimizer fail to compute path");
        }

        return true;
    }

  protected:
    std::string name_;

    std::shared_ptr<MotionModel> motion_model_;

    CriticManager critic_manager_;
    NoiseGenerator noise_generator_;

    models::OptimizerSettings settings_;
    unsigned int active_time_steps_{0};
    bool regenerate_noises_{true};

    models::State state_;
    models::ControlSequence control_sequence_;
    models::Trajectories generated_trajectories_;
    models::Path path_;
    models::Vector costs_;
    CriticCostBreakdown critic_cost_breakdown_;

    CriticData critics_data_ = {
        state_, generated_trajectories_, path_, costs_, settings_.model_dt, false, nullptr, std::nullopt, std::nullopt,
        nullptr};  /// Referenced state and scoring buffers must outlive this view.
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_
