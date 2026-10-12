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
 * Modifications: Decoupled the controller implementation from Nav2 nodes and adapted planning, configuration,
 *   and reset behavior to FineNav.
 */

#include "finenav_mppi_controller/controller.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "finenav_core/algo_register_macros.hpp"
#include "finenav_mppi_controller/optimizer.hpp"
#include "finenav_mppi_controller/tools/trajectory_visualizer.hpp"

namespace finenav_mppi_controller {

namespace {
template <typename DataPacketT>
nav_msgs::msg::Path toPath(const DataPacketT& data, const std::string& frame_id) {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = rclcpp::Clock().now();
    if constexpr (requires { data.poses; }) {
        path.poses.reserve(data.poses.size());
        for (const auto& pose : data.poses) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose = pose;
            path.poses.push_back(ps);
        }
    }
    return path;
}
}  // namespace

MPPIController::MPPIController() = default;

MPPIController::~MPPIController() = default;

void MPPIController::setLogger(const rclcpp::Logger& logger) {
    logger_ = logger;
    optimizer_.setLogger(logger);
}

void MPPIController::setVisualizer(const finenav::viz::FineNavVisualizer& visualizer) {
    visualizer_.setVisualizer(visualizer);
    if (configured_ && visualizer_enabled_) {
        visualizer_.configure(visualizer_config_);
    }
}

void MPPIController::configure(const ConfigType& params) {
    optimizer_.setLogger(logger_);
    if (params.CostCritic.num_threads <= 0 || params.CostCritic.chunk_size <= 0) {
        throw std::invalid_argument("CostCritic.num_threads and CostCritic.chunk_size must be greater than zero");
    }
    if (params.visualize && (params.visualizer_trajectory_step <= 0 || params.visualizer_time_step <= 0 ||
                             params.visualizer_max_trajectories <= 0 || params.visualizer_publish_rate <= 0.0)) {
        throw std::invalid_argument("Visualizer sampling, maximum trajectories, and publish rate must be greater than zero");
    }
    if (!std::isfinite(params.ax_min) || !std::isfinite(params.ax_max) || !std::isfinite(params.ay_min) ||
        !std::isfinite(params.ay_max) || !std::isfinite(params.az_max) || params.ax_min > 0.0 ||
        params.ax_max < 0.0 || params.ay_min > 0.0 || params.ay_max < 0.0 || params.az_max < 0.0) {
        throw std::invalid_argument("Acceleration limits must satisfy ax_min <= 0 <= ax_max, ax_min <= 0 <= ay_max, and az_max >= 0");
    }

    critics_.clear();
    critics_.push_back(std::make_unique<mppi::critics::ConstraintCritic>());
    mppi::critics::CostCriticExecutionOptions options;
    options.num_threads = static_cast<std::size_t>(params.CostCritic.num_threads);
    options.chunk_size = static_cast<std::size_t>(params.CostCritic.chunk_size);
    critics_.push_back(std::make_unique<mppi::critics::CostCritic>(options));
    critics_.push_back(std::make_unique<mppi::critics::GoalAngleCritic>());
    critics_.push_back(std::make_unique<mppi::critics::GoalCritic>());
    critics_.push_back(std::make_unique<mppi::critics::PathAlignCritic>());
    critics_.push_back(std::make_unique<mppi::critics::PathAngleCritic>());
    critics_.push_back(std::make_unique<mppi::critics::PathFollowCritic>());
    critics_.push_back(std::make_unique<mppi::critics::PreferForwardCritic>());
    critics_.push_back(std::make_unique<mppi::critics::TwirlingCritic>());
    critics_.push_back(std::make_unique<mppi::critics::VelocityDeadbandCritic>());

    visualizer_enabled_ = params.visualize;
    visualizer_config_.frame_id = params.visualizer_frame_id;
    visualizer_config_.trajectory_topic = params.visualizer_trajectory_topic;
    visualizer_config_.trajectory_step = static_cast<std::size_t>(params.visualizer_trajectory_step);
    visualizer_config_.time_step = static_cast<std::size_t>(params.visualizer_time_step);
    visualizer_config_.max_trajectories = static_cast<std::size_t>(params.visualizer_max_trajectories);
    visualizer_config_.publish_rate = params.visualizer_publish_rate;
    if (params.visualize && visualizer_.hasVisualizer()) {
        visualizer_.configure(visualizer_config_);
    } else {
        visualization_generation_.fetch_add(1, std::memory_order_relaxed);
        visualizer_.reset();
    }

    RCLCPP_INFO(logger_, "Initializing MPPI optimizer with %zu critics", critics_.size());
    optimizer_.initialize("MPPIController", critics_, params);
    configured_ = true;
}

std::optional<finenav::core::Trajectory<MPPIController::OutputProfile>>
MPPIController::plan(const Context& ctx, const IWorldView& world_view) {
    if (ctx.ref_traj.poses.empty()) return std::nullopt;

    using Clock = std::chrono::steady_clock;
    using Ms = std::chrono::duration<double, std::milli>;
    const auto started = Clock::now();
    finenav::core::Trajectory<OutputProfile> output;
    const auto& robot_state = ctx.robot_state;
    auto path = toPath(ctx.ref_traj, "map");
    setPlan(path);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose = robot_state.pose;
    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = "map";

    const auto optimize_started = Clock::now();
    const auto result = optimizer_.evalControl(pose, robot_state.twist, path_, world_view);
    const double optimize_ms = Ms(Clock::now() - optimize_started).count();

    if (visualizer_enabled_) {
        visualizer_.submit(optimizer_.getGeneratedTrajectories(), optimizer_.getCriticCostBreakdown(), result.trajectory,
                           path, visualization_generation_.load(std::memory_order_relaxed));
    }

    const auto& trajectory = result.trajectory;
    const auto& controls = result.controls;
    const std::size_t count = static_cast<std::size_t>(trajectory.rows());
    output.poses.reserve(count);
    output.twists.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = trajectory(static_cast<Eigen::Index>(i), 0);
        pose_msg.position.y = trajectory(static_cast<Eigen::Index>(i), 1);
        const float yaw = trajectory(static_cast<Eigen::Index>(i), 2);
        pose_msg.orientation.z = std::sin(yaw * 0.5F);
        pose_msg.orientation.w = std::cos(yaw * 0.5F);
        output.poses.push_back(pose_msg);

        geometry_msgs::msg::Twist twist;
        twist.linear.x = controls(static_cast<Eigen::Index>(i), 0);
        twist.angular.z = controls(static_cast<Eigen::Index>(i), 1);
        twist.linear.y = controls(static_cast<Eigen::Index>(i), 2);
        output.twists.push_back(twist);
    }

    static rclcpp::Clock throttle_clock{RCL_STEADY_TIME};
    RCLCPP_INFO_THROTTLE(logger_, throttle_clock, 2000,
                         "[MPPIController] evalControl: %.2f ms, total: %.2f ms",
                         optimize_ms, Ms(Clock::now() - started).count());
    return output;
}

void MPPIController::setPlan(const nav_msgs::msg::Path& path) { path_ = path; }

void MPPIController::setSpeedLimit(const double&, const bool&) {}

void MPPIController::reset() {
    const auto generation = visualization_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (visualizer_enabled_) visualizer_.clear(generation);
    optimizer_.reset();
}

}  // namespace finenav_mppi_controller

FINENAV_REGISTER_PLANNER(finenav_mppi_controller::MPPIController, finenav_mppi_controller)
