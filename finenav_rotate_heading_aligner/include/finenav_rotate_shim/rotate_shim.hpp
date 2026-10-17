/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *   Copyright (c) 2021 Samsung Research America
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
 * Modifications Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * This file has been modified from the original Nav2 source.
 * Modifications: Adapted rotation-before-tracking behavior to the FineNav
 *   Planner interface, parameter library, and planning context.
 */

#pragma once

#include <cmath>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "finenav_core/plugin_planner.hpp"
#include "finenav_rotate_shim/finenav_rotate_shim_params.hpp"

namespace finenav {
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::Trajectory;
using finenav::core::OutputTrajectoryT;

/**
 * @brief 原地旋转，使机器人朝向对准参考路径上的前方采样点。
 * 朝向误差小于 angular_dist_threshold 时，plan() 返回 nullopt 结束旋转。
 */
class RotateShim {
public:
  RotateShim() = default;

  using ConfigType = finenav_rotate_shim::Params;
  using Context    = PlanningContext<RotateShim>;

  static constexpr IOProfile InputProfile  = { .pose = true, .vel = false, .accel = false, .time = false };
  static constexpr IOProfile OutputProfile = { .pose = true, .vel = true,  .accel = false, .time = false };

  void configure(const ConfigType& params);
  void reset();

  /**
   * @brief 执行一次旋转规划。
   * @param ctx 包含 robot_state 和 ref_traj
   * @return 已对齐时返回 nullopt；空路径时返回当前位姿和零速度；否则返回纯旋转轨迹
   */
  std::optional<Trajectory<OutputProfile>> plan(const Context& ctx);

private:
  /**
   * @brief 返回与路径起点的平面直线距离达到采样距离的第一个点。
   * @param ref_poses 非空参考路径
   * @param sampling_distance 采样距离，单位 m
   * @return 满足距离条件的点；无满足条件的点时返回路径终点
   */
  static geometry_msgs::msg::Point getSampledPathPt(
      const std::vector<geometry_msgs::msg::Pose>& ref_poses,
      double sampling_distance);

  /** @brief 从四元数提取 yaw 角 */
  static double yawFromQuat(const geometry_msgs::msg::Quaternion& q);

  /** @brief 归一化角度到 [-pi, pi] */
  static double normalizeAngle(double angle);

  double forward_sampling_distance_ = 0.5;
  double angular_speed_             = 0.8;
  double angular_dist_threshold_    = 0.1;
};

}  // namespace finenav
