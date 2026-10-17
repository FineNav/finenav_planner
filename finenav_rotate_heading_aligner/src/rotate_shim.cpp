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

#include "finenav_rotate_shim/rotate_shim.hpp"

#include <algorithm>
#include <cmath>

#include "finenav_core/algo_register_macros.hpp"

namespace finenav {

void RotateShim::configure(const ConfigType& params)
{
  forward_sampling_distance_ = params.forward_sampling_distance;
  angular_speed_             = params.angular_speed;
  angular_dist_threshold_    = params.angular_dist_threshold;
}

void RotateShim::reset()
{
}

std::optional<Trajectory<RotateShim::OutputProfile>> RotateShim::plan(
    const Context& ctx)
{
  const auto& ref_traj = ctx.ref_traj;

  // 空路径时输出当前位姿和零速度，使跟踪循环继续等待参考路径。
  if (ref_traj.poses.empty()) {
    Trajectory<RotateShim::OutputProfile> output;
    geometry_msgs::msg::Pose p;
    p.position.x = ctx.robot_state.pose.position.x;
    p.position.y = ctx.robot_state.pose.position.y;
    p.position.z = ctx.robot_state.pose.position.z;
    p.orientation.w = ctx.robot_state.pose.orientation.w;
    p.orientation.x = ctx.robot_state.pose.orientation.x;
    p.orientation.y = ctx.robot_state.pose.orientation.y;
    p.orientation.z = ctx.robot_state.pose.orientation.z;
    output.poses.push_back(p);
    output.twists.push_back(geometry_msgs::msg::Twist{});
    return output;
  }

  const auto sampled_pt = getSampledPathPt(ref_traj.poses, forward_sampling_distance_);

  // 世界坐标系中，机器人到采样点的位移。
  const double dx = sampled_pt.x - ctx.robot_state.pose.position.x;
  const double dy = sampled_pt.y - ctx.robot_state.pose.position.y;

  geometry_msgs::msg::Quaternion q;
  q.w = ctx.robot_state.pose.orientation.w;
  q.x = ctx.robot_state.pose.orientation.x;
  q.y = ctx.robot_state.pose.orientation.y;
  q.z = ctx.robot_state.pose.orientation.z;
  const double current_yaw = yawFromQuat(q);

  const double target_yaw = std::atan2(dy, dx);
  const double angular_distance = normalizeAngle(target_yaw - current_yaw);

  if (std::abs(angular_distance) < angular_dist_threshold_) {
    return std::nullopt;
  }

  Trajectory<RotateShim::OutputProfile> output;
  const double sign = (angular_distance > 0.0) ? 1.0 : -1.0;

  // 输出当前位姿和纯旋转速度。
  geometry_msgs::msg::Pose p;
  p.position.x = ctx.robot_state.pose.position.x;
  p.position.y = ctx.robot_state.pose.position.y;
  p.position.z = ctx.robot_state.pose.position.z;
  p.orientation.w = ctx.robot_state.pose.orientation.w;
  p.orientation.x = ctx.robot_state.pose.orientation.x;
  p.orientation.y = ctx.robot_state.pose.orientation.y;
  p.orientation.z = ctx.robot_state.pose.orientation.z;
  output.poses.push_back(p);

  geometry_msgs::msg::Twist twist;
  twist.linear.x   = 0.0;
  twist.linear.y   = 0.0;
  twist.linear.z   = 0.0;
  twist.angular.x  = 0.0;
  twist.angular.y  = 0.0;
  twist.angular.z  = sign * angular_speed_;
  output.twists.push_back(twist);

  return output;
}

geometry_msgs::msg::Point RotateShim::getSampledPathPt(
    const std::vector<geometry_msgs::msg::Pose>& ref_poses,
    double sampling_distance)
{
  if (ref_poses.size() < 2) {
    // 路径过短，直接返回唯一点的位置
    return ref_poses.front().position;
  }

  const auto& start = ref_poses.front().position;

  // 按路径顺序检查各点到起点的平面直线距离。
  for (size_t i = 1; i < ref_poses.size(); ++i) {
    const double dx = ref_poses[i].position.x - start.x;
    const double dy = ref_poses[i].position.y - start.y;
    if (std::hypot(dx, dy) >= sampling_distance) {
      return ref_poses[i].position;
    }
  }

  // 无点达到采样距离时返回路径终点。
  return ref_poses.back().position;
}

double RotateShim::yawFromQuat(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double RotateShim::normalizeAngle(double angle)
{
  while (angle >  M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }
  return angle;
}

}  // namespace finenav

FINENAV_REGISTER_PLANNER(finenav::RotateShim, finenav_rotate_shim)
