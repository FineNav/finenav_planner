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
 *   Copyright (c) 2023 Open Navigation LLC
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
 * Modifications: Migrated scoring to Eigen and adapted parameters and geometry to FineNav interfaces.
 */

#include "finenav_mppi_controller/critics/goal_critic.hpp"

namespace mppi::critics {
using namespace nav2_mppi_controller;

void GoalCritic::initialize(const finenav_mppi_controller::Params& p) {
    logger_ = rclcpp::get_logger("GoalCritic");
    const auto& c = p.GoalCritic;
    enabled_ = c.enabled;
    power_ = static_cast<unsigned int>(c.cost_power);
    weight_ = static_cast<float>(c.cost_weight);
    threshold_to_consider_ = static_cast<float>(p.threshold_to_consider);
}

void GoalCritic::score(CriticData& data, const IMapView& map_view) {
    if (!this->enabled_) {
        return;
    }

    // Only activate when the local path is short enough for final goal approach
    if (data.state.local_path_length > threshold_to_consider_) {
        return;
    }

    const auto goal_idx = data.path.x.size() - 1;

    const auto goal_x = data.path.x(goal_idx);
    const auto goal_y = data.path.y(goal_idx);

    const auto dists = ((data.trajectories.x - goal_x).square() + (data.trajectories.y - goal_y).square()).sqrt();

    data.costs += (dists.rowwise().mean() * weight_).pow(power_);
    RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics
