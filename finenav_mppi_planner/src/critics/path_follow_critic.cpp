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
 * Modifications: Migrated scoring to Eigen, adapted FineNav path data, and fixed path-index bounds.
 */

#include "finenav_mppi_controller/critics/path_follow_critic.hpp"

namespace mppi::critics {
using namespace finenav_mppi_controller;

void PathFollowCritic::initialize(const finenav_mppi_controller::Params& p) {
    logger_ = rclcpp::get_logger("PathFollowCritic");
    const auto& c = p.PathFollowCritic;
    enabled_ = c.enabled;
    power_ = static_cast<unsigned int>(c.cost_power);
    weight_ = static_cast<float>(c.cost_weight);
    offset_from_furthest_ = static_cast<size_t>(c.offset_from_furthest);
    threshold_to_consider_ = static_cast<float>(p.threshold_to_consider);
}

void PathFollowCritic::score(CriticData& data, const IWorldView& world_view) {
    if (!this->enabled_ || data.path.x.size() < 2 || data.state.local_path_length < threshold_to_consider_) {
        return;
    }
    utils::setPathFurthestPointIfNotSet(data);
    utils::setPathCostsIfNotSet(data, world_view);
    const size_t path_size = data.path.x.size() - 1;

    auto offseted_idx = std::min(*data.furthest_reached_path_point + offset_from_furthest_, path_size - 1);
    // Drive to the first valid path point, in case of dynamic obstacles on path
    // we want to drive past it, not through it
    bool valid = false;
    while (!valid && offseted_idx < path_size - 1) {
        valid = (*data.path_pts_valid)[offseted_idx];
        if (!valid) {
            offseted_idx++;
        }
    }

    const auto path_x = data.path.x(offseted_idx);
    const auto path_y = data.path.y(offseted_idx);

    const auto last_x = data.trajectories.x.col(data.trajectories.x.cols() - 1);
    const auto last_y = data.trajectories.y.col(data.trajectories.y.cols() - 1);

    const models::Vector dists = ((last_x.array() - path_x).square() + (last_y.array() - path_y).square()).sqrt();

    data.costs += (weight_ * dists).pow(power_);
    RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics
