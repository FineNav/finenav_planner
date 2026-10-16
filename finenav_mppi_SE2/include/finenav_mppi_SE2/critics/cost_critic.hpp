/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *   Apache License 2.0
 *
 *   Copyright (c) 2023 Robocc Brice Renaudeau
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
 * Modifications: Decoupled cost evaluation from Nav2 costmaps and added collision-model and parallel-scoring
 *   configuration.
 */

#ifndef FINENAV_MPPI_SE2__CRITICS__COST_CRITIC_HPP_
#define FINENAV_MPPI_SE2__CRITICS__COST_CRITIC_HPP_

#include <cstddef>
#include <functional>

#include "finenav_mppi_SE2/critic_function.hpp"
#include "finenav_mppi_SE2/models/state.hpp"
#include "finenav_mppi_SE2/tools/utils.hpp"

namespace mppi::critics {

struct CostCriticExecutionOptions {
    std::size_t num_threads{1};
    std::size_t chunk_size{1};
};

/**
 * @class mppi::critics::CostCritic
 * @brief Critic objective function for avoiding obstacles using costmap's inflated cost
 */

class CostCritic : public CriticFunction {
  public:
    explicit CostCritic(CostCriticExecutionOptions options = {});
    ~CostCritic() override;

    /**
    * @brief Initialize critic
    */
    void initialize(const finenav_mppi_SE2::Params& params) override;

    /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
    void score(CriticData& data, const finenav_mppi_SE2::IWorldView& world_view) override;

  protected:
    void executeTrajectories(std::size_t count, const std::function<void(std::size_t)>& callback);

    /**
    * @brief Checks if cost represents a collision
    * @param cost Point cost at pose center
    * @param x X of pose
    * @param y Y of pose
    * @param theta theta of pose
    * @return bool if in collision
    */
    bool inCollision(float x, float y, float theta, const finenav_mppi_SE2::IWorldView& world_view) const {
        return world_view.isCollision(x, y, theta);
    }

  protected:
    //  bool consider_footprint_{true};
    float collision_cost_{0};
    float critical_cost_{0};
    float weight_{0};

    float near_goal_distance_;
    unsigned int power_{0};
    CostCriticExecutionOptions execution_options_;
    int observed_openmp_threads_{1};
};

}  // namespace mppi::critics

#endif  // FINENAV_MPPI_SE2__CRITICS__COST_CRITIC_HPP_
