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
 * Modifications: Adapted configuration to generated FineNav parameters and removed obsolete Nav2 dependencies.
 */

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__GOAL_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__GOAL_CRITIC_HPP_

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

namespace mppi::critics {

/**
 * @class mppi::critics::GoalCritic
 * @brief Critic objective function for driving towards goal
 */

class GoalCritic : public CriticFunction {
  public:
    /**
    * @brief Initialize critic
    */
    void initialize(const finenav_mppi_controller::Params& params) override;

    /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
    void score(CriticData& data, const nav2_mppi_controller::IMapView& map_view) override;

  protected:
    unsigned int power_{0};
    float weight_{0};
    float threshold_to_consider_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__GOAL_CRITIC_HPP_
