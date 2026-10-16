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
 * Modifications: Adapted configuration to generated FineNav parameters and removed obsolete Nav2 state.
 */

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_

#include "finenav_mppi_SE2/critic_function.hpp"
#include "finenav_mppi_SE2/models/state.hpp"
#include "finenav_mppi_SE2/tools/utils.hpp"

namespace mppi::critics {

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for enforcing feasible constraints
 */
class ConstraintCritic : public CriticFunction {
  public:
    /**
    * @brief Initialize critic
    */
    void initialize(const finenav_mppi_SE2::Params& params) override;

    /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
    void score(CriticData& data, const finenav_mppi_SE2::IWorldView& world_view) override;

  protected:
    unsigned int power_{0};
    float weight_{0};
    float min_vel_;
    float max_vel_;
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
