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
 * Modifications: Migrated trajectory storage from xtensor to Eigen for the FineNav planning pipeline.
 */

#ifndef NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_

#include "finenav_mppi_SE2/models/eigen_types.hpp"

namespace mppi::models {

/**
 * @class mppi::models::Trajectories
 * @brief Candidate Trajectories
 */
struct Trajectories {
    BatchArray x;
    BatchArray y;
    BatchArray yaws;

    /**
    * @brief Reset state data
    */
    void reset(unsigned int batch_size, unsigned int time_steps) {
        x.resize(batch_size, time_steps);
        y.resize(batch_size, time_steps);
        yaws.resize(batch_size, time_steps);

        x.setZero();
        y.setZero();
        yaws.setZero();
    }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_
