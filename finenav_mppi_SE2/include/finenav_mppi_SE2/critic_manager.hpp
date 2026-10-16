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
 * Modifications: Refactored critic registration and configuration for FineNav and added per-critic cost
 *   reporting.
 */

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

//#include "finenav_mppi_SE2/tools/parameters_handler.hpp"
#include "finenav_mppi_SE2/critic_data.hpp"
#include "finenav_mppi_SE2/critic_function.hpp"
#include "finenav_mppi_SE2/tools/utils.hpp"

#include "finenav_mppi_SE2/critics/constraint_critic.hpp"
#include "finenav_mppi_SE2/critics/cost_critic.hpp"
#include "finenav_mppi_SE2/critics/goal_angle_critic.hpp"
#include "finenav_mppi_SE2/critics/goal_critic.hpp"
#include "finenav_mppi_SE2/critics/path_align_critic.hpp"
#include "finenav_mppi_SE2/critics/path_angle_critic.hpp"
#include "finenav_mppi_SE2/critics/path_follow_critic.hpp"
#include "finenav_mppi_SE2/critics/prefer_forward_critic.hpp"
#include "finenav_mppi_SE2/critics/twirling_critic.hpp"
#include "finenav_mppi_SE2/critics/velocity_deadband_critic.hpp"

namespace mppi {

/**
 * @class mppi::CriticManager
 * @brief Manager of objective function plugins for scoring trajectories
 */

class CriticManager {
  public:
    /**
    * @brief Constructor for mppi::CriticManager
    */
    CriticManager() = default;

    /**
    * @brief Virtual Destructor for mppi::CriticManager
    */
    virtual ~CriticManager() = default;

    /**
    * @brief Configure critic manager on bringup and load plugins
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
    void on_configure(std::vector<std::unique_ptr<critics::CriticFunction>>& critics,
                      const finenav_mppi_SE2::Params& params);

    /**
    * @brief Score trajectories by the set of loaded critic functions
    * @param CriticData Struct of necessary information to pass to the critic functions
    */

    void evalTrajectoriesScores(CriticData& data, const finenav_mppi_SE2::IWorldView& world_view) const;

  protected:
    std::vector<std::unique_ptr<critics::CriticFunction>> critics_;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
