/* Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef FINENAV_WORLD_VIEW_DEF_HPP
#define FINENAV_WORLD_VIEW_DEF_HPP

namespace finenav_mppi_SE2 {
/**
 * @brief Read-only world view used by MPPI critics.
 *
 * Implementations used with CostCritic's OpenMPDynamic mode must support
 * concurrent const queries on the same instance, including synchronized caches.
 */
class IWorldView {
  public:
    IWorldView() = default;
    virtual ~IWorldView() = default;
    virtual bool isCollision(float x, float y, float theta) const = 0;

    // Cost at the queried pose in the world view's coordinate frame.
    virtual int getCost(float x, float y, float theta) const = 0;

};
}  // namespace finenav_mppi_SE2

#endif
