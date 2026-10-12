/* Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef FINENAV_WORLD_VIEW_DEF_HPP
#define FINENAV_WORLD_VIEW_DEF_HPP

#include "finenav_mppi_controller/critic_data.hpp"

namespace finenav_mppi_controller {
/**
     * @brief Read-only world view used by MPPI critics.
     *
     * CostCritic's OpenMPDynamic execution calls getCost() and isCollision()
     * concurrently on the same instance. Implementations used
     * with this execution mode must therefore support concurrent const reads and must
     * not mutate an unsynchronized cache from these methods.
     */
class IWorldView {
  public:
    IWorldView() = default;
    virtual ~IWorldView() = default;
    // 是否将未知区域视为可通行
    virtual bool isTrackingUnknown() const = 0;

    virtual bool isCollision(float x, float y, float theta) const = 0;

    // 获取地图索引处的代价值（根据项目需要可改为返回 unsigned char 等）
    virtual int getCost(float x, float y, float theta) const = 0;

    // Optional batch score supplied by a WorldView; return true when costs were changed.
    virtual bool score(mppi::CriticData&) const { return false; }
};
}  // namespace finenav_mppi_controller

#endif
