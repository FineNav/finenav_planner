/* Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <type_traits>

#include <gtest/gtest.h>

#include "finenav_mppi_controller/world_view_def.hpp"

TEST(WorldViewInterfaceTest, DeclaresAbstractPlannerWorldViewContract) {
    static_assert(std::is_abstract_v<finenav_mppi_controller::IWorldView>);
    EXPECT_TRUE(std::is_polymorphic_v<finenav_mppi_controller::IWorldView>);
}
