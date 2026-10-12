/* Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef FINENAV_MPPI_CONTROLLER__MODELS__EIGEN_TYPES_HPP_
#define FINENAV_MPPI_CONTROLLER__MODELS__EIGEN_TYPES_HPP_

#include <Eigen/Core>

namespace mppi::models {

using BatchArray = Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Vector = Eigen::ArrayXf;

}  // namespace mppi::models

#endif  // FINENAV_MPPI_CONTROLLER__MODELS__EIGEN_TYPES_HPP_
