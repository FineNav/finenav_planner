// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <algorithm>
#include "finenav_route_planner/common/vec3.hpp"

namespace astar {

enum class Heuristic {
    // h = 0 : degrades A* to Dijkstra's algorithm (weighted BFS).
    // On a uniform-cost grid this is equivalent to classic BFS.
    // On a weighted grid it finds the minimum-cost path without
    // directional guidance, at the cost of more node expansions.
    Zero,
    Euclidean,
    Manhattan,
    Chebyshev,
    // 3-D octile (diagonal) distance. Admissible for 8-connected grids when
    // the diagonal step cost = √2 (and 3-D corner step = √3). Recommended as
    // the D* Lite default heuristic.
    Diagonal
};

// Heuristic operates on Vec3 cells.
// For 2-D grids z is always 0, so the 3-D formula degrades correctly.
inline double calculateHeuristic(
    Heuristic   type,
    const Vec3& from,
    const Vec3& to)
{
    switch (type)
    {
    case Heuristic::Zero:
        return 0.0;
    case Heuristic::Euclidean: {
        double dx = from.x - to.x, dy = from.y - to.y, dz = from.z - to.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    case Heuristic::Manhattan:
        return std::abs(from.x - to.x) + std::abs(from.y - to.y) + std::abs(from.z - to.z);
    case Heuristic::Chebyshev:
        return std::max({std::abs(from.x - to.x),
                         std::abs(from.y - to.y),
                         std::abs(from.z - to.z)});
    case Heuristic::Diagonal: {
        // Octile / diagonal distance: moving along 2 axes at once costs √2.
        // Shared with D* Lite (recommended default heuristic for 8-connectivity).
        const double dx = std::abs(from.x - to.x);
        const double dy = std::abs(from.y - to.y);
        const double dz = std::abs(from.z - to.z);
        const double dmin = std::min({dx, dy, dz});
        const double dmax = std::max({dx, dy, dz});
        const double dmid = dx + dy + dz - dmin - dmax;
        return dmin * std::sqrt(3.0)
             + (dmid - dmin) * std::sqrt(2.0)
             + (dmax - dmid);
    }
    default:
        return 0.0;
    }
}

} // namespace astar

