// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <algorithm>
#include "finenav_route_planner/vec3.hpp"

namespace astar {

enum class Heuristic {
    // Dijkstra search; equivalent to BFS when all edge weights are equal.
    Zero,
    Euclidean,
    Manhattan,
    Chebyshev
};

// Cell-space distance; 2-D grids use z = 0.
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
    default:
        return 0.0;
    }
}

} // namespace astar

