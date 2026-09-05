#pragma once

#include "types.hpp"

namespace uav::collision {

bool bezierCollisionFree(
    const BezierSegment& segment,
    const Map3D& map,
    const UAVDimensions& dimensions
);

} // namespace uav::collision