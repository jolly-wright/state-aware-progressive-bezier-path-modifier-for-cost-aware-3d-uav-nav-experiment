#include "collision.hpp"
#include "config.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace uav::collision {

namespace {

// ============================================================================
// LOCAL BÉZIER SUBDIVISION REPRESENTATION
// ============================================================================

struct BezierControlPoints
{
    Vec3 P0;
    Vec3 P1;
    Vec3 P2;
    Vec3 P3;
};


// ============================================================================
// CONTROL-POINT AABB
// ============================================================================
//
// The Bézier curve lies inside the convex hull of its control points.
// Therefore the AABB of the control points conservatively contains
// the complete Bézier sub-curve.
//

AABB makeControlPointAABB(
    const BezierControlPoints& curve)
{
    AABB box;

    box.min = {
        std::min({
            curve.P0.x,
            curve.P1.x,
            curve.P2.x,
            curve.P3.x
        }),

        std::min({
            curve.P0.y,
            curve.P1.y,
            curve.P2.y,
            curve.P3.y
        }),

        std::min({
            curve.P0.z,
            curve.P1.z,
            curve.P2.z,
            curve.P3.z
        })
    };

    box.max = {
        std::max({
            curve.P0.x,
            curve.P1.x,
            curve.P2.x,
            curve.P3.x
        }),

        std::max({
            curve.P0.y,
            curve.P1.y,
            curve.P2.y,
            curve.P3.y
        }),

        std::max({
            curve.P0.z,
            curve.P1.z,
            curve.P2.z,
            curve.P3.z
        })
    };

    return box;
}


// ============================================================================
// EXPAND CURVE AABB BY UAV HALF-EXTENTS
// ============================================================================

AABB expandByUAVDimensions(
    const AABB& curveBox,
    const UAVDimensions& dimensions)
{
    const Vec3 half =
        dimensions.halfExtents();

    return {
        {
            curveBox.min.x - half.x,
            curveBox.min.y - half.y,
            curveBox.min.z - half.z
        },

        {
            curveBox.max.x + half.x,
            curveBox.max.y + half.y,
            curveBox.max.z + half.z
        }
    };
}


// ============================================================================
// UAV-SCALE TERMINATION
// ============================================================================
//
// The unexpanded Bézier control-point AABB is compared with the physical
// UAV dimensions.
//
// If the expanded box still intersects an obstacle at this resolution,
// the candidate is conservatively classified as colliding.
//

bool reachedUAVScale(
    const AABB& curveBox,
    const UAVDimensions& dimensions)
{
    const double sizeX =
        curveBox.max.x - curveBox.min.x;

    const double sizeY =
        curveBox.max.y - curveBox.min.y;

    const double sizeZ =
        curveBox.max.z - curveBox.min.z;

    return
        sizeX <= dimensions.x &&
        sizeY <= dimensions.y &&
        sizeZ <= dimensions.z;
}


// ============================================================================
// OBSTACLE INTERSECTION
// ============================================================================

bool intersectsAnyObstacle(
    const AABB& expandedCurveBox,
    const Map3D& map)
{
    for (const Obstacle& obstacle :
         map.obstacles)
    {
        if (expandedCurveBox.intersects(
                obstacle.bounds))
        {
            return true;
        }
    }

    return false;
}


// ============================================================================
// DE CASTELJAU SUBDIVISION AT u = 0.5
// ============================================================================

void subdivideBezier(
    const BezierControlPoints& curve,
    BezierControlPoints& left,
    BezierControlPoints& right)
{
    const Vec3 A =
        (curve.P0 + curve.P1) * 0.5;

    const Vec3 B =
        (curve.P1 + curve.P2) * 0.5;

    const Vec3 C =
        (curve.P2 + curve.P3) * 0.5;

    const Vec3 D =
        (A + B) * 0.5;

    const Vec3 E =
        (B + C) * 0.5;

    const Vec3 M =
        (D + E) * 0.5;

    left = {
        curve.P0,
        A,
        D,
        M
    };

    right = {
        M,
        E,
        C,
        curve.P3
    };
}


// ============================================================================
// RECURSIVE COLLISION TEST
// ============================================================================

bool subCurveCollisionFree(
    const BezierControlPoints& curve,
    const Map3D& map,
    const UAVDimensions& dimensions,
    std::size_t depth,
    double uMin,
    double uMax)
{
    const AABB curveBox =
        makeControlPointAABB(
            curve
        );

    const AABB expandedCurveBox =
        expandByUAVDimensions(
            curveBox,
            dimensions
        );

    // ------------------------------------------------------------------------
    // No obstacle intersects this conservative region.
    // ------------------------------------------------------------------------

    if (!intersectsAnyObstacle(
            expandedCurveBox,
            map))
    {
        return true;
    }

    // ------------------------------------------------------------------------
    // Region is already at UAV scale.
    // ------------------------------------------------------------------------

    if (reachedUAVScale(
            curveBox,
            dimensions))
    {
        return false;
    }

    // ------------------------------------------------------------------------
    // Subdivision limits.
    // ------------------------------------------------------------------------

    const double parameterWidth =
        uMax - uMin;

    if (depth >=
            config::MAX_BEZIER_SUBDIVISION_DEPTH ||
        parameterWidth <=
            config::MIN_BEZIER_PARAMETER_INTERVAL)
    {
        return false;
    }

    // ------------------------------------------------------------------------
    // Subdivide using de Casteljau.
    // ------------------------------------------------------------------------

    BezierControlPoints left;
    BezierControlPoints right;

    subdivideBezier(
        curve,
        left,
        right
    );

    const double uMid =
        0.5 * (uMin + uMax);

    // ------------------------------------------------------------------------
    // Both child curves must be collision-free.
    // ------------------------------------------------------------------------

    if (!subCurveCollisionFree(
            left,
            map,
            dimensions,
            depth + 1,
            uMin,
            uMid))
    {
        return false;
    }

    if (!subCurveCollisionFree(
            right,
            map,
            dimensions,
            depth + 1,
            uMid,
            uMax))
    {
        return false;
    }

    return true;
}

} // namespace


// ============================================================================
// PUBLIC COLLISION INTERFACE
// ============================================================================

bool bezierCollisionFree(
    const BezierSegment& segment,
    const Map3D& map,
    const UAVDimensions& dimensions)
{
    // ------------------------------------------------------------------------
    // Validate Bézier control points.
    // ------------------------------------------------------------------------

    if (!segment.P0.isFinite() ||
        !segment.P1.isFinite() ||
        !segment.P2.isFinite() ||
        !segment.P3.isFinite())
    {
        return false;
    }

    // ------------------------------------------------------------------------
    // Validate UAV dimensions.
    // ------------------------------------------------------------------------

    if (!std::isfinite(dimensions.x) ||
        !std::isfinite(dimensions.y) ||
        !std::isfinite(dimensions.z))
    {
        return false;
    }

    if (dimensions.x < 0.0 ||
        dimensions.y < 0.0 ||
        dimensions.z < 0.0)
    {
        return false;
    }

    // ------------------------------------------------------------------------
    // Empty obstacle map.
    // ------------------------------------------------------------------------

    if (map.obstacles.empty())
    {
        return true;
    }

    // ------------------------------------------------------------------------
    // Convert public segment into local subdivision representation.
    // ------------------------------------------------------------------------

    const BezierControlPoints curve{
        segment.P0,
        segment.P1,
        segment.P2,
        segment.P3
    };

    // ------------------------------------------------------------------------
    // Recursively evaluate the complete Bézier curve.
    // ------------------------------------------------------------------------

    return subCurveCollisionFree(
        curve,
        map,
        dimensions,
        0,
        0.0,
        1.0
    );
}

} // namespace uav::collision