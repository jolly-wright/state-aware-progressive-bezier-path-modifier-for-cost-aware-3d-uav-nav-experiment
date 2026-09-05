#include "bezier.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace uav::bezier {

namespace {

// ============================================================================
// RAY / AABB INTERSECTION
// ============================================================================

bool rayAABBEntryDistance(
    const Vec3& origin,
    const Vec3& direction,
    const AABB& box,
    double& entryDistance,
    double eps
)
{
    double tMin = 0.0;
    double tMax = std::numeric_limits<double>::infinity();

    const double originValues[3] = {
        origin.x, origin.y, origin.z
    };

    const double directionValues[3] = {
        direction.x, direction.y, direction.z
    };

    const double minValues[3] = {
        box.min.x, box.min.y, box.min.z
    };

    const double maxValues[3] = {
        box.max.x, box.max.y, box.max.z
    };

    for (int axis = 0; axis < 3; ++axis)
    {
        const double o = originValues[axis];
        const double d = directionValues[axis];

        if (std::abs(d) <= eps)
        {
            if (o < minValues[axis] || o > maxValues[axis])
            {
                return false;
            }

            continue;
        }

        double t1 = (minValues[axis] - o) / d;
        double t2 = (maxValues[axis] - o) / d;

        if (t1 > t2)
        {
            std::swap(t1, t2);
        }

        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);

        if (tMin > tMax)
        {
            return false;
        }
    }

    if (tMax < 0.0)
    {
        return false;
    }

    entryDistance = std::max(0.0, tMin);
    return true;
}


// ============================================================================
// FIRST OBSTACLE BOUNDARY
// ============================================================================

bool firstObstacleBoundaryDistance(
    const Vec3& origin,
    const Vec3& direction,
    const Map3D& map,
    const UAVDimensions& dimensions,
    double& boundaryDistance,
    double eps
)
{
    const Vec3 half = dimensions.halfExtents();

    bool found = false;
    double nearest = std::numeric_limits<double>::infinity();

    for (const Obstacle& obstacle : map.obstacles)
    {
        AABB expanded{
            {
                obstacle.bounds.min.x - half.x,
                obstacle.bounds.min.y - half.y,
                obstacle.bounds.min.z - half.z
            },
            {
                obstacle.bounds.max.x + half.x,
                obstacle.bounds.max.y + half.y,
                obstacle.bounds.max.z + half.z
            }
        };

        double distance = 0.0;

        if (rayAABBEntryDistance(
                origin,
                direction,
                expanded,
                distance,
                eps))
        {
            if (distance < nearest)
            {
                nearest = distance;
                found = true;
            }
        }
    }

    if (!found)
    {
        return false;
    }

    boundaryDistance = nearest;
    return true;
}


// ============================================================================
// CURVATURE EVALUATION
// ============================================================================

double curvatureAt(
    const BezierSegment& segment,
    double u,
    double epsDerivative,
    double epsCurvature
)
{
    const Vec3 d1 = segment.firstDerivative(u);
    const Vec3 d2 = segment.secondDerivative(u);

    const double speedSquared = d1.squaredNorm();

    if (!std::isfinite(speedSquared) ||
        speedSquared < epsDerivative * epsDerivative)
    {
        return 0.0;
    }

    const double speed = std::sqrt(speedSquared);

    const Vec3 crossProduct = cross(d1, d2);
    const double numerator = crossProduct.norm();

    if (!std::isfinite(numerator))
    {
        return 0.0;
    }

    const double denominator = speed * speed * speed;

    if (denominator <= epsCurvature)
    {
        return 0.0;
    }

    const double curvature = numerator / denominator;

    if (!std::isfinite(curvature))
    {
        return 0.0;
    }

    return curvature;
}


// ============================================================================
// ADAPTIVE CURVATURE SUBDIVISION
// ============================================================================

double adaptiveCurvature(
    const BezierSegment& segment,
    double uLeft,
    double uRight,
    double kLeft,
    double kMid,
    double kRight,
    std::size_t depth,
    double epsDerivative,
    double epsCurvature
)
{
    const double currentMaximum =
        std::max({kLeft, kMid, kRight});

    const double currentMinimum =
        std::min({kLeft, kMid, kRight});

    const double spread =
        currentMaximum - currentMinimum;

    const double intervalWidth =
        uRight - uLeft;

    if (depth >= config::MAX_CURVATURE_SUBDIVISION_DEPTH ||
        intervalWidth <= config::MIN_CURVATURE_PARAMETER_INTERVAL ||
        spread <= config::CURVATURE_TOLERANCE)
    {
        return currentMaximum;
    }

    const double uMid =
        0.5 * (uLeft + uRight);

    const double uQuarter =
        0.5 * (uLeft + uMid);

    const double uThreeQuarter =
        0.5 * (uMid + uRight);

    const double kQuarter =
        curvatureAt(
            segment,
            uQuarter,
            epsDerivative,
            epsCurvature
        );

    const double kThreeQuarter =
        curvatureAt(
            segment,
            uThreeQuarter,
            epsDerivative,
            epsCurvature
        );

    const double leftMaximum =
        adaptiveCurvature(
            segment,
            uLeft,
            uMid,
            kLeft,
            kQuarter,
            kMid,
            depth + 1,
            epsDerivative,
            epsCurvature
        );

    const double rightMaximum =
        adaptiveCurvature(
            segment,
            uMid,
            uRight,
            kMid,
            kThreeQuarter,
            kRight,
            depth + 1,
            epsDerivative,
            epsCurvature
        );

    return std::max(leftMaximum, rightMaximum);
}

} // namespace


// ============================================================================
// P1 CONSTRUCTION
// ============================================================================

Vec3 makeP1(
    const Vec3& P0,
    const Vec3& corner,
    const UAVState& state,
    const config::VehicleConfig& vehicle,
    const Map3D& map,
    double eps
)
{
    const Vec3 toCorner = corner - P0;
    const double cornerDistance = toCorner.norm();

    if (!std::isfinite(cornerDistance) ||
        cornerDistance < eps)
    {
        return P0;
    }

    const Vec3 cHat = toCorner / cornerDistance;

    // UAVState::moving() requires the geometric epsilon.
    if (!state.moving(eps))
    {
        return P0 + cHat * config::D_UAV;
    }

    const double speed = state.speed();

    if (!std::isfinite(speed) || speed < eps)
    {
        return P0 + cHat * config::D_UAV;
    }

    const Vec3 tHat = state.velocity / speed;

    // eta_theta = (1 + t_hat^T c_hat) / 2
    const double etaTheta =
        0.5 * (1.0 + dot(tHat, cHat));

    // d1_max = max(D_UAV, ||C_i-P0|| - D_UAV)
    const double d1Max =
        std::max(
            config::D_UAV,
            cornerDistance - config::D_UAV
        );

    // |v| / V_MAX
    double speedRatio = 0.0;

    if (vehicle.V_MAX > eps)
    {
        speedRatio =
            std::clamp(
                speed / vehicle.V_MAX,
                0.0,
                1.0
            );
    }

    // d1_nom =
    // D_UAV +
    // (d1_max - D_UAV)
    // (|v| / V_MAX)
    // eta_theta
    const double d1Nominal =
        config::D_UAV +
        (d1Max - config::D_UAV) *
        speedRatio *
        etaTheta;

    const Vec3 nominalP1 =
        P0 + tHat * d1Nominal;

    // If P0 -> P1 enters an expanded obstacle,
    // shorten P1 to the first obstacle boundary.
    double boundaryDistance = 0.0;

    if (firstObstacleBoundaryDistance(
            P0,
            tHat,
            map,
            vehicle.dimensions,
            boundaryDistance,
            eps))
    {
        const double d1 =
            std::min(
                d1Nominal,
                boundaryDistance
            );

        return P0 + tHat * d1;
    }

    return nominalP1;
}


// ============================================================================
// P3 CONSTRUCTION
// ============================================================================

Vec3 makeP3(
    const Vec3& previousCorner,
    const Vec3& corner,
    const Vec3& nextCorner,
    double D_UAV,
    double eps
)
{
    const Vec3 incoming =
        corner - previousCorner;

    const Vec3 outgoing =
        nextCorner - corner;

    const double incomingLength =
        incoming.norm();

    const double outgoingLength =
        outgoing.norm();

    if (!std::isfinite(incomingLength) ||
        !std::isfinite(outgoingLength) ||
        incomingLength < eps ||
        outgoingLength < eps)
    {
        return corner;
    }

    const Vec3 incomingDirection =
        incoming / incomingLength;

    const Vec3 outgoingDirection =
        outgoing / outgoingLength;

    const double cosTheta =
        std::clamp(
            dot(incomingDirection, outgoingDirection),
            -1.0,
            1.0
        );

    const double theta =
        std::acos(cosTheta);

    // Paper: if distance < 2 D_UAV, use midpoint.
    if (outgoingLength < 2.0 * D_UAV)
    {
        return corner + outgoing * 0.5;
    }

    // d3_min = D_UAV
    // d3_max = L_i,i+1 - D_UAV
    const double d3Min = D_UAV;
    const double d3Max = outgoingLength - D_UAV;

    // d3 =
    // d3_min +
    // (d3_max - d3_min) * (1 + cos(theta)) / 2
    const double d3 =
        d3Min +
        (d3Max - d3Min) *
        (1.0 + std::cos(theta)) *
        0.5;

    return corner + outgoingDirection * d3;
}


// ============================================================================
// SEGMENT CONSTRUCTION
// ============================================================================

BezierSegment makeSegment(
    const Vec3& P0,
    const Vec3& P1,
    const Vec3& P2,
    const Vec3& P3,
    std::size_t cornerIndex
)
{
    BezierSegment segment;

    segment.P0 = P0;
    segment.P1 = P1;
    segment.P2 = P2;
    segment.P3 = P3;
    segment.cornerIndex = cornerIndex;
    segment.generationTimeMs = 0.0;

    return segment;
}


// ============================================================================
// MAXIMUM CURVATURE
// ============================================================================

double maximumCurvature(
    const BezierSegment& segment,
    double epsDerivative,
    double epsCurvature
)
{
    const double k0 =
        curvatureAt(
            segment,
            0.0,
            epsDerivative,
            epsCurvature
        );

    const double kMid =
        curvatureAt(
            segment,
            0.5,
            epsDerivative,
            epsCurvature
        );

    const double k1 =
        curvatureAt(
            segment,
            1.0,
            epsDerivative,
            epsCurvature
        );

    return adaptiveCurvature(
        segment,
        0.0,
        1.0,
        k0,
        kMid,
        k1,
        0,
        epsDerivative,
        epsCurvature
    );
}


// ============================================================================
// ARC LENGTH
// ============================================================================

double arcLength(
    const BezierSegment& segment,
    std::size_t samples
)
{
    if (samples == 0)
    {
        return 0.0;
    }

    // Simpson's rule requires an even number of intervals.
    if (samples % 2 != 0)
    {
        ++samples;
    }

    const double h =
        1.0 / static_cast<double>(samples);

    auto speedAt = [&segment](double u) -> double
    {
        return segment.firstDerivative(u).norm();
    };

    double sum =
        speedAt(0.0) +
        speedAt(1.0);

    for (std::size_t i = 1; i < samples; ++i)
    {
        const double u =
            static_cast<double>(i) * h;

        const double weight =
            (i % 2 == 0) ? 2.0 : 4.0;

        sum += weight * speedAt(u);
    }

    return (h / 3.0) * sum;
}

} // namespace uav::bezier