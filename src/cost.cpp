#include "cost.hpp"

#include <algorithm>
#include <cmath>

namespace uav::cost {

// ============================================================================
// DIRECTIONAL COST
// ============================================================================
//
// Paper:
//
//   J_d = 1/2 * (1 - (v^T B'(0)) / (|v| |B'(0)|))
//
// For a stationary UAV, J_d = 0 because no velocity direction exists.
//
// ============================================================================

double directionalCost(
    const BezierSegment& segment,
    const Vec3& velocity,
    double eps)
{
    const double velocityNorm =
        velocity.norm();

    const Vec3 tangent =
        segment.firstDerivative(0.0);

    const double tangentNorm =
        tangent.norm();

    // No instantaneous velocity direction exists for a stationary UAV.
    if (velocityNorm < eps ||
        tangentNorm < eps)
    {
        return 0.0;
    }

    const double cosine =
        clamp(
            dot(velocity, tangent) /
            (velocityNorm * tangentNorm),
            -1.0,
            1.0
        );

    return 0.5 * (1.0 - cosine);
}


// ============================================================================
// CURVATURE-DEMAND COST
// ============================================================================
//
// Paper:
//
//   J_k = kappa_max * |v|^2 / A_MAX
//
// ============================================================================

double curvatureCost(
    double kappaMax,
    const Vec3& velocity,
    const config::VehicleConfig& vehicle,
    double eps)
{
    const double speed =
        velocity.norm();

    // No lateral-acceleration demand at zero velocity.
    if (speed < eps)
    {
        return 0.0;
    }

    // Prevent division by zero or invalid acceleration configuration.
    if (vehicle.A_MAX <= eps)
    {
        return INF;
    }

    return
        kappaMax * speed * speed /
        vehicle.A_MAX;
}


// ============================================================================
// PATH-LENGTH EXCESS COST
// ============================================================================
//
// Paper:
//
//   J_l = max(0, (L_path - L_up) / L_up)
//
// L_path:
//     Length of the candidate Bézier path.
//
// L_up:
//     Length of the corresponding upstream path.
//
// ============================================================================

double pathLengthCost(
    double pathLength,
    double upstreamPathLength,
    double eps)
{
    // Avoid division by an effectively zero upstream path length.
    if (upstreamPathLength < eps)
    {
        return 0.0;
    }

    return std::max(
        0.0,
        (pathLength - upstreamPathLength) /
        upstreamPathLength
    );
}


// ============================================================================
// TOTAL COST
// ============================================================================
//
// Paper:
//
//   J = w_d J_d + w_k J_k + w_l J_l
//
// ============================================================================

double totalCost(
    double Jd,
    double Jk,
    double Jl,
    double wD,
    double wK,
    double wL)
{
    return
        wD * Jd +
        wK * Jk +
        wL * Jl;
}


// ============================================================================
// COMPLETE CANDIDATE COST EVALUATION
// ============================================================================

void evaluateCost(
    Candidate& candidate,
    const Vec3& velocity,
    const config::VehicleConfig& vehicle,
    double pathLength,
    double upstreamPathLength,
    double wD,
    double wK,
    double wL,
    double epsGeometry)
{
    // ------------------------------------------------------------------------
    // Directional continuity
    // ------------------------------------------------------------------------

    candidate.Jd =
        directionalCost(
            candidate.segment,
            velocity,
            epsGeometry
        );


    // ------------------------------------------------------------------------
    // Curvature demand
    // ------------------------------------------------------------------------

    candidate.Jk =
        curvatureCost(
            candidate.kappaMax,
            velocity,
            vehicle,
            epsGeometry
        );


    // ------------------------------------------------------------------------
    // Path-length excess
    // ------------------------------------------------------------------------

    candidate.Jl =
        pathLengthCost(
            pathLength,
            upstreamPathLength,
            epsGeometry
        );


    // ------------------------------------------------------------------------
    // Final weighted candidate cost
    // ------------------------------------------------------------------------

    candidate.J =
        totalCost(
            candidate.Jd,
            candidate.Jk,
            candidate.Jl,
            wD,
            wK,
            wL
        );
}

} // namespace uav::cost