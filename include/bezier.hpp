#pragma once

#include "types.hpp"
#include "config.hpp"

namespace uav::bezier {

// ============================================================================
// CONTROL-POINT CONSTRUCTION
// ============================================================================

// Construct P1 according to the paper's state-aware initialization rule.
//
// For a moving UAV:
//   - P1 is aligned with the current velocity.
//   - Its distance is determined by UAV size, current speed,
//     and heading agreement with the current upstream corner.
//   - If the P0 -> P1 segment enters an obstacle expanded by the
//     UAV half-extents, P1 is shortened to the first obstacle boundary.
//
// For a stationary UAV:
//   - P1 is placed D_UAV from P0 toward the current corner.
Vec3 makeP1(
    const Vec3& P0,
    const Vec3& corner,
    const UAVState& state,
    const config::VehicleConfig& vehicle,
    const Map3D& map,
    double eps = config::EPS_GEOMETRY
);


// Construct P3 for a corner according to the paper.
//
// previousCorner = C_{i-1}
// corner         = C_i
// nextCorner    = C_{i+1}
Vec3 makeP3(
    const Vec3& previousCorner,
    const Vec3& corner,
    const Vec3& nextCorner,
    double D_UAV,
    double eps = config::EPS_GEOMETRY
);


// ============================================================================
// BÉZIER SEGMENT CONSTRUCTION
// ============================================================================

BezierSegment makeSegment(
    const Vec3& P0,
    const Vec3& P1,
    const Vec3& P2,
    const Vec3& P3,
    std::size_t cornerIndex = 0
);


// ============================================================================
// CURVATURE
// ============================================================================

// Estimate the maximum curvature of a cubic Bézier segment.
//
// Curvature:
//
//   kappa(u) = ||B'(u) x B''(u)|| / ||B'(u)||^3
//
// Evaluation ignores parameter locations where ||B'(u)|| < epsilon.
// Adaptive subdivision uses the implementation parameters in config.hpp.
double maximumCurvature(
    const BezierSegment& segment,
    double epsDerivative = config::EPS_BEZIER_DERIVATIVE,
    double epsCurvature = config::EPS_CURVATURE
);


// ============================================================================
// ARC LENGTH
// ============================================================================

// Numerically estimate:
//
//   L = integral_0^1 ||B'(u)|| du
//
// The number of integration subdivisions is supplied by config.hpp.
double arcLength(
    const BezierSegment& segment,
    std::size_t samples = config::ARC_LENGTH_SAMPLES
);

} // namespace uav::bezier