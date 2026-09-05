#pragma once

#include "types.hpp"
#include "config.hpp"

namespace uav::cost {

// ============================================================================
// COST COMPONENTS
// ============================================================================

// Directional continuity cost:
//
//   Jd = 1/2 * (1 - cos(theta))
//
// Measures deviation between the UAV's current velocity direction and
// the initial Bézier tangent.
//
// Returns 0 for aligned directions and 1 for opposite directions.
// For a stationary UAV, returns 0 because no velocity direction exists.
double directionalCost(
    const BezierSegment& segment,
    const Vec3& velocity,
    double eps = 1e-9
);


// ============================================================================

// Curvature-demand cost:
//
//   Jk = kappa_max * |v|^2 / A_MAX
//
// This represents the lateral-acceleration demand relative to the
// vehicle's allowable lateral acceleration.
//
// NOTE:
// Jk is NOT a rejection criterion.
// It may exceed 1 during execution when the UAV must reduce its velocity
// to satisfy A_MAX.
//
// kappaMax is supplied by the curvature-evaluation module.
double curvatureCost(
    double kappaMax,
    const Vec3& velocity,
    const config::VehicleConfig& vehicle,
    double eps = 1e-9
);


// ============================================================================

// Path-length excess cost:
//
//   Jl = 1 - ||P3 - P0|| / L_path
//
// Measures excess curve length relative to the direct endpoint distance.
//
// Returns 0 for a straight connection.
double pathLengthCost(
    double pathLength,
    double upstreamPathLength,
    double eps = 1e-9
);


// ============================================================================
// TOTAL COST
// ============================================================================

// Weighted candidate cost:
//
//   J = W_D * Jd + W_K * Jk + W_L * Jl
//
// The weights are supplied explicitly so the cost module does not depend
// on scenario-specific or global vehicle state.
double totalCost(
    double Jd,
    double Jk,
    double Jl,
    double wD,
    double wK,
    double wL
);


// ============================================================================
// COMPLETE CANDIDATE EVALUATION
// ============================================================================

// Calculates all three normalized cost components and the final weighted
// cost for a candidate.
//
// The candidate's kappaMax must already have been determined by the
// curvature-evaluation module.
//
// Collision status is deliberately NOT handled here.
void evaluateCost(
    Candidate& candidate,
    const Vec3& velocity,
    const config::VehicleConfig& vehicle,
    double pathLength,
    double upstreamPathLength,
    double wD,
    double wK,
    double wL,
    double epsGeometry = 1e-9
);

} // namespace uav::cost