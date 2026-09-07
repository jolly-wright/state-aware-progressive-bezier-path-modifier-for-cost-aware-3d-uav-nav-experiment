#pragma once

#include "types.hpp"

namespace uav::config {

// ============================================================================
// UAV PARAMETERS
// ============================================================================

// Full physical dimensions of the simulated UAV [m].
// Replace with the actual UAV dimensions used in the experiment.
inline constexpr UAVDimensions UAV_DIMENSIONS{
    0.0,    // x
    0.0,    // y
    0.0     // z
};

// D_UAV = maximum physical dimension of the UAV [m].
// Do NOT independently enter this if it can be derived from UAV_DIMENSIONS.
inline constexpr double D_UAV =
    std::max({
        UAV_DIMENSIONS.x,
        UAV_DIMENSIONS.y,
        UAV_DIMENSIONS.z
    });


// ============================================================================
// VEHICLE MOTION LIMITS
// ============================================================================

// Configured maximum operating velocity [m/s].
// Maximum allowable lateral acceleration [m/s^2].
// This is vehicle-specific and is also used for curvature-based
// velocity regulation during simulated execution.
struct VehicleConfig
{
    UAVDimensions dimensions;
    double V_MAX;
    double A_MAX;
};

// Initial UAV speed used by the experiment [m/s].
inline constexpr double INITIAL_SPEED = 0.0;


// ============================================================================
// COST FUNCTION
// ============================================================================

// Fixed globally across all experiments.
inline constexpr double W_D = 0.1;
inline constexpr double W_K = 0.2;
inline constexpr double W_L = 0.7;


// ============================================================================
// UPSTREAM MAP
// ============================================================================

// Upstream map spatial resolution [m].
// This is used by the Stage-3 angular-refinement termination criterion.
inline constexpr double MAP_RESOLUTION = 0.0;


// ============================================================================
// GEOMETRIC NUMERICAL SAFEGUARDS
// ============================================================================

// Minimum vector magnitude accepted for normalization [m] or equivalent
// geometric scale.
//
// This corresponds to the epsilon_g concept in the paper.
inline constexpr double EPS_GEOMETRY = 1e-9;

// Minimum Bézier derivative magnitude accepted during curvature evaluation.
// Corresponds to epsilon_B in the formulation.
inline constexpr double EPS_BEZIER_DERIVATIVE = 1e-9;

// Minimum curvature used in velocity regulation to avoid division by zero.
// Corresponds to epsilon_kappa.
inline constexpr double EPS_CURVATURE = 1e-9;


// ============================================================================
// STAGE 1 — COARSE SEARCH
// ============================================================================

// Number of coarse rays at each anchor.
//
// Explicitly fixed by the methodology:
// N_theta = 4.
inline constexpr std::size_t NUM_COARSE_RAYS = 4;

// Angular separation between the four uniformly distributed rays.
// 360 / 4 = 90 degrees.
inline constexpr double COARSE_RAY_SPACING =
    2.0 * PI / static_cast<double>(NUM_COARSE_RAYS);

// Rotation applied at alternating anchors.
// Explicitly defined as 45 degrees.
inline constexpr double ALTERNATING_RAY_ROTATION =
    radians(45.0);

// Anchor spacing along P1 -> A_C.
// Explicitly D_UAV.
inline constexpr double ANCHOR_SPACING_MULTIPLIER = 1;

// Radial candidate spacing.
// Explicitly D_UAV.
inline constexpr double RADIAL_SPACING_MULTIPLIER = 2;


// ============================================================================
// STAGE 1 — SEARCH LIMITS
// ============================================================================

// Maximum radial search distance is L13/2.
//
// This is NOT a free constant; it is computed for every segment as:
//
// R_search = ||P3 - P1|| / 2
//
// Therefore there is intentionally no numerical R_SEARCH here.


// ============================================================================
// STAGE 3 — ANGULAR REFINEMENT
// ============================================================================

// Number of rays in each refinement stencil.
//
// Fixed by the methodology:
//
// {theta_L,
//  (3 theta_L + theta_R)/4,
//  (theta_L + theta_R)/2,
//  (theta_L + 3 theta_R)/4,
//  theta_R}
inline constexpr std::size_t NUM_REFINEMENT_RAYS = 5;

// Number of genuinely NEW rays per refinement iteration.
//
// The two boundary rays are reused.
inline constexpr std::size_t NEW_RAYS_PER_REFINEMENT = 3;


// ============================================================================
// COLLISION-CHECKING SUBDIVISION
// ============================================================================

// These are implementation parameters rather than new methodological
// constants. The paper states that Bézier candidates are recursively
// subdivided using de Casteljau subdivision.
//
// Maximum recursion depth is exposed so the implementation remains
// deterministic.
inline constexpr std::size_t MAX_BEZIER_SUBDIVISION_DEPTH = 12;

// Minimum parameter interval width at which subdivision stops.
// This is an implementation resolution and should be recorded if it is
// experimentally important.
inline constexpr double MIN_BEZIER_PARAMETER_INTERVAL = 1e-3;


// ============================================================================
// CURVATURE ESTIMATION
// ============================================================================

// Maximum adaptive subdivision depth used when estimating kappa_max.
//
// The paper specifies adaptive subdivision but does not prescribe a
// numerical depth, so this must remain an implementation parameter.
inline constexpr std::size_t MAX_CURVATURE_SUBDIVISION_DEPTH = 12;

// Parameter interval width below which curvature subdivision terminates.
inline constexpr double MIN_CURVATURE_PARAMETER_INTERVAL = 1e-3;

// Curvature convergence tolerance.
//
// Again, this is an implementation parameter, not a claimed paper constant.
inline constexpr double CURVATURE_TOLERANCE = 1e-6;


// ============================================================================
// TRAJECTORY SAMPLING
// ============================================================================

// Number of samples used when converting a generated trajectory into
// evaluation samples.
//
// This is NOT the candidate-search resolution.
inline constexpr std::size_t TRAJECTORY_SAMPLES_PER_SEGMENT = 200;


// ============================================================================
// PATH LENGTH NUMERICAL INTEGRATION
// ============================================================================

// Number of subdivisions used to numerically estimate Bézier arc length.
//
// Replace with a more rigorous adaptive integrator later if necessary.
inline constexpr std::size_t ARC_LENGTH_SAMPLES = 200;


// ============================================================================
// EXECUTION SIMULATION
// ============================================================================

// Integration time step [s].
//
// This controls the numerical simulation of UAV traversal, not the
// trajectory-generation algorithm.
inline constexpr double SIMULATION_DT = 0.001;

// Minimum velocity used to prevent numerical stagnation [m/s].
inline constexpr double MIN_SIMULATION_SPEED = 1e-6;


// ============================================================================
// EXPERIMENT CONFIGURATION
// ============================================================================

// Number of scenarios used for the broad evaluation.
// Set this to the actual number used in the paper.
inline constexpr std::size_t NUM_SCENARIOS = 100;

// Fixed random seed if scenarios are procedurally generated.
// Use a fixed seed so the experiment is reproducible.
inline constexpr unsigned int RANDOM_SEED = 12345;

// ============================================================================
// DERIVED EXECUTION QUANTITIES
// ============================================================================

// Curvature-limited velocity:
//
// v_curv = sqrt(A_MAX / max(kappa, EPS_CURVATURE))
//
// Commanded velocity:
//
// v_cmd = min(V_MAX, v_curv)
//
// These are kept as functions because they depend on the local trajectory.
inline double curvatureLimitedVelocity(double curvature, const VehicleConfig& vehicle)
{
    return std::sqrt(
        vehicle.A_MAX /
        std::max(curvature, EPS_CURVATURE)
    );
}

inline double commandedVelocity(double curvature, const VehicleConfig& vehicle)
{
    return std::min(
        vehicle.V_MAX,
        curvatureLimitedVelocity(curvature, vehicle)
    );
}

} // namespace uav::config