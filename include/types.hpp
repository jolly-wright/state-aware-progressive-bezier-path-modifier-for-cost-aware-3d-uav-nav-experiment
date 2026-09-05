#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>
#include <string>
#include <array>
#include <algorithm>

#ifdef ARDUINO
#undef PI
#undef radians
#undef degrees
#endif

namespace uav {

// ============================================================================
// Basic constants
// ============================================================================

constexpr double PI = 3.1415926535897932384626433832795;
constexpr double INF = std::numeric_limits<double>::infinity();


// ============================================================================
// 3-D vector
// ============================================================================

struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3() = default;

    Vec3(double x_, double y_, double z_)
        : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vec3 operator-(const Vec3& other) const
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vec3 operator*(double s) const
    {
        return {x * s, y * s, z * s};
    }

    Vec3 operator/(double s) const
    {
        return {x / s, y / s, z / s};
    }

    Vec3& operator+=(const Vec3& other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vec3& operator*=(double s)
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    double squaredNorm() const
    {
        return x * x + y * y + z * z;
    }

    double norm() const
    {
        return std::sqrt(squaredNorm());
    }

    bool isFinite() const
    {
        return std::isfinite(x) &&
               std::isfinite(y) &&
               std::isfinite(z);
    }
};


// ============================================================================
// Vector mathematics
// ============================================================================

inline double dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x +
           a.y * b.y +
           a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

inline double distance(const Vec3& a, const Vec3& b)
{
    return (a - b).norm();
}

inline Vec3 normalized(const Vec3& v, double eps)
{
    const double n = v.norm();

    if (n < eps)
        return {};

    return v / n;
}

inline double clamp(double value, double lo, double hi)
{
    return std::max(lo, std::min(value, hi));
}

inline double clamp01(double value)
{
    return clamp(value, 0.0, 1.0);
}

constexpr double radians(double degrees)
{
    return degrees * PI / 180.0;
}

constexpr double degrees(double radiansValue)
{
    return radiansValue * 180.0 / PI;
}


// ============================================================================
// Axis-aligned bounding box
// ============================================================================

struct AABB
{
    Vec3 min;
    Vec3 max;

    bool intersects(const AABB& other) const
    {
        return
            min.x <= other.max.x && max.x >= other.min.x &&
            min.y <= other.max.y && max.y >= other.min.y &&
            min.z <= other.max.z && max.z >= other.min.z;
    }
};


// ============================================================================
// UAV geometry
// ============================================================================

struct UAVDimensions
{
    // Full physical dimensions of the vehicle.
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    // Paper defines D_UAV as the maximum physical dimension.
    double maxDimension() const
    {
        return std::max({x, y, z});
    }

    Vec3 halfExtents() const
    {
        return {
            0.5 * x,
            0.5 * y,
            0.5 * z
        };
    }
};


// ============================================================================
// Obstacle
// ============================================================================

struct Obstacle
{
    // Axis-aligned obstacle bounds in the common world frame.
    AABB bounds;

    // Optional identifier for debugging / scenario logging.
    std::size_t id = 0;
};


// ============================================================================
// Map
// ============================================================================

struct Map3D
{
    // Spatial resolution of the upstream map.
    double resolution = 0.0;

    // Overall map bounds.
    AABB bounds;

    // Obstacles represented as axis-aligned boxes.
    std::vector<Obstacle> obstacles;
};


// ============================================================================
// Upstream waypoint representation
// ============================================================================

struct WaypointPath
{
    // C = {C0, C1, ..., CN}
    std::vector<Vec3> points;

    bool empty() const
    {
        return points.empty();
    }

    std::size_t size() const
    {
        return points.size();
    }

    const Vec3& operator[](std::size_t i) const
    {
        return points[i];
    }
};


// ============================================================================
// UAV state at a segment-generation event
// ============================================================================

struct UAVState
{
    Vec3 position;
    Vec3 velocity;

    double speed() const
    {
        return velocity.norm();
    }

    bool moving(double eps) const
    {
        return speed() > eps;
    }
};


// ============================================================================
// Cubic Bézier segment
// ============================================================================

struct BezierSegment
{
    Vec3 P0;
    Vec3 P1;
    Vec3 P2;
    Vec3 P3;

    // Index of the upstream corner that this segment reaches.
    std::size_t cornerIndex = 0;

    // Optional generation timestamp/duration for experiments.
    double generationTimeMs = 0.0;

    Vec3 evaluate(double u) const
    {
        u = clamp01(u);

        const double oneMinusU = 1.0 - u;

        return
            P0 * (oneMinusU * oneMinusU * oneMinusU) +
            P1 * (3.0 * oneMinusU * oneMinusU * u) +
            P2 * (3.0 * oneMinusU * u * u) +
            P3 * (u * u * u);
    }

    Vec3 firstDerivative(double u) const
    {
        u = clamp01(u);

        const double a = 1.0 - u;

        return
            (P1 - P0) * (3.0 * a * a) +
            (P2 - P1) * (6.0 * a * u) +
            (P3 - P2) * (3.0 * u * u);
    }

    Vec3 secondDerivative(double u) const
    {
        u = clamp01(u);

        return
            (P2 - P1 * 2.0 + P0) * (6.0 * (1.0 - u)) +
            (P3 - P2 * 2.0 + P1) * (6.0 * u);
    }
};


// ============================================================================
// Complete proposed trajectory
// ============================================================================

struct BezierTrajectory
{
    std::vector<BezierSegment> segments;

    Vec3 endpoint() const
    {
        if (segments.empty())
            return {};

        return segments.back().P3;
    }

    std::size_t segmentCount() const
    {
        return segments.size();
    }
};


// ============================================================================
// Generic sampled trajectory
//
// Used so the simulator can evaluate A*, B-spline and Bézier trajectories
// through the same downstream execution/evaluation interface.
// ============================================================================

struct TrajectorySample
{
    double u = 0.0;
    Vec3 position;
    Vec3 tangent;
    double curvature = 0.0;
};

struct SampledTrajectory
{
    std::vector<TrajectorySample> samples;

    double pathLength = 0.0;
    double maximumLateralAcceleration = 0.0;
    double executionTime = 0.0;
};


// ============================================================================
// Candidate representation used by the proposed search
// ============================================================================

struct Candidate
{
    Vec3 P2;

    // Candidate's associated Bézier segment.
    BezierSegment segment;

    // Maximum estimated curvature.
    double kappaMax = 0.0;

    // Lateral acceleration at the current speed.
    double lateralAcceleration = 0.0;

    // Normalized cost terms.
    double Jd = 0.0;
    double Jk = 0.0;
    double Jl = 0.0;

    // Total weighted candidate cost.
    double J = INF;

    // Search bookkeeping.
    std::size_t anchorIndex = 0;
    std::size_t rayIndex = 0;
    std::size_t radialIndex = 0;

    bool collisionFree = false;
    bool curvatureFeasible = false;

    bool feasible() const
    {
        return collisionFree;
    }
};


// ============================================================================
// Anchor
// ============================================================================

struct Anchor
{
    Vec3 position;
    std::size_t index = 0;
};


// ============================================================================
// Ray
// ============================================================================

struct Ray
{
    Vec3 direction;

    // Angular coordinate in the local normal plane.
    double angle = 0.0;

    std::size_t index = 0;
};


// ============================================================================
// Stage-3 refinement interval
// ============================================================================

struct AngularInterval
{
    double thetaLeft = 0.0;
    double thetaRight = 0.0;

    // Lowest-cost candidate currently associated with each boundary.
    Candidate leftCandidate;
    Candidate rightCandidate;
};


// ============================================================================
// Search result for one selected corner
// ============================================================================

struct CornerSearchResult
{
    bool success = false;

    std::size_t cornerIndex = 0;

    Vec3 selectedP2;

    Candidate bestCandidate;

    // Second-best direction used to initialize Stage 3.
    Candidate secondBestCandidate;

    // Number of candidate evaluations performed.
    std::size_t candidateEvaluations = 0;

    // Number of Stage-3 refinement iterations.
    std::size_t refinementIterations = 0;
};


// ============================================================================
// Progressive modifier result
// ============================================================================

struct ModifierResult
{
    bool success = false;

    BezierTrajectory trajectory;

    // Number of upstream corners incorporated.
    std::size_t incorporatedCorners = 0;

    // Number of generated Bézier segments.
    std::size_t generatedSegments = 0;

    // Sum of generation times of accepted segments.
    double generationTimeMs = 0.0;

    // Search statistics.
    std::size_t stage1CandidateEvaluations = 0;
    std::size_t stage3CandidateEvaluations = 0;
    std::size_t refinementIterations = 0;
};


// ============================================================================
// Trajectory-level measurements
// ============================================================================

struct TrajectoryMetrics
{
    double pathLength = 0.0;

    // Maximum actual lateral acceleration under the simulated velocity
    // profile. This is NOT treated as a failure criterion.
    double maximumLateralAcceleration = 0.0;

    // Method-specific trajectory generation time.
    double generationTimeMs = 0.0;

    // Simulated time to traverse the generated trajectory.
    double executionTimeMs = 0.0;

    // Defined as generation + execution for the experiment.
    double totalCompletionTimeMs = 0.0;
};


// ============================================================================
// Scenario-level result
// ============================================================================

struct MethodResult
{
    std::string methodName;

    TrajectoryMetrics metrics;

    bool trajectoryGenerated = false;
};


// ============================================================================
// Complete experiment result
// ============================================================================

struct ScenarioResult
{
    std::size_t scenarioId = 0;

    MethodResult rawAStar;
    MethodResult bSpline;
    MethodResult proposed;
};

} // namespace uav