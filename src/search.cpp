#include "search.hpp"

#include "bezier.hpp"
#include "collision.hpp"
#include "cost.hpp"
#include "config.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace uav::search {

namespace {


// ============================================================================
// SEARCH TIMER
// ============================================================================

class SearchTimer
{
public:

    SearchTimer()
    {
#ifdef ARDUINO
        startMicros_ = micros();
#else
        startTime_ =
            std::chrono::steady_clock::now();
#endif
    }

    double elapsedMilliseconds() const
    {
#ifdef ARDUINO

        const unsigned long now =
            micros();

        return static_cast<double>(
            now - startMicros_
        ) * 0.001;

#else

        const auto now =
            std::chrono::steady_clock::now();

        return
            std::chrono::duration<double, std::milli>(
                now - startTime_
            ).count();

#endif
    }

private:

#ifdef ARDUINO

    unsigned long startMicros_ = 0;

#else

    std::chrono::steady_clock::time_point startTime_;

#endif
};


// ============================================================================
// HORIZON GUARD
// ============================================================================

class HorizonGuard
{
public:

    HorizonGuard(
        const SearchRuntime& runtime,
        const SearchTimer& timer)
        : runtime_(runtime),
          timer_(timer)
    {
    }

    bool expired() const
    {
        if (!std::isfinite(
                runtime_.remainingExecutionTime))
        {
            return false;
        }

        if (runtime_.remainingExecutionTime < 0.0)
        {
            return true;
        }

        return
            timer_.elapsedMilliseconds() >=
            runtime_.remainingExecutionTime;
    }

    bool cornerReachable(
        double distanceToCorner) const
    {
        if (!std::isfinite(
                runtime_.availableTravelDistance))
        {
            return true;
        }

        if (runtime_.availableTravelDistance < 0.0)
        {
            return false;
        }

        return
            distanceToCorner <=
            runtime_.availableTravelDistance +
            config::EPS_GEOMETRY;
    }

private:

    const SearchRuntime& runtime_;
    const SearchTimer& timer_;
};


// ============================================================================
// PROGRESS REPORTING
// ============================================================================

void reportProgress(
    const SearchInput& input,
    const SearchProgress& progress)
{
    if (input.progressCallback == nullptr)
    {
        return;
    }

    input.progressCallback(
        progress,
        input.progressUserData
    );
}


// ============================================================================
// INPUT VALIDATION
// ============================================================================

bool validVehicle(
    const config::VehicleConfig& vehicle)
{
    return
        vehicle.dimensions.x > 0.0 &&
        vehicle.dimensions.y > 0.0 &&
        vehicle.dimensions.z > 0.0 &&

        std::isfinite(
            vehicle.dimensions.x
        ) &&

        std::isfinite(
            vehicle.dimensions.y
        ) &&

        std::isfinite(
            vehicle.dimensions.z
        ) &&

        std::isfinite(
            vehicle.V_MAX
        ) &&

        std::isfinite(
            vehicle.A_MAX
        ) &&

        vehicle.V_MAX > 0.0 &&
        vehicle.A_MAX > 0.0;
}


bool validSearchInput(
    const SearchInput& input)
{
    if (input.upstreamPath == nullptr)
    {
        return false;
    }

    if (input.map == nullptr)
    {
        return false;
    }

    if (input.upstreamPath->size() < 2)
    {
        return false;
    }

    if (!validVehicle(
            input.vehicle))
    {
        return false;
    }

    if (!input.state.position.isFinite())
    {
        return false;
    }

    if (!input.state.velocity.isFinite())
    {
        return false;
    }

    if (!input.P0.isFinite())
    {
        return false;
    }

    if (!std::isfinite(
            input.map->resolution) ||
        input.map->resolution <= 0.0)
    {
        return false;
    }

    if (input.startCornerIndex >=
        input.upstreamPath->size())
    {
        return false;
    }

    if (input.hasPreviousSegment)
    {
        if (!input.previousSegment.P0.isFinite() ||
            !input.previousSegment.P1.isFinite() ||
            !input.previousSegment.P2.isFinite() ||
            !input.previousSegment.P3.isFinite())
        {
            return false;
        }

        if (!input.previousCorner.isFinite())
        {
            return false;
        }
    }

    return true;
}


// ============================================================================
// ANGLE UTILITIES
// ============================================================================

double normalizeAngle(
    double angle)
{
    while (angle < 0.0)
    {
        angle += 2.0 * PI;
    }

    while (angle >= 2.0 * PI)
    {
        angle -= 2.0 * PI;
    }

    return angle;
}


double angleOfVector(
    const Vec3& vector,
    const Vec3& n1,
    const Vec3& n2)
{
    const double x =
        dot(vector, n1);

    const double y =
        dot(vector, n2);

    return normalizeAngle(
        std::atan2(y, x)
    );
}


// ============================================================================
// NORMAL BASIS
// ============================================================================

struct NormalBasis
{
    Vec3 n1;
    Vec3 n2;
};


NormalBasis makeNormalBasis(
    const Vec3& e13)
{
    const std::array<Vec3, 3> axes = {
        Vec3{1.0, 0.0, 0.0},
        Vec3{0.0, 1.0, 0.0},
        Vec3{0.0, 0.0, 1.0}
    };

    std::size_t leastAligned = 0;

    double smallestAbsDot =
        std::abs(
            dot(
                e13,
                axes[0]
            )
        );

    for (std::size_t i = 1;
         i < axes.size();
         ++i)
    {
        const double value =
            std::abs(
                dot(
                    e13,
                    axes[i]
                )
            );

        if (value < smallestAbsDot)
        {
            smallestAbsDot = value;
            leastAligned = i;
        }
    }

    const Vec3 n1 =
        normalized(
            cross(
                e13,
                axes[leastAligned]
            ),
            config::EPS_GEOMETRY
        );

    const Vec3 n2 =
        normalized(
            cross(
                e13,
                n1
            ),
            config::EPS_GEOMETRY
        );

    return {
        n1,
        n2
    };
}


// ============================================================================
// COARSE RAYS
// ============================================================================

std::vector<Ray> makeCoarseRays(
    const NormalBasis& basis,
    std::size_t anchorIndex)
{
    std::vector<Ray> rays;

    rays.reserve(
        config::NUM_COARSE_RAYS
    );

    const double rotation =
        (anchorIndex % 2 == 1)
            ? config::ALTERNATING_RAY_ROTATION
            : 0.0;

    for (std::size_t i = 0;
         i < config::NUM_COARSE_RAYS;
         ++i)
    {
        const double theta =
            rotation +
            static_cast<double>(i) *
            config::COARSE_RAY_SPACING;

        const Vec3 direction =
            normalized(
                basis.n1 * std::cos(theta) +
                basis.n2 * std::sin(theta),
                config::EPS_GEOMETRY
            );

        rays.push_back({
            direction,
            normalizeAngle(theta),
            i
        });
    }

    return rays;
}


// ============================================================================
// ANCHORS
// ============================================================================
//
// P3 is the actual upstream corner.
//
// Anchor line:
//
// P1 ---------------- P3
//
// The final anchor is exactly P3.
//

std::vector<Anchor> makeAnchors(
    const Vec3& P1,
    const Vec3& P3,
    double D_UAV)
{
    std::vector<Anchor> anchors;

    const Vec3 P3MinusP1 =
        P3 - P1;

    const double L13 =
        P3MinusP1.norm();

    if (L13 <=
        config::EPS_GEOMETRY)
    {
        return anchors;
    }

    const Vec3 e13 =
        P3MinusP1 / L13;

    if (L13 < D_UAV)
    {
        anchors.push_back({
            (P1 + P3) * 0.5,
            0
        });

        return anchors;
    }

    std::size_t anchorIndex = 0;

    for (double travelled = 0.0;
         travelled <
             L13 -
             config::EPS_GEOMETRY;
         travelled +=
             D_UAV *
             config::ANCHOR_SPACING_MULTIPLIER)
    {
        anchors.push_back({
            P1 + e13 * travelled,
            anchorIndex++
        });
    }

    anchors.push_back({
        P3,
        anchorIndex
    });

    return anchors;
}


// ============================================================================
// CONTINUATION P1
// ============================================================================

Vec3 makeContinuationP1(
    const Vec3& P0,
    const BezierSegment& previousSegment)
{
    const Vec3 terminalDerivative =
        previousSegment.firstDerivative(
            1.0
        );

    const double derivativeLength =
        terminalDerivative.norm();

    const double previousLength =
        distance(
            previousSegment.P0,
            previousSegment.P3
        );

    if (derivativeLength <=
            config::EPS_GEOMETRY ||
        previousLength <=
            config::EPS_GEOMETRY)
    {
        return P0;
    }

    const Vec3 terminalTangent =
        terminalDerivative /
        derivativeLength;

    return
        P0 +
        terminalTangent *
        previousLength;
}


// ============================================================================
// LINE / AABB ENTRY
// ============================================================================

bool lineAABBEntry(
    const Vec3& P0,
    const Vec3& P1,
    const AABB& box,
    double& tEntry)
{
    const Vec3 direction =
        P1 - P0;

    double tMin = 0.0;
    double tMax = 1.0;

    const double origin[3] = {
        P0.x,
        P0.y,
        P0.z
    };

    const double delta[3] = {
        direction.x,
        direction.y,
        direction.z
    };

    const double boxMin[3] = {
        box.min.x,
        box.min.y,
        box.min.z
    };

    const double boxMax[3] = {
        box.max.x,
        box.max.y,
        box.max.z
    };

    for (std::size_t axis = 0;
         axis < 3;
         ++axis)
    {
        if (std::abs(delta[axis]) <=
            config::EPS_GEOMETRY)
        {
            if (origin[axis] <
                    boxMin[axis] ||
                origin[axis] >
                    boxMax[axis])
            {
                return false;
            }

            continue;
        }

        const double inverseDelta =
            1.0 / delta[axis];

        double t1 =
            (boxMin[axis] -
             origin[axis]) *
            inverseDelta;

        double t2 =
            (boxMax[axis] -
             origin[axis]) *
            inverseDelta;

        if (t1 > t2)
        {
            std::swap(
                t1,
                t2
            );
        }

        tMin =
            std::max(
                tMin,
                t1
            );

        tMax =
            std::min(
                tMax,
                t2
            );

        if (tMin > tMax)
        {
            return false;
        }
    }

    tEntry = tMin;

    return
        tEntry >=
            -config::EPS_GEOMETRY &&
        tEntry <=
            1.0 +
            config::EPS_GEOMETRY;
}


// ============================================================================
// LIMIT CONTINUATION P1
// ============================================================================

Vec3 limitP1ByNearestObstacle(
    const Vec3& P0,
    const Vec3& rawP1,
    const Map3D& map)
{
    const Vec3 direction =
        rawP1 - P0;

    const double length =
        direction.norm();

    if (length <=
        config::EPS_GEOMETRY)
    {
        return rawP1;
    }

    double nearestT =
        std::numeric_limits<double>::infinity();

    bool obstacleFound = false;

    for (const Obstacle& obstacle :
         map.obstacles)
    {
        double tEntry = 0.0;

        if (!lineAABBEntry(
                P0,
                rawP1,
                obstacle.bounds,
                tEntry))
        {
            continue;
        }

        if (tEntry < nearestT)
        {
            nearestT = tEntry;
            obstacleFound = true;
        }
    }

    if (!obstacleFound)
    {
        return rawP1;
    }

    return
        P0 +
        direction *
        nearestT;
}


// ============================================================================
// SEARCH P1
// ============================================================================

Vec3 makeSearchP1(
    const Vec3& P0,
    const Vec3& corner,
    const UAVState& state,
    const config::VehicleConfig& vehicle,
    const Map3D& map,
    bool hasPreviousSegment,
    const BezierSegment& previousSegment)
{
    if (!hasPreviousSegment)
    {
        return bezier::makeP1(
            P0,
            corner,
            state,
            vehicle,
            map,
            config::EPS_GEOMETRY
        );
    }

    const Vec3 rawP1 =
        makeContinuationP1(
            P0,
            previousSegment
        );

    return limitP1ByNearestObstacle(
        P0,
        rawP1,
        map
    );
}


// ============================================================================
// UPSTREAM LENGTH
// ============================================================================

double exactUpstreamLength(
    const Vec3& P0,
    const Vec3& P3)
{
    return distance(
        P0,
        P3
    );
}


// ============================================================================
// CANDIDATE EVALUATION
// ============================================================================

Candidate evaluateCandidate(
    const Vec3& P0,
    const Vec3& P1,
    const Vec3& P2,
    const Vec3& P3,
    std::size_t cornerIndex,
    const Map3D& map,
    const config::VehicleConfig& vehicle,
    const UAVState& state,
    double L_up)
{
    Candidate candidate;

    candidate.P2 =
        P2;

    candidate.segment.P0 =
        P0;

    candidate.segment.P1 =
        P1;

    candidate.segment.P2 =
        P2;

    candidate.segment.P3 =
        P3;

    candidate.segment.cornerIndex =
        cornerIndex;

    candidate.collisionFree =
        collision::bezierCollisionFree(
            candidate.segment,
            map,
            vehicle.dimensions
        );

    if (!candidate.collisionFree)
    {
        return candidate;
    }

    candidate.kappaMax =
        bezier::maximumCurvature(
            candidate.segment
        );

    const double speed =
        state.velocity.norm();

    candidate.lateralAcceleration =
        candidate.kappaMax *
        speed *
        speed;

    candidate.curvatureFeasible =
        speed <=
            config::EPS_GEOMETRY ||
        candidate.lateralAcceleration <=
            vehicle.A_MAX +
            config::EPS_GEOMETRY;

    const double pathLength =
        bezier::arcLength(
            candidate.segment
        );

    cost::evaluateCost(
        candidate,
        state.velocity,
        vehicle,
        pathLength,
        L_up,
        config::W_D,
        config::W_K,
        config::W_L,
        config::EPS_GEOMETRY
    );

    return candidate;
}


// ============================================================================
// STAGE 1 SEARCH
// ============================================================================

bool stage1SearchWithCorner(
    const Vec3& P0,
    const Vec3& P1,
    const Vec3& P3,
    std::size_t cornerIndex,
    const Map3D& map,
    const config::VehicleConfig& vehicle,
    const UAVState& state,
    double L_up,
    Candidate& best,
    Candidate& secondBest,
    SearchStatistics& statistics,
    const HorizonGuard& horizon)
{
    const double D_UAV =
        vehicle.dimensions.maxDimension();

    if (D_UAV <=
        config::EPS_GEOMETRY)
    {
        return false;
    }

    const Vec3 P3MinusP1 =
        P3 - P1;

    const double L13 =
        P3MinusP1.norm();

    if (L13 <=
        config::EPS_GEOMETRY)
    {
        return false;
    }

    const Vec3 e13 =
        P3MinusP1 / L13;

    const double Rsearch =
        L13 * 0.5;

    const NormalBasis basis =
        makeNormalBasis(
            e13
        );

    const std::vector<Anchor> anchors =
        makeAnchors(
            P1,
            P3,
            D_UAV
        );

    bool foundBest = false;
    bool foundSecond = false;

    for (const Anchor& anchor :
         anchors)
    {
        if (horizon.expired())
        {
            break;
        }

        const std::vector<Ray> rays =
            makeCoarseRays(
                basis,
                anchor.index
            );

        for (const Ray& ray :
             rays)
        {
            if (horizon.expired())
            {
                break;
            }

            for (std::size_t radialIndex = 1;
                 ;
                 ++radialIndex)
            {
                const double radius =
                    static_cast<double>(
                        radialIndex
                    ) *
                    D_UAV *
                    config::RADIAL_SPACING_MULTIPLIER;

                if (radius >
                    Rsearch +
                    config::EPS_GEOMETRY)
                {
                    break;
                }

                const Vec3 P2 =
                    anchor.position +
                    ray.direction *
                    radius;

                Candidate candidate =
                    evaluateCandidate(
                        P0,
                        P1,
                        P2,
                        P3,
                        cornerIndex,
                        map,
                        vehicle,
                        state,
                        L_up
                    );

                candidate.anchorIndex =
                    anchor.index;

                candidate.rayIndex =
                    ray.index;

                candidate.radialIndex =
                    radialIndex;

                ++statistics.stage1CandidateEvaluations;

                if (!candidate.feasible())
                {
                    continue;
                }

                if (!foundBest ||
                    candidate.J <
                    best.J)
                {
                    if (foundBest)
                    {
                        secondBest =
                            best;

                        foundSecond =
                            true;
                    }

                    best =
                        candidate;

                    foundBest =
                        true;
                }
                else if (!foundSecond ||
                         candidate.J <
                         secondBest.J)
                {
                    secondBest =
                        candidate;

                    foundSecond =
                        true;
                }
            }
        }

        if (horizon.expired())
        {
            break;
        }
    }

    return foundBest;
}


// ============================================================================
// STAGE 3 REFINEMENT
// ============================================================================

Candidate stage3Refine(
    const Vec3& P0,
    const Vec3& P1,
    const Vec3& P3,
    std::size_t cornerIndex,
    const Map3D& map,
    const config::VehicleConfig& vehicle,
    const UAVState& state,
    double L_up,
    Candidate best,
    Candidate secondBest,
    SearchStatistics& statistics,
    const HorizonGuard& horizon)
{
    if (!best.feasible() ||
        !secondBest.feasible())
    {
        return best;
    }

    if (best.anchorIndex !=
        secondBest.anchorIndex)
    {
        return best;
    }

    const Vec3 P3MinusP1 =
        P3 - P1;

    const double L13 =
        P3MinusP1.norm();

    if (L13 <=
        config::EPS_GEOMETRY)
    {
        return best;
    }

    const Vec3 e13 =
        P3MinusP1 / L13;

    const double Rsearch =
        L13 * 0.5;

    const NormalBasis basis =
        makeNormalBasis(
            e13
        );

    const double bestRadius =
        distance(
            best.segment.P1,
            best.segment.P2
        );

    const double secondRadius =
        distance(
            secondBest.segment.P1,
            secondBest.segment.P2
        );

    if (std::abs(
            bestRadius -
            secondRadius) >
        config::EPS_GEOMETRY)
    {
        return best;
    }

    double thetaBest =
        angleOfVector(
            best.segment.P2 -
            best.segment.P1,
            basis.n1,
            basis.n2
        );

    double thetaSecond =
        angleOfVector(
            secondBest.segment.P2 -
            secondBest.segment.P1,
            basis.n1,
            basis.n2
        );

    double thetaLeft =
        thetaBest;

    double thetaRight =
        thetaSecond;

    double delta =
        normalizeAngle(
            thetaRight -
            thetaLeft
        );

    if (delta > PI)
    {
        std::swap(
            thetaLeft,
            thetaRight
        );

        delta =
            normalizeAngle(
                thetaRight -
                thetaLeft
            );
    }

    if (2.0 *
        Rsearch *
        std::sin(
            delta * 0.5
        )
        <=
        map.resolution +
        config::EPS_GEOMETRY)
    {
        return best;
    }

    for (;;)
    {
        if (horizon.expired())
        {
            return best;
        }

        double interval =
            thetaRight -
            thetaLeft;

        if (interval < 0.0)
        {
            interval +=
                2.0 * PI;
        }

        if (2.0 *
            Rsearch *
            std::sin(
                interval * 0.5
            )
            <=
            map.resolution +
            config::EPS_GEOMETRY)
        {
            break;
        }

        const double theta2 =
            thetaLeft +
            interval *
            0.25;

        const double theta3 =
            thetaLeft +
            interval *
            0.50;

        const double theta4 =
            thetaLeft +
            interval *
            0.75;

        Candidate candidates[5];

        candidates[0] =
            best;

        candidates[4] =
            secondBest;

        const double newAngles[3] = {
            theta2,
            theta3,
            theta4
        };

        for (std::size_t i = 0;
             i < 3;
             ++i)
        {
            if (horizon.expired())
            {
                return best;
            }

            const double theta =
                newAngles[i];

            const Vec3 ray =
                normalized(
                    basis.n1 *
                        std::cos(theta) +
                    basis.n2 *
                        std::sin(theta),
                    config::EPS_GEOMETRY
                );

            const Vec3 P2 =
                best.segment.P1 +
                ray *
                Rsearch;

            candidates[i + 1] =
                evaluateCandidate(
                    P0,
                    P1,
                    P2,
                    P3,
                    cornerIndex,
                    map,
                    vehicle,
                    state,
                    L_up
                );

            candidates[i + 1].anchorIndex =
                best.anchorIndex;

            ++statistics.stage3CandidateEvaluations;
        }

        Candidate localBest =
            best;

        Candidate localSecond =
            secondBest;

        bool haveBest =
            localBest.feasible();

        bool haveSecond =
            localSecond.feasible();

        for (const Candidate& candidate :
             candidates)
        {
            if (!candidate.feasible())
            {
                continue;
            }

            if (!haveBest ||
                candidate.J <
                localBest.J)
            {
                if (haveBest)
                {
                    localSecond =
                        localBest;

                    haveSecond =
                        true;
                }

                localBest =
                    candidate;

                haveBest =
                    true;
            }
            else if (!haveSecond ||
                     candidate.J <
                     localSecond.J)
            {
                localSecond =
                    candidate;

                haveSecond =
                    true;
            }
        }

        if (!haveBest)
        {
            return best;
        }

        best =
            localBest;

        if (!haveSecond)
        {
            break;
        }

        secondBest =
            localSecond;

        thetaBest =
            angleOfVector(
                best.segment.P2 -
                best.segment.P1,
                basis.n1,
                basis.n2
            );

        thetaSecond =
            angleOfVector(
                secondBest.segment.P2 -
                secondBest.segment.P1,
                basis.n1,
                basis.n2
            );

        thetaLeft =
            thetaBest;

        thetaRight =
            thetaSecond;

        delta =
            normalizeAngle(
                thetaRight -
                thetaLeft
            );

        if (delta > PI)
        {
            std::swap(
                thetaLeft,
                thetaRight
            );
        }

        ++statistics.refinementIterations;
    }

    return best;
}

} // namespace


// ============================================================================
// PUBLIC PROGRESSIVE MODIFIER
// ============================================================================

ModifierResult progressiveModify(
    const SearchInput& input)
{
    ModifierResult result;

    SearchStatistics statistics;

    SearchTimer totalTimer;

    HorizonGuard horizon(
        input.runtime,
        totalTimer
    );

    // ------------------------------------------------------------------------
    // Validate input.
    // ------------------------------------------------------------------------

    if (!validSearchInput(
            input))
    {
        result.success =
            false;

        result.generationTimeMs =
            totalTimer.elapsedMilliseconds();

        return result;
    }

    const WaypointPath& upstreamPath =
        *input.upstreamPath;

    const Map3D& map =
        *input.map;

    Vec3 P0 =
        input.P0;

    BezierSegment previousSegment =
        input.previousSegment;

    Vec3 previousCorner =
        input.previousCorner;

    bool hasPreviousSegment =
        input.hasPreviousSegment;

    std::size_t firstCorner =
        input.startCornerIndex;

    if (firstCorner == 0)
    {
        firstCorner = 1;
    }

    if (firstCorner >=
        upstreamPath.size())
    {
        result.success =
            false;

        result.generationTimeMs =
            totalTimer.elapsedMilliseconds();

        return result;
    }

    // =========================================================================
    // CORNER-BY-CORNER PROGRESSIVE SEARCH
    // =========================================================================

    for (std::size_t cornerIndex = firstCorner;
         cornerIndex < upstreamPath.size();
         ++cornerIndex)
    {
        if (horizon.expired())
        {
            break;
        }

        // ---------------------------------------------------------------------
        // Exact upstream corner.
        //
        // P3 = upstreamPath[cornerIndex]
        // ---------------------------------------------------------------------

        const Vec3 corner =
            upstreamPath[cornerIndex];

        const Vec3 P3 =
            corner;

        // ---------------------------------------------------------------------
        // Travel-horizon check.
        // ---------------------------------------------------------------------

        const double distanceToCorner =
            distance(
                P0,
                P3
            );

        if (!horizon.cornerReachable(
                distanceToCorner))
        {
            break;
        }

        // ---------------------------------------------------------------------
        // Segment generation timer.
        // ---------------------------------------------------------------------

        SearchTimer segmentTimer;

        // ---------------------------------------------------------------------
        // Construct P1.
        // ---------------------------------------------------------------------

        const Vec3 P1 =
            makeSearchP1(
                P0,
                corner,
                input.state,
                input.vehicle,
                map,
                hasPreviousSegment,
                previousSegment
            );

        if (distance(
                P1,
                P3
            ) <=
            config::EPS_GEOMETRY)
        {
            break;
        }

        // ---------------------------------------------------------------------
        // Corner-to-corner upstream length.
        // ---------------------------------------------------------------------

        const double L_up =
            exactUpstreamLength(
                P0,
                P3
            );

        if (L_up <=
            config::EPS_GEOMETRY)
        {
            break;
        }

        // =====================================================================
        // STAGE 1
        // =====================================================================

        Candidate best;
        Candidate secondBest;

        const std::size_t stage1Before =
            statistics.stage1CandidateEvaluations;

        const bool foundStage1 =
            stage1SearchWithCorner(
                P0,
                P1,
                P3,
                cornerIndex,
                map,
                input.vehicle,
                input.state,
                L_up,
                best,
                secondBest,
                statistics,
                horizon
            );

        const std::size_t stage1Evaluations =
            statistics.stage1CandidateEvaluations -
            stage1Before;

        // ---------------------------------------------------------------------
        // Report actual Stage 1 completion.
        // ---------------------------------------------------------------------

        SearchProgress stage1Progress;

        stage1Progress.stage =
            SearchStage::Stage1Done;

        stage1Progress.cornerIndex =
            cornerIndex;

        stage1Progress.stage1CandidateEvaluations =
            stage1Evaluations;

        reportProgress(
            input,
            stage1Progress
        );

        if (!foundStage1)
        {
            break;
        }

        // =====================================================================
        // STAGE 3
        // =====================================================================

        Candidate finalCandidate =
            best;

        const bool refinementEligible =
            secondBest.feasible() &&
            secondBest.anchorIndex ==
                best.anchorIndex;

        if (refinementEligible)
        {
            SearchProgress stage3Start;

            stage3Start.stage =
                SearchStage::Stage3Started;

            stage3Start.cornerIndex =
                cornerIndex;

            stage3Start.stage1CandidateEvaluations =
                stage1Evaluations;

            reportProgress(
                input,
                stage3Start
            );

            const std::size_t stage3Before =
                statistics.stage3CandidateEvaluations;

            const std::size_t refinementBefore =
                statistics.refinementIterations;

            finalCandidate =
                stage3Refine(
                    P0,
                    P1,
                    P3,
                    cornerIndex,
                    map,
                    input.vehicle,
                    input.state,
                    L_up,
                    best,
                    secondBest,
                    statistics,
                    horizon
                );

            const std::size_t stage3Evaluations =
                statistics.stage3CandidateEvaluations -
                stage3Before;

            const std::size_t refinementIterations =
                statistics.refinementIterations -
                refinementBefore;

            SearchProgress stage3Progress;

            stage3Progress.stage =
                SearchStage::Stage3Done;

            stage3Progress.cornerIndex =
                cornerIndex;

            stage3Progress.stage1CandidateEvaluations =
                stage1Evaluations;

            stage3Progress.stage3CandidateEvaluations =
                stage3Evaluations;

            stage3Progress.refinementIterations =
                refinementIterations;

            if (finalCandidate.feasible())
            {
                stage3Progress.maxCurvature =
                    finalCandidate.kappaMax;

                stage3Progress.lateralAcceleration =
                    finalCandidate.lateralAcceleration;

                stage3Progress.Jd =
                    finalCandidate.Jd;

                stage3Progress.Jk =
                    finalCandidate.Jk;

                stage3Progress.Jl =
                    finalCandidate.Jl;

                stage3Progress.J =
                    finalCandidate.J;
            }

            reportProgress(
                input,
                stage3Progress
            );
        }
        else
        {
            SearchProgress stage3Skipped;

            stage3Skipped.stage =
                SearchStage::Stage3Skipped;

            stage3Skipped.cornerIndex =
                cornerIndex;

            stage3Skipped.stage1CandidateEvaluations =
                stage1Evaluations;

            reportProgress(
                input,
                stage3Skipped
            );
        }

        // =====================================================================
        // ACCEPT FINAL CANDIDATE
        // =====================================================================

        if (!finalCandidate.feasible())
        {
            break;
        }

        if (horizon.expired())
        {
            break;
        }

        // ---------------------------------------------------------------------
        // Individual segment generation time.
        // ---------------------------------------------------------------------

        finalCandidate.segment.generationTimeMs =
            segmentTimer.elapsedMilliseconds();

        // ---------------------------------------------------------------------
        // Commit segment.
        // ---------------------------------------------------------------------

        result.trajectory.segments.push_back(
            finalCandidate.segment
        );

        P0 =
            finalCandidate.segment.P3;

        previousSegment =
            finalCandidate.segment;

        previousCorner =
            corner;

        hasPreviousSegment =
            true;

        ++result.incorporatedCorners;
        ++result.generatedSegments;

        // ---------------------------------------------------------------------
        // Report accepted segment.
        // ---------------------------------------------------------------------

        SearchProgress segmentProgress;

        segmentProgress.stage =
            SearchStage::SegmentAccepted;

        segmentProgress.cornerIndex =
            cornerIndex;

        segmentProgress.stage1CandidateEvaluations =
            stage1Evaluations;

        segmentProgress.maxCurvature =
            finalCandidate.kappaMax;

        segmentProgress.lateralAcceleration =
            finalCandidate.lateralAcceleration;

        segmentProgress.Jd =
            finalCandidate.Jd;

        segmentProgress.Jk =
            finalCandidate.Jk;

        segmentProgress.Jl =
            finalCandidate.Jl;

        segmentProgress.J =
            finalCandidate.J;

        segmentProgress.segmentGenerationTimeMs =
            finalCandidate.segment.generationTimeMs;

        reportProgress(
            input,
            segmentProgress
        );
    }

    // =========================================================================
    // FINAL STATISTICS
    // =========================================================================

    result.stage1CandidateEvaluations =
        statistics.stage1CandidateEvaluations;

    result.stage3CandidateEvaluations =
        statistics.stage3CandidateEvaluations;

    result.refinementIterations =
        statistics.refinementIterations;

    result.generationTimeMs =
        totalTimer.elapsedMilliseconds();

    // ------------------------------------------------------------------------
    // Success means the complete upstream path was incorporated.
    //
    // This prevents a partially generated trajectory from being reported as
    // a successful full-path search.
    // ------------------------------------------------------------------------

    const std::size_t requiredSegments =
        upstreamPath.size() - firstCorner;

    result.success =
        result.generatedSegments ==
        requiredSegments;

    // ------------------------------------------------------------------------
    // Final search progress event.
    // ------------------------------------------------------------------------

    SearchProgress finished;

    finished.stage =
        SearchStage::SearchFinished;

    finished.cornerIndex =
        result.generatedSegments;

    reportProgress(
        input,
        finished
    );

    return result;
}

} // namespace uav::search