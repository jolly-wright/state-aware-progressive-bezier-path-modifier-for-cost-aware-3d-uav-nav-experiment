#include <Arduino.h>
#include <ArduinoJson.h>
#include "types.hpp"
#include "config.hpp"
#include "search.hpp"
#include "bezier.hpp"
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <vector>
namespace
{
using namespace uav;
// ============================================================================
// SERIAL PROTOCOL
// ============================================================================
constexpr unsigned long SERIAL_BAUD = 115200;
constexpr const char* CMD_RUN =
    "run";
constexpr const char* MSG_READY =
    "READY";
constexpr const char* MSG_WAITING =
    "WAITING_FOR_RUN";
constexpr const char* MSG_READY_FOR_SCENARIO =
    "READY_FOR_SCENARIO";
constexpr const char* MSG_RESULT_BEGIN =
    "RESULT_BEGIN";
constexpr const char* MSG_RESULT_END =
    "RESULT_END";
// ============================================================================
// SCENARIO DATA
// ============================================================================
struct ScenarioData
{
    std::size_t scenarioId = 0;
    WaypointPath path;
    Map3D map;
    UAVState state;
    config::VehicleConfig vehicle;
};
// ============================================================================
// VECTOR JSON HELPER
// ============================================================================
bool readVec3(
    JsonVariantConst value,
    Vec3& result
)
{
    JsonArrayConst array =
        value.as<JsonArrayConst>();
    if (array.isNull() ||
        array.size() != 3)
    {
        return false;
    }
    result.x =
        array[0].as<double>();
    result.y =
        array[1].as<double>();
    result.z =
        array[2].as<double>();
    return result.isFinite();
}
// ============================================================================
// LOAD SCENARIO JSON
// ============================================================================
bool loadScenarioFromJson(
    const std::vector<char>& jsonBuffer,
    ScenarioData& scenario
)
{
    JsonDocument document;
    const DeserializationError error =
        deserializeJson(
            document,
            jsonBuffer.data(),
            jsonBuffer.size()
        );
    if (error)
    {
        Serial.print(
            "ERROR: JSON_PARSE_FAILED: "
        );
        Serial.println(
            error.c_str()
        );
        return false;
    }
    // ========================================================================
    // SCENARIO ID
    // ========================================================================
    if (!document["scenario_id"].is<std::size_t>())
    {
        Serial.println(
            "ERROR: MISSING_SCENARIO_ID"
        );
        return false;
    }
    scenario.scenarioId =
        document["scenario_id"]
            .as<std::size_t>();
    // ========================================================================
    // UAV
    // ========================================================================
    JsonObjectConst uavJson =
        document["uav"]
            .as<JsonObjectConst>();
    if (uavJson.isNull())
    {
        Serial.println(
            "ERROR: MISSING_UAV"
        );
        return false;
    }
    JsonArrayConst dimensions =
        uavJson["dimensions"]
            .as<JsonArrayConst>();
    if (dimensions.isNull() ||
        dimensions.size() != 3)
    {
        Serial.println(
            "ERROR: INVALID_UAV_DIMENSIONS"
        );
        return false;
    }
    scenario.vehicle.dimensions.x =
        dimensions[0].as<double>();
    scenario.vehicle.dimensions.y =
        dimensions[1].as<double>();
    scenario.vehicle.dimensions.z =
        dimensions[2].as<double>();
    if (!std::isfinite(
            scenario.vehicle.dimensions.x) ||
        !std::isfinite(
            scenario.vehicle.dimensions.y) ||
        !std::isfinite(
            scenario.vehicle.dimensions.z))
    {
        Serial.println(
            "ERROR: NONFINITE_UAV_DIMENSIONS"
        );
        return false;
    }
    if (scenario.vehicle.dimensions.x <= 0.0 ||
        scenario.vehicle.dimensions.y <= 0.0 ||
        scenario.vehicle.dimensions.z <= 0.0)
    {
        Serial.println(
            "ERROR: INVALID_UAV_DIMENSIONS"
        );
        return false;
    }
    if (!readVec3(
            uavJson["position"],
            scenario.state.position))
    {
        Serial.println(
            "ERROR: INVALID_UAV_POSITION"
        );
        return false;
    }
    if (!readVec3(
            uavJson["velocity"],
            scenario.state.velocity))
    {
        Serial.println(
            "ERROR: INVALID_UAV_VELOCITY"
        );
        return false;
    }
    scenario.vehicle.V_MAX =
        uavJson["max_velocity"]
            .as<double>();
    scenario.vehicle.A_MAX =
        uavJson["max_lateral_acceleration"]
            .as<double>();
    if (!std::isfinite(
            scenario.vehicle.V_MAX) ||
        !std::isfinite(
            scenario.vehicle.A_MAX))
    {
        Serial.println(
            "ERROR: NONFINITE_VEHICLE_LIMIT"
        );
        return false;
    }
    if (scenario.vehicle.V_MAX <= 0.0 ||
        scenario.vehicle.A_MAX <= 0.0)
    {
        Serial.println(
            "ERROR: INVALID_VEHICLE_LIMIT"
        );
        return false;
    }
    // ========================================================================
    // MAP
    // ========================================================================
    JsonObjectConst mapJson =
        document["map"]
            .as<JsonObjectConst>();
    if (mapJson.isNull())
    {
        Serial.println(
            "ERROR: MISSING_MAP"
        );
        return false;
    }
    const double resolutionCm =
        mapJson["resolution_cm"]
            .as<double>();
    if (!std::isfinite(resolutionCm) ||
        resolutionCm <= 0.0)
    {
        Serial.println(
            "ERROR: INVALID_MAP_RESOLUTION"
        );
        return false;
    }
    // Scenario resolution is given in centimetres.
    // Map3D stores resolution in metres.
    scenario.map.resolution =
        resolutionCm * 0.01;
    // ========================================================================
    // OPTIONAL MAP BOUNDS
    // ========================================================================
    JsonObjectConst boundsJson =
        mapJson["bounds"]
            .as<JsonObjectConst>();
    if (!boundsJson.isNull())
    {
        if (!readVec3(
                boundsJson["min"],
                scenario.map.bounds.min))
        {
            Serial.println(
                "ERROR: INVALID_MAP_BOUNDS_MIN"
            );
            return false;
        }
        if (!readVec3(
                boundsJson["max"],
                scenario.map.bounds.max))
        {
            Serial.println(
                "ERROR: INVALID_MAP_BOUNDS_MAX"
            );
            return false;
        }
        if (scenario.map.bounds.min.x >
                scenario.map.bounds.max.x ||
            scenario.map.bounds.min.y >
                scenario.map.bounds.max.y ||
            scenario.map.bounds.min.z >
                scenario.map.bounds.max.z)
        {
            Serial.println(
                "ERROR: INVALID_MAP_BOUNDS"
            );
            return false;
        }
    }
    // ========================================================================
    // OBSTACLES
    // ========================================================================
    JsonArrayConst obstacles =
        mapJson["obstacles"]
            .as<JsonArrayConst>();
    if (obstacles.isNull())
    {
        Serial.println(
            "ERROR: MISSING_OBSTACLES"
        );
        return false;
    }
    scenario.map.obstacles.clear();
    std::size_t obstacleId = 0;
    for (JsonObjectConst obstacleJson :
         obstacles)
    {
        Vec3 center;
        Vec3 dimensions;
        if (!readVec3(
                obstacleJson["center"],
                center))
        {
            Serial.println(
                "ERROR: INVALID_OBSTACLE_CENTER"
            );
            return false;
        }
        if (!readVec3(
                obstacleJson["dimensions"],
                dimensions))
        {
            Serial.println(
                "ERROR: INVALID_OBSTACLE_DIMENSIONS"
            );
            return false;
        }
        if (dimensions.x <= 0.0 ||
            dimensions.y <= 0.0 ||
            dimensions.z <= 0.0)
        {
            Serial.println(
                "ERROR: INVALID_OBSTACLE_DIMENSIONS"
            );
            return false;
        }
        const Vec3 half =
            dimensions * 0.5;
        Obstacle obstacle;
        obstacle.id =
            obstacleId++;
        obstacle.bounds.min =
            center - half;
        obstacle.bounds.max =
            center + half;
        scenario.map.obstacles.push_back(
            obstacle
        );
    }
    // ========================================================================
    // PATH
    // ========================================================================
    JsonObjectConst pathJson =
        document["path"]
            .as<JsonObjectConst>();
    if (pathJson.isNull())
    {
        Serial.println(
            "ERROR: MISSING_PATH"
        );
        return false;
    }
    JsonArrayConst waypoints =
        pathJson["waypoints"]
            .as<JsonArrayConst>();
    if (waypoints.isNull() ||
        waypoints.size() < 2)
    {
        Serial.println(
            "ERROR: INVALID_WAYPOINT_PATH"
        );
        return false;
    }
    scenario.path.points.clear();
    for (JsonVariantConst waypoint :
         waypoints)
    {
        Vec3 point;
        if (!readVec3(
                waypoint,
                point))
        {
            Serial.println(
                "ERROR: INVALID_WAYPOINT"
            );
            return false;
        }
        scenario.path.points.push_back(
            point
        );
    }
    // ========================================================================
    // INPUT CONSISTENCY
    // ========================================================================
    if (distance(
            scenario.state.position,
            scenario.path.points.front()
        ) > config::EPS_GEOMETRY)
    {
        Serial.println(
            "ERROR: UAV_POSITION_DOES_NOT_MATCH_PATH_START"
        );
        return false;
    }
    return true;
}
// ============================================================================
// READ ONE SERIAL LINE
// ============================================================================
bool readSerialLine(
    char* buffer,
    std::size_t capacity
)
{
    if (capacity == 0)
    {
        return false;
    }
    std::size_t length = 0;
    while (true)
    {
        while (!Serial.available())
        {
            delay(1);
        }
        const char c =
            static_cast<char>(
                Serial.read()
            );
        if (c == '\r')
        {
            continue;
        }
        if (c == '\n')
        {
            buffer[length] =
                '\0';
            return true;
        }
        if (length + 1 >= capacity)
        {
            buffer[0] =
                '\0';
            return false;
        }
        buffer[length++] =
            c;
    }
}
// ============================================================================
// RECEIVE EXACT JSON PAYLOAD
// ============================================================================
bool receiveScenarioJson(
    std::vector<char>& jsonBuffer
)
{
    char lengthBuffer[32];
    if (!readSerialLine(
            lengthBuffer,
            sizeof(lengthBuffer)))
    {
        return false;
    }
    char* endPointer = nullptr;
    const unsigned long length =
        std::strtoul(
            lengthBuffer,
            &endPointer,
            10
        );
    if (endPointer == lengthBuffer ||
        *endPointer != '\0' ||
        length == 0)
    {
        return false;
    }
    jsonBuffer.clear();
    jsonBuffer.resize(
        static_cast<std::size_t>(length)
    );
    std::size_t received = 0;
    while (received <
           static_cast<std::size_t>(length))
    {
        if (!Serial.available())
        {
            delay(1);
            continue;
        }
        const int value =
            Serial.read();
        if (value < 0)
        {
            continue;
        }
        jsonBuffer[received++] =
            static_cast<char>(value);
    }
    return true;
}
// ============================================================================
// PRINT SEARCH START
// ============================================================================
void printSearchStart(
    const ScenarioData& scenario
)
{
    Serial.println();
    Serial.println(
        "=========================================="
    );
    Serial.print(
        "SCENARIO "
    );
    Serial.println(
        static_cast<unsigned long>(
            scenario.scenarioId
        )
    );
    Serial.println(
        "=========================================="
    );
    Serial.println(
        "SEARCH START"
    );
    Serial.print(
        "Start position: "
    );
    Serial.print(
        scenario.state.position.x,
        4
    );
    Serial.print(", ");
    Serial.print(
        scenario.state.position.y,
        4
    );
    Serial.print(", ");
    Serial.println(
        scenario.state.position.z,
        4
    );
    Serial.print(
        "Goal: "
    );
    const Vec3& goal =
        scenario.path[
            scenario.path.size() - 1
        ];
    Serial.print(
        goal.x,
        4
    );
    Serial.print(", ");
    Serial.print(
        goal.y,
        4
    );
    Serial.print(", ");
    Serial.println(
        goal.z,
        4
    );
    Serial.print(
        "Upstream corners: "
    );
    Serial.println(
        static_cast<unsigned long>(
            scenario.path.size() - 1
        )
    );
}
// ============================================================================
// PRINT SEARCH SUMMARY
// ============================================================================
void printSearchSummary(
    const ScenarioData& scenario,
    const ModifierResult& result
)
{
    Serial.println();
    Serial.println(
        "------------------------------------------"
    );
    Serial.println(
        "SEARCH COMPLETE"
    );
    // ------------------------------------------------------------------------
    // Stage 1
    // ------------------------------------------------------------------------
    Serial.print(
        "STAGE 1 DONE"
    );
    Serial.print(
        " | candidate evaluations="
    );
    Serial.println(
        static_cast<unsigned long>(
            result.stage1CandidateEvaluations
        )
    );
    // ------------------------------------------------------------------------
    // Stage 3
    // ------------------------------------------------------------------------
    Serial.print(
        "STAGE 3 DONE"
    );
    Serial.print(
        " | candidate evaluations="
    );
    Serial.print(
        static_cast<unsigned long>(
            result.stage3CandidateEvaluations
        )
    );
    Serial.print(
        " | refinement iterations="
    );
    Serial.println(
        static_cast<unsigned long>(
            result.refinementIterations
        )
    );
    // ------------------------------------------------------------------------
    // Search result
    // ------------------------------------------------------------------------
    Serial.print(
        "Generated segments: "
    );
    Serial.println(
        static_cast<unsigned long>(
            result.generatedSegments
        )
    );
    Serial.print(
        "Incorporated corners: "
    );
    Serial.println(
        static_cast<unsigned long>(
            result.incorporatedCorners
        )
    );
    Serial.print(
        "WHOLE SEARCH TIME: "
    );
    Serial.print(
        result.generationTimeMs,
        3
    );
    Serial.println(
        " ms"
    );
    // ------------------------------------------------------------------------
    // Goal
    // ------------------------------------------------------------------------
    bool goalReached = false;
    if (!result.trajectory.segments.empty())
    {
        const Vec3 endpoint =
            result.trajectory.endpoint();
        const Vec3& goal =
            scenario.path[
                scenario.path.size() - 1
            ];
        goalReached =
            distance(
                endpoint,
                goal
            ) <= config::EPS_GEOMETRY;
    }
    Serial.print(
        "GOAL: "
    );
    if (goalReached)
    {
        Serial.println(
            "REACHED"
        );
    }
    else
    {
        Serial.println(
            "NOT REACHED"
        );
    }
    Serial.println(
        "------------------------------------------"
    );
}
// ============================================================================
// PRINT SEGMENT INFORMATION
// ============================================================================
double printSegmentInformation(
    const BezierSegment& segment,
    std::size_t segmentNumber
)
{
    const double kappaMax =
        bezier::maximumCurvature(
            segment
        );
    const double speed =
        segmentNumber == 0
            ? 0.0
            : 0.0;
    (void)speed;
    Serial.println();
    Serial.print(
        "SEGMENT "
    );
    Serial.println(
        static_cast<unsigned long>(
            segmentNumber + 1
        )
    );
    Serial.print(
        "  corner index: "
    );
    Serial.println(
        static_cast<unsigned long>(
            segment.cornerIndex
        )
    );
    Serial.print(
        "  generation time: "
    );
    Serial.print(
        segment.generationTimeMs,
        3
    );
    Serial.println(
        " ms"
    );
    Serial.print(
        "  maximum curvature: "
    );
    Serial.println(
        kappaMax,
        9
    );
    Serial.print(
        "  P0: "
    );
    Serial.print(
        segment.P0.x,
        4
    );
    Serial.print(", ");
    Serial.print(
        segment.P0.y,
        4
    );
    Serial.print(", ");
    Serial.println(
        segment.P0.z,
        4
    );
    Serial.print(
        "  P1: "
    );
    Serial.print(
        segment.P1.x,
        4
    );
    Serial.print(", ");
    Serial.print(
        segment.P1.y,
        4
    );
    Serial.print(", ");
    Serial.println(
        segment.P1.z,
        4
    );
    Serial.print(
        "  P2: "
    );
    Serial.print(
        segment.P2.x,
        4
    );
    Serial.print(", ");
    Serial.print(
        segment.P2.y,
        4
    );
    Serial.print(", ");
    Serial.println(
        segment.P2.z,
        4
    );
    Serial.print(
        "  P3: "
    );
    Serial.print(
        segment.P3.x,
        4
    );
    Serial.print(", ");
    Serial.print(
        segment.P3.y,
        4
    );
    Serial.print(", ");
    Serial.println(
        segment.P3.z,
        4
    );
    return segment.generationTimeMs;
}
// ============================================================================
// PRINT ALL SEGMENTS
// ============================================================================
double printAllSegments(
    const BezierTrajectory& trajectory
)
{
    Serial.println();
    Serial.println(
        "=========================================="
    );
    Serial.println(
        "SEGMENT RESULTS"
    );
    Serial.println(
        "=========================================="
    );
    double summedGenerationTimeMs =
        0.0;
    for (std::size_t i = 0;
         i < trajectory.segments.size();
         ++i)
    {
        summedGenerationTimeMs +=
            printSegmentInformation(
                trajectory.segments[i],
                i
            );
    }
    Serial.println();
    Serial.print(
        "SUM OF SEGMENT GENERATION TIMES: "
    );
    Serial.print(
        summedGenerationTimeMs,
        3
    );
    Serial.println(
        " ms"
    );
    return summedGenerationTimeMs;
}
// ============================================================================
// WRITE VEC3 TO JSON
// ============================================================================
void writeVec3(
    JsonObject object,
    const char* key,
    const Vec3& value
)
{
    JsonArray array =
        object[key].to<JsonArray>();
    array.add(value.x);
    array.add(value.y);
    array.add(value.z);
}
// ============================================================================
// WRITE DIMENSIONS TO JSON
// ============================================================================
void writeDimensions(
    JsonObject object,
    const char* key,
    const UAVDimensions& dimensions
)
{
    JsonArray array =
        object[key].to<JsonArray>();
    array.add(dimensions.x);
    array.add(dimensions.y);
    array.add(dimensions.z);
}
// ============================================================================
// SEND RESULT JSON
// ============================================================================
void sendResult(
    const ScenarioData& scenario,
    const ModifierResult& result,
    double summedSegmentGenerationTimeMs
)
{
    JsonDocument document;
    JsonObject root =
        document.to<JsonObject>();
    root["scenario_id"] =
        scenario.scenarioId;
    root["success"] =
        result.success;
    // ========================================================================
    // TIMING
    // ========================================================================
    JsonObject timing =
        root["timing"].to<JsonObject>();
    timing["generation_time_ms"] =
        result.generationTimeMs;
    timing["sum_segment_generation_time_ms"] =
        summedSegmentGenerationTimeMs;
    // ========================================================================
    // SEARCH STATISTICS
    // ========================================================================
    JsonObject statistics =
        root["search_statistics"]
            .to<JsonObject>();
    statistics["generated_segments"] =
        result.generatedSegments;
    statistics["incorporated_corners"] =
        result.incorporatedCorners;
    statistics["stage1_candidate_evaluations"] =
        result.stage1CandidateEvaluations;
    statistics["stage3_candidate_evaluations"] =
        result.stage3CandidateEvaluations;
    statistics["refinement_iterations"] =
        result.refinementIterations;
    // ========================================================================
    // UAV
    // ========================================================================
    JsonObject uav =
        root["uav"].to<JsonObject>();
    writeVec3(
        uav,
        "position",
        scenario.state.position
    );
    writeVec3(
        uav,
        "velocity",
        scenario.state.velocity
    );
    writeDimensions(
        uav,
        "dimensions",
        scenario.vehicle.dimensions
    );
    uav["max_velocity_mps"] =
        scenario.vehicle.V_MAX;
    uav["max_lateral_acceleration_mps2"] =
        scenario.vehicle.A_MAX;
    // ========================================================================
    // GOAL
    // ========================================================================
    const Vec3& goal =
        scenario.path[
            scenario.path.size() - 1
        ];
    JsonObject goalObject =
        root["goal"].to<JsonObject>();
    writeVec3(
        goalObject,
        "position",
        goal
    );
    bool goalReached = false;
    if (!result.trajectory.segments.empty())
    {
        goalReached =
            distance(
                result.trajectory.endpoint(),
                goal
            ) <= config::EPS_GEOMETRY;
    }
    root["goal_reached"] =
        goalReached;
    // ========================================================================
    // MAP
    // ========================================================================
    JsonObject mapObject =
        root["map"].to<JsonObject>();
    mapObject["resolution_m"] =
        scenario.map.resolution;
    JsonArray obstacles =
        mapObject["obstacles"]
            .to<JsonArray>();
    for (const Obstacle& obstacle :
         scenario.map.obstacles)
    {
        JsonObject obstacleObject =
            obstacles.add<JsonObject>();
        obstacleObject["id"] =
            obstacle.id;
        writeVec3(
            obstacleObject,
            "min",
            obstacle.bounds.min
        );
        writeVec3(
            obstacleObject,
            "max",
            obstacle.bounds.max
        );
    }
    // ========================================================================
    // UPSTREAM PATH
    // ========================================================================
    JsonArray upstreamPath =
        root["upstream_path"]
            .to<JsonArray>();
    for (const Vec3& point :
         scenario.path.points)
    {
        JsonArray waypoint =
            upstreamPath.add<JsonArray>();
        waypoint.add(point.x);
        waypoint.add(point.y);
        waypoint.add(point.z);
    }
    // ========================================================================
    // FINAL BÉZIER TRAJECTORY
    // ========================================================================
    JsonArray segments =
        root["final_bezier_segments"]
            .to<JsonArray>();
    for (const BezierSegment& segment :
         result.trajectory.segments)
    {
        JsonObject segmentObject =
            segments.add<JsonObject>();
        segmentObject["corner_index"] =
            segment.cornerIndex;
        segmentObject["generation_time_ms"] =
            segment.generationTimeMs;
        segmentObject["maximum_curvature"] =
            bezier::maximumCurvature(
                segment
            );
            // ------------------------------------------------------------------------
            // Initial-direction deviation from the goal-corner direction.
            //
            // Initial segment direction: P0 -> P1
            // Goal-corner direction:     P0 -> P3
            // ------------------------------------------------------------------------

            const Vec3 initialDirection =
                segment.P1 -
                segment.P0;

            const Vec3 goalDirection =
                segment.P3 -
                segment.P0;

            const double initialDirectionLength =
                initialDirection.norm();

            const double goalDirectionLength =
                goalDirection.norm();

            double goalDirectionDeviation =
                0.0;

            if (initialDirectionLength >
                    config::EPS_GEOMETRY &&
                goalDirectionLength >
                    config::EPS_GEOMETRY)
            {
                double cosine =
                    dot(
                        initialDirection,
                        goalDirection
                    ) /
                    (
                        initialDirectionLength *
                        goalDirectionLength
                    );

                cosine =
                    std::clamp(
                        cosine,
                        -1.0,
                        1.0
                    );

                goalDirectionDeviation =
                    std::acos(
                        cosine
                    ) *
                    180.0 /
                    M_PI;
            }

            segmentObject["goal_direction_deviation_deg"] =
                goalDirectionDeviation;
        writeVec3(
            segmentObject,
            "P0",
            segment.P0
        );
        writeVec3(
            segmentObject,
            "P1",
            segment.P1
        );
        writeVec3(
            segmentObject,
            "P2",
            segment.P2
        );
        writeVec3(
            segmentObject,
            "P3",
            segment.P3
        );
    }
    // ========================================================================
    // SERIAL PROTOCOL
    // ========================================================================
    Serial.println(
        MSG_RESULT_BEGIN
    );
    serializeJson(
        document,
        Serial
    );
    Serial.println();
    Serial.println(
        MSG_RESULT_END
    );
}
// ============================================================================
// RUN ONE SCENARIO
// ============================================================================
void runScenario(
    const ScenarioData& scenario
)
{
    printSearchStart(
        scenario
    );
    // ========================================================================
    // BUILD SEARCH INPUT
    // ========================================================================
    search::SearchInput input;
    input.upstreamPath =
        &scenario.path;
    input.map =
        &scenario.map;
    input.state =
        scenario.state;
    input.vehicle =
        scenario.vehicle;
    input.runtime.remainingExecutionTime =
        INF;
    input.runtime.availableTravelDistance =
        INF;
    input.startCornerIndex =
        1;
    input.P0 =
        scenario.state.position;
    input.hasPreviousSegment =
        false;
    // These are valid fields in the current SearchInput.
    input.previousSegment =
        BezierSegment{};
    input.previousCorner =
        scenario.path.points[0];
    // ========================================================================
    // COMPLETE PROGRESSIVE SEARCH
    // ========================================================================
    //
    // progressiveModify() itself processes the corners from
    // startCornerIndex through the end of the upstream path.
    //
    // Therefore this single call performs the complete search through
    // the final goal waypoint.
    const ModifierResult result =
        search::progressiveModify(
            input
        );
    // ========================================================================
    // SEARCH SUMMARY
    // ========================================================================
    printSearchSummary(
        scenario,
        result
    );
    // ========================================================================
    // PER-SEGMENT INFORMATION
    // ========================================================================
    double summedSegmentGenerationTimeMs =
        0.0;
    if (!result.trajectory.segments.empty())
    {
        summedSegmentGenerationTimeMs =
            printAllSegments(
                result.trajectory
            );
    }
    else
    {
        Serial.println();
        Serial.println(
            "NO BÉZIER SEGMENTS GENERATED"
        );
    }
    // ========================================================================
    // FINAL GOAL CHECK
    // ========================================================================
    bool goalReached = false;
    if (!result.trajectory.segments.empty())
    {
        const Vec3 endpoint =
            result.trajectory.endpoint();
        const Vec3& goal =
            scenario.path[
                scenario.path.size() - 1
            ];
        goalReached =
            distance(
                endpoint,
                goal
            ) <= config::EPS_GEOMETRY;
    }
    Serial.println();
    Serial.println(
        "=========================================="
    );
    if (result.success &&
        goalReached)
    {
        Serial.println(
            "WHOLE SEARCH: SUCCESS"
        );
        Serial.println(
            "FINAL GOAL REACHED"
        );
    }
    else
    {
        Serial.println(
            "WHOLE SEARCH: FAILED"
        );
        Serial.println(
            "FINAL GOAL NOT REACHED"
        );
    }
    Serial.println(
        "=========================================="
    );
    // ========================================================================
    // RESULT JSON
    // ========================================================================
    sendResult(
        scenario,
        result,
        summedSegmentGenerationTimeMs
    );
}
// ============================================================================
// WAIT FOR RUN COMMAND
// ============================================================================
bool waitForRunCommand()
{
    char command[32];
    while (true)
    {
        if (!readSerialLine(
                command,
                sizeof(command)))
        {
            continue;
        }
        if (std::strcmp(
                command,
                CMD_RUN
            ) == 0)
        {
            return true;
        }
        Serial.println(
            "IGNORED_COMMAND"
        );
        Serial.println(
            MSG_WAITING
        );
    }
}
} // namespace
// ============================================================================
// SETUP
// ============================================================================
void setup()
{
    Serial.begin(
        SERIAL_BAUD
    );
    delay(1000);
    Serial.println(
        MSG_READY
    );
    Serial.println(
        MSG_WAITING
    );
}
// ============================================================================
// LOOP
// ============================================================================
void loop()
{
    // ========================================================================
    // WAIT FOR run FROM run.py
    // ========================================================================
    waitForRunCommand();
    Serial.println(
        MSG_READY_FOR_SCENARIO
    );
    // ========================================================================
    // RECEIVE SCENARIO JSON
    // ========================================================================
    std::vector<char> jsonBuffer;
    if (!receiveScenarioJson(
            jsonBuffer))
    {
        Serial.println(
            "ERROR: INVALID_SCENARIO_PACKET"
        );
        Serial.println(
            MSG_WAITING
        );
        return;
    }
    // ========================================================================
    // PARSE SCENARIO
    // ========================================================================
    ScenarioData scenario;
    if (!loadScenarioFromJson(
            jsonBuffer,
            scenario))
    {
        Serial.println(
            "ERROR: INVALID_SCENARIO_JSON"
        );
        Serial.println(
            MSG_WAITING
        );
        return;
    }
    // ========================================================================
    // RUN COMPLETE SEARCH
    // ========================================================================
    runScenario(
        scenario
    );
    // ========================================================================
    // READY FOR NEXT SCENARIO
    // ========================================================================
    Serial.println(
        MSG_WAITING
    );
}