import json
import os
import glob
import sys
import math

import plotly.graph_objects as go


# ============================================================================
# CONFIGURATION
# ============================================================================

SCENARIO_DIR = "scenarios"
RESULT_DIR = "teensy_results"
OUTPUT_DIR = "output_visualization"

CURVE_SAMPLES_PER_SEGMENT = 100

# UAV and goal visualization size multiplier.
BOX_SIZE_MULTIPLIER = 2.0

# Arrow visualization.
VELOCITY_ARROW_LENGTH = 1.0
VELOCITY_ARROW_WIDTH = 0.08
VELOCITY_ARROW_HEAD_LENGTH = 0.25
VELOCITY_ARROW_HEAD_WIDTH = 0.18


# ============================================================================
# FILE DISCOVERY
# ============================================================================

def list_result_files():
    if not os.path.isdir(RESULT_DIR):
        return []

    files = glob.glob(
        os.path.join(
            RESULT_DIR,
            "scenario_*.json"
        )
    )

    files.sort()

    return files


def find_scenario_file(scenario_id):
    filename = os.path.join(
        SCENARIO_DIR,
        f"scenario_{scenario_id:03d}.json"
    )

    if os.path.isfile(filename):
        return filename

    return None


# ============================================================================
# JSON
# ============================================================================

def load_json(filename):
    with open(
        filename,
        "r",
        encoding="utf-8"
    ) as file:
        return json.load(file)


# ============================================================================
# VECTOR
# ============================================================================

def vec3(value):
    return (
        float(value[0]),
        float(value[1]),
        float(value[2])
    )


def vector_norm(vector):
    return math.sqrt(
        vector[0] * vector[0]
        + vector[1] * vector[1]
        + vector[2] * vector[2]
    )


def normalize(vector):
    magnitude = vector_norm(vector)

    if magnitude == 0.0:
        return (
            0.0,
            0.0,
            0.0
        )

    return (
        vector[0] / magnitude,
        vector[1] / magnitude,
        vector[2] / magnitude
    )


# ============================================================================
# CUBIC BEZIER
# ============================================================================

def evaluate_bezier(
    p0,
    p1,
    p2,
    p3,
    u
):
    one_minus_u = 1.0 - u

    b0 = one_minus_u ** 3
    b1 = 3.0 * one_minus_u ** 2 * u
    b2 = 3.0 * one_minus_u * u ** 2
    b3 = u ** 3

    return (
        b0 * p0[0]
        + b1 * p1[0]
        + b2 * p2[0]
        + b3 * p3[0],

        b0 * p0[1]
        + b1 * p1[1]
        + b2 * p2[1]
        + b3 * p3[1],

        b0 * p0[2]
        + b1 * p1[2]
        + b2 * p2[2]
        + b3 * p3[2]
    )


def generate_segment_curve(segment):
    p0 = vec3(segment["P0"])
    p1 = vec3(segment["P1"])
    p2 = vec3(segment["P2"])
    p3 = vec3(segment["P3"])

    points = []

    for i in range(
        CURVE_SAMPLES_PER_SEGMENT + 1
    ):
        u = (
            i
            / CURVE_SAMPLES_PER_SEGMENT
        )

        point = evaluate_bezier(
            p0,
            p1,
            p2,
            p3,
            u
        )

        points.append(point)

    return points


def generate_complete_bezier_path(result):
    segments = result[
        "final_bezier_segments"
    ]

    path = []

    for segment_index, segment in enumerate(
        segments
    ):
        segment_points = generate_segment_curve(
            segment
        )

        if segment_index > 0:
            # P0 is identical to the previous segment's P3.
            # Do not duplicate that point.
            segment_points = segment_points[1:]

        path.extend(
            segment_points
        )

    return path


# ============================================================================
# CUBOID
# ============================================================================

def cuboid_mesh(
    center,
    dimensions
):
    cx, cy, cz = center
    dx, dy, dz = dimensions

    hx = dx * 0.5
    hy = dy * 0.5
    hz = dz * 0.5

    x = [
        cx - hx,
        cx + hx,
        cx + hx,
        cx - hx,
        cx - hx,
        cx + hx,
        cx + hx,
        cx - hx
    ]

    y = [
        cy - hy,
        cy - hy,
        cy + hy,
        cy + hy,
        cy - hy,
        cy - hy,
        cy + hy,
        cy + hy
    ]

    z = [
        cz - hz,
        cz - hz,
        cz - hz,
        cz - hz,
        cz + hz,
        cz + hz,
        cz + hz,
        cz + hz
    ]

    i = [
        0, 0,
        4, 4,
        0, 0,
        1, 1,
        2, 2,
        3, 3
    ]

    j = [
        1, 2,
        5, 6,
        1, 5,
        2, 6,
        3, 7,
        0, 4
    ]

    k = [
        2, 3,
        6, 7,
        5, 4,
        6, 5,
        7, 6,
        4, 7
    ]

    return x, y, z, i, j, k


def add_box(
    figure,
    center,
    dimensions,
    name,
    color,
    opacity
):
    x, y, z, i, j, k = cuboid_mesh(
        center,
        dimensions
    )

    figure.add_trace(
        go.Mesh3d(
            x=x,
            y=y,
            z=z,
            i=i,
            j=j,
            k=k,
            name=name,
            color=color,
            opacity=opacity,
            flatshading=True,
            hovertemplate=(
                f"{name}"
                "<br>x=%{x:.2f}"
                "<br>y=%{y:.2f}"
                "<br>z=%{z:.2f}"
                "<extra></extra>"
            )
        )
    )


# ============================================================================
# VELOCITY ARROW
# ============================================================================

def add_velocity_arrow(
    figure,
    position,
    velocity
):
    velocity_direction = normalize(
        velocity
    )

    velocity_magnitude = vector_norm(
        velocity
    )

    # ------------------------------------------------------------------------
    # No arrow for a stationary UAV.
    # ------------------------------------------------------------------------

    if velocity_magnitude == 0.0:
        return


    # ------------------------------------------------------------------------
    # Start the arrow slightly above the UAV center so that it appears
    # physically on top of the purple UAV box rather than through it.
    # ------------------------------------------------------------------------

    arrow_start = (
        position[0],
        position[1],
        position[2]
        + 0.5
    )


    # ------------------------------------------------------------------------
    # Arrow endpoint.
    # ------------------------------------------------------------------------

    arrow_end = (
        arrow_start[0]
        + velocity_direction[0]
        * VELOCITY_ARROW_LENGTH,

        arrow_start[1]
        + velocity_direction[1]
        * VELOCITY_ARROW_LENGTH,

        arrow_start[2]
        + velocity_direction[2]
        * VELOCITY_ARROW_LENGTH
    )


    # ------------------------------------------------------------------------
    # Main flat arrow shaft.
    #
    # Cone gives us a flat 3D arrowhead while the line gives the shaft.
    # ------------------------------------------------------------------------

    figure.add_trace(
        go.Scatter3d(
            x=[
                arrow_start[0],
                arrow_end[0]
            ],

            y=[
                arrow_start[1],
                arrow_end[1]
            ],

            z=[
                arrow_start[2],
                arrow_end[2]
            ],

            mode="lines",

            name="Initial velocity",

            line=dict(
                color="green",
                width=VELOCITY_ARROW_WIDTH * 20
            ),

            hovertemplate=(
                "Initial velocity"
                "<br>Vx = "
                f"{velocity[0]:.3f} m/s"
                "<br>Vy = "
                f"{velocity[1]:.3f} m/s"
                "<br>Vz = "
                f"{velocity[2]:.3f} m/s"
                "<extra></extra>"
            )
        )
    )


    # ------------------------------------------------------------------------
    # Flat 3D arrowhead.
    # ------------------------------------------------------------------------

    figure.add_trace(
        go.Cone(
            x=[
                arrow_end[0]
            ],

            y=[
                arrow_end[1]
            ],

            z=[
                arrow_end[2]
            ],

            u=[
                velocity_direction[0]
            ],

            v=[
                velocity_direction[1]
            ],

            w=[
                velocity_direction[2]
            ],

            sizemode="absolute",

            sizeref=VELOCITY_ARROW_HEAD_LENGTH,

            anchor="tip",

            colorscale=[
                [0, "green"],
                [1, "green"]
            ],

            showscale=False,

            name="Initial velocity",

            hoverinfo="skip"
        )
    )


# ============================================================================
# CREATE FIGURE
# ============================================================================

def create_figure(
    scenario,
    result
):
    scenario_id = int(
        scenario["scenario_id"]
    )


    # ========================================================================
    # INPUT WAYPOINTS
    # ========================================================================

    waypoints = [
        vec3(point)
        for point in scenario[
            "path"
        ][
            "waypoints"
        ]
    ]

    waypoint_x = [
        point[0]
        for point in waypoints
    ]

    waypoint_y = [
        point[1]
        for point in waypoints
    ]

    waypoint_z = [
        point[2]
        for point in waypoints
    ]


    # ========================================================================
    # GENERATED BEZIER PATH
    # ========================================================================

    bezier_path = generate_complete_bezier_path(
        result
    )

    if not bezier_path:
        raise ValueError(
            "Result contains no final_bezier_segments."
        )

    bezier_x = [
        point[0]
        for point in bezier_path
    ]

    bezier_y = [
        point[1]
        for point in bezier_path
    ]

    bezier_z = [
        point[2]
        for point in bezier_path
    ]


    # ========================================================================
    # FIGURE
    # ========================================================================

    figure = go.Figure()


    # ========================================================================
    # ORIGINAL WAYPOINT PATH
    # ========================================================================

    figure.add_trace(
        go.Scatter3d(
            x=waypoint_x,
            y=waypoint_y,
            z=waypoint_z,

            mode="lines",

            name="Original waypoint path",

            line=dict(
                color="blue",
                width=4,
                dash="dot"
            ),

            hoverinfo="skip"
        )
    )


    # ========================================================================
    # ORIGINAL WAYPOINTS
    # ========================================================================

    figure.add_trace(
        go.Scatter3d(
            x=waypoint_x,
            y=waypoint_y,
            z=waypoint_z,

            mode="markers",

            name="Waypoints",

            marker=dict(
                color="blue",
                size=7,
                symbol="circle"
            ),

            text=[
                f"Waypoint {index}"
                for index in range(
                    len(waypoints)
                )
            ],

            hovertemplate=(
                "%{text}"
                "<br>X = %{x:.3f} m"
                "<br>Y = %{y:.3f} m"
                "<br>Z = %{z:.3f} m"
                "<extra></extra>"
            )
        )
    )


    # ========================================================================
    # WHOLE GENERATED BEZIER PATH
    # ========================================================================

    figure.add_trace(
        go.Scatter3d(
            x=bezier_x,
            y=bezier_y,
            z=bezier_z,

            mode="lines",

            name="Generated Bézier path",

            line=dict(
                color="grey",
                width=8
            ),

            hovertemplate=(
                "Generated path"
                "<br>X = %{x:.3f} m"
                "<br>Y = %{y:.3f} m"
                "<br>Z = %{z:.3f} m"
                "<extra></extra>"
            )
        )
    )


    # ========================================================================
    # UAV
    # ========================================================================

    uav = scenario["uav"]

    uav_position = vec3(
        uav["position"]
    )

    uav_dimensions = vec3(
        uav["dimensions"]
    )

    # 2× the original visualization dimensions.
    enlarged_uav_dimensions = (
        uav_dimensions[0]
        * BOX_SIZE_MULTIPLIER,

        uav_dimensions[1]
        * BOX_SIZE_MULTIPLIER,

        uav_dimensions[2]
        * BOX_SIZE_MULTIPLIER
    )

    add_box(
        figure,
        uav_position,
        enlarged_uav_dimensions,
        "UAV",
        "purple",
        0.70
    )


    # ========================================================================
    # INITIAL VELOCITY ARROW
    # ========================================================================

    initial_velocity = vec3(
        uav["velocity"]
    )

    add_velocity_arrow(
        figure,
        uav_position,
        initial_velocity
    )


    # ========================================================================
    # GOAL
    # ========================================================================

    goal = waypoints[-1]

    # 2× the original visualization dimensions.
    enlarged_goal_dimensions = (
        uav_dimensions[0]
        * BOX_SIZE_MULTIPLIER,

        uav_dimensions[1]
        * BOX_SIZE_MULTIPLIER,

        uav_dimensions[2]
        * BOX_SIZE_MULTIPLIER
    )

    add_box(
        figure,
        goal,
        enlarged_goal_dimensions,
        "Goal",
        "green",
        0.55
    )


    # ========================================================================
    # OBSTACLES
    # ========================================================================

    obstacles = scenario[
        "map"
    ].get(
        "obstacles",
        []
    )

    for obstacle_index, obstacle in enumerate(
        obstacles
    ):
        center = vec3(
            obstacle["center"]
        )

        dimensions = vec3(
            obstacle["dimensions"]
        )

        obstacle_id = obstacle.get(
            "id",
            obstacle_index
        )

        add_box(
            figure,
            center,
            dimensions,
            f"Obstacle {obstacle_id}",
            "red",
            0.30
        )


    # ========================================================================
    # LAYOUT
    # ========================================================================

    title = (
        f"UAV Path Modification — "
        f"Scenario {scenario_id}"
    )

    if not result.get(
        "success",
        False
    ):
        title += " — SEARCH FAILED"


    figure.update_layout(
        title=dict(
            text=title,
            x=0.5
        ),

        scene=dict(
            xaxis=dict(
                title="X (m)",
                showgrid=True,
                zeroline=True
            ),

            yaxis=dict(
                title="Y (m)",
                showgrid=True,
                zeroline=True
            ),

            zaxis=dict(
                title="Z (m)",
                showgrid=True,
                zeroline=True
            ),

            aspectmode="data"
        ),

        legend=dict(
            x=0.01,
            y=0.99
        ),

        margin=dict(
            l=0,
            r=0,
            t=60,
            b=0
        ),

        template="plotly_white"
    )

    return figure


# ============================================================================
# HTML
# ============================================================================

def write_html(
    figure,
    filename,
    scenario_id
):
    plot_div = figure.to_html(
        include_plotlyjs="cdn",
        full_html=False,
        config={
            "displaylogo": False,
            "responsive": True
        }
    )

    marker = '<div id="'

    start = plot_div.find(
        marker
    )

    if start == -1:
        raise RuntimeError(
            "Could not locate Plotly graph."
        )

    start += len(marker)

    end = plot_div.find(
        '"',
        start
    )

    if end == -1:
        raise RuntimeError(
            "Could not determine Plotly graph ID."
        )

    graph_id = plot_div[
        start:end
    ]


    html = f"""<!DOCTYPE html>
<html lang="en">

<head>

<meta charset="UTF-8">

<meta
    name="viewport"
    content="width=device-width, initial-scale=1.0"
>

<title>
UAV Path Modification — Scenario {scenario_id}
</title>

<style>

html,
body {{
    width: 100%;
    height: 100%;
    margin: 0;
    padding: 0;
    overflow: hidden;
    font-family: Arial, sans-serif;
}}

#plot-container {{
    width: 100%;
    height: 100%;
}}

#download-buttons {{
    position: fixed;
    top: 12px;
    right: 12px;
    z-index: 1000;

    display: flex;
    gap: 8px;
}}

#download-buttons button {{
    border: 1px solid #888;
    background: white;

    padding: 8px 12px;

    border-radius: 4px;

    cursor: pointer;

    font-size: 13px;
}}

#download-buttons button:hover {{
    background: #eeeeee;
}}

</style>

</head>

<body>

<div id="download-buttons">

<button onclick="downloadPNG()">
    Download PNG
</button>

<button onclick="downloadSVG()">
    Download SVG
</button>

</div>

<div id="plot-container">

{plot_div}

</div>

<script>

function downloadPNG() {{

    Plotly.downloadImage(
        document.getElementById("{graph_id}"),
        {{
            format: "png",
            filename: "scenario_{scenario_id:03d}",
            width: 1600,
            height: 1000,
            scale: 2
        }}
    );

}}

function downloadSVG() {{

    Plotly.downloadImage(
        document.getElementById("{graph_id}"),
        {{
            format: "svg",
            filename: "scenario_{scenario_id:03d}",
            width: 1600,
            height: 1000
        }}
    );

}}

</script>

</body>

</html>
"""

    with open(
        filename,
        "w",
        encoding="utf-8"
    ) as file:
        file.write(
            html
        )


# ============================================================================
# PROCESS ONE RESULT
# ============================================================================

def process_result(
    result_filename
):
    result = load_json(
        result_filename
    )

    if "scenario_id" not in result:
        raise ValueError(
            f"{result_filename} does not contain scenario_id."
        )

    scenario_id = int(
        result["scenario_id"]
    )

    scenario_filename = find_scenario_file(
        scenario_id
    )

    if scenario_filename is None:
        raise FileNotFoundError(
            f"Could not find "
            f"{SCENARIO_DIR}/scenario_{scenario_id:03d}.json"
        )

    scenario = load_json(
        scenario_filename
    )

    if int(
        scenario["scenario_id"]
    ) != scenario_id:
        raise ValueError(
            "Scenario ID mismatch."
        )

    figure = create_figure(
        scenario,
        result
    )

    os.makedirs(
        OUTPUT_DIR,
        exist_ok=True
    )

    output_filename = os.path.join(
        OUTPUT_DIR,
        f"scenario_{scenario_id:03d}.html"
    )

    write_html(
        figure,
        output_filename,
        scenario_id
    )

    return output_filename


# ============================================================================
# MAIN
# ============================================================================

def main():

    print(
        "=========================================="
    )
    print(
        " UAV Path Modifier - Plot Generator"
    )
    print(
        "=========================================="
    )
    print()

    result_files = list_result_files()

    if not result_files:
        print(
            f"No result JSON files found in "
            f"'{RESULT_DIR}'."
        )

        sys.exit(1)

    print(
        f"Found {len(result_files)} result file(s)."
    )

    print()

    successful = 0
    failed = 0

    for result_filename in result_files:

        print(
            "------------------------------------------"
        )

        print(
            f"Processing:"
        )

        print(
            f"  {result_filename}"
        )

        try:

            output_filename = process_result(
                result_filename
            )

            print(
                "Created:"
            )

            print(
                f"  {output_filename}"
            )

            successful += 1

        except Exception as error:

            print(
                "ERROR:"
            )

            print(
                f"  {error}"
            )

            failed += 1

    print()

    print(
        "=========================================="
    )

    print(
        "PLOT GENERATION FINISHED"
    )

    print(
        "=========================================="
    )

    print(
        f"Successful: {successful}"
    )

    print(
        f"Failed:     {failed}"
    )

    print(
        f"Output:     {OUTPUT_DIR}/"
    )


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    main()