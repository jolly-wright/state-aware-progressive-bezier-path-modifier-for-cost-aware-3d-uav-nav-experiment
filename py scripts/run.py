import json
import os
import sys
import time

import serial
import serial.tools.list_ports


# ============================================================================
# CONFIGURATION
# ============================================================================

BAUD_RATE = 115200

SCENARIO_DIR = "scenarios"
OUTPUT_DIR = "teensy_results"


# ============================================================================
# TEENSY DETECTION
# ============================================================================

def find_teensy_port():
    ports = list(
        serial.tools.list_ports.comports()
    )

    if not ports:
        return None

    for port in ports:
        description = (
            port.description or ""
        ).lower()

        manufacturer = (
            port.manufacturer or ""
        ).lower()

        if (
            "teensy" in description
            or "teensy" in manufacturer
        ):
            return port.device

    return None


# ============================================================================
# SCENARIO DISCOVERY
# ============================================================================

def list_scenarios():
    if not os.path.isdir(
        SCENARIO_DIR
    ):
        return []

    scenarios = []

    for filename in os.listdir(
        SCENARIO_DIR
    ):
        if (
            filename.startswith("scenario_")
            and filename.endswith(".json")
        ):
            scenarios.append(
                os.path.join(
                    SCENARIO_DIR,
                    filename
                )
            )

    scenarios.sort()

    return scenarios


# ============================================================================
# SERIAL READING
# ============================================================================

def read_line(ser):
    line = ser.readline()

    if not line:
        return None

    return line.decode(
        "utf-8",
        errors="replace"
    ).strip()


# ============================================================================
# SEND SCENARIO
# ============================================================================

def send_scenario(
    ser,
    filename
):
    with open(
        filename,
        "r",
        encoding="utf-8"
    ) as file:
        json_text = file.read()

    # Validate the scenario before sending it.
    scenario = json.loads(
        json_text
    )

    # Re-serialize only for validation/format consistency.
    # The original JSON text is what is sent to the Teensy.
    del scenario

    json_bytes = json_text.encode(
        "utf-8"
    )

    print()
    print(
        "Sending scenario:"
    )
    print(
        f"  {filename}"
    )
    print(
        f"  JSON size: {len(json_bytes)} bytes"
    )

    # ------------------------------------------------------------------------
    # Protocol:
    #
    #   <JSON byte count>\n
    #   <exact JSON bytes>
    #
    # ------------------------------------------------------------------------

    length_line = (
        f"{len(json_bytes)}\n"
        .encode("ascii")
    )

    ser.write(
        length_line
    )

    ser.write(
        json_bytes
    )

    ser.flush()

    print(
        "Scenario JSON sent."
    )


# ============================================================================
# RECEIVE RESULT
# ============================================================================

def receive_result(ser):
    json_lines = []

    inside_result = False

    while True:
        line = read_line(
            ser
        )

        if line is None:
            continue

        print(
            f"< {line}"
        )

        # --------------------------------------------------------------------
        # Beginning of result JSON
        # --------------------------------------------------------------------

        if line == "RESULT_BEGIN":
            inside_result = True
            json_lines.clear()
            continue

        # --------------------------------------------------------------------
        # End of result JSON
        # --------------------------------------------------------------------

        if line == "RESULT_END":
            if not inside_result:
                continue

            json_text = "\n".join(
                json_lines
            )

            try:
                return json.loads(
                    json_text
                )

            except json.JSONDecodeError as error:
                print()
                print(
                    "ERROR: Teensy returned invalid JSON."
                )
                print(
                    f"JSON error: {error}"
                )
                print()
                print(
                    "Received JSON:"
                )
                print(
                    json_text
                )

                raise

        # --------------------------------------------------------------------
        # Store only lines inside RESULT_BEGIN / RESULT_END.
        # --------------------------------------------------------------------

        if inside_result:
            json_lines.append(
                line
            )


# ============================================================================
# SAVE RESULT
# ============================================================================

def save_result(result):
    os.makedirs(
        OUTPUT_DIR,
        exist_ok=True
    )

    scenario_id = result.get(
        "scenario_id"
    )

    if scenario_id is None:
        raise ValueError(
            "Teensy result does not contain 'scenario_id'."
        )

    filename = os.path.join(
        OUTPUT_DIR,
        f"scenario_{int(scenario_id):03d}_result.json"
    )

    with open(
        filename,
        "w",
        encoding="utf-8"
    ) as file:
        json.dump(
            result,
            file,
            indent=4
        )

        file.write(
            "\n"
        )

    return filename


# ============================================================================
# WAIT FOR MESSAGE
# ============================================================================

def wait_for_message(
    ser,
    expected
):
    while True:
        line = read_line(
            ser
        )

        if line is None:
            continue

        print(
            f"< {line}"
        )

        if line == expected:
            return


# ============================================================================
# MAIN
# ============================================================================

def main():

    print(
        "=========================================="
    )
    print(
        " UAV Path Modifier - Teensy Runner"
    )
    print(
        "=========================================="
    )
    print()


    # ========================================================================
    # FIND TEENSY
    # ========================================================================

    port = find_teensy_port()

    if port is None:
        print(
            "ERROR: No Teensy detected."
        )

        print()
        print(
            "Available serial ports:"
        )

        ports = list(
            serial.tools.list_ports.comports()
        )

        if not ports:
            print(
                "  None"
            )
        else:
            for available_port in ports:
                print(
                    f"  {available_port.device}: "
                    f"{available_port.description}"
                )

        sys.exit(1)

    print(
        f"Teensy detected on {port}"
    )


    # ========================================================================
    # FIND SCENARIOS BEFORE OPENING SERIAL
    # ========================================================================

    scenarios = list_scenarios()

    if not scenarios:
        print()
        print(
            f"ERROR: No scenario JSON files found in "
            f"'{SCENARIO_DIR}'."
        )
        sys.exit(1)

    print()
    print(
        f"Found {len(scenarios)} scenario(s):"
    )

    for filename in scenarios:
        print(
            f"  {filename}"
        )


    # ========================================================================
    # OPEN SERIAL
    # ========================================================================

    try:
        ser = serial.Serial(
            port=port,
            baudrate=BAUD_RATE,
            timeout=1
        )

    except serial.SerialException as error:
        print()
        print(
            "ERROR: Could not open Teensy serial port."
        )
        print(
            f"  {error}"
        )
        sys.exit(1)


    # ========================================================================
    # TEENSY MAY RESET WHEN SERIAL OPENS
    # ========================================================================

    time.sleep(
        1.0
    )


    print()
    print(
        "Waiting for Teensy..."
    )


    # ========================================================================
    # WAIT FOR INITIAL READY
    # ========================================================================

    try:
        wait_for_message(
            ser,
            "READY"
        )

    except KeyboardInterrupt:
        print()
        print(
            "Interrupted."
        )
        ser.close()
        return


    print()
    print(
        "Teensy is ready."
    )


    # ========================================================================
    # USER CONTROL
    # ========================================================================

    print()
    print(
        "Type 'run' to execute all scenarios."
    )
    print(
        "Type 'q' to quit."
    )

    while True:

        try:
            command = input(
                "\n> "
            ).strip().lower()

        except KeyboardInterrupt:
            print()
            print(
                "Exiting."
            )
            ser.close()
            return

        if command == "q":
            print(
                "Exiting."
            )
            ser.close()
            return

        if command == "run":
            break

        print(
            "Nothing executed. "
            "Type exactly 'run' or 'q'."
        )


    # ========================================================================
    # EXECUTE ALL SCENARIOS
    # ========================================================================

    completed = 0

    try:

        for scenario_index, filename in enumerate(
            scenarios,
            start=1
        ):

            print()
            print(
                "=========================================="
            )
            print(
                f"Scenario {scenario_index}/{len(scenarios)}"
            )
            print(
                "=========================================="
            )


            # ----------------------------------------------------------------
            # Tell Teensy to prepare for a scenario.
            # ----------------------------------------------------------------

            ser.write(
                b"run\n"
            )

            ser.flush()


            # ----------------------------------------------------------------
            # Wait for Teensy to accept the command.
            # ----------------------------------------------------------------

            print(
                "Waiting for Teensy to accept scenario..."
            )

            wait_for_message(
                ser,
                "READY_FOR_SCENARIO"
            )


            # ----------------------------------------------------------------
            # Send the scenario JSON.
            # ----------------------------------------------------------------
            
            send_scenario(
                ser,
                filename
            )


            # ----------------------------------------------------------------
            # Teensy now executes the complete scenario.
            #
            # Any normal progress messages printed by Teensy are displayed
            # live by receive_result().
            #
            # receive_result() returns only after:
            #
            #   RESULT_BEGIN
            #   <JSON>
            #   RESULT_END
            #
            # ----------------------------------------------------------------

            print()
            print(
                "Waiting for Teensy result..."
            )

            result = receive_result(
                ser
            )


            # ----------------------------------------------------------------
            # Save returned result JSON.
            # ----------------------------------------------------------------

            output_file = save_result(
                result
            )

            completed += 1

            print()
            print(
                "Result received successfully."
            )
            print(
                f"Saved result to:"
            )
            print(
                f"  {output_file}"
            )


            # ----------------------------------------------------------------
            # Wait for Teensy to finish returning to its idle state.
            # ----------------------------------------------------------------

            print()
            print(
                "Waiting for Teensy..."
            )

            wait_for_message(
                ser,
                "WAITING_FOR_RUN"
            )

            print(
                "Teensy is idle."
            )


    except KeyboardInterrupt:

        print()
        print(
            "Interrupted by user."
        )

        print(
            f"Completed scenarios: "
            f"{completed}/{len(scenarios)}"
        )

        ser.close()
        return


    except json.JSONDecodeError:

        print()
        print(
            "Scenario/result JSON error."
        )

        ser.close()
        sys.exit(1)


    except (OSError, serial.SerialException) as error:

        print()
        print(
            "Serial communication error:"
        )
        print(
            f"  {error}"
        )

        ser.close()
        sys.exit(1)


    except Exception as error:

        print()
        print(
            "ERROR:"
        )
        print(
            f"  {error}"
        )

        ser.close()
        sys.exit(1)


    # ========================================================================
    # ALL SCENARIOS COMPLETE
    # ========================================================================

    print()
    print(
        "=========================================="
    )
    print(
        "ALL SCENARIOS FINISHED"
    )
    print(
        "=========================================="
    )
    print(
        f"Completed: {completed}/{len(scenarios)}"
    )
    print(
        f"Results saved in: {OUTPUT_DIR}/"
    )
    print()


    ser.close()


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    main()