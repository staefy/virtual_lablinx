"""
Client demo for processing all camera rigs on the Virtual StackLink.

This script connects to the TCP server implemented by server.py and
performs a realistic batch workflow:

  1. Dispense plates from the input stack (lift 1) until either all
     camera stops contain a plate or the input stack is empty.
  2. For each plate dispensed, move it from the lift to a designated
     camera stop and actuate the camera (using IGNORESTOP to simulate
     the photo process).
  3. Once all plates are staged at their camera stops, de-actuate the
     camera, move each plate to the output lift (stop 4) and return it
     to the output stack (lift 2).
  4. Repeat steps 1–3 until no more plates are available in the input
     stack. Any error code (except for 2000 indicating an empty input
     stack) triggers an interactive prompt: the script asks you to resolve
     the error (e.g. by clearing a mock error or unblocking the track in
     the web UI) and press Enter to retry. No automatic clearing of
     error flags is performed; if the error remains active, the same
     prompt will appear again until you resolve it.

Command responses are printed for transparency. A one-second pause is
inserted between every command to allow you to observe changes in the
web UI.

Run this script while server.py is running:

    python client_demo_all_rigs.py

The script connects to the simulator over the TCP port and the HTTP API at
port 8000. It relies on the HTTP API to determine whether a DISPENSE 2000
error code is due to the input stack being empty or a simulated jam. No
automatic clearing of error flags is performed; you must clear mock
errors via the web UI before hitting Enter when prompted.

"""

import socket
import time
import urllib.request
import urllib.parse
import json
from typing import Tuple, List, Optional


HOST = "localhost"
PORT = 7000

# List of camera stops in the order plates should be filled. To avoid
# movement blocking, we start with the farthest stop (8) and work
# backwards toward the closest (2). Adjust the list if your layout changes.
CAMERA_STOPS: List[int] = [8, 7, 6, 5, 2]


def send_command(conn: socket.socket, command: str) -> Tuple[int, str]:
    """Send a command and return a tuple of (error_code, full_response)."""
    print(f"\n> {command}")
    conn.sendall((command + "\r\n").encode("utf-8"))
    # Read until we get a CRLF-terminated response. We'll accumulate up to
    # some reasonable buffer size; the server sends all responses at once.
    response = conn.recv(4096).decode("utf-8", errors="replace")
    print(response.strip())
    # Parse the second line of the response to extract the error code
    lines = [line for line in response.split("\r\n") if line]
    if len(lines) < 2:
        return 9999, response
    # Lines[0] is echo, lines[1] begins with the code
    try:
        code = int(lines[1].split()[0])
    except Exception:
        code = 9999
    return code, response


def clear_error_flag_for_code(code: int) -> None:
    """Deprecated: clearing error flags is left to the user via the UI.

    This function is retained for compatibility but no longer performs any
    action. The simulator's error flags must be toggled off by the user in
    the web UI; the client script will simply prompt until the error is
    resolved.
    """
    pass


def get_error_flags() -> dict:
    """Fetch the current error flags from the simulator.

    Returns a dictionary mapping flag names to boolean values. If the
    request fails, an empty dictionary is returned.
    """
    url = f"http://{HOST}:8000/api/errors"
    try:
        with urllib.request.urlopen(url, timeout=5) as resp:
            data = resp.read().decode()
            return json.loads(data)
    except Exception:
        return {}


def get_stack_count(stack_num: int) -> int:
    """Return the current count of plates in the specified stack.

    This helper queries the simulator's /api/state endpoint and extracts
    the plate count for the given stack index (as a string key). If the
    request fails, zero is returned. Only stack 1 is currently used in
    this script.
    """
    url = f"http://{HOST}:8000/api/state"
    try:
        with urllib.request.urlopen(url, timeout=5) as resp:
            data = resp.read().decode()
            state = json.loads(data)
            stacks = state.get("stacks", {})
            info = stacks.get(str(stack_num), None)
            if info is not None:
                return int(info.get("count", 0))
    except Exception:
        pass
    return 0


def handle_error(code: int, command: str) -> None:
    """Handle a non-zero error code by prompting the user to resolve the issue.

    This function prints a message indicating the error code and the last
    attempted command, then waits for the user to press Enter before
    returning. Unlike previous versions, it does not clear any error flags
    automatically. The user must open the web UI, untick the relevant mock
    error checkbox or otherwise fix the problem, then press Enter to allow
    the script to retry the command. If the error is not cleared, the
    command will return the same error again and this function will prompt
    again.
    """
    print(f"Encountered error code {code} while executing '{command}'.")
    print(
        "Please resolve the error using the simulation settings (e.g. turn off a mock "
        "error or clear a jam) and press Enter to retry."
    )
    # Wait for user confirmation
    try:
        input()
    except KeyboardInterrupt:
        pass


def attempt_command(conn: socket.socket, command: str, empty_ok_code: Optional[int] = None) -> Optional[int]:
    """Send a command and repeat on error until success or an allowed empty code.

    Parameters:
        conn: the socket to send the command on.
        command: the command string to send.
        empty_ok_code: if provided, an error code that should be treated
            as a non-fatal 'empty' condition (e.g. 2000 for DISPENSE). When this
            code is returned, the function stops retrying and returns the code.

    Returns:
        The error code returned by the command (0 on success). If the
        empty_ok_code is returned, that code is returned. Otherwise, this
        function does not return until a success or empty_ok_code occurs.
    """
    while True:
        code, _ = send_command(conn, command)
        if code == 0:
            return 0
        # Treat specified empty code as a terminal condition for this command.
        # For DISPENSE, code 2000 normally indicates that the input stack is
        # empty. However, if the 'dispense_failure' error flag is active, 2000
        # reflects a simulated jam and should be treated as an error. Check
        # the current error flags to distinguish these cases.
        if empty_ok_code is not None and code == empty_ok_code:
            # Special handling when the returned code matches the designated
            # 'empty' code (e.g. 2000 for DISPENSE). Not all 2000 responses
            # indicate an actual empty stack; some may arise from jam or
            # other simulated faults. To distinguish, inspect the current
            # error flags and the remaining plate count. If any error flag
            # is active or there are still plates in the input stack, we
            # treat this as an error and prompt the user to resolve it.
            if empty_ok_code == 2000:
                # Query all error flags. If any flag is true, we assume
                # the 2000 is due to a simulated jam/fault rather than
                # genuine emptiness.
                flags = get_error_flags()
                if any(flags.values()):
                    handle_error(code, command)
                    continue
                # If there are still plates in the input stack, treat
                # 2000 as a jam and prompt for resolution.
                remaining = get_stack_count(1)
                if remaining > 0:
                    handle_error(code, command)
                    continue
            # If no flags are active and the stack is empty, treat as
            # legitimate empty condition and return the code.
            return code
        # Any other error should prompt the user and retry
        handle_error(code, command)
        # Loop will retry the command after user input



def process_batch(conn: socket.socket) -> bool:
    """Process a single batch of plates across all camera stops.

    Returns True if any plates were processed in this batch, or False if
    the input stack was empty at the start of the batch.
    """
    filled: List[int] = []
    # Dispense and move a plate to each camera stop
    for stop in CAMERA_STOPS:
        # Dispense a plate from lift 1 (input stack). Treat code 2000 as
        # indicating no plates remain to dispense.
        code = attempt_command(conn, "DISPENSE 1,1", empty_ok_code=2000)
        if code == 0:
            # Move the plate from lift (stop 3) to the camera stop
            time.sleep(1)
            move_cmd = f"MOVEPLATE 1,3,{stop}"
            move_code = attempt_command(conn, move_cmd)
            if move_code != 0:
                return True  # An unrecoverable error occurred
            # Actuate the camera rig: set ignore to true
            time.sleep(1)
            ignore_on = f"IGNORESTOP 1,{stop},true"
            ignore_code = attempt_command(conn, ignore_on)
            if ignore_code != 0:
                return True
            # Pause to allow imaging
            time.sleep(1)
            filled.append(stop)
        elif code == 2000:
            # No object was dispensed; input stack is empty
            print("Input stack empty; ending batch processing.")
            break
        else:
            # Unexpected error; abort sequence
            print("Error during DISPENSE; aborting.")
            return False
        # Delay after each DISPENSE sequence
        time.sleep(1)
    if not filled:
        # Nothing to process
        return False
    # Retrieve and return plates from camera stops to output stack.
    # To avoid blocking, we process the closest plates first. Iterate in reverse order.
    for stop in reversed(filled):
        # Deactivate the camera for this stop
        ignore_off_cmd = f"IGNORESTOP 1,{stop},false"
        ignore_code = attempt_command(conn, ignore_off_cmd)
        if ignore_code != 0:
            return True
        time.sleep(1)
        # Move plate back to output lift (stop 4)
        move_back_cmd = f"MOVEPLATE 1,{stop},4"
        move_back_code = attempt_command(conn, move_back_cmd)
        if move_back_code != 0:
            return True
        time.sleep(1)
        # Return plate to output stack (lift 2)
        return_cmd = "RETURN 1,2"
        return_code = attempt_command(conn, return_cmd)
        if return_code != 0:
            return True
        time.sleep(1)
    return True


def main() -> None:
    with socket.create_connection((HOST, PORT)) as conn:
        batch_num = 1
        while True:
            print(f"\n--- Starting batch {batch_num} ---")
            processed = process_batch(conn)
            if not processed:
                break
            batch_num += 1
        print("\nAll batches complete.")


if __name__ == "__main__":
    main()