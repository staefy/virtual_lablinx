"""
Virtual StackLink device simulator.

This script implements two main services:

1. A TCP server that listens on port 7000 and accepts plain‑text StackLink
   commands. The server responds by echoing the command followed by a
   four‑digit status code and message. It supports a subset of commands
   described in Hudson Robotics' StackLink documentation. Commands that are
   not recognized return an error.

2. A simple web UI served over HTTP on port 8000. The UI
   visualizes the current state of the virtual StackLink, including plate
   positions on the track, inventory levels for each stack, and whether
   individual stops are ignored. The page refreshes automatically every
   few seconds to reflect state changes due to incoming commands.

Usage:
    python server.py

Once running, you can connect to the simulator via telnet or your
application code on localhost:7000 and issue commands. Open a browser
to http://localhost:8000 to view the device state.

This implementation is intentionally verbose and modular to ease
development. It can be refactored later as needed.
"""

import logging
import re
import socket
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# Import the command registry for unimplemented commands
try:
    from . import stacklink_commands
except ImportError:
    # When running as a script without package context, fall back to
    # relative import. This allows the module to be executed directly.
    import stacklink_commands  # type: ignore

# We avoid external dependencies like Flask by using Python's built‑in HTTP server.
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from pathlib import Path


logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s")


def plural(n: int, singular: str, plural: Optional[str] = None) -> str:
    """Return a properly pluralized string."""
    if n == 1:
        return f"{n} {singular}"
    return f"{n} {plural or singular + 's'}"


@dataclass
class TrackStop:
    """Represents a single stop on the track."""
    index: int
    has_plate: bool = False
    ignored: bool = False
    # Unique identifier for the plate currently at this stop, if any. When a
    # plate occupies a stop, plate_id is a positive integer; otherwise None.
    plate_id: Optional[int] = None

    def __repr__(self) -> str:
        status = "Object" if self.has_plate else "Empty"
        if self.ignored:
            status += " (ignored)"
        return status


@dataclass
class Stack:
    """Represents a plate stack with a finite capacity."""
    index: int
    capacity: int = 30
    count: int = 0

    def dispense(self) -> bool:
        """Remove a plate from the stack, if any remain."""
        if self.count > 0:
            self.count -= 1
            return True
        return False

    def return_plate(self) -> bool:
        """Add a plate to the stack, if not full."""
        if self.count < self.capacity:
            self.count += 1
            return True
        return False

    def __repr__(self) -> str:
        return f"Stack {self.index}: {self.count}/{self.capacity} plates"


class StackLinkState:
    """Mutable state of the virtual StackLink."""

    def __init__(self, num_stops: int = 8, num_stacks: int = 2):
        # Create stops numbered 1..num_stops
        self.stops: Dict[int, TrackStop] = {i: TrackStop(i) for i in range(1, num_stops + 1)}
        # Map lifts (stack indices) to specific stops. Lifts are configured as
        # stops #3 and #4 by default. Adjust as needed.
        self.lift_map: Dict[int, int] = {1: 3, 2: 4}
        # Initialize stacks. By default, stack 1 (input) starts with 15 plates
        # and stack 2 (output) starts empty. Additional stacks, if present,
        # default to zero plates.
        self.stacks: Dict[int, Stack] = {}
        for i in range(1, num_stacks + 1):
            default_count = 15 if i == 1 else 0
            self.stacks[i] = Stack(i, capacity=30, count=default_count)
        # Version string for VERSION command
        self.version_info = "StackLink Virtual 1.0.0 (mock)"
        # Supported command list (names only) for LISTCOMMANDS
        self.commands: List[str] = [
            "AcknowledgeSend trackNumber",
            "AddCoordinates pointName, [constrained], [keyValueInput...]",
            "AddStop trackNumber, portName, positionName, [flags]",
            "CompoundShift trackNumber, direction, [times], [receive], [mask]",
            "DISPENSE trackNumber,liftNumber",
            "RETURN trackNumber,liftNumber",
            "MOVEPLATE trackNumber,sourceStop,destinationStop",
            "SHIFTPLATES trackNumber,direction",
            "SENDPLATE trackNumber,stopNumber",
            "RECEIVEPLATE trackNumber,stopNumber",
            "ACKNOWLEDGESEND trackNumber",
            "IGNORESTOP trackNumber,stopNumber,ignore",
            "IGNORESTOPRANGE trackNumber,startStop,endStop,ignore",
            "IGNOREALLSTOPS trackNumber,ignore",
            "GETSTOPSENSORS trackNumber",
            "HASPLATE trackNumber,stopNumber",
            "GETIGNORESTOP trackNumber,stopNumber",
            "GETIGNORESTOPS trackNumber",
            "LISTCOMMANDS [filter]",
            "VERSION",
        ]

        # Counter to assign unique identifiers to plates as they are dispensed.
        # Each call to DISPENSE increments this counter. The IDs are used for
        # display purposes in the UI.
        self.next_plate_id: int = 1

        # Store the initial number of plates per stack so we can reset later.
        self.default_stack_counts: Dict[int, int] = {idx: stack.count for idx, stack in self.stacks.items()}

        # Flags for injecting faults into the simulator. The web UI can toggle
        # these booleans via the /api/set_error endpoint. When set, they
        # override normal behavior and force specific error codes. See
        # individual command handlers for usage. Keys correspond to the
        # checkboxes presented in the settings panel.
        self.error_flags: Dict[str, bool] = {
            # When true, DISPENSE always fails with code 2000 regardless of stack
            # inventory. Use this to simulate a jammed lift or empty magazine.
            "dispense_failure": False,
            # When true, DISPENSE and RECEIVEPLATE fail with 2001 indicating
            # the lift is blocked. Useful to simulate a plate stuck on the lift.
            "lift_blocked": False,
            # When true, RETURN always fails with 2003 even if the stack has
            # space. Use to simulate a full output stack or latch failure.
            "stack_full": False,
            # When true, MOVEPLATE and SHIFTPLATES fail with error code 57
            # regardless of path availability. This simulates an obstruction
            # on the track.
            "movement_blocked": False,
        }

    def set_error_flag(self, name: str, value: bool) -> None:
        """Update an error flag if it exists."""
        if name in self.error_flags:
            self.error_flags[name] = value

    def set_plate_presence(self, stop: int, present: bool) -> bool:
        """Manually set the presence of a plate at a given stop. Returns False if stop is invalid."""
        if stop not in self.stops:
            return False
        self.stops[stop].has_plate = present
        return True

    def get_error_flags(self) -> Dict[str, bool]:
        """Return a copy of the current error flags."""
        return dict(self.error_flags)

    # --- New state management helpers ---
    def reset_state(self, stack_counts: Optional[Dict[int, int]] = None) -> None:
        """Reset the entire device state to a clean configuration.

        If stack_counts is provided, it must be a mapping of stack index to
        the desired plate count. Otherwise each stack is reset to its
        default initial count.
        """
        # Clear all stops
        for stop in self.stops.values():
            stop.has_plate = False
            stop.plate_id = None
            stop.ignored = False
        # Reset stacks to specified counts or defaults
        for idx, stack in self.stacks.items():
            if stack_counts and idx in stack_counts:
                count = stack_counts[idx]
            else:
                # Use per‑stack default counts captured at initialization
                count = self.default_stack_counts.get(idx, 0)
            # Clamp count within capacity
            if count < 0:
                count = 0
            elif count > stack.capacity:
                count = stack.capacity
            stack.count = count
        # Reset error flags
        for key in list(self.error_flags.keys()):
            self.error_flags[key] = False
        # Reset plate ID counter
        self.next_plate_id = 1

    def set_stack_count(self, index: int, count: int) -> bool:
        """Set the number of plates in a stack. Returns True on success."""
        stack = self.stacks.get(index)
        if not stack:
            return False
        # Clamp count within capacity
        if count < 0:
            count = 0
        elif count > stack.capacity:
            count = stack.capacity
        stack.count = count
        return True

    # ---- Helper methods for state introspection ----
    def stops_status_string(self) -> str:
        """Return a comma‑separated list of stop statuses (e.g., '1:Empty, 2:Object')."""
        parts = []
        for i in sorted(self.stops.keys()):
            stop = self.stops[i]
            status = "Object" if stop.has_plate else "Empty"
            parts.append(f"{i}:{status}")
        return ", ".join(parts)

    def ignored_status_string(self) -> str:
        """Return a comma‑separated list of ignored stops."""
        ignored_indices = [str(i) for i, stop in self.stops.items() if stop.ignored]
        return ",".join(ignored_indices) if ignored_indices else "None"

    # ---- Command handlers ----
    def handle_command(self, raw: str) -> Tuple[List[str], str, List[str]]:
        """
        Process a single command line. Returns a tuple of (echo_lines, code_message, extra_lines).

        - echo_lines: always contains the command itself.
        - code_message: string beginning with the four‑digit error code and message.
        - extra_lines: for LIST commands, additional lines of output that precede 'End of List'.
        """
        command = raw.strip()
        echo = [command]

        if not command:
            return echo, "0001 Empty command", []

        # Split command and its parameters
        parts = command.split()
        name = parts[0].upper()
        args_str = command[len(parts[0]):].strip()

        # Dispatch
        handler_name = f"cmd_{name.lower()}"
        handler = getattr(self, handler_name, None)
        # If no built‑in handler exists, consult the external command registry
        if handler is None:
            # Determine if this command is recognised but not implemented
            # Use the command registry to get a fallback handler
            if name.upper() in getattr(stacklink_commands, "COMMAND_LIST", []):
                handler = stacklink_commands.get_handler(name)
            else:
                # Unknown command; return unrecognized
                return echo, "0001 Unrecognized command", []
        try:
            code, message, extra = handler(args_str)
            return echo, f"{code:04d} {message}", extra
        except Exception:
            logging.exception("Error handling command '%s'", command)
            return echo, "9999 Internal error", []

    # Command implementations return (error_code, message, extra_lines)

    def cmd_version(self, args: str) -> Tuple[int, str, List[str]]:
        return 0, self.version_info, []

    def cmd_listcommands(self, args: str) -> Tuple[int, str, List[str]]:
        # Filter commands if a substring is provided
        query = args.strip()
        if query:
            filtered = [c for c in self.commands if query.lower() in c.lower()]
        else:
            filtered = list(self.commands)
        extra = [c for c in filtered]
        msg = f"Command list ({len(filtered)} commands)"
        return 0, msg, extra

    def cmd_getstopsensors(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: GETSTOPSENSORS trackNumber
        try:
            track = int(args.split(",")[0])  # noqa: F841
        except Exception:
            return 1, "Invalid parameters", []
        # Simply return current sensor status for each stop
        return 0, self.stops_status_string(), []

    def cmd_hasplate(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: HASPLATE trackNumber,stopNumber
        try:
            track_str, stop_str = args.split(",")  # noqa: F841
            stop = int(stop_str)
        except Exception:
            return 1, "Invalid parameters", []
        if stop not in self.stops:
            return 1, "Stop out of range", []
        # Determine plate presence
        has_plate = self.stops[stop].has_plate
        status = "Object" if has_plate else "Empty"
        return 0, status, []

    def cmd_getignorestop(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: GETIGNORESTOP trackNumber,stopNumber
        try:
            _, stop_str = args.split(",")
            stop = int(stop_str)
        except Exception:
            return 1, "Invalid parameters", []
        if stop not in self.stops:
            return 1, "Stop out of range", []
        status = "True" if self.stops[stop].ignored else "False"
        return 0, status, []

    def cmd_getignorestops(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: GETIGNORESTOPS trackNumber
        ignored = self.ignored_status_string()
        return 0, f"Ignored stops: {ignored}", []

    def cmd_ignorestop(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: IGNORESTOP trackNumber,stopNumber,ignore
        try:
            parts = [x.strip() for x in args.split(",")]
            track = int(parts[0])  # noqa: F841
            stop = int(parts[1])
            ignore = parts[2].lower() in ("true", "1", "yes")
        except Exception:
            return 1, "Invalid parameters", []
        if stop not in self.stops:
            return 1, "Stop out of range", []
        self.stops[stop].ignored = ignore
        return 0, "Success", []

    def cmd_ignorestoprange(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: IGNORESTOPRANGE trackNumber,startStop,endStop,ignore
        try:
            parts = [x.strip() for x in args.split(",")]
            track = int(parts[0])  # noqa: F841
            start = int(parts[1])
            end = int(parts[2])
            ignore = parts[3].lower() in ("true", "1", "yes")
        except Exception:
            return 1, "Invalid parameters", []
        for stop_id in range(start, end + 1):
            if stop_id in self.stops:
                self.stops[stop_id].ignored = ignore
        return 0, "Success", []

    def cmd_ignoreallstops(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: IGNOREALLSTOPS trackNumber,ignore
        try:
            parts = [x.strip() for x in args.split(",")]
            track = int(parts[0])  # noqa: F841
            ignore = parts[1].lower() in ("true", "1", "yes")
        except Exception:
            return 1, "Invalid parameters", []
        for stop in self.stops.values():
            stop.ignored = ignore
        return 0, "Success", []

    def cmd_dispense(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: DISPENSE trackNumber,liftNumber
        try:
            track_str, lift_str = args.split(",")  # noqa: F841
            lift = int(lift_str)
        except Exception:
            return 1, "Invalid parameters", []
        if lift not in self.lift_map:
            return 1, "Unknown lift", []
        stop_id = self.lift_map[lift]
        stack = self.stacks.get(lift)
        stop = self.stops[stop_id]
        # Fault injection: lift blocked forces a 2001 error before any other checks
        if self.error_flags.get("lift_blocked", False):
            return 2001, "Cannot dispense; lift is blocked", []
        # Fault injection: dispense failure forces a 2000 error regardless of stack
        if self.error_flags.get("dispense_failure", False):
            return 2000, "No object was dispensed", []
        if stop.has_plate:
            return 2001, "Cannot dispense; lift is blocked", []
        # If no stack or stack is empty, return 2000
        if not stack or not stack.dispense():
            return 2000, "No object was dispensed", []
        # Assign a unique ID to the new plate
        plate_id = self.next_plate_id
        self.next_plate_id += 1
        stop.has_plate = True
        stop.plate_id = plate_id
        return 0, "Success", []

    def cmd_return(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: RETURN trackNumber,liftNumber
        try:
            track_str, lift_str = args.split(",")  # noqa: F841
            lift = int(lift_str)
        except Exception:
            return 1, "Invalid parameters", []
        if lift not in self.lift_map:
            return 1, "Unknown lift", []
        stop_id = self.lift_map[lift]
        stack = self.stacks.get(lift)
        stop = self.stops[stop_id]
        # Fault injection: lift blocked prevents returning a plate
        if self.error_flags.get("lift_blocked", False):
            return 2001, "Cannot dispense; lift is blocked", []
        # Fault injection: stack_full forces a 2003 error regardless of capacity
        if self.error_flags.get("stack_full", False):
            return 2003, "Stack full", []
        if not stop.has_plate:
            return 2002, "No plate at lift", []
        # Remove plate from track and return it to the stack
        stop.has_plate = False
        stop.plate_id = None
        if not stack.return_plate():
            return 2003, "Stack full", []
        return 0, "Success", []

    def cmd_moveplate(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: MOVEPLATE trackNumber,sourceStop,destinationStop
        try:
            parts_int = [int(x.strip()) for x in args.split(",")]
            track, source, dest = parts_int  # noqa: F841
        except Exception:
            return 1, "Invalid parameters", []
        if source not in self.stops or dest not in self.stops:
            return 1, "Stop out of range", []
        # Fault injection: movement_blocked forces a 57 error regardless of path availability
        if self.error_flags.get("movement_blocked", False):
            return 57, "Movement blocked", []
        if not self.stops[source].has_plate:
            return 2004, "No plate at source", []
        # Determine direction and path
        if dest > source:
            path = range(source + 1, dest + 1)
        else:
            path = range(dest, source)
        for i in path:
            if i == source:
                continue
            if i in self.stops and self.stops[i].has_plate:
                return 57, "Movement blocked", []
        # Move the plate and its ID from source to destination
        self.stops[source].has_plate = False
        # Transfer plate ID
        plate_id = self.stops[source].plate_id
        self.stops[source].plate_id = None
        self.stops[dest].has_plate = True
        self.stops[dest].plate_id = plate_id
        return 0, self.stops_status_string(), []

    def cmd_shiftplates(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: SHIFTPLATES trackNumber,direction
        try:
            track_str, direction = [x.strip() for x in args.split(",")]  # noqa: F841
        except Exception:
            return 1, "Invalid parameters", []
        direction = direction.lower()
        if direction not in ("forward", "fwd", "f", "reverse", "rev", "r"):
            return 1, "Invalid direction", []
        forward = direction in ("forward", "fwd", "f")
        indices = sorted(self.stops.keys())
        moved = False
        if forward:
            for i in reversed(indices):
                if self.stops[i].has_plate:
                    next_idx = i + 1
                    if next_idx not in self.stops or self.stops[next_idx].has_plate:
                        continue
                    self.stops[i].has_plate = False
                    self.stops[next_idx].has_plate = True
                    moved = True
        else:
            for i in indices:
                if self.stops[i].has_plate:
                    next_idx = i - 1
                    if next_idx not in self.stops or self.stops[next_idx].has_plate:
                        continue
                    self.stops[i].has_plate = False
                    self.stops[next_idx].has_plate = True
                    moved = True
        # Fault injection: movement_blocked forces a 57 error
        if self.error_flags.get("movement_blocked", False):
            return 57, "Movement blocked", []
        if not moved:
            return 2005, "No plates moved", []
        return 0, self.stops_status_string(), []

    def cmd_sendplate(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: SENDPLATE trackNumber,stopNumber
        try:
            track_str, stop_str = args.split(",")  # noqa: F841
            stop = int(stop_str)
        except Exception:
            return 1, "Invalid parameters", []
        if stop not in self.stops:
            return 1, "Stop out of range", []
        if not self.stops[stop].has_plate:
            return 2004, "No plate at source", []
        self.stops[stop].has_plate = False
        self.stops[stop].plate_id = None
        return 0, "Success", []

    def cmd_receiveplate(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: RECEIVEPLATE trackNumber,stopNumber
        try:
            track_str, stop_str = args.split(",")  # noqa: F841
            stop = int(stop_str)
        except Exception:
            return 1, "Invalid parameters", []
        if stop not in self.stops:
            return 1, "Stop out of range", []
        # Fault injection: lift_blocked prevents receiving a plate
        if self.error_flags.get("lift_blocked", False):
            return 2001, "Cannot dispense; lift is blocked", []
        if self.stops[stop].has_plate:
            return 2001, "Cannot dispense; lift is blocked", []
        self.stops[stop].has_plate = True
        # Received plates have no identifier in this mock implementation
        self.stops[stop].plate_id = None
        return 0, "Success", []

    def cmd_acknowledgesend(self, args: str) -> Tuple[int, str, List[str]]:
        # Syntax: ACKNOWLEDGESEND trackNumber
        return 0, "Success", []

    def cmd_getnumtracks(self, args: str) -> Tuple[int, str, List[str]]:
        """Return the number of tracks in this virtual device.

        Syntax: GETNUMTRACKS

        In this simulator, there is only one track, so this command returns
        "1". Additional arguments are ignored.
        """
        return 0, "1", []


class TCPServer(threading.Thread):
    """A simple threaded TCP server for handling StackLink commands."""

    def __init__(self, state: StackLinkState, host: str = "0.0.0.0", port: int = 7000):
        super().__init__(daemon=True)
        self.state = state
        self.host = host
        self.port = port
        self.should_stop = threading.Event()

    def run(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, self.port))
            sock.listen(5)
            logging.info("TCP server listening on %s:%d", self.host, self.port)
            while not self.should_stop.is_set():
                try:
                    conn, addr = sock.accept()
                except Exception:
                    continue
                logging.info("Accepted connection from %s:%d", *addr)
                threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True).start()

    def handle_client(self, conn: socket.socket, addr: Tuple[str, int]) -> None:
        with conn:
            buffer = ""
            while True:
                try:
                    data = conn.recv(1024)
                except Exception:
                    break
                if not data:
                    break
                buffer += data.decode("utf-8", errors="replace")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.rstrip("\r")
                    if not line:
                        continue
                    echo_lines, code_message, extra = self.state.handle_command(line)
                    response_lines: List[str] = []
                    response_lines.extend(echo_lines)
                    response_lines.append(code_message)
                    for ex in extra:
                        response_lines.append(ex)
                    if extra:
                        response_lines.append("End of List")
                    response_text = "\r\n".join(response_lines) + "\r\n"
                    conn.sendall(response_text.encode("utf-8"))
        logging.info("Connection closed from %s:%d", *addr)

    def stop(self) -> None:
        self.should_stop.set()


class WebRequestHandler(BaseHTTPRequestHandler):
    """HTTP request handler serving the dashboard and state API."""

    def __init__(self, *args, state: StackLinkState, **kwargs):
        self.state = state
        super().__init__(*args, **kwargs)

    def do_GET(self):
        """Dispatch GET requests based on path."""
        # Settings and state APIs
        if self.path == "/" or self.path.startswith("/index.html"):
            self.serve_index()
        elif self.path == "/api/state":
            self.serve_state()
        elif self.path.startswith("/api/errors"):
            # Return current error flags as JSON
            self.serve_errors()
        elif self.path.startswith("/api/set_error"):
            # Toggle a specific error flag via query parameters
            self.set_error()
        elif self.path.startswith("/api/set_plate"):
            # Manually set plate presence at a stop
            self.set_plate()
        elif self.path.startswith("/api/reset_state"):
            # Reset the entire simulator state
            self.reset_state_endpoint()
        elif self.path.startswith("/api/set_stack"):
            # Adjust the number of plates in a specific stack
            self.set_stack_endpoint()
        else:
            self.send_response(404)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"Not Found")

    def serve_index(self):
        # Load the template file relative to this script
        # Attempt to read the index.html template. If missing, fall back to a minimal page.
        try:
            template_path = Path(__file__).resolve().parent / "templates" / "index.html"
            with open(template_path, "rb") as f:
                content = f.read()
            template_str = content.decode("utf-8")
        except Exception:
            template_str = None
        # Build state JSON for injection
        state_json = json.dumps(
            {
                "stops": {
                    str(i): {
                        "has_plate": stop.has_plate,
                        "ignored": stop.ignored,
                        "plate_id": stop.plate_id,
                    }
                    for i, stop in self.state.stops.items()
                },
                "stacks": {
                    str(i): {"count": stack.count, "capacity": stack.capacity}
                    for i, stack in self.state.stacks.items()
                },
            }
        )
        if template_str is not None:
            html = template_str.replace("__STATE_JSON__", state_json)
        else:
            # Minimal HTML fallback if template is missing
            html = (
                "<!doctype html><html><head><meta charset='UTF-8'><title>Virtual StackLink</title>"
                "<style>body{font-family:Arial, sans-serif; margin:20px;} pre{background:#f5f5f5; padding:10px;}</style>"
                "</head><body>"
                "<h1>Virtual StackLink</h1>"
                "<p>Template not found. Showing current device state as JSON:</p>"
                f"<pre>{state_json}</pre>"
                "</body></html>"
            )
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(html.encode("utf-8"))

    def serve_state(self):
        stops = {
            i: {
                "has_plate": stop.has_plate,
                "ignored": stop.ignored,
                "plate_id": stop.plate_id,
            }
            for i, stop in self.state.stops.items()
        }
        stacks = {i: {"count": stack.count, "capacity": stack.capacity} for i, stack in self.state.stacks.items()}
        data = json.dumps({"stops": stops, "stacks": stacks}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def serve_errors(self) -> None:
        """Return JSON of the current error flags."""
        flags = self.state.get_error_flags()
        data = json.dumps(flags).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def set_error(self) -> None:
        """Enable or disable an error flag via query parameters.

        Expected parameters:
            name: the error flag key (e.g. 'dispense_failure')
            value: 'true' or 'false'
        """
        from urllib.parse import urlparse, parse_qs
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)
        name = qs.get("name", [None])[0]
        value_str = qs.get("value", [None])[0]
        if name is None or value_str is None:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b"Missing parameters")
            return
        value = value_str.lower() in ("true", "1", "yes", "on")
        self.state.set_error_flag(name, value)
        # Respond with the updated flag state
        resp = {name: self.state.error_flags.get(name, None)}
        data = json.dumps(resp).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def set_plate(self) -> None:
        """Manually set the presence of a plate at a stop via query parameters.

        Expected parameters:
            stop: integer stop number
            present: 'true' or 'false' (default true)
        """
        from urllib.parse import urlparse, parse_qs
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)
        stop_str = qs.get("stop", [None])[0]
        if stop_str is None:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b"Missing stop parameter")
            return
        try:
            stop = int(stop_str)
        except Exception:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b"Invalid stop parameter")
            return
        value_str = qs.get("present", ["true"])[0]
        present = value_str.lower() in ("true", "1", "yes", "on")
        success = self.state.set_plate_presence(stop, present)
        if not success:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b"Stop out of range")
            return
        data = json.dumps({"stop": stop, "present": present}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def reset_state_endpoint(self) -> None:
        """HTTP endpoint to reset the simulator to a clean state."""
        # We could allow optional stack counts via query parameters
        from urllib.parse import urlparse, parse_qs
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)
        stack_counts = {}
        for key in qs:
            if key.startswith("stack"):
                try:
                    idx = int(key.replace("stack", ""))
                    cnt = int(qs[key][0])
                    stack_counts[idx] = cnt
                except Exception:
                    continue
        if stack_counts:
            self.state.reset_state(stack_counts)
        else:
            self.state.reset_state()
        data = json.dumps({"status": "reset"}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def set_stack_endpoint(self) -> None:
        """HTTP endpoint to set the number of plates in a stack."""
        from urllib.parse import urlparse, parse_qs
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)
        stack_str = qs.get("stack", [None])[0]
        count_str = qs.get("count", [None])[0]
        if stack_str is None or count_str is None:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b"Missing parameters")
            return
        try:
            idx = int(stack_str)
            cnt = int(count_str)
        except Exception:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b"Invalid parameters")
            return
        success = self.state.set_stack_count(idx, cnt)
        if not success:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b"Unknown stack")
            return
        data = json.dumps({"stack": idx, "count": cnt}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def run_http_server(state: StackLinkState, host: str = "0.0.0.0", port: int = 8000):
    """Run a simple HTTP server for the dashboard."""
    from functools import partial
    handler_class = partial(WebRequestHandler, state=state)
    httpd = HTTPServer((host, port), handler_class)
    logging.info("HTTP server listening on http://%s:%d", host, port)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        httpd.server_close()


def main() -> None:
    state = StackLinkState()
    tcp_server = TCPServer(state)
    tcp_server.start()
    # Start HTTP server in main thread
    # The web UI listens on port 8000 by default to avoid conflicts with other services.
    run_http_server(state, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()