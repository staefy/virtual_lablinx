"""
Virtual StackLink device simulator.

This script implements two main services:

1. A TCP server that listens on port 7000 and accepts plain-text StackLink
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
import time
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

    def __init__(self, config_path: str = "config.json"):
        self.config = self._load_config(config_path)
        
        # Initialize stops
        self.stops: Dict[int, TrackStop] = {}
        for stop_cfg in self.config["stops"]:
            stop_id = stop_cfg["id"]
            self.stops[stop_id] = TrackStop(stop_id)
        
        # Initialize stacks and lift_map
        self.stacks: Dict[int, Stack] = {}
        self.lift_map: Dict[int, int] = {}
        for stop_cfg in self.config["stops"]:
            if stop_cfg["type"] == "stack":
                lift_idx = stop_cfg["lift_index"]
                stop_id = stop_cfg["id"]
                self.lift_map[lift_idx] = stop_id
                self.stacks[lift_idx] = Stack(
                    lift_idx, 
                    capacity=stop_cfg.get("capacity", 30), 
                    count=stop_cfg.get("initial_count", 0)
                )

        # Version string for VERSION command
        self.version_info = "StackLink Virtual 1.1.0 (configurable)"
        
        # Supported command list (names only) for LISTCOMMANDS
        self.commands: List[str] = getattr(stacklink_commands, "COMMAND_LIST", [])

        # Counter to assign unique identifiers to plates
        self.next_plate_id: int = 1

        # Store the initial number of plates per stack
        self.default_stack_counts: Dict[int, int] = {idx: stack.count for idx, stack in self.stacks.items()}

        # Initial global error flags
        self.error_flags: Dict[str, bool] = {
            "movement_blocked": False,
        }
        # Add per-stacker flags
        for lift_idx in self.stacks.keys():
            self.error_flags[f"dispense_failure_{lift_idx}"] = False
            self.error_flags[f"lift_blocked_{lift_idx}"] = False
            self.error_flags[f"stack_full_{lift_idx}"] = False

        # Configuration-driven timings (default to 1s)
        timings = self.config.get("timings", {})
        self.dispense_time = float(timings.get("dispense_time", 1.0))
        self.return_time = float(timings.get("return_time", 1.0))
        self.move_time_per_segment = float(timings.get("move_time_per_segment", 1.0))

        # Track active movements for animation [plate_id] -> {source, dest, start_time, duration}
        self.active_moves: Dict[int, dict] = {}

    def _load_config(self, path_str: str) -> dict:
        """Load the layout configuration from a JSON file."""
        try:
            with open(path_str, "r") as f:
                return json.load(f)
        except Exception as e:
            logging.error(f"Failed to load config from {path_str}: {e}")
            # Fallback to a minimal default config if loading fails
            return {
                "stops": [{"id": i, "type": "camera"} for i in range(1, 9)],
                "connectors": []
            }

    def set_error_flag(self, name: str, value: bool) -> None:
        """Update an error flag if it exists. Replaces existing flag if name starts with prefix."""
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
        """Return a comma-separated list of stop statuses (e.g., '1:Empty, 2:Object')."""
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

    # ---- Command handler delegator ----
    def handle_command(self, raw: str) -> Tuple[List[str], str, List[str]]:
        """Process a single command line by delegating to stacklink_commands module."""
        command = raw.strip()
        echo = [command]

        if not command:
            return echo, "0001 Empty command", []

        parts = command.split()
        name = parts[0].upper()
        args_str = command[len(parts[0]):].strip()

        # Get the handler from the external registry (case-insensitive check)
        cmd_list_upper = {c.upper() for c in getattr(stacklink_commands, "COMMAND_LIST", [])}
        if name in cmd_list_upper:
            handler = stacklink_commands.get_handler(name)
        else:
            return echo, "0001 Unrecognized command", []

        try:
            code, message, extra = handler(self, args_str)
            return echo, f"{code:04d} {message}", extra
        except Exception:
            logging.exception("Error handling command '%s'", command)
            return echo, "9999 Internal error", []


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
                    try:
                        conn.sendall(response_text.encode("utf-8"))
                    except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
                        logging.info("Client %s:%d disconnected during response", *addr)
                        return
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
        if self.path == "/" or self.path.startswith("/?") or self.path.startswith("/index.html"):
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
                "config": self.state.config
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
                "type": next((s["type"] for s in self.state.config["stops"] if s["id"] == i), "camera")
            }
            for i, stop in self.state.stops.items()
        }
        stacks = {i: {"count": stack.count, "capacity": stack.capacity} for i, stack in self.state.stacks.items()}
        data = json.dumps({
            "stops": stops,
            "stacks": stacks,
            "active_moves": self.state.active_moves,
            "config": self.state.config
        }).encode("utf-8")
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
    import sys
    
    # Create the state object from config.json
    state = StackLinkState("config.json")

    # Start the TCP server
    try:
        tcp_server = TCPServer(state)
        tcp_server.start()
    except OSError as e:
        if e.errno == 10048 or "address already in use" in str(e).lower():
            print("\n" + "="*60)
            print("ERROR: Port 7000 is already in use!")
            print("Another simulator instance may already be running.")
            print("Please stop the other instance before starting a new one.")
            print("="*60 + "\n")
            sys.exit(1)
        raise

    # Start the HTTP server
    def run_http_server_thread():
        try:
            server_address = ("", 8000)
            httpd = HTTPServer(server_address, lambda *args, **kwargs: WebRequestHandler(*args, state=state, **kwargs))
            logging.info("HTTP server listening on port 8000")
            httpd.serve_forever()
        except OSError as e:
            if e.errno == 10048 or "address already in use" in str(e).lower():
                print("\n" + "="*60)
                print("ERROR: Port 8000 is already in use!")
                print("Another simulator instance may already be running.")
                print("Please stop the other instance before starting a new one.")
                print("="*60 + "\n")
                import sys
                sys.exit(1)
            raise

    # Run HTTP server in a separate thread to not block the main thread
    http_thread = threading.Thread(target=run_http_server_thread, daemon=True)
    http_thread.start()

    # Keep the main thread alive, e.g., by waiting for the TCP server thread (if it were not daemon)
    # or by a simple loop/sleep if there's nothing else for the main thread to do.
    # For daemon threads, the program will exit when the main thread exits.
    # A common pattern is to join non-daemon threads or use a KeyboardInterrupt handler.
    try:
        while True:
            time.sleep(1) # Keep main thread alive
    except KeyboardInterrupt:
        logging.info("Shutting down servers...")
        tcp_server.stop()
        # httpd.server_close() would be called by the http_thread if it caught KeyboardInterrupt,
        # but since it's daemon, it will just terminate with the main thread.
        # If http_thread was not daemon, we would need to signal it to stop and join it.


if __name__ == "__main__":
    main()