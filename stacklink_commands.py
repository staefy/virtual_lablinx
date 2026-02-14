"""StackLink command registry for the virtual simulator.

This module defines the full set of supported StackLink commands and provides
a generic handler for unimplemented commands. The intent is to centralize
command definitions separate from the main server implementation. If a
command has a corresponding implementation in the server's StackLinkState
class (e.g. `cmd_dispense`), that implementation will be invoked. If the
command is recognised but there is no mock implementation, a 9999 error
will be returned. Unknown commands are left to the server to handle as
"unrecognized" (0001).

The COMMAND_LIST below was derived from Hudson Robotics' StackLink Command
Manual (2025-10-14) and covers diagnostics, hardware management, logging,
general movement, lift operations, meta operations, plate movement, and
various settings categories. Only a subset of these commands are
implemented in the virtual simulator; the remainder will return
``9999 No mock implementation for this command`` when invoked.
"""

import logging
import threading
import time
from typing import List, Tuple, Dict, Callable, Optional, Any

# List of all commands recognised by the simulator.
COMMAND_LIST: List[str] = [
    # Diagnostics
    "ClearCounter", "ClearCounters", "DumpInputState", "DumpOutputState",
    "DumpStops", "GetFileData", "GetFileList", "GetFileSize",
    "GetLoopCounter", "GetStackSensors", "GetStopControlStates",
    "GetStopOutputStates", "GetStopSensors", "IncrementCounter",
    "ListCounters", "ListLabLinxException", "ListMetrics",
    "ListRawMetrics", "MovePlateTest", "TestLift", "TestStops",
    # Hardware management
    "GetClockTime", "GetClockTimezone", "GetCpuInfo", "GetDiskInfo",
    "GetFirmwareUptime", "GetMemoryUsage", "GetOsInfo", "GetSystemUptime",
    "GetUptime", "ListClockTimezones", "ListComPorts", "SendTo",
    "SetClockTime", "SetClockTimezone", "Shutdown", "Terminal",
    # Logging
    "GetLogLevel", "GetLogPath", "ListBootLog", "ListLog", "ListLogSources",
    "LogCommands", "SetLogLevel",
    # General direct I/O
    "ListIOs", "ReadAnalog", "ReadInp", "ReadInput", "WriteAnalog",
    "WriteOut", "WriteOutput",
    # General movement
    "CalculateMoveSpeed", "CalculateMoveTime", "EStop", "GetAutoHome",
    "GetPos", "GetPosition", "GetSpeed", "Halt", "Home", "Jog",
    "Move", "Move_Abs", "MoveAxes", "MoveAxis", "MoveFast",
    "SetAutoHome", "SetSpeed", "ShiftedMove", "Status",
    # Lift commands
    "Dispense", "Return",
    # Meta commands
    "Can", "DeleteSettingsFile", "GetSerialNumber", "GetSettings",
    "ListCommands", "ListParameters", "ListParams", "ListSettings", "Version",
    # Plate movement
    "AcknowledgeSend", "CompoundShift", "ConveyorOff", "ConveyorOn",
    "GetIgnoreStop", "GetIgnoreStops", "HasPlate", "IgnoreAllStops",
    "IgnoreStop", "IgnoreStopRange", "MovePlate", "ReceivePlate",
    "SendPlate", "ShiftPlates",
    # Settings – motor configuration
    "ClearMotionOverrides", "DeleteMotionProfile", "GetAxes",
    "GetAxisProfile", "GetDefaultMotion", "GetEStopAxes",
    "GetHomeMotion", "GetLimits", "GetMotionProfile", "GetPrimaryAxes",
    "GetSecondaryAxes", "GetStepsPerUnit", "GetSynchronizeMotion",
    "ListAxisProfiles", "ListDefaultMotions", "ListHomeMotions",
    "ListMotionProfiles", "SetAxisProfile", "SetDefaultMotion",
    "SetEStopAxes", "SetHomeMotion", "SetLimits", "SetMotionProfile",
    "SetSynchronizeMotion",
    # Settings – network configuration
    "GetCurrentIP", "GetDefaultGateway", "GetDNS", "GetHostName",
    "GetIP", "GetIPAddress", "GetSubnetMask", "GetUseDHCP",
    "ListNetworkSettings", "SetDefaultGateway", "SetDNS", "SetHostName",
    "SetIP", "SetIPAddress", "SetSubnetMask", "SetUseDHCP",
    # Settings – point management
    "AddCoordinates", "ClearPoints", "ConstrainPoint", "DeletePoint",
    "GetPoint", "Here", "ListPoints", "LoadPoint", "RemoveCoordinates",
    "Set", "SetShifted", "Shift",
    # Settings – stop management
    "AddStop", "GetStopFlags", "GetStopName", "GetStopPort", "InsertStop",
    "ListStops", "MoveStop", "RemoveAllStops", "RemoveStop",
    "RenameStop", "SetStopFlags", "SetStopName", "SetStopPort", "SwapStops",
    # Settings – track configuration
    "GetAutoStop", "GetAutoStopTime", "GetHighSpeedDistance",
    "GetMoveTime", "GetNumTracks", "GetPartners", "GetShiftTimeout",
    "GetTrackSettings", "ListTrackSettings", "SendToPartner",
    "SetAutoStop", "SetAutoStopTime", "SetHighSpeedDistance",
    "SetMoveTime", "SetPartners", "SetShiftTimeout", "SetTrackSettings",
]


def not_implemented(state: Any, args: str) -> Tuple[int, str, List[str]]:
    """Return a 9999 error for unimplemented commands.

    Parameters:
        state: The StackLinkState instance (not used).
        args: The argument string provided to the command.

    Returns:
        A tuple containing the 9999 error code, a descriptive message, and
        an empty list for extra lines.
    """
    return 9999, "No mock implementation for this command", []


def get_handler(name: str) -> Callable[[Any, str], Tuple[int, str, List[str]]]:
    """Return a handler function for the given command name.

    If the command is implemented in this module (e.g. `cmd_dispense`),
    the returned handler will invoke that function.
    """
    name_upper = name.strip().split()[0].upper()
    handler_func = globals().get(f"cmd_{name_upper.lower()}")
    if handler_func:
        return handler_func
    return not_implemented


# --- Command Implementations ---

def cmd_version(state: Any, args: str) -> Tuple[int, str, List[str]]:
    return 0, state.version_info, []


def cmd_listcommands(state: Any, args: str) -> Tuple[int, str, List[str]]:
    query = args.strip()
    if query:
        filtered = [c for c in state.commands if query.lower() in c.lower()]
    else:
        filtered = list(state.commands)
    extra = [c for c in filtered]
    msg = f"Command list ({len(filtered)} commands)"
    return 0, msg, extra


def cmd_getstopsensors(state: Any, args: str) -> Tuple[int, str, List[str]]:
    try:
        parts = args.split(",")
        if parts:
            track = int(parts[0])  # noqa: F841
    except Exception:
        return 1, "Invalid parameters", []
    return 0, state.stops_status_string(), []


def cmd_hasplate(state: Any, args: str) -> Tuple[int, str, List[str]]:
    try:
        track_str, stop_str = args.split(",")
        stop = int(stop_str)
    except Exception:
        return 1, "Invalid parameters", []
    if stop not in state.stops:
        return 1, "Stop out of range", []
    has_plate = state.stops[stop].has_plate
    status = "Object" if has_plate else "Empty"
    return 0, status, []


def cmd_getignorestop(state: Any, args: str) -> Tuple[int, str, List[str]]:
    try:
        _, stop_str = args.split(",")
        stop = int(stop_str)
    except Exception:
        return 1, "Invalid parameters", []
    if stop not in state.stops:
        return 1, "Stop out of range", []
    status = "True" if state.stops[stop].ignored else "False"
    return 0, status, []


def cmd_getignorestops(state: Any, args: str) -> Tuple[int, str, List[str]]:
    ignored = state.ignored_status_string()
    return 0, f"Ignored stops: {ignored}", []


def cmd_ignorestop(state: Any, args: str) -> Tuple[int, str, List[str]]:
    try:
        parts = [x.strip() for x in args.split(",")]
        track = int(parts[0])  # noqa: F841
        stop = int(parts[1])
        ignore = parts[2].lower() in ("true", "1", "yes")
    except Exception:
        return 1, "Invalid parameters", []
    if stop not in state.stops:
        return 1, "Stop out of range", []
    state.stops[stop].ignored = ignore
    return 0, "Success", []


def cmd_ignorestoprange(state: Any, args: str) -> Tuple[int, str, List[str]]:
    try:
        parts = [x.strip() for x in args.split(",")]
        track = int(parts[0])  # noqa: F841
        start = int(parts[1])
        end = int(parts[2])
        ignore = parts[3].lower() in ("true", "1", "yes")
    except Exception:
        return 1, "Invalid parameters", []
    for stop_id in range(start, end + 1):
        if stop_id in state.stops:
            state.stops[stop_id].ignored = ignore
    return 0, "Success", []


def cmd_ignoreallstops(state: Any, args: str) -> Tuple[int, str, List[str]]:
    try:
        parts = [x.strip() for x in args.split(",")]
        track = int(parts[0])  # noqa: F841
        ignore = parts[1].lower() in ("true", "1", "yes")
    except Exception:
        return 1, "Invalid parameters", []
    for stop in state.stops.values():
        stop.ignored = ignore
    return 0, "Success", []


def cmd_dispense(state: Any, args: str) -> Tuple[int, str, List[str]]:
    try:
        track_str, lift_str = args.split(",")
        lift = int(lift_str)
    except Exception:
        return 1, "Invalid parameters", []
    if lift not in state.lift_map:
        return 1, "Unknown lift", []
    stop_id = state.lift_map[lift]
    stack = state.stacks.get(lift)
    stop = state.stops[stop_id]
    if state.error_flags.get(f"lift_blocked_{lift}", False):
        return 2001, "Cannot dispense; lift is blocked", []
    if state.error_flags.get(f"dispense_failure_{lift}", False):
        return 2000, "No object was dispensed", []
    if stop.has_plate:
        return 2001, "Cannot dispense; lift is blocked", []
    if not stack or not stack.dispense():
        return 2000, "No object was dispensed", []
    plate_id = state.next_plate_id
    state.next_plate_id += 1
    
    # Synchronous hardware delay
    time.sleep(state.dispense_time)
    
    stop.has_plate = True
    stop.plate_id = plate_id
    return 0, "Success", []


def cmd_return(state: Any, args: str) -> Tuple[int, str, List[str]]:
    try:
        track_str, lift_str = args.split(",")
        lift = int(lift_str)
    except Exception:
        return 1, "Invalid parameters", []
    if lift not in state.lift_map:
        return 1, "Unknown lift", []
    stop_id = state.lift_map[lift]
    stack = state.stacks.get(lift)
    stop = state.stops[stop_id]
    if state.error_flags.get(f"lift_blocked_{lift}", False):
        return 2001, "Cannot dispense; lift is blocked", []
    if state.error_flags.get(f"stack_full_{lift}", False):
        return 2003, "Stack full", []
    if not stop.has_plate:
        return 2002, "No plate at lift", []
    stop.has_plate = False
    stop.plate_id = None
    
    # Synchronous hardware delay
    time.sleep(state.return_time)
    
    if not stack.return_plate():
        # This shouldn't happen if we checked before, but for safety
        return 2003, "Stack full", []
    return 0, "Success", []


def cmd_moveplate(state: Any, args: str) -> Tuple[int, str, List[str]]:
    try:
        parts_int = [int(x.strip()) for x in args.split(",")]
        track, source, dest = parts_int
    except Exception:
        return 1, "Invalid parameters", []
    if source not in state.stops or dest not in state.stops:
        return 1, "Stop out of range", []
    if state.error_flags.get("movement_blocked", False):
        return 57, "Movement blocked", []
    if not state.stops[source].has_plate:
        return 2004, "No plate at source", []
    if dest > source:
        path = range(source + 1, dest + 1)
    else:
        path = range(dest, source)
    for i in path:
        if i == source:
            continue
        if i in state.stops and state.stops[i].has_plate:
            return 57, "Movement blocked", []
    state.stops[source].has_plate = False
    plate_id = state.stops[source].plate_id
    state.stops[source].plate_id = None
    
    # Synchronous hardware delay with active tracking for animation
    distance = abs(dest - source)
    duration = distance * state.move_time_per_segment
    
    if plate_id is not None:
        state.active_moves[plate_id] = {
            "source": source,
            "dest": dest,
            "duration": duration,
            "start_time": time.time()
        }
    
    try:
        time.sleep(duration)
    finally:
        if plate_id is not None:
            state.active_moves.pop(plate_id, None)
    
    state.stops[dest].has_plate = True
    state.stops[dest].plate_id = plate_id
    return 0, state.stops_status_string(), []


def cmd_shiftplates(state: Any, args: str) -> Tuple[int, str, List[str]]:
    try:
        track_str, direction = [x.strip() for x in args.split(",")]
    except Exception:
        return 1, "Invalid parameters", []
    direction = direction.lower()
    if direction not in ("forward", "fwd", "f", "reverse", "rev", "r"):
        return 1, "Invalid direction", []
    forward = direction in ("forward", "fwd", "f")
    indices = sorted(state.stops.keys())
    moved = False
    if forward:
        for i in reversed(indices):
            if state.stops[i].has_plate:
                next_idx = i + 1
                if next_idx not in state.stops or state.stops[next_idx].has_plate:
                    continue
                state.stops[i].has_plate = False
                state.stops[next_idx].has_plate = True
                moved = True
    else:
        for i in indices:
            if state.stops[i].has_plate:
                next_idx = i - 1
                if next_idx not in state.stops or state.stops[next_idx].has_plate:
                    continue
                state.stops[i].has_plate = False
                state.stops[next_idx].has_plate = True
                moved = True
    if state.error_flags.get("movement_blocked", False):
        return 57, "Movement blocked", []
    if not moved:
        return 2005, "No plates moved", []
    return 0, state.stops_status_string(), []


def cmd_sendplate(state: Any, args: str) -> Tuple[int, str, List[str]]:
    try:
        track_str, stop_str = args.split(",")
        stop = int(stop_str)
    except Exception:
        return 1, "Invalid parameters", []
    if stop not in state.stops:
        return 1, "Stop out of range", []
    if not state.stops[stop].has_plate:
        return 2004, "No plate at source", []
    state.stops[stop].has_plate = False
    state.stops[stop].plate_id = None
    return 0, "Success", []


def cmd_receiveplate(state: Any, args: str) -> Tuple[int, str, List[str]]:
    try:
        track_str, stop_str = args.split(",")
        stop = int(stop_str)
    except Exception:
        return 1, "Invalid parameters", []
    if stop not in state.stops:
        return 1, "Stop out of range", []
    # Find lift index for this stop if it's a stack stop
    lift_idx = next((k for k, v in state.lift_map.items() if v == stop), None)
    if lift_idx is not None and state.error_flags.get(f"lift_blocked_{lift_idx}", False):
        return 2001, "Cannot dispense; lift is blocked", []
    if state.stops[stop].has_plate:
        return 2001, "Cannot dispense; lift is blocked", []
    state.stops[stop].has_plate = True
    state.stops[stop].plate_id = None
    return 0, "Success", []


def cmd_acknowledgesend(state: Any, args: str) -> Tuple[int, str, List[str]]:
    return 0, "Success", []


def cmd_getnumtracks(state: Any, args: str) -> Tuple[int, str, List[str]]:
    return 0, "1", []


def cmd_liststops(state: Any, args: str) -> Tuple[int, str, List[str]]:
    """List all stops on the track with their configuration.
    
    Returns stop info matching the StackLink LISTSTOPS format:
    Position N: Port X: Name 'StopName'; [flags]
    """
    try:
        parts = args.split(",")
        if parts and parts[0].strip():
            track = int(parts[0].strip())
            if track != 1:
                return 1, "Invalid track number", []
    except Exception:
        return 1, "Invalid parameters", []
    
    # Build the list of stops from config
    stop_configs = state.config.get("stops", [])
    num_stops = len(stop_configs)
    
    extra_lines = []
    for idx, stop_cfg in enumerate(sorted(stop_configs, key=lambda x: x["id"]), start=1):
        stop_id = stop_cfg["id"]
        stop_type = stop_cfg.get("type", "empty")
        
        # Determine port letter (A, B, C, ...) based on position
        port_letter = chr(ord('A') + (stop_id - 1) % 26)
        
        # Build name and flags based on type
        if stop_type == "stack":
            lift_idx = stop_cfg.get("lift_index", stop_id)
            name = f"Lift{lift_idx}"
            flags = "Lift"
        elif stop_type == "waste":
            name = "Waste"
            flags = "NoSensor"
        else:
            # Generic empty stop or camera position (simulator doesn't know about cameras)
            name = stop_cfg.get("name", f"Stop{stop_id}")
            flags = ""
        
        line = f"Position {idx}: Port {port_letter}: Name '{name}'"
        if flags:
            line += f"; {flags}"
        extra_lines.append(line)
    
    return 0, f"Track 1 has {num_stops} stops:", extra_lines
