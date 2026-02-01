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

from typing import List, Tuple, Dict, Callable, Optional, Any

# List of all commands recognised by the simulator. If a command name
# appears here but does not have an implementation (either in
# StackLinkState or in this module), calling it will result in a 9999
# error.
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
    "ListCommands", "ListParameters", "ListParams", "ListSettings",
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

    If the command is implemented by the StackLinkState class (i.e., it has a
    method named ``cmd_<name.lower()>``), the returned handler will invoke
    that method. Otherwise, if the command is recognised but not
    implemented, the returned handler will call :func:`not_implemented`.
    Unknown commands (not listed in COMMAND_LIST) should be handled by
    the server's default logic.
    """
    # Normalize the command name
    name_upper = name.strip().split()[0].upper()
    def handler(state: Any, args: str) -> Tuple[int, str, List[str]]:
        # Check if state has a method for this command
        method_name = f"cmd_{name_upper.lower()}"
        if hasattr(state, method_name):
            method = getattr(state, method_name)
            # Invoke the existing implementation
            return method(args)
        # If command is known but no implementation exists, return stub
        return not_implemented(state, args)
    return handler
