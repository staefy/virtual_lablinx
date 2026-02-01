# Virtual LabLinx Simulator

A configurable virtual simulator for the StackLink device, featuring a TCP command server and a web-based dashboard for visual monitoring.

## Features
- **Configurable Layout**: Define stops, stacks, and track connectors in `config.json`.
- **Command Registry**: All command logic is centralized in `stacklink_commands.py` for easy reference and extension.
- **Web UI**: Real-time visualization of plates, stacks, and track status on port 8000.
- **TCP Server**: Accepts standard StackLink commands on port 7000.

## Getting Started

1. **Configure**: Edit `config.json` to define your physical laboratory layout.
2. **Run**:
   ```bash
   python server.py
   ```
3. **Visualize**: Open `http://localhost:8000` in your browser.
4. **Interact**: Send commands via Telnet or code to `localhost:7000`.
   - Examples: `DISPENSE 1,1`, `MOVEPLATE 1,3,4`, `SHIFTPLATES 1,fwd`.

## Command Implementation
To see which commands are implemented or to add new ones, refer to `stacklink_commands.py`. The `COMMAND_LIST` contains all recognized commands, and functions prefixed with `cmd_` provide the mock logic.