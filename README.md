# Virtual LabLinx Simulator

A configurable virtual simulator for the StackLink device, featuring a TCP command server and a web-based dashboard for visual monitoring.

## Features
- **Configurable Layout**: Define stops, stacks, and track connectors in `config.json`.
- **Command Registry**: All command logic is centralized in `stacklink_commands.py` for easy reference and extension.
- **Web UI**: Real-time visualization of plates, stacks, and track status on port 8000.
- **TCP Server**: Accepts standard StackLink commands on port 7000.

Visualisation based on config file:
<img width="2036" height="428" alt="image" src="https://github.com/user-attachments/assets/752b8a5a-e2d5-452b-8e9d-46e0ccc5393a" />

and some UI to control simulation state:

<img width="323" height="369" alt="image" src="https://github.com/user-attachments/assets/cfa534a4-68c4-4f22-a856-2c473bcc6956" />

Real device - A stop:
<img width="549" height="254" alt="image" src="https://github.com/user-attachments/assets/73c77a25-d13a-4a20-b866-3a9852d4666f" />

Real device - Stackers and track:
<img width="505" height="602" alt="image" src="https://github.com/user-attachments/assets/638d41e8-c275-4da2-b646-b958ae862a28" />


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
