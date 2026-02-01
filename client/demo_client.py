"""
Demo client for the Virtual StackLink simulator.

This script connects to a running Virtual StackLink TCP server and
performs a series of operations that mimic the basic plate transfer
sequence described in the StackLink programming introduction. Between
each command the client pauses for one second so you can observe
state changes in the web UI.

Usage:
    python demo_client.py

Ensure the server is running (e.g. `python server.py`) before
executing this script.
"""

import socket
import time


def send_command(conn: socket.socket, command: str) -> None:
    """Send a command to the server and print the response."""
    # Each command must be terminated by CRLF
    print(f"\n> {command}")
    conn.sendall((command + "\r\n").encode("utf-8"))
    # Read echo and response lines (at least two)
    echo = conn.recv(1024).decode("utf-8", errors="replace")
    # Print raw response
    print(echo.strip())


def main():
    host = "localhost"
    port = 7000
    commands = [
        # 1. Dispense a plate from stack 1 (lift 1 -> stop 3)
        "DISPENSE 1,1",
        # 2. Move the plate from stop 3 (lift 1) to stop 5 (instrument)
        "MOVEPLATE 1,3,5",
        # 3. Release the plate so the instrument can pick it up
        "IGNORESTOP 1,5,true",
        # 4. Capture the plate again after instrument has finished
        "IGNORESTOP 1,5,false",
        # 5. Move the plate from stop 5 back to lift 2 (stop 4)
        "MOVEPLATE 1,5,4",
        # 6. Push the plate back into stack 2 (lift 2)
        "RETURN 1,2",
    ]
    with socket.create_connection((host, port)) as conn:
        for cmd in commands:
            send_command(conn, cmd)
            time.sleep(1)
    print("\nDemo sequence complete.")


if __name__ == "__main__":
    main()