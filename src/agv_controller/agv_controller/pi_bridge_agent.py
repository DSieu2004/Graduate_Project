#!/usr/bin/env python3
"""
Pi Cloud Bridge Agent
=====================
Runs on the Raspberry Pi.  Connects OUTBOUND to the FastAPI server's
/ws/robot-agent/{robot_code} endpoint and relays every message to/from
the LOCAL rosbridge WebSocket (ws://localhost:9090).

This lets the dashboard control the robot over the internet without
any port-forwarding or VPN on the Pi side.

Usage:
  python3 pi_bridge_agent.py --server wss://your-server.com --robot AMR_01

  # With auth token (if BRIDGE_TOKEN is set on the server):
  python3 pi_bridge_agent.py --server wss://your-server.com --robot AMR_01 --token mysecret

  # Custom local rosbridge port:
  python3 pi_bridge_agent.py --server wss://your-server.com --robot AMR_01 --rosbridge-port 9090

Requirements (Pi side):
  pip install websockets
  rosbridge_server must be running:  ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""

import argparse
import asyncio
import signal
import sys

try:
    import websockets
except ImportError:
    print(
        "ERROR: 'websockets' package required.  Install with:  pip install websockets"
    )
    sys.exit(1)

# ── Config ────────────────────────────────────────────────────────────────────

DEFAULT_ROSBRIDGE_PORT = 9090
RECONNECT_DELAY = 5  # seconds between reconnect attempts
MAX_MSG_SIZE = 16 * 1024 * 1024  # 16 MB (large maps)

shutdown_event = asyncio.Event()


# ── Bridge loop ───────────────────────────────────────────────────────────────


async def bridge(server_url: str, rosbridge_url: str):
    """
    Connect to both the cloud server and local rosbridge, then relay
    messages bidirectionally until one side disconnects.
    """
    print(f"[BRIDGE] Connecting to server:    {server_url}")
    print(f"[BRIDGE] Connecting to rosbridge: {rosbridge_url}")

    async with websockets.connect(
        server_url, max_size=MAX_MSG_SIZE, ping_interval=20, ping_timeout=40
    ) as ws_server, websockets.connect(rosbridge_url, max_size=MAX_MSG_SIZE) as ws_ros:
        print("[BRIDGE] Both connections established — relaying messages")

        async def server_to_ros():
            """Dashboard → server → Pi agent → local rosbridge."""
            try:
                async for msg in ws_server:
                    await ws_ros.send(msg)
            except websockets.ConnectionClosed:
                pass

        async def ros_to_server():
            """Local rosbridge → Pi agent → server → dashboard."""
            try:
                async for msg in ws_ros:
                    await ws_server.send(msg)
            except websockets.ConnectionClosed:
                pass

        done, pending = await asyncio.wait(
            [
                asyncio.create_task(server_to_ros()),
                asyncio.create_task(ros_to_server()),
            ],
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in pending:
            task.cancel()

    print("[BRIDGE] Session ended")


async def run_forever(server_url: str, rosbridge_url: str):
    """Keep reconnecting until the process is killed."""
    while not shutdown_event.is_set():
        try:
            await bridge(server_url, rosbridge_url)
        except (OSError, websockets.InvalidURI, websockets.InvalidHandshake) as exc:
            print(f"[BRIDGE] Connection failed: {exc}")
        except Exception as exc:
            print(f"[BRIDGE] Unexpected error: {exc}")

        if not shutdown_event.is_set():
            print(f"[BRIDGE] Reconnecting in {RECONNECT_DELAY}s …")
            try:
                await asyncio.wait_for(shutdown_event.wait(), timeout=RECONNECT_DELAY)
            except asyncio.TimeoutError:
                pass


# ── Entrypoint ────────────────────────────────────────────────────────────────


def parse_args():
    p = argparse.ArgumentParser(
        description="Pi Cloud Bridge Agent — relay local rosbridge to remote server"
    )
    p.add_argument(
        "--server",
        required=True,
        help="Server base URL, e.g. wss://myserver.com or ws://192.168.1.100:8000",
    )
    p.add_argument("--robot", required=True, help="Robot code, e.g. AMR_01")
    p.add_argument(
        "--token", default="", help="Bridge auth token (matches BRIDGE_TOKEN on server)"
    )
    p.add_argument(
        "--rosbridge-port",
        type=int,
        default=DEFAULT_ROSBRIDGE_PORT,
        help=f"Local rosbridge WebSocket port (default: {DEFAULT_ROSBRIDGE_PORT})",
    )
    return p.parse_args()


def main():
    args = parse_args()

    # Build URLs
    base = args.server.rstrip("/")
    token_qs = f"?token={args.token}" if args.token else ""
    server_url = f"{base}/ws/robot-agent/{args.robot}{token_qs}"
    rosbridge_url = f"ws://localhost:{args.rosbridge_port}"

    print("=" * 60)
    print("  Pi Cloud Bridge Agent")
    print(f"  Robot:     {args.robot}")
    print(f"  Server:    {server_url.split('?')[0]}")
    print(f"  Rosbridge: {rosbridge_url}")
    print("=" * 60)

    loop = asyncio.new_event_loop()

    # Graceful shutdown on Ctrl+C / SIGTERM
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, shutdown_event.set)
        except NotImplementedError:
            # Windows doesn't support add_signal_handler
            signal.signal(sig, lambda s, f: shutdown_event.set())

    try:
        loop.run_until_complete(run_forever(server_url, rosbridge_url))
    except KeyboardInterrupt:
        pass
    finally:
        print("[BRIDGE] Shutting down")
        loop.close()


if __name__ == "__main__":
    main()