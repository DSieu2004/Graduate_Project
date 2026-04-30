"""
slam_manager_node.py — SLAM & Navigation lifecycle manager for the warehouse robot.

Provides topic-based control so the dashboard (via rosbridge) can
start/stop SLAM, save maps, list saved maps, and launch Nav2 with
a chosen map — all without SSH access to the Pi.

SLAM and Navigation are **mutually exclusive**: starting one will
automatically stop the other.

Subscribes:
    /slam/command  (std_msgs/String)
        "start_slam"           — launch async SLAM toolbox
        "stop_slam"            — kill SLAM process
        "save_map:<name>"      — save map to ~/maps/<name>.yaml + .pgm
        "list_maps"            — scan ~/maps/ and publish list
        "load_map:<name>"      — stop SLAM, launch Nav2 with ~/maps/<name>.yaml
        "stop_nav"             — kill Nav2 process

Publishes:
    /slam/status   (std_msgs/String)   — 1 Hz heartbeat
        "IDLE"                 — nothing running
        "MAPPING"              — SLAM active
        "SAVING"               — map save in progress
        "SAVED:<name>"         — map saved successfully
        "NAV:<name>"           — Nav2 running with <name>
        "LOADING:<name>"       — Nav2 starting up
        "ERROR:<message>"      — something went wrong

    /slam/map_list (std_msgs/String)   — JSON array of map names
        '["warehouse_map","office_floor"]'
"""

import json
import os
import signal
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SlamManager(Node):
    def __init__(self):
        super().__init__("slam_manager")

        self.status_pub = self.create_publisher(String, "/slam/status", 10)
        self.map_list_pub = self.create_publisher(String, "/slam/map_list", 10)
        self.create_subscription(String, "/slam/command", self.on_command, 10)

        # Publish status at 1 Hz so dashboard always has current state
        self.create_timer(1.0, self.publish_status)

        self.slam_process = None
        self.nav_process = None
        self.state = "IDLE"  # see docstring for values
        self.active_map = None  # map name when Nav2 is running

        # Ensure ~/maps directory exists
        self.maps_dir = os.path.expanduser("~/maps")
        os.makedirs(self.maps_dir, exist_ok=True)

        self.get_logger().info("SLAM Manager ready — listening on /slam/command")
        self.publish_status()

    # ── Status publisher ───────────────────────────────────────
    def publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)

    def set_state(self, new_state: str):
        self.state = new_state
        self.get_logger().info(f"State → {new_state}")
        self.publish_status()

    # ── Command handler ────────────────────────────────────────
    def on_command(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f"Command received: {cmd}")

        if cmd == "start_slam":
            self.start_slam()
        elif cmd == "stop_slam":
            self.stop_slam()
        elif cmd.startswith("save_map:"):
            filename = cmd.split(":", 1)[1].strip() or "warehouse_map"
            self.save_map(filename)
        elif cmd == "list_maps":
            self.list_maps()
        elif cmd.startswith("load_map:"):
            mapname = cmd.split(":", 1)[1].strip()
            if mapname:
                self.load_map(mapname)
            else:
                self.set_state("ERROR:no map name provided")
        elif cmd == "stop_nav":
            self.stop_nav()
        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

    # ── Helper: kill a process group ───────────────────────────
    def _kill_process(self, proc, label="process"):
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)
            self.get_logger().info(f"{label} terminated (SIGTERM)")
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                self.get_logger().info(f"{label} killed (SIGKILL)")
            except Exception:
                pass

    # ── Start SLAM ─────────────────────────────────────────────
    def start_slam(self):
        if self.slam_process is not None and self.slam_process.poll() is None:
            self.get_logger().warn("SLAM already running")
            self.set_state("MAPPING")
            return

        # Mutual exclusivity: stop Nav2 first
        if self.nav_process is not None and self.nav_process.poll() is None:
            self.get_logger().info("Stopping Nav2 before starting SLAM…")
            self._kill_process(self.nav_process, "Nav2")
            self.nav_process = None
            self.active_map = None

        try:
            self.slam_process = subprocess.Popen(
                ["ros2", "launch", "agv_controller", "handheld_slam.launch.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            self.set_state("MAPPING")
            self.get_logger().info(f"SLAM started (PID {self.slam_process.pid})")
        except Exception as e:
            self.set_state(f"ERROR:{e}")

    # ── Stop SLAM ──────────────────────────────────────────────
    def stop_slam(self):
        if self.slam_process is None or self.slam_process.poll() is not None:
            self.get_logger().info("SLAM not running — nothing to stop")
            self.slam_process = None
            self.set_state("IDLE")
            return

        self._kill_process(self.slam_process, "SLAM")
        self.slam_process = None
        self.set_state("IDLE")

    # ── Save Map ───────────────────────────────────────────────
    def save_map(self, filename: str):
        self.set_state("SAVING")

        def _do_save():
            filepath = os.path.join(self.maps_dir, filename)
            try:
                result = subprocess.run(
                    [
                        "ros2",
                        "run",
                        "nav2_map_server",
                        "map_saver_cli",
                        "-f",
                        filepath,
                        "--ros-args",
                        "-p",
                        "save_map_timeout:=5.0",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=15,
                )
                if result.returncode == 0:
                    self.set_state(f"SAVED:{filename}")
                    self.get_logger().info(f"Map saved → {filepath}.yaml")
                    # Publish updated list so dashboard sees the new map
                    self.list_maps()
                else:
                    err = result.stderr.strip()[:100] or "Unknown error"
                    self.set_state(f"ERROR:save failed — {err}")
            except subprocess.TimeoutExpired:
                self.set_state("ERROR:map save timed out")
            except Exception as e:
                self.set_state(f"ERROR:{e}")

        threading.Thread(target=_do_save, daemon=True).start()

    # ── List Maps ──────────────────────────────────────────────
    def list_maps(self):
        """Scan ~/maps/ for .yaml files and publish the list."""
        try:
            maps = sorted(
                os.path.splitext(f)[0]
                for f in os.listdir(self.maps_dir)
                if f.endswith(".yaml")
            )
        except Exception as e:
            self.get_logger().error(f"Failed to list maps: {e}")
            maps = []

        msg = String()
        msg.data = json.dumps(maps)
        self.map_list_pub.publish(msg)
        self.get_logger().info(f"Published map list: {maps}")

    # ── Load Map → Launch Nav2 ─────────────────────────────────
    def load_map(self, mapname: str):
        yaml_path = os.path.join(self.maps_dir, mapname + ".yaml")
        if not os.path.isfile(yaml_path):
            self.set_state(f"ERROR:map not found — {mapname}.yaml")
            return

        # Mutual exclusivity: stop SLAM first
        if self.slam_process is not None and self.slam_process.poll() is None:
            self.get_logger().info("Stopping SLAM before loading map…")
            self._kill_process(self.slam_process, "SLAM")
            self.slam_process = None

        # Stop existing Nav2 if running
        if self.nav_process is not None and self.nav_process.poll() is None:
            self.get_logger().info("Stopping previous Nav2…")
            self._kill_process(self.nav_process, "Nav2")
            self.nav_process = None

        self.set_state(f"LOADING:{mapname}")

        def _do_launch():
            try:
                self.nav_process = subprocess.Popen(
                    [
                        "ros2",
                        "launch",
                        "agv_controller",
                        "nav2_custom.launch.py",
                        f"map:={yaml_path}",
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    preexec_fn=os.setsid,
                )
                self.active_map = mapname
                self.get_logger().info(
                    f"Nav2 started with map '{mapname}' (PID {self.nav_process.pid})"
                )
                # Wait a moment for Nav2 to initialize, then set NAV state
                self.nav_process.poll()
                if self.nav_process.returncode is not None:
                    self.set_state(
                        f"ERROR:Nav2 exited immediately (code {self.nav_process.returncode})"
                    )
                    self.nav_process = None
                    self.active_map = None
                else:
                    self.set_state(f"NAV:{mapname}")
            except Exception as e:
                self.set_state(f"ERROR:{e}")
                self.nav_process = None
                self.active_map = None

        threading.Thread(target=_do_launch, daemon=True).start()

    # ── Stop Navigation ────────────────────────────────────────
    def stop_nav(self):
        if self.nav_process is None or self.nav_process.poll() is not None:
            self.get_logger().info("Nav2 not running — nothing to stop")
            self.nav_process = None
            self.active_map = None
            self.set_state("IDLE")
            return

        self._kill_process(self.nav_process, "Nav2")
        self.nav_process = None
        self.active_map = None
        self.set_state("IDLE")


def main(args=None):
    rclpy.init(args=args)
    node = SlamManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up any running processes
        if node.slam_process and node.slam_process.poll() is None:
            try:
                os.killpg(os.getpgid(node.slam_process.pid), signal.SIGTERM)
            except Exception:
                pass
        if node.nav_process and node.nav_process.poll() is None:
            try:
                os.killpg(os.getpgid(node.nav_process.pid), signal.SIGTERM)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
