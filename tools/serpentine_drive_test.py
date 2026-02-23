"""
Serpentine Track Drive Test with Real-Time Visualization

Drives the Pololu 3pi+ robot through the serpentine test pattern while
visualizing its position in real-time using matplotlib.  Connects to the
robot via the ESP32-C3 WiFi bridge (auto-discovery or manual IP).

Position data comes from the robot's own encoder/IMU odometry relayed
through the bridge, and optionally from the quadrascopic tracker.

Usage:
    python serpentine_drive_test.py                     # auto-discover bridge
    python serpentine_drive_test.py --ip 192.168.7.210  # manual IP
    python serpentine_drive_test.py --dry-run            # no robot, just show the track
    python serpentine_drive_test.py --csv pololu-robot.csv  # overlay ground truth

Author: Janis
"""

import argparse
import csv
import json
import logging
import math
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation

# ---------------------------------------------------------------------------
# Constants -- serpentine geometry (must match pattern_generator.py)
# ---------------------------------------------------------------------------

BOARD_W_MM = 1220.0
BOARD_H_MM = 800.0
MARGIN_MM = 100.0
TRACK_SPACING_MM = 100.0
TAPE_WIDTH_MM = 25.0
UTURN_RADIUS_MM = 50.0
RETURN_CLEARANCE_MM = 100.0

X_MIN = MARGIN_MM + RETURN_CLEARANCE_MM + UTURN_RADIUS_MM   # 250
X_MAX = BOARD_W_MM - MARGIN_MM - UTURN_RADIUS_MM - TAPE_WIDTH_MM / 2  # 1057.5
Y_MIN = MARGIN_MM + TAPE_WIDTH_MM / 2    # 112.5
Y_MAX = BOARD_H_MM - MARGIN_MM - TAPE_WIDTH_MM / 2   # 687.5

NUM_PASSES = int((Y_MAX - Y_MIN) / TRACK_SPACING_MM) + 1  # 6

# Track Y-coordinates
TRACK_YS = [Y_MIN + i * TRACK_SPACING_MM for i in range(NUM_PASSES)]
# 112.5, 212.5, 312.5, 412.5, 512.5, 612.5

# Starting point (on the exterior return leg)
START_X = MARGIN_MM + UTURN_RADIUS_MM  # 150
START_Y = Y_MIN                        # 112.5

# ---------------------------------------------------------------------------
# Network constants
# ---------------------------------------------------------------------------

DEFAULT_ROBOT_IP = "192.168.7.210"
DEFAULT_ROBOT_PORT = 5006
DISCOVERY_PORT = 5004
SOCKET_TIMEOUT = 0.1

# Drive parameters (defaults, overridable via --speed)
DEFAULT_DRIVE_SPEED_PCT = 50   # % of max motor speed for straight legs
UTURN_SPEED_PCT = 30           # % for U-turns
POSITION_POLL_HZ = 10          # how often to request position from robot

# Robot geometry (Pololu 3pi+ 32U4)
WHEEL_TRACK_MM = 96.0          # approximate distance between wheel centres
TURN_90_ARC_MM = (WHEEL_TRACK_MM / 2) * (math.pi / 2)  # ~75.4 mm encoder dist

# U-turn smooth-arc parameters (empirically tuned on test board)
# The robot traces a smooth semicircular arc instead of sharp
# point-turn + forward + point-turn.  Each step is a short forward
# followed by a small in-place turn.
# Left and right U-turns have separate tuning.
UTURN_LEFT_STEPS = 5             # fwd+turn iterations for LEFT U-turn
UTURN_LEFT_FWD_MM = 25.0         # forward distance per step (mm)
UTURN_LEFT_TURN_DEG = 16.0       # turn angle per step (degrees)
UTURN_LEFT_EXTRA_FWD_MM = 15.0   # extra forward after arc completes

UTURN_RIGHT_STEPS = 5            # fwd+turn iterations for RIGHT U-turn
UTURN_RIGHT_FWD_MM = 25.0        # forward distance per step (mm)
UTURN_RIGHT_TURN_DEG = 16.0      # turn angle per step (degrees)
UTURN_RIGHT_EXTRA_DEG = 0.0      # extra CW correction after right arc
UTURN_RIGHT_EXTRA_FWD_MM = 15.0  # extra forward after arc completes

UTURN_ARC_FWD_SPEED_PCT = 30     # % speed for arc forward segments
UTURN_ARC_TURN_SPEED_PCT = 40    # % speed for arc turn segments

# Line-follow distances (empirical -- tape is shorter than board geometry)
LF_STRAIGHT_DIST_MM = 720.0      # tape length between U-turns
LF_SEARCH_MM = 200.0             # search distance when acquiring line

# Return path parameters (exterior return along board left edge)
# The return path consists of:
#   1. Approach leg:  continue along last track from X_MIN to START_X
#   2. Arc 1:         quarter-circle LEFT arc (heading 180 -> -90)
#   3. Straight leg:  corridor at X = MARGIN_MM from Y ~562 to Y ~162
#   4. Arc 2:         quarter-circle LEFT arc (heading -90  -> 0)
# Arc tuning from empirical tests: 3 x (fwd 38 mm + left 12 deg) ~ 90 deg.
RETURN_ARC_STEPS = 3              # fwd+turn iterations per quarter-circle
RETURN_ARC_FWD_MM = 38.0          # forward distance per step (mm)
RETURN_ARC_TURN_DEG = 12.0        # left-turn angle per step (degrees)
RETURN_CORRIDOR_X = MARGIN_MM     # 100 mm -- X position of the return corridor
RETURN_APPROACH_DIST_MM = X_MIN - START_X   # 100 mm approach to arc 1 entry
RETURN_STRAIGHT_DIST_MM = (                 # ~400 mm corridor between arcs
    (TRACK_YS[-1] - UTURN_RADIUS_MM) - (TRACK_YS[0] + UTURN_RADIUS_MM)
)

# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class Waypoint:
    """A target the robot drives toward."""
    x_mm: float
    y_mm: float
    label: str = ""


@dataclass
class OdometryRecord:
    """Timestamped position sample."""
    t: float            # time.time()
    x_mm: float
    y_mm: float
    heading_deg: float
    source: str = "robot"   # "robot" or "tracker"


# ---------------------------------------------------------------------------
# Bridge discovery (standalone, no dependency on the RPi robot_controller.py)
# ---------------------------------------------------------------------------

def discover_bridge(timeout: float = 5.0) -> Optional[Tuple[str, int]]:
    """Listen for ESP32-C3 discovery beacons on the local network.

    Returns (ip, data_port) or None.
    """
    logger = logging.getLogger("discovery")
    logger.info("Listening for discovery beacons on port %d (%.0fs) ...",
                DISCOVERY_PORT, timeout)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.5)
    try:
        # Bind to all interfaces so we receive broadcasts on any subnet
        sock.bind(("0.0.0.0", DISCOVERY_PORT))
    except OSError as exc:
        logger.error("Cannot bind port %d: %s  "
                     "(is another instance running?)", DISCOVERY_PORT, exc)
        sock.close()
        return None

    deadline = time.time() + timeout
    result = None
    packets_received = 0

    try:
        while time.time() < deadline:
            try:
                data, addr = sock.recvfrom(1024)
                packets_received += 1
                raw = data.decode()
                logger.debug("RX from %s:%d -> %s", addr[0], addr[1],
                             raw[:200])
                msg = json.loads(raw)
            except socket.timeout:
                continue
            except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                logger.debug("Non-JSON packet from %s: %s", addr, exc)
                continue

            if msg.get("type") != "discovery":
                logger.debug("Ignoring type=%s from %s",
                             msg.get("type"), addr[0])
                continue
            if msg.get("service") != "pololu-3pi-bridge":
                logger.debug("Ignoring service=%s from %s",
                             msg.get("service"), addr[0])
                continue

            bridge_ip = msg.get("ip", addr[0])
            bridge_port = msg.get("port", DEFAULT_ROBOT_PORT)
            logger.info("Discovered bridge at %s:%d  (RSSI %s, MAC %s)",
                        bridge_ip, bridge_port,
                        msg.get("rssi", "?"), msg.get("mac", "?"))

            # Send acknowledgement
            ack = json.dumps({"cmd": "discover_ack"}).encode()
            sock.sendto(ack, addr)
            logger.debug("Sent discover_ack to %s:%d", *addr)

            # Wait briefly for confirmation
            ack_deadline = time.time() + 1.0
            while time.time() < ack_deadline:
                try:
                    cdata, _ = sock.recvfrom(1024)
                    cmsg = json.loads(cdata.decode())
                    if cmsg.get("type") == "discover_confirm":
                        logger.info("Bridge confirmed discovery handshake")
                        break
                except (socket.timeout, json.JSONDecodeError):
                    continue

            result = (bridge_ip, bridge_port)
            break
    finally:
        sock.close()

    if result is None:
        logger.warning("No beacon received (packets seen: %d). "
                       "Check that the ESP32 and this PC are on the "
                       "same WiFi network/subnet.", packets_received)
    return result


# ---------------------------------------------------------------------------
# Lightweight robot client (self-contained for this tool)
# ---------------------------------------------------------------------------

class SimpleRobotClient:
    """Minimal UDP client for commanding the 3pi+ through the ESP32 bridge.

    Handles sending JSON commands and receiving responses on a background
    thread.  Records every position update for the visualization.
    """

    def __init__(self, ip: str, port: int):
        self.logger = logging.getLogger("robot")
        self.ip = ip
        self.port = port

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(SOCKET_TIMEOUT)

        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._connected = False
        self._sequence = 0

        # Position state
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.heading_deg = 0.0
        self._odom_lock = threading.Lock()

        # Precision-move completion
        self._pmove_done = threading.Event()
        self._pmove_result: Optional[Dict] = None

        # Recorded trajectory
        self.trajectory: List[OdometryRecord] = []
        self._traj_lock = threading.Lock()

        # All received messages (for debugging log)
        self.message_log: List[Dict] = []

        # Heartbeat tracking
        self.last_heartbeat_t = 0.0
        self.battery_mv = 0

        # Ack tracking (for verifying commands reach the robot)
        self._last_ack_cmd = ""
        self._ack_event = threading.Event()
        self._robot_pong = threading.Event()

    # -- lifecycle --

    def start(self) -> None:
        self._running = True
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name="robot-rx")
        self._rx_thread.start()

    def stop(self) -> None:
        self.send({"cmd": "stop"})
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=2.0)
        self._sock.close()

    @property
    def is_connected(self) -> bool:
        if not self._connected:
            return False
        return (time.time() - self.last_heartbeat_t) < 10.0

    # -- send --

    def send(self, cmd: Dict) -> bool:
        try:
            # Use compact separators (no spaces) so the ESP32 bridge's
            # strstr() checks match the JSON key/value pairs exactly.
            payload = json.dumps(cmd, separators=(',', ':')).encode()
            self._sock.sendto(payload, (self.ip, self.port))
            self._sequence += 1
            return True
        except Exception as exc:
            self.logger.error("Send failed: %s", exc)
            return False

    def ping(self) -> bool:
        return self.send({
            "cmd": "ping",
            "seq": self._sequence,
            "ts": int(time.time() * 1000),
        })

    def bridge_ping(self) -> bool:
        """Ping the ESP32 bridge directly (no UART forwarding to robot)."""
        return self.send({
            "cmd": "bridge_ping",
            "seq": self._sequence,
            "ts": int(time.time() * 1000),
        })

    def reset_position(self) -> bool:
        return self.send({"cmd": "resetpos"})

    def get_position(self) -> bool:
        return self.send({"cmd": "getpos"})

    def precision_move(self, direction: str, speed_pct: int,
                       distance_mm: float = 0, duration_ms: int = 0) -> bool:
        cmd: Dict = {
            "cmd": "pmove",
            "dir": direction,
            "speed": max(1, min(100, speed_pct)),
        }
        if distance_mm > 0:
            cmd["dist"] = int(distance_mm)
        elif duration_ms > 0:
            cmd["dur"] = duration_ms
        else:
            return False
        self._pmove_done.clear()
        self._pmove_result = None
        return self.send(cmd)

    def line_follow(self, speed_pct: int,
                    distance_mm: float = 0, duration_ms: int = 0,
                    search_mm: float = 0) -> bool:
        """Start a line-follow move using the front IR sensors for tracking.

        The robot must have been calibrated first (calibrate_line_sensors).
        Completion is signalled by an lf_done message -- use wait_for_pmove()
        which also listens for lf_done.

        If *search_mm* > 0, the robot first drives slowly forward up to that
        distance looking for the line.  PID tracking only starts once the
        line is confidently detected.  This is essential after U-turns where
        the robot may not be perfectly centred on the next track.
        """
        cmd: Dict = {
            "cmd": "linefollow",
            "speed": max(1, min(100, speed_pct)),
        }
        if distance_mm > 0:
            cmd["dist"] = int(distance_mm)
        elif duration_ms > 0:
            cmd["dur"] = duration_ms
        else:
            return False
        if search_mm > 0:
            cmd["search"] = int(search_mm)
        self._pmove_done.clear()
        self._pmove_result = None
        return self.send(cmd)

    def wait_for_pmove(self, timeout: float = 30.0) -> Optional[Dict]:
        if self._pmove_done.wait(timeout):
            return self._pmove_result
        self.logger.warning("pmove timed out (%.1fs)", timeout)
        return None

    def set_heading(self, heading_deg: float) -> bool:
        """Set the robot's heading to a specific value (corrects gyro drift)."""
        return self.send({"cmd": "setheading", "h": int(heading_deg)})

    def calibrate_line_sensors(self) -> bool:
        return self.send({"cmd": "calibrate", "sensor": "line"})

    def wait_for_connection(self, timeout: float = 5.0) -> bool:
        """Block until a response arrives from the bridge.

        Sends bridge_ping (handled by the ESP32 itself, does NOT depend
        on the 3pi+ robot UART) and retries every 0.5 s.
        """
        self.bridge_ping()
        deadline = time.time() + timeout
        next_ping = time.time() + 0.5
        while time.time() < deadline:
            if self.is_connected:
                return True
            if time.time() >= next_ping:
                self.bridge_ping()
                next_ping = time.time() + 0.5
            time.sleep(0.05)
        return self.is_connected

    def verify_robot(self, timeout: float = 3.0) -> bool:
        """Verify that the 3pi+ robot is reachable through the UART bridge.

        Sends a regular 'ping' (forwarded by the ESP32 to the robot via
        UART) and waits for the 'pong' response -- proving the full
        WiFi -> ESP32 -> UART -> 3pi+ -> UART -> ESP32 -> WiFi path.
        """
        self._robot_pong.clear()
        self.ping()
        deadline = time.time() + timeout
        next_ping = time.time() + 0.8
        while time.time() < deadline:
            if self._robot_pong.wait(timeout=0.1):
                return True
            if time.time() >= next_ping:
                self.ping()
                next_ping = time.time() + 0.8
        return False

    def wait_for_ack(self, cmd: str, timeout: float = 5.0) -> bool:
        """Wait until the robot sends an ack for *cmd*."""
        self._ack_event.clear()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._ack_event.wait(timeout=0.1):
                if self._last_ack_cmd == cmd:
                    return True
                self._ack_event.clear()
        return False

    # -- receive --

    def _rx_loop(self) -> None:
        while self._running:
            try:
                data, addr = self._sock.recvfrom(2048)
                raw = data.decode()
                self.logger.debug("RX from %s: %s", addr, raw[:200])
                msg = json.loads(raw)
                self._dispatch(msg)
            except socket.timeout:
                continue
            except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                self.logger.debug("RX decode error: %s  data=%r",
                                  exc, data[:100] if data else b"")
                continue
            except OSError:
                break

    def _dispatch(self, msg: Dict) -> None:
        self.message_log.append(msg)
        self._connected = True
        self.last_heartbeat_t = time.time()

        msg_type = msg.get("type", "")

        if msg_type == "position":
            self._update_position(msg.get("x", 0), msg.get("y", 0),
                                  msg.get("h", 0))
        elif msg_type == "pmove_done":
            pos = msg.get("pos", {})
            self._update_position(pos.get("x", 0), pos.get("y", 0),
                                  pos.get("h", 0))
            self._pmove_result = msg
            self._pmove_done.set()
            self.logger.info("pmove_done: dist=%.1f  pos=(%.1f, %.1f)",
                             msg.get("dist", 0),
                             pos.get("x", 0), pos.get("y", 0))
        elif msg_type == "lf_done":
            pos = msg.get("pos", {})
            self._update_position(pos.get("x", 0), pos.get("y", 0),
                                  pos.get("h", 0))
            self._pmove_result = msg
            self._pmove_done.set()
            reason = msg.get("reason", "complete")
            self.logger.info("lf_done (%s): dist=%.1f  pos=(%.1f, %.1f)",
                             reason, msg.get("dist", 0),
                             pos.get("x", 0), pos.get("y", 0))
        elif msg_type == "lf_status":
            # Periodic line-follow telemetry -- update position
            pos = msg.get("pos", {})
            if pos:
                self._update_position(pos.get("x", 0), pos.get("y", 0),
                                      pos.get("h", 0))
            self.logger.info("lf_status: line=%d err=%d sns=%s spd=%s "
                             "dist=%.1f h=%.1f",
                             msg.get("line", 0), msg.get("err", 0),
                             msg.get("sns", []),
                             msg.get("spd", []), msg.get("dist", 0),
                             pos.get("h", 0))
        elif msg_type == "heartbeat":
            self.battery_mv = msg.get("battery", 0)
        elif msg_type == "bridge_status":
            self.logger.debug("Bridge status: IP=%s RSSI=%s heap=%s",
                              msg.get("ip", "?"), msg.get("rssi", "?"),
                              msg.get("heap", "?"))
        elif msg_type == "bridge_pong":
            client_ts = msg.get("client_ts", 0)
            now_ms = int(time.time() * 1000)
            rtt = now_ms - client_ts if client_ts else 0
            self.logger.debug("Bridge pong: RTT=%d ms", rtt)
        elif msg_type == "ack":
            self._last_ack_cmd = msg.get("cmd", "")
            self._ack_event.set()
            self.logger.debug("Ack: cmd=%s", self._last_ack_cmd)
        elif msg_type == "pong":
            client_ts = msg.get("client_ts", 0)
            rtt = int(time.time() * 1000) - client_ts if client_ts else 0
            self.logger.debug("Pong RTT=%d ms", rtt)
            self._robot_pong.set()
        elif msg_type == "event":
            event = msg.get("event", "")
            if event == "bump":
                self.logger.warning("BUMP %s -- motors stopped",
                                    msg.get("side", ""))
            elif event == "search_ok":
                self.logger.info("Line search: found at %.1f mm",
                                 msg.get("d", 0))
            elif event == "line_lost":
                self.logger.warning("Line lost (sensors below threshold)")
            elif event == "line_found":
                self.logger.info("Line re-acquired")
            else:
                self.logger.info("Event: %s", event)
        elif msg_type == "error":
            self.logger.warning("Robot error: %s", msg.get("error", "?"))

    def _update_position(self, x: float, y: float, h: float) -> None:
        with self._odom_lock:
            self.x_mm = x
            self.y_mm = y
            self.heading_deg = h
        rec = OdometryRecord(
            t=time.time(), x_mm=x, y_mm=y, heading_deg=h, source="robot")
        with self._traj_lock:
            self.trajectory.append(rec)


# ---------------------------------------------------------------------------
# Serpentine path builder
# ---------------------------------------------------------------------------

def build_serpentine_waypoints() -> List[Waypoint]:
    """Compute the sequence of waypoints for the serpentine traversal.

    The robot is assumed to start at (START_X, START_Y), already on the
    first track.  Waypoints are the endpoints of each straight leg and the
    exit points of each U-turn.

    For simplicity the U-turns are commanded as timed turns (dur) rather
    than distance, since the 3pi firmware's pmove does not support arcs.
    We break U-turns into: turn 90 deg -> move 100 mm -> turn 90 deg.
    """
    waypoints: List[Waypoint] = []

    for i in range(NUM_PASSES):
        y = TRACK_YS[i]
        left_to_right = (i % 2 == 0)

        if left_to_right:
            # Drive straight right
            dist = X_MAX - (X_MIN if i > 0 else START_X)
            waypoints.append(Waypoint(X_MAX, y, f"pass{i+1}-end-R"))
        else:
            # Drive straight left
            dist = X_MAX - X_MIN
            waypoints.append(Waypoint(X_MIN, y, f"pass{i+1}-end-L"))

        # U-turn to next track (except after the last pass)
        if i < NUM_PASSES - 1:
            next_y = TRACK_YS[i + 1]
            if left_to_right:
                waypoints.append(Waypoint(X_MAX, next_y,
                                          f"uturn{i+1}-exit-R"))
            else:
                waypoints.append(Waypoint(X_MIN, next_y,
                                          f"uturn{i+1}-exit-L"))

    return waypoints


def compute_move_for_waypoint(
    current_x: float, current_y: float, current_h: float,
    target: Waypoint,
) -> List[Tuple[str, int, float, int]]:
    """Return a list of (direction, speed_pct, distance_mm, duration_ms)
    commands to reach *target* from the current pose.

    Simple strategy:
      - If same Y: move forward/backward along X
      - If same X, different Y: this is a U-turn segment, move along Y
    """
    dx = target.x_mm - current_x
    dy = target.y_mm - current_y
    moves = []

    if abs(dy) < 1.0:
        # Horizontal leg
        dist = abs(dx)
        direction = "w" if dx > 0 else "s"
        moves.append((direction, DEFAULT_DRIVE_SPEED_PCT, dist, 0))
    elif abs(dx) < 1.0:
        # Vertical segment of a U-turn
        # The 3pi+ precision move uses 'w' for forward and 's' for backward.
        # We need to turn first, then move straight, then turn back.
        # Simplified: use timed moves for the 180-degree U-turn.
        dist = abs(dy)
        # Just command a forward move covering the Y distance --
        # the real robot heading matters here, but since pmove corrects heading
        # we trust the IMU to keep us straight after each turn command.
        direction = "w"
        moves.append((direction, UTURN_SPEED_PCT, dist, 0))
    else:
        # Diagonal -- should not happen in our serpentine
        dist = math.sqrt(dx * dx + dy * dy)
        moves.append(("w", DEFAULT_DRIVE_SPEED_PCT, dist, 0))

    return moves


# ---------------------------------------------------------------------------
# Ground-truth CSV loader
# ---------------------------------------------------------------------------

def load_ground_truth(csv_path: str) -> Tuple[List[float], List[float]]:
    """Load the X, Y columns from the pattern ground-truth CSV."""
    xs, ys = [], []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row["x_mm"]))
            ys.append(float(row["y_mm"]))
    return xs, ys


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------

class SerpentineVisualizer:
    """Real-time matplotlib visualization of the serpentine track and robot."""

    def __init__(self, robot: Optional[SimpleRobotClient] = None,
                 ground_truth_csv: Optional[str] = None):
        self.robot = robot
        self.gt_csv = ground_truth_csv
        self._stop_event = threading.Event()

        # Trajectory data (thread-safe copy for plotting)
        self._plot_xs: List[float] = []
        self._plot_ys: List[float] = []

    def run(self) -> None:
        """Show the matplotlib window and start the animation loop."""
        fig, ax = plt.subplots(figsize=(14, 8))
        fig.canvas.manager.set_window_title("Serpentine Drive Test")
        self._setup_track(ax)
        self._setup_legend(ax, fig)

        # Robot trajectory line (updated by animation)
        (traj_line,) = ax.plot([], [], "b-", linewidth=1.5, alpha=0.8,
                               label="Robot odometry")
        # Current position marker
        (pos_marker,) = ax.plot([], [], "bo", markersize=8)
        # Heading indicator
        (heading_line,) = ax.plot([], [], "b-", linewidth=2)

        # Status text
        status_text = ax.text(
            0.02, 0.98, "", transform=ax.transAxes,
            fontsize=9, verticalalignment="top", fontfamily="monospace",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))

        def update(frame):
            if self.robot:
                with self.robot._traj_lock:
                    self._plot_xs = [r.x_mm + START_X
                                     for r in self.robot.trajectory]
                    self._plot_ys = [r.y_mm + START_Y
                                     for r in self.robot.trajectory]

                traj_line.set_data(self._plot_xs, self._plot_ys)

                with self.robot._odom_lock:
                    cx = self.robot.x_mm + START_X
                    cy = self.robot.y_mm + START_Y
                    ch = self.robot.heading_deg

                pos_marker.set_data([cx], [cy])

                # Heading arrow (30 mm long)
                hrad = math.radians(ch)
                hx = cx + 30.0 * math.cos(hrad)
                hy = cy + 30.0 * math.sin(hrad)
                heading_line.set_data([cx, hx], [cy, hy])

                conn = "OK" if self.robot.is_connected else "LOST"
                n = len(self._plot_xs)
                bat = self.robot.battery_mv
                status_text.set_text(
                    f"Connection: {conn}   Battery: {bat} mV\n"
                    f"Position: ({cx:.1f}, {cy:.1f}) mm  "
                    f"Heading: {ch:.1f} deg\n"
                    f"Samples: {n}")

            return traj_line, pos_marker, heading_line, status_text

        self._anim = FuncAnimation(fig, update, interval=100, blit=False)
        plt.tight_layout()
        plt.show()

    def _setup_track(self, ax) -> None:
        """Draw the serpentine track, board outline, markers."""
        # Board outline
        board = plt.Rectangle((0, 0), BOARD_W_MM, BOARD_H_MM,
                               linewidth=2, edgecolor="black",
                               facecolor="#f5f5dc", zorder=0)
        ax.add_patch(board)

        # Margin rectangle
        margin = plt.Rectangle(
            (MARGIN_MM, MARGIN_MM),
            BOARD_W_MM - 2 * MARGIN_MM, BOARD_H_MM - 2 * MARGIN_MM,
            linewidth=0.5, edgecolor="gray", facecolor="none",
            linestyle="--", zorder=1)
        ax.add_patch(margin)

        # Serpentine tracks (straight segments)
        for i in range(NUM_PASSES):
            y = TRACK_YS[i]
            left_to_right = (i % 2 == 0)
            x_start = X_MIN if i > 0 else START_X
            x_end = X_MAX
            if not left_to_right:
                x_start, x_end = X_MAX, X_MIN
            ax.plot([x_start if left_to_right else X_MAX,
                     x_end if left_to_right else X_MIN],
                    [y, y], color="gray", linewidth=TAPE_WIDTH_MM * 0.3,
                    alpha=0.4, zorder=2)
            # Direction arrow
            mid_x = (X_MIN + X_MAX) / 2
            dx = 20 if left_to_right else -20
            ax.annotate("", xy=(mid_x + dx, y), xytext=(mid_x - dx, y),
                        arrowprops=dict(arrowstyle="->", color="gray",
                                        lw=1.5))
            ax.text(X_MIN - 40, y, f"P{i+1}", fontsize=8, ha="right",
                    va="center", color="gray")

        # U-turn arcs (visual only)
        for i in range(NUM_PASSES - 1):
            y_start = TRACK_YS[i]
            y_end = TRACK_YS[i + 1]
            cy = (y_start + y_end) / 2
            left_to_right = (i % 2 == 0)
            if left_to_right:
                cx = X_MAX
                theta1, theta2 = -90, 90
            else:
                cx = X_MIN
                theta1, theta2 = 90, 270
            arc = mpatches.Arc((cx, cy), UTURN_RADIUS_MM * 2,
                               UTURN_RADIUS_MM * 2,
                               angle=0, theta1=theta1, theta2=theta2,
                               color="gray", linewidth=1.5, alpha=0.4,
                               zorder=2)
            ax.add_patch(arc)

        # Sync markers (red dots at U-turn entry/exit)
        sync_coords = []
        for i in range(NUM_PASSES - 1):
            left_to_right = (i % 2 == 0)
            x = X_MAX if left_to_right else X_MIN
            sync_coords.append((x, TRACK_YS[i]))
            sync_coords.append((x, TRACK_YS[i + 1]))
        for sx, sy in sync_coords:
            ax.plot(sx, sy, "r+", markersize=10, markeredgewidth=2, zorder=5)

        # ArUco marker positions (corners)
        aruco_positions = [
            (50, 50, 0), (BOARD_W_MM - 50, 50, 1),
            (50, BOARD_H_MM - 50, 2), (BOARD_W_MM - 50, BOARD_H_MM - 50, 3),
        ]
        for ax_x, ay, aid in aruco_positions:
            rect = plt.Rectangle((ax_x - 30, ay - 30), 60, 60,
                                  linewidth=1, edgecolor="black",
                                  facecolor="lightgray", zorder=3)
            ax.add_patch(rect)
            ax.text(ax_x, ay, f"A{aid}", fontsize=7, ha="center",
                    va="center", zorder=4)

        # Start position
        ax.plot(START_X, START_Y, "g^", markersize=12, zorder=6,
                label="Start")

        # Ground truth from CSV
        if self.gt_csv and os.path.isfile(self.gt_csv):
            gx, gy = load_ground_truth(self.gt_csv)
            ax.plot(gx, gy, color="green", linewidth=1, alpha=0.5,
                    linestyle="--", label="Ground truth", zorder=2)

        ax.set_xlim(-20, BOARD_W_MM + 20)
        ax.set_ylim(-20, BOARD_H_MM + 20)
        ax.set_aspect("equal")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_title("Serpentine Track -- Robot Odometry Visualization")
        ax.grid(True, alpha=0.2)

    def _setup_legend(self, ax, fig) -> None:
        ax.legend(loc="upper right", fontsize=8)


# ---------------------------------------------------------------------------
# Drive executor
# ---------------------------------------------------------------------------

class SerpentineDriveExecutor:
    """Commands the robot through the serpentine path in a background thread.

    The executor issues precision-move commands for each straight leg and
    simplified U-turn maneuvers between tracks.
    """

    def __init__(self, robot: SimpleRobotClient,
                 speed_pct: int = DEFAULT_DRIVE_SPEED_PCT,
                 use_line_follow: bool = True):
        self.robot = robot
        self.speed_pct = speed_pct
        self.use_line_follow = use_line_follow
        self.logger = logging.getLogger("drive")
        self._thread: Optional[threading.Thread] = None
        self._abort = threading.Event()

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="drive-exec")
        self._thread.start()

    def abort(self) -> None:
        self._abort.set()
        self.robot.send({"cmd": "stop"})

    def _run(self) -> None:
        """Execute the full serpentine traversal."""
        robot = self.robot

        self.logger.info("=== Serpentine drive starting ===")

        # ---- Pre-flight: verify robot is reachable via UART ----
        self.logger.info("Verifying robot UART link (ping -> pong) ...")
        if not robot.verify_robot(timeout=4.0):
            self.logger.error(
                "Robot is NOT responding via UART!\n"
                "  The ESP32 bridge is reachable, but the 3pi+ robot\n"
                "  did not reply to a forwarded ping.\n"
                "  Check:\n"
                "    1. 3pi+ is powered on (green power LED)\n"
                "    2. UART wiring: ESP32 GPIO5 <-> 3pi+ Pin 0,\n"
                "                    ESP32 GPIO4 <-> 3pi+ Pin 1,\n"
                "                    GND <-> GND\n"
                "    3. 3pi+ firmware is flashed (pio run -e pololu-3pi -t upload)")
            return
        self.logger.info("Robot UART link OK (battery %d mV)", robot.battery_mv)

        # Reset odometry
        robot.reset_position()
        if not robot.wait_for_ack("resetpos", timeout=2.0):
            self.logger.warning("No ack for resetpos (continuing)")
        time.sleep(0.2)

        # Calibrate line sensors before driving if using line-follow
        if self.use_line_follow:
            cal_ok = False
            for attempt in range(3):
                self.logger.info("Calibrating line sensors (robot will spin)"
                                 " ... [attempt %d/3]", attempt + 1)
                robot.calibrate_line_sensors()
                if robot.wait_for_ack("calibrate", timeout=10.0):
                    self.logger.info(
                        "Line sensor calibration confirmed by robot")
                    cal_ok = True
                    break
                self.logger.warning(
                    "No ack for calibrate (attempt %d/3)", attempt + 1)
                time.sleep(0.5)
            if not cal_ok:
                self.logger.error(
                    "Calibration failed after 3 attempts.  Check UART "
                    "wiring and try running with -v for diagnostics.")
                return

        # The firmware resets heading + position after calibration, so
        # no separate resetpos needed.  Sync heading from robot:
        time.sleep(0.3)
        self._poll_position()

        # Build waypoints
        waypoints = build_serpentine_waypoints()
        self.logger.info("Waypoints: %d", len(waypoints))

        # IMPORTANT: Waypoints are in board coordinates (mm from board corner).
        # The robot's encoder odometry is in its *own* local frame (0,0 at
        # power-on/reset) and will drift.  We CANNOT compare robot odometry
        # x/y against board waypoints.  Instead we use the *planned* board
        # position (we know where on the board the robot is after each
        # commanded segment) and only read the robot's gyro heading (which
        # is corrected after each segment via setheading).
        current_x = START_X   # planned board position
        current_y = START_Y
        current_h = 0.0       # nominal heading after calibration reset
        self.logger.info("Start pose: planned=(%.1f, %.1f) h=%.1f",
                         current_x, current_y, current_h)

        for idx, wp in enumerate(waypoints):
            if self._abort.is_set():
                self.logger.warning("Drive aborted")
                return

            self.logger.info("[%d/%d] Target: (%7.1f, %7.1f) %s",
                             idx + 1, len(waypoints),
                             wp.x_mm, wp.y_mm, wp.label)

            dx = wp.x_mm - current_x
            dy = wp.y_mm - current_y

            if abs(dy) < 1.0:
                # ------- Horizontal straight leg -------
                dist = abs(dx)
                if dist < 1.0:
                    continue

                # Determine if we need to face right (+X) or left (-X)
                need_heading = 0.0 if dx > 0 else 180.0

                # If heading differs significantly, do a point turn first
                heading_diff = need_heading - current_h
                while heading_diff > 180:
                    heading_diff -= 360
                while heading_diff < -180:
                    heading_diff += 360

                if abs(heading_diff) > 5.0:
                    self._point_turn(heading_diff)
                    time.sleep(0.2)
                    # Correct heading to expected after turn
                    robot.set_heading(need_heading)
                    time.sleep(0.15)
                    current_h = need_heading

                # Drive straight -- use line-follow if enabled
                lf_dist = LF_STRAIGHT_DIST_MM if self.use_line_follow else dist
                self.logger.info("  Straight %.1f mm  heading=%.1f  mode=%s",
                                 lf_dist, current_h,
                                 "line-follow" if self.use_line_follow
                                 else "pmove")
                if self.use_line_follow:
                    robot.line_follow(self.speed_pct,
                                      distance_mm=lf_dist,
                                      search_mm=LF_SEARCH_MM)
                else:
                    robot.precision_move("w", self.speed_pct,
                                        distance_mm=dist)
                result = robot.wait_for_pmove(60.0)
                if result is None:
                    self.logger.error("  Move timeout -- stopping robot")
                    robot.send({"cmd": "stop"})
                    time.sleep(0.3)
                elif result.get("reason") == "no_line":
                    self.logger.error("  Line not found during search -- "
                                      "stopping")
                    robot.send({"cmd": "stop"})
                    time.sleep(0.3)

                # CRITICAL: Correct gyro heading after line-follow.
                # During line-follow the PID steers via differential motors,
                # but the gyro heading drifts freely and becomes unreliable.
                # Set it back to the expected value so U-turns work.
                robot.set_heading(need_heading)
                time.sleep(0.15)
                current_h = need_heading

            else:
                # ------- U-turn (smooth arc) -------
                # Trace a smooth arc: repeat (fwd + small turn) N times.
                # Direction depends on current heading:
                #   heading ~0   (was going right) -> LEFT  U-turn (CCW)
                #   heading ~180 (was going left)  -> RIGHT U-turn (CW)
                turn_left = (abs(current_h) < 90)
                turn_dir = "a" if turn_left else "d"

                # Select per-direction parameters
                if turn_left:
                    n_steps = UTURN_LEFT_STEPS
                    step_fwd = UTURN_LEFT_FWD_MM
                    step_deg = UTURN_LEFT_TURN_DEG
                    extra_fwd = UTURN_LEFT_EXTRA_FWD_MM
                else:
                    n_steps = UTURN_RIGHT_STEPS
                    step_fwd = UTURN_RIGHT_FWD_MM
                    step_deg = UTURN_RIGHT_TURN_DEG
                    extra_fwd = UTURN_RIGHT_EXTRA_FWD_MM

                # Determine heading for the NEXT straight leg
                next_pass_idx = idx + 1
                if next_pass_idx < len(waypoints):
                    next_dx = waypoints[next_pass_idx].x_mm - wp.x_mm
                    next_heading = 0.0 if next_dx > 0 else 180.0
                else:
                    next_heading = current_h

                self.logger.info(
                    "  U-turn: %s arc (%d x [fwd %.0f mm + turn %.0f deg])"
                    " + fwd %.0f mm%s",
                    "LEFT" if turn_left else "RIGHT",
                    n_steps, step_fwd, step_deg, extra_fwd,
                    f" + extra {UTURN_RIGHT_EXTRA_DEG:.0f} deg"
                    if not turn_left else "")

                arc_mm = TURN_90_ARC_MM * step_deg / 90.0
                for step in range(n_steps):
                    # Forward segment
                    robot.precision_move("w", UTURN_ARC_FWD_SPEED_PCT,
                                         distance_mm=step_fwd)
                    robot.wait_for_pmove(10.0)
                    # Turn segment
                    robot.precision_move(turn_dir, UTURN_ARC_TURN_SPEED_PCT,
                                         distance_mm=arc_mm)
                    robot.wait_for_pmove(10.0)

                # Right U-turns need a small extra correction
                if not turn_left and UTURN_RIGHT_EXTRA_DEG > 0:
                    extra_arc = (TURN_90_ARC_MM
                                 * UTURN_RIGHT_EXTRA_DEG / 90.0)
                    robot.precision_move("d", UTURN_ARC_TURN_SPEED_PCT,
                                         distance_mm=extra_arc)
                    robot.wait_for_pmove(10.0)

                # Extra forward to clear the arc and reach the next tape
                if extra_fwd > 0:
                    robot.precision_move("w", UTURN_ARC_FWD_SPEED_PCT,
                                         distance_mm=extra_fwd)
                    robot.wait_for_pmove(10.0)

                # Lock heading to the expected value for next straight
                robot.set_heading(next_heading)
                time.sleep(0.2)
                current_h = next_heading

            # Advance planned position to the target waypoint (board coords).
            self._poll_position()
            current_x = wp.x_mm
            current_y = wp.y_mm

            self.logger.info("  Arrived planned=(%.1f, %.1f) odom=(%.1f, %.1f) h=%.1f",
                             current_x, current_y,
                             robot.x_mm, robot.y_mm, current_h)
            time.sleep(0.3)  # brief pause between segments

        # ---- Return path: drive back to start position ----
        # The return tape runs along the board left edge:
        #   approach (last track to arc entry) -> quarter-arc 1 (180->-90)
        #   -> straight corridor at X=100 -> quarter-arc 2 (-90->0)
        if self._abort.is_set():
            return

        self.logger.info("=== Return path ===")
        arc_mm = TURN_90_ARC_MM * RETURN_ARC_TURN_DEG / 90.0

        # -- Step 1: Approach -- continue along last track to arc entry --
        if RETURN_APPROACH_DIST_MM > 1.0:
            self.logger.info("  Approach: %.1f mm along last track",
                             RETURN_APPROACH_DIST_MM)
            if self.use_line_follow:
                robot.line_follow(self.speed_pct,
                                  distance_mm=RETURN_APPROACH_DIST_MM,
                                  search_mm=LF_SEARCH_MM)
            else:
                robot.precision_move("w", self.speed_pct,
                                     distance_mm=RETURN_APPROACH_DIST_MM)
            result = robot.wait_for_pmove(30.0)
            if result is None:
                self.logger.error("  Approach timeout")
                robot.send({"cmd": "stop"})
        robot.set_heading(180.0)
        time.sleep(0.15)
        current_h = 180.0
        current_x = START_X  # 150

        # -- Step 2: Arc 1 -- quarter-circle LEFT (heading 180 -> -90) --
        # Moves from ~(150, 612.5) to ~(100, 562.5), radius 50 mm
        self.logger.info(
            "  Arc 1: %d x [fwd %.0f mm + left %.0f deg]",
            RETURN_ARC_STEPS, RETURN_ARC_FWD_MM, RETURN_ARC_TURN_DEG)
        for step in range(RETURN_ARC_STEPS):
            if self._abort.is_set():
                return
            robot.precision_move("w", UTURN_ARC_FWD_SPEED_PCT,
                                 distance_mm=RETURN_ARC_FWD_MM)
            robot.wait_for_pmove(10.0)
            robot.precision_move("a", UTURN_ARC_TURN_SPEED_PCT,
                                 distance_mm=arc_mm)
            robot.wait_for_pmove(10.0)

        robot.set_heading(-90.0)
        time.sleep(0.2)
        current_h = -90.0
        current_x = RETURN_CORRIDOR_X           # 100
        current_y = TRACK_YS[-1] - UTURN_RADIUS_MM  # 562.5

        # -- Step 3: Straight corridor -- X = 100, heading -90 --
        # Drive from ~(100, 562.5) to ~(100, 162.5)
        self.logger.info("  Corridor: %.1f mm at X=%.0f, heading -90",
                         RETURN_STRAIGHT_DIST_MM, RETURN_CORRIDOR_X)
        if self.use_line_follow:
            robot.line_follow(self.speed_pct,
                              distance_mm=RETURN_STRAIGHT_DIST_MM,
                              search_mm=LF_SEARCH_MM)
        else:
            robot.precision_move("w", self.speed_pct,
                                 distance_mm=RETURN_STRAIGHT_DIST_MM)
        result = robot.wait_for_pmove(120.0)
        if result is None:
            self.logger.error("  Corridor drive timeout")
            robot.send({"cmd": "stop"})

        robot.set_heading(-90.0)
        time.sleep(0.15)
        current_y = TRACK_YS[0] + UTURN_RADIUS_MM   # 162.5

        # -- Step 4: Arc 2 -- quarter-circle LEFT (heading -90 -> 0) --
        # Moves from ~(100, 162.5) to ~(150, 112.5), radius 50 mm
        self.logger.info(
            "  Arc 2: %d x [fwd %.0f mm + left %.0f deg]",
            RETURN_ARC_STEPS, RETURN_ARC_FWD_MM, RETURN_ARC_TURN_DEG)
        for step in range(RETURN_ARC_STEPS):
            if self._abort.is_set():
                return
            robot.precision_move("w", UTURN_ARC_FWD_SPEED_PCT,
                                 distance_mm=RETURN_ARC_FWD_MM)
            robot.wait_for_pmove(10.0)
            robot.precision_move("a", UTURN_ARC_TURN_SPEED_PCT,
                                 distance_mm=arc_mm)
            robot.wait_for_pmove(10.0)

        robot.set_heading(0.0)
        time.sleep(0.2)
        current_h = 0.0
        current_x = START_X    # 150
        current_y = START_Y    # 112.5
        self._poll_position()
        self.logger.info("  Back at start: planned=(%.1f, %.1f) h=%.1f",
                         current_x, current_y, current_h)

        self.logger.info("=== Serpentine drive complete ===")
        self.logger.info("Total samples recorded: %d",
                         len(robot.trajectory))

    def _point_turn(self, degrees: float) -> None:
        """Rotate in place by *degrees* (positive = CCW, a-turn;
        negative = CW, d-turn).

        Uses encoder-distance-based moves.  For the 3pi+ with ~96 mm
        wheel track, a 90-degree turn requires each wheel to travel
        ~75.4 mm of arc.  Scale linearly for other angles.
        """
        if abs(degrees) < 1.0:
            return
        direction = "a" if degrees > 0 else "d"
        arc_mm = TURN_90_ARC_MM * abs(degrees) / 90.0
        self.logger.debug("  Point turn: %s dist=%.1f mm (%.0f deg)",
                          direction, arc_mm, degrees)
        self.robot.precision_move(direction, 40, distance_mm=arc_mm)
        self.robot.wait_for_pmove(10.0)

    def _poll_position(self) -> None:
        """Request fresh position from the robot and wait a moment."""
        self.robot.get_position()
        time.sleep(0.15)


# ---------------------------------------------------------------------------
# JSONL log writer
# ---------------------------------------------------------------------------

def save_log(robot: SimpleRobotClient, path: str) -> None:
    """Write all received messages and the trajectory to a JSONL file."""
    with open(path, "w") as f:
        for rec in robot.trajectory:
            entry = {
                "t": rec.t,
                "x_mm": rec.x_mm,
                "y_mm": rec.y_mm,
                "heading_deg": rec.heading_deg,
                "source": rec.source,
            }
            f.write(json.dumps(entry) + "\n")
    logging.getLogger("log").info("Saved %d records to %s",
                                  len(robot.trajectory), path)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Serpentine drive test with real-time visualization")
    parser.add_argument("--ip", default=None,
                        help="ESP32-C3 bridge IP (skip auto-discovery)")
    parser.add_argument("--port", type=int, default=DEFAULT_ROBOT_PORT,
                        help="ESP32-C3 bridge UDP port")
    parser.add_argument("--discover-timeout", type=float, default=8.0,
                        help="Auto-discovery timeout in seconds")
    parser.add_argument("--csv", default=None,
                        help="Path to ground-truth CSV (pololu-robot.csv)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Show the track visualization without a robot")
    parser.add_argument("--no-drive", action="store_true",
                        help="Connect but do not auto-drive (manual control)")
    parser.add_argument("--no-line-follow", action="store_true",
                        help="Use dead-reckoning pmove instead of line-follow")
    parser.add_argument("--speed", type=int, default=DEFAULT_DRIVE_SPEED_PCT,
                        help="Straight-leg speed percent (default 50)")
    parser.add_argument("--log", default=None,
                        help="Path for JSONL trajectory log")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(name)-10s] %(levelname)s  %(message)s",
        datefmt="%H:%M:%S",
    )
    logger = logging.getLogger("main")

    drive_speed = args.speed

    # Resolve ground-truth CSV (default: same directory as this script)
    csv_path = args.csv
    if csv_path is None:
        default_csv = Path(__file__).parent / "pololu-robot.csv"
        if default_csv.is_file():
            csv_path = str(default_csv)

    robot: Optional[SimpleRobotClient] = None
    executor: Optional[SerpentineDriveExecutor] = None

    if not args.dry_run:
        # Resolve bridge address
        ip = args.ip
        port = args.port

        if ip is None:
            logger.info("No --ip specified, attempting auto-discovery ...")
            result = discover_bridge(timeout=args.discover_timeout)
            if result is None:
                logger.error("Auto-discovery failed.  "
                             "Use --ip to specify the bridge manually.")
                sys.exit(1)
            ip, port = result

        logger.info("Connecting to bridge at %s:%d", ip, port)
        robot = SimpleRobotClient(ip, port)
        robot.start()

        if not robot.wait_for_connection(5.0):
            # Show local network info to help diagnose subnet mismatches
            diag = ""
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect((ip, port))
                local_ip = s.getsockname()[0]
                s.close()
                diag = f"  Local IP reaching bridge: {local_ip}"
            except Exception:
                diag = "  (could not determine local IP toward bridge)"
            logger.error(
                "No response from bridge at %s:%d after 5 s.\n%s\n"
                "  Troubleshooting:\n"
                "    1. Verify the ESP32-C3 LED blinked 3x on boot (WiFi OK)\n"
                "    2. Confirm this PC and the ESP32 are on the SAME subnet\n"
                "    3. Try:  nc -u %s %d   then type {\"cmd\":\"bridge_ping\"}\n"
                "    4. Check firewall:  sudo iptables -L -n | grep -i drop\n"
                "    5. Run with -v for packet-level debug logging",
                ip, port, diag, ip, port)
            robot.stop()
            sys.exit(1)

        logger.info("Bridge connected")

        # Quick robot UART check (non-blocking, just advisory)
        logger.info("Checking robot UART path ...")
        if robot.verify_robot(timeout=3.0):
            logger.info("Robot responding (battery %d mV)", robot.battery_mv)
        else:
            logger.warning(
                "Robot did NOT respond to ping via UART.  "
                "The ESP32 bridge is working, but the 3pi+ robot "
                "may not be powered or wired correctly.")
            if not args.no_drive:
                logger.error("Cannot drive without a responding robot. "
                             "Use --no-drive to just visualize.")
                robot.stop()
                sys.exit(1)

        if not args.no_drive:
            executor = SerpentineDriveExecutor(
                robot,
                speed_pct=drive_speed,
                use_line_follow=not args.no_line_follow,
            )
            executor.start()

    # Launch visualization (blocks until window closed)
    viz = SerpentineVisualizer(robot=robot, ground_truth_csv=csv_path)
    try:
        viz.run()
    except KeyboardInterrupt:
        pass
    finally:
        if executor:
            executor.abort()
        if robot:
            # Save trajectory log
            log_path = args.log or f"serpentine_log_{int(time.time())}.jsonl"
            if robot.trajectory:
                save_log(robot, log_path)
            robot.stop()

    logger.info("Done")


if __name__ == "__main__":
    main()
