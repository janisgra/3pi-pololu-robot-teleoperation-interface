#!/usr/bin/env python3
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
DEFAULT_ROBOT_PORT = 5005
DISCOVERY_PORT = 5004
SOCKET_TIMEOUT = 0.1

# Drive parameters (defaults, overridable via --speed)
DEFAULT_DRIVE_SPEED_PCT = 50   # % of max motor speed for straight legs
UTURN_SPEED_PCT = 30           # % for U-turns
POSITION_POLL_HZ = 10          # how often to request position from robot

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
    logger.info("Listening for discovery beacons on port %d ...", DISCOVERY_PORT)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.5)
    try:
        sock.bind(("", DISCOVERY_PORT))
    except OSError as exc:
        logger.error("Cannot bind port %d: %s", DISCOVERY_PORT, exc)
        sock.close()
        return None

    deadline = time.time() + timeout
    result = None

    try:
        while time.time() < deadline:
            try:
                data, addr = sock.recvfrom(1024)
                msg = json.loads(data.decode())
            except socket.timeout:
                continue
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

            if msg.get("type") != "discovery":
                continue
            if msg.get("service") != "pololu-3pi-bridge":
                continue

            bridge_ip = msg.get("ip", addr[0])
            bridge_port = msg.get("port", DEFAULT_ROBOT_PORT)
            logger.info("Discovered bridge at %s:%d  (RSSI %s, MAC %s)",
                        bridge_ip, bridge_port,
                        msg.get("rssi", "?"), msg.get("mac", "?"))

            # Send acknowledgement
            ack = json.dumps({"cmd": "discover_ack"}).encode()
            sock.sendto(ack, addr)

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
            payload = json.dumps(cmd).encode()
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

    def wait_for_pmove(self, timeout: float = 30.0) -> Optional[Dict]:
        if self._pmove_done.wait(timeout):
            return self._pmove_result
        self.logger.warning("pmove timed out (%.1fs)", timeout)
        return None

    def calibrate_line_sensors(self) -> bool:
        return self.send({"cmd": "calibrate", "sensor": "line"})

    def wait_for_connection(self, timeout: float = 5.0) -> bool:
        self.ping()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_connected:
                return True
            time.sleep(0.05)
        return self.is_connected

    # -- receive --

    def _rx_loop(self) -> None:
        while self._running:
            try:
                data, _ = self._sock.recvfrom(2048)
                msg = json.loads(data.decode())
                self._dispatch(msg)
            except socket.timeout:
                continue
            except (json.JSONDecodeError, UnicodeDecodeError):
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
        elif msg_type == "heartbeat":
            self.battery_mv = msg.get("battery", 0)
        elif msg_type == "pong":
            client_ts = msg.get("client_ts", 0)
            rtt = int(time.time() * 1000) - client_ts if client_ts else 0
            self.logger.debug("Pong RTT=%d ms", rtt)
        elif msg_type == "event":
            event = msg.get("event", "")
            if event == "bump":
                self.logger.warning("BUMP %s -- motors stopped",
                                    msg.get("side", ""))
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
                    self._plot_xs = [r.x_mm for r in self.robot.trajectory]
                    self._plot_ys = [r.y_mm for r in self.robot.trajectory]

                traj_line.set_data(self._plot_xs, self._plot_ys)

                with self.robot._odom_lock:
                    cx, cy = self.robot.x_mm, self.robot.y_mm
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
                 speed_pct: int = DEFAULT_DRIVE_SPEED_PCT):
        self.robot = robot
        self.speed_pct = speed_pct
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

        # Reset odometry
        robot.reset_position()
        time.sleep(0.3)

        # Build waypoints
        waypoints = build_serpentine_waypoints()
        self.logger.info("Waypoints: %d", len(waypoints))

        current_x = START_X
        current_y = START_Y
        current_h = 0.0  # heading in degrees (0 = facing +X = right)

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
                    current_h = need_heading

                # Drive straight
                self.logger.info("  Straight %.1f mm  heading=%.1f",
                                 dist, current_h)
                robot.precision_move("w", self.speed_pct,
                                     distance_mm=dist)
                result = robot.wait_for_pmove(60.0)
                if result is None:
                    self.logger.error("  pmove timeout -- aborting leg")
                    continue

                # Update current pose from robot
                self._poll_position()
                current_x = robot.x_mm
                current_y = robot.y_mm
                current_h = robot.heading_deg

            else:
                # ------- U-turn (vertical transition between tracks) -------
                # Strategy: turn 90 deg toward next track, drive dist, turn 90 deg
                turn_dir = 90.0 if dy > 0 else -90.0
                vert_dist = abs(dy)

                self.logger.info("  U-turn: turn %.0f deg, fwd %.1f mm, "
                                 "turn %.0f deg", turn_dir, vert_dist,
                                 -turn_dir)

                # First 90-degree turn
                self._point_turn(turn_dir)
                current_h += turn_dir
                time.sleep(0.2)

                # Drive the vertical segment
                robot.precision_move("w", UTURN_SPEED_PCT,
                                     distance_mm=vert_dist)
                result = robot.wait_for_pmove(30.0)
                time.sleep(0.2)

                # Second 90-degree turn (back to horizontal)
                self._point_turn(-turn_dir)
                current_h -= turn_dir
                time.sleep(0.2)

                self._poll_position()
                current_x = robot.x_mm
                current_y = robot.y_mm
                current_h = robot.heading_deg

            self.logger.info("  Arrived pos=(%.1f, %.1f) h=%.1f",
                             current_x, current_y, current_h)
            time.sleep(0.3)  # brief pause between segments

        self.logger.info("=== Serpentine drive complete ===")
        self.logger.info("Total samples recorded: %d",
                         len(robot.trajectory))

    def _point_turn(self, degrees: float) -> None:
        """Rotate in place by *degrees* (positive = CCW, a-turn;
        negative = CW, d-turn).

        Uses a timed move since the 3pi+ pmove doesn't support pure rotation.
        Empirical: at speed 40%, ~800 ms per 90 deg on hard surface.
        """
        if abs(degrees) < 1.0:
            return
        direction = "a" if degrees > 0 else "d"
        # Scale duration linearly with angle
        dur_ms = int(abs(degrees) / 90.0 * 800)
        self.logger.debug("  Point turn: %s for %d ms (%.0f deg)",
                          direction, dur_ms, degrees)
        self.robot.precision_move(direction, 40, duration_ms=dur_ms)
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
            logger.error("No response from bridge -- check that the ESP32-C3 "
                         "is powered and on the same network.")
            robot.stop()
            sys.exit(1)

        logger.info("Bridge connected (battery %d mV)", robot.battery_mv)

        if not args.no_drive:
            executor = SerpentineDriveExecutor(robot, speed_pct=drive_speed)
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
