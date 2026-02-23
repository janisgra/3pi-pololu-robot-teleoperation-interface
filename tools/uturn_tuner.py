"""
U-Turn Tuning Tool for Pololu 3pi+ Robot

Interactive terminal tool for tuning point-turn arc distances, forward
distances, and speeds for the serpentine U-turn maneuver.  Connects to
the robot through the ESP32-C3 WiFi bridge and lets you command
individual moves, read sensor values, and iterate quickly.

Usage:
    python tools/uturn_tuner.py                     # auto-discover bridge
    python tools/uturn_tuner.py --ip 192.168.7.210  # manual IP

Once connected you get an interactive prompt.  Type 'help' for commands.

Author: Janis
"""

import argparse
import json
import logging
import math
import readline  # enables arrow-key history in input()
import socket
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Network constants (same as serpentine_drive_test.py)
# ---------------------------------------------------------------------------

DEFAULT_ROBOT_IP = "192.168.7.210"
DEFAULT_ROBOT_PORT = 5005
DISCOVERY_PORT = 5004
SOCKET_TIMEOUT = 0.1

# Robot geometry
WHEEL_TRACK_MM = 96.0
TURN_90_ARC_MM = (WHEEL_TRACK_MM / 2) * (math.pi / 2)  # ~75.4 mm

# U-turn smooth-arc parameters (empirically tuned)
# Left and right U-turns have separate tuning.
UTURN_LEFT_STEPS = 5             # fwd+turn iterations for LEFT U-turn
UTURN_LEFT_FWD_MM = 25.0         # forward distance per step (mm)
UTURN_LEFT_TURN_DEG = 20.0       # turn angle per step (degrees)
UTURN_LEFT_EXTRA_FWD_MM = 15.0   # extra forward after arc completes

UTURN_RIGHT_STEPS = 5            # fwd+turn iterations for RIGHT U-turn
UTURN_RIGHT_FWD_MM = 25.0        # forward distance per step (mm)
UTURN_RIGHT_TURN_DEG = 20.0      # turn angle per step (degrees)
UTURN_RIGHT_EXTRA_DEG = 9.0      # extra CW correction after right arc
UTURN_RIGHT_EXTRA_FWD_MM = 15.0  # extra forward after arc completes

UTURN_ARC_FWD_SPEED_PCT = 30     # % speed for arc forward segments
UTURN_ARC_TURN_SPEED_PCT = 40    # % speed for arc turn segments

# Line-follow distances
LF_STRAIGHT_DIST_MM = 720.0      # empirical tape length between U-turns
LF_SEARCH_MM = 200.0             # search distance when acquiring line

# ---------------------------------------------------------------------------
# Bridge discovery (same as serpentine tool)
# ---------------------------------------------------------------------------

def discover_bridge(timeout: float = 8.0) -> Optional[Tuple[str, int]]:
    logger = logging.getLogger("discovery")
    logger.info("Listening for discovery beacons on port %d (%.0fs) ...",
                DISCOVERY_PORT, timeout)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.5)
    try:
        sock.bind(("0.0.0.0", DISCOVERY_PORT))
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
            except (socket.timeout, json.JSONDecodeError, UnicodeDecodeError):
                continue
            if (msg.get("type") == "discovery" and
                    msg.get("service") == "pololu-3pi-bridge"):
                ip = msg.get("ip", addr[0])
                port = msg.get("port", DEFAULT_ROBOT_PORT)
                logger.info("Discovered bridge at %s:%d", ip, port)
                ack = json.dumps({"cmd": "discover_ack"}).encode()
                sock.sendto(ack, addr)
                result = (ip, port)
                break
    finally:
        sock.close()
    return result


# ---------------------------------------------------------------------------
# Minimal robot client (stripped from serpentine_drive_test.py)
# ---------------------------------------------------------------------------

class RobotClient:
    def __init__(self, ip: str, port: int):
        self.logger = logging.getLogger("robot")
        self.ip = ip
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(SOCKET_TIMEOUT)
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._connected = False
        self._seq = 0

        # Position
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.heading_deg = 0.0
        self._odom_lock = threading.Lock()

        # Move completion
        self._pmove_done = threading.Event()
        self._pmove_result: Optional[Dict] = None

        # Ack / pong
        self._last_ack_cmd = ""
        self._ack_event = threading.Event()
        self._robot_pong = threading.Event()

        self.last_heartbeat_t = 0.0
        self.battery_mv = 0

        # Last status message (for sensor readout)
        self.last_status: Optional[Dict] = None

    def start(self):
        self._running = True
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name="rx")
        self._rx_thread.start()

    def stop(self):
        self.send({"cmd": "stop"})
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=2)
        self._sock.close()

    @property
    def is_connected(self):
        return self._connected and (time.time() - self.last_heartbeat_t) < 10

    def send(self, cmd: Dict) -> bool:
        try:
            payload = json.dumps(cmd, separators=(',', ':')).encode()
            self._sock.sendto(payload, (self.ip, self.port))
            self._seq += 1
            return True
        except Exception as exc:
            self.logger.error("Send failed: %s", exc)
            return False

    def bridge_ping(self):
        self.send({"cmd": "bridge_ping", "seq": self._seq,
                   "ts": int(time.time() * 1000)})

    def ping(self):
        self.send({"cmd": "ping", "seq": self._seq,
                   "ts": int(time.time() * 1000)})

    def wait_for_connection(self, timeout=5.0):
        self.bridge_ping()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_connected:
                return True
            self.bridge_ping()
            time.sleep(0.3)
        return self.is_connected

    def verify_robot(self, timeout=3.0):
        self._robot_pong.clear()
        self.ping()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._robot_pong.wait(timeout=0.1):
                return True
            if time.time() > deadline - 1.5:
                self.ping()
        return False

    def wait_for_ack(self, cmd, timeout=5.0):
        self._ack_event.clear()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._ack_event.wait(timeout=0.1):
                if self._last_ack_cmd == cmd:
                    return True
                self._ack_event.clear()
        return False

    def wait_for_pmove(self, timeout=30.0):
        if self._pmove_done.wait(timeout):
            return self._pmove_result
        self.logger.warning("Move timed out (%.1fs)", timeout)
        return None

    # Commands
    def calibrate(self):
        self.send({"cmd": "calibrate", "sensor": "line"})

    def get_status(self):
        self.last_status = None
        self.send({"cmd": "status"})

    def get_position(self):
        self.send({"cmd": "getpos"})

    def reset_position(self):
        self.send({"cmd": "resetpos"})

    def set_heading(self, h):
        self.send({"cmd": "setheading", "h": int(h)})

    def pmove(self, direction, speed_pct, distance_mm=0, duration_ms=0):
        cmd = {"cmd": "pmove", "dir": direction,
               "speed": max(1, min(100, speed_pct))}
        if distance_mm > 0:
            cmd["dist"] = int(distance_mm)
        elif duration_ms > 0:
            cmd["dur"] = duration_ms
        else:
            return
        self._pmove_done.clear()
        self._pmove_result = None
        self.send(cmd)

    def line_follow(self, speed_pct, distance_mm=0, duration_ms=0,
                    search_mm=0):
        cmd = {"cmd": "linefollow",
               "speed": max(1, min(100, speed_pct))}
        if distance_mm > 0:
            cmd["dist"] = int(distance_mm)
        elif duration_ms > 0:
            cmd["dur"] = duration_ms
        else:
            return
        if search_mm > 0:
            cmd["search"] = int(search_mm)
        self._pmove_done.clear()
        self._pmove_result = None
        self.send(cmd)

    def stop_motors(self):
        self.send({"cmd": "stop"})

    # RX loop
    def _rx_loop(self):
        while self._running:
            try:
                data, addr = self._sock.recvfrom(2048)
                msg = json.loads(data.decode())
                self._dispatch(msg)
            except socket.timeout:
                continue
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            except OSError:
                break

    def _dispatch(self, msg):
        self._connected = True
        self.last_heartbeat_t = time.time()
        t = msg.get("type", "")

        if t == "position":
            self._update_pos(msg)
        elif t == "pmove_done":
            pos = msg.get("pos", {})
            self._update_pos(pos)
            self._pmove_result = msg
            self._pmove_done.set()
            self.logger.info("pmove_done: dist=%.1f  pos=(%.1f,%.1f) h=%.1f",
                             msg.get("dist", 0), pos.get("x", 0),
                             pos.get("y", 0), pos.get("h", 0))
        elif t == "lf_done":
            pos = msg.get("pos", {})
            self._update_pos(pos)
            self._pmove_result = msg
            self._pmove_done.set()
            self.logger.info("lf_done (%s): dist=%.1f  h=%.1f",
                             msg.get("reason", "ok"),
                             msg.get("dist", 0), pos.get("h", 0))
        elif t == "lf_status":
            pos = msg.get("pos", {})
            self._update_pos(pos)
            self.logger.info("lf: line=%d err=%d sns=%s spd=%s d=%.1f h=%.1f",
                             msg.get("line", 0), msg.get("err", 0),
                             msg.get("sns", []), msg.get("spd", []),
                             msg.get("dist", 0), pos.get("h", 0))
        elif t == "heartbeat":
            self.battery_mv = msg.get("battery", 0)
        elif t == "bridge_pong":
            pass
        elif t == "ack":
            self._last_ack_cmd = msg.get("cmd", "")
            self._ack_event.set()
            self.logger.debug("Ack: %s", self._last_ack_cmd)
        elif t == "pong":
            self._robot_pong.set()
        elif t == "status":
            self.last_status = msg
        elif t == "event":
            ev = msg.get("event", "")
            if ev == "bump":
                self.logger.warning("BUMP %s", msg.get("side", ""))
            elif ev == "search_ok":
                self.logger.info("Search: line found at %.1f mm",
                                 msg.get("d", 0))
            elif ev == "line_lost":
                self.logger.warning("Line lost")
            elif ev == "line_found":
                self.logger.info("Line re-acquired")
            else:
                self.logger.info("Event: %s", ev)
        elif t == "error":
            self.logger.warning("Error: %s", msg.get("error", "?"))

    def _update_pos(self, d):
        with self._odom_lock:
            self.x_mm = d.get("x", self.x_mm)
            self.y_mm = d.get("y", self.y_mm)
            self.heading_deg = d.get("h", self.heading_deg)


# ---------------------------------------------------------------------------
# Interactive command loop
# ---------------------------------------------------------------------------

HELP_TEXT = """
Commands (all distances in mm, angles in degrees, speeds in %):

  cal                   Calibrate line sensors (robot spins in place)
  pos                   Print current odometry position and heading
  resetpos              Reset odometry to (0, 0, 0)
  seth <degrees>        Set heading to specific value
  status                Request and print full status (sensors, encoders)

  fwd <mm> [speed]      Drive forward (default speed 30%)
  back <mm> [speed]     Drive backward
  left <deg> [speed]    Turn left (CCW) by degrees (default speed 40%)
  right <deg> [speed]   Turn right (CW) by degrees
  stop                  Emergency stop

  lf <mm> [speed] [search_mm]   Line-follow for distance
                                (default speed 50, search 0)
  lfm [speed] [search_mm]       Line-follow until end of tape (line lost)
                                (default speed 50, search 0)

  uturn <dy_mm> [speed] [turn_speed]
                        Execute full U-turn as smooth arc:
                        5 steps of (fwd 25mm + turn 20deg)
                        Positive dy = turn left.  Negative = right.
                        Right U-turns get an extra 9-deg correction.
                        Default speed=30 turn_speed=40

  seq <commands>        Run multiple commands separated by semicolons
                        Example: seq fwd 100; left 90; fwd 100

  repeat <n> <command>  Repeat a command n times
                        Example: repeat 3 fwd 50

  help                  Show this help
  quit / exit           Disconnect and exit
"""


def parse_float(s: str, default: float = 0.0) -> float:
    try:
        return float(s)
    except (ValueError, IndexError):
        return default


def execute_command(robot: RobotClient, line: str) -> bool:
    """Execute one command.  Returns False to quit."""
    line = line.strip()
    if not line or line.startswith("#"):
        return True

    parts = line.split()
    cmd = parts[0].lower()

    if cmd in ("quit", "exit", "q"):
        return False

    elif cmd == "help":
        print(HELP_TEXT)

    elif cmd == "cal":
        print("Calibrating (robot will spin) ...")
        robot.calibrate()
        for attempt in range(3):
            if robot.wait_for_ack("calibrate", timeout=6.0):
                print("Calibration OK")
                robot.reset_position()
                time.sleep(0.2)
                robot.set_heading(0)
                time.sleep(0.1)
                return True
            if attempt < 2:
                print(f"  No ack (attempt {attempt+1}), retrying ...")
                robot.calibrate()
        print("Calibration failed (no ack after 3 attempts)")

    elif cmd == "pos":
        robot.get_position()
        time.sleep(0.2)
        print(f"  x={robot.x_mm:.1f}  y={robot.y_mm:.1f}  "
              f"h={robot.heading_deg:.1f}")

    elif cmd == "resetpos":
        robot.reset_position()
        time.sleep(0.15)
        print("Position reset to (0, 0, 0)")

    elif cmd == "seth":
        if len(parts) < 2:
            print("Usage: seth <degrees>")
            return True
        h = parse_float(parts[1])
        robot.set_heading(h)
        time.sleep(0.15)
        print(f"  Heading set to {h:.0f}")

    elif cmd == "status":
        robot.get_status()
        time.sleep(0.3)
        if robot.last_status:
            s = robot.last_status
            print(f"  battery:  {s.get('battery', 0)} mV")
            print(f"  encoders: {s.get('enc', [])}")
            print(f"  bump:     {s.get('bump', [])}")
            print(f"  line:     {s.get('line', [])}")
            print(f"  motors:   {s.get('mtr', [])}")
        else:
            print("  (no status received)")

    elif cmd in ("fwd", "forward"):
        if len(parts) < 2:
            print("Usage: fwd <mm> [speed%]")
            return True
        dist = parse_float(parts[1])
        speed = int(parse_float(parts[2], 30)) if len(parts) > 2 else 30
        print(f"  Forward {dist:.0f} mm @ {speed}% ...")
        robot.pmove("w", speed, distance_mm=dist)
        result = robot.wait_for_pmove(30)
        if result:
            pos = result.get("pos", {})
            print(f"  Done: dist={result.get('dist',0):.1f}  "
                  f"pos=({pos.get('x',0):.1f}, {pos.get('y',0):.1f})  "
                  f"h={pos.get('h',0):.1f}")
        else:
            print("  Timed out!")

    elif cmd in ("back", "backward"):
        if len(parts) < 2:
            print("Usage: back <mm> [speed%]")
            return True
        dist = parse_float(parts[1])
        speed = int(parse_float(parts[2], 30)) if len(parts) > 2 else 30
        print(f"  Backward {dist:.0f} mm @ {speed}% ...")
        robot.pmove("s", speed, distance_mm=dist)
        result = robot.wait_for_pmove(30)
        if result:
            pos = result.get("pos", {})
            print(f"  Done: dist={result.get('dist',0):.1f}  "
                  f"h={pos.get('h',0):.1f}")

    elif cmd == "left":
        if len(parts) < 2:
            print("Usage: left <degrees> [speed%]")
            return True
        deg = parse_float(parts[1])
        speed = int(parse_float(parts[2], 40)) if len(parts) > 2 else 40
        arc = TURN_90_ARC_MM * deg / 90.0
        print(f"  Turn left {deg:.0f} deg (arc={arc:.1f} mm) @ {speed}% ...")
        robot.pmove("a", speed, distance_mm=arc)
        result = robot.wait_for_pmove(15)
        if result:
            pos = result.get("pos", {})
            print(f"  Done: arc={result.get('dist',0):.1f}  "
                  f"h={pos.get('h',0):.1f}")

    elif cmd == "right":
        if len(parts) < 2:
            print("Usage: right <degrees> [speed%]")
            return True
        deg = parse_float(parts[1])
        speed = int(parse_float(parts[2], 40)) if len(parts) > 2 else 40
        arc = TURN_90_ARC_MM * deg / 90.0
        print(f"  Turn right {deg:.0f} deg (arc={arc:.1f} mm) @ {speed}% ...")
        robot.pmove("d", speed, distance_mm=arc)
        result = robot.wait_for_pmove(15)
        if result:
            pos = result.get("pos", {})
            print(f"  Done: arc={result.get('dist',0):.1f}  "
                  f"h={pos.get('h',0):.1f}")

    elif cmd == "stop":
        robot.stop_motors()
        print("  Motors stopped")

    elif cmd == "lf":
        if len(parts) < 2:
            print("Usage: lf <mm> [speed%] [search_mm]")
            return True
        dist = parse_float(parts[1])
        speed = int(parse_float(parts[2], 50)) if len(parts) > 2 else 50
        search = parse_float(parts[3], 0) if len(parts) > 3 else 0
        print(f"  Line-follow {dist:.0f} mm @ {speed}%"
              f"{' search=' + str(int(search)) if search > 0 else ''} ...")
        robot.line_follow(speed, distance_mm=dist, search_mm=search)
        result = robot.wait_for_pmove(60)
        if result:
            reason = result.get("reason", "ok")
            pos = result.get("pos", {})
            print(f"  Done ({reason}): dist={result.get('dist',0):.1f}  "
                  f"h={pos.get('h',0):.1f}")
        else:
            print("  Timed out!")
            robot.stop_motors()

    elif cmd == "lfm":
        speed = int(parse_float(parts[1], 50)) if len(parts) > 1 else 50
        search = parse_float(parts[2], 0) if len(parts) > 2 else 0
        print(f"  Line-follow until end of tape @ {speed}%"
              f"{' search=' + str(int(search)) if search > 0 else ''} ...")
        robot.line_follow(speed, distance_mm=9999, search_mm=search)
        result = robot.wait_for_pmove(120)
        if result:
            reason = result.get("reason", "ok")
            pos = result.get("pos", {})
            print(f"  Done ({reason}): dist={result.get('dist',0):.1f}  "
                  f"h={pos.get('h',0):.1f}")
        else:
            print("  Timed out!")
            robot.stop_motors()

    elif cmd == "uturn":
        if len(parts) < 2:
            print("Usage: uturn <dy_mm> [fwd_speed%] [turn_speed%]")
            print("  Positive dy = turn left, negative = right")
            print("  Left:  %d x (fwd %.0f mm + turn %.0f deg) + fwd %.0f mm"
                  % (UTURN_LEFT_STEPS, UTURN_LEFT_FWD_MM,
                     UTURN_LEFT_TURN_DEG, UTURN_LEFT_EXTRA_FWD_MM))
            print("  Right: %d x (fwd %.0f mm + turn %.0f deg) + extra %.0f deg + fwd %.0f mm"
                  % (UTURN_RIGHT_STEPS, UTURN_RIGHT_FWD_MM,
                     UTURN_RIGHT_TURN_DEG, UTURN_RIGHT_EXTRA_DEG,
                     UTURN_RIGHT_EXTRA_FWD_MM))
            return True
        dy = parse_float(parts[1])
        fwd_speed = int(parse_float(parts[2], UTURN_ARC_FWD_SPEED_PCT)) \
            if len(parts) > 2 else UTURN_ARC_FWD_SPEED_PCT
        turn_speed = int(parse_float(parts[3], UTURN_ARC_TURN_SPEED_PCT)) \
            if len(parts) > 3 else UTURN_ARC_TURN_SPEED_PCT
        turn_left = (dy > 0)
        turn_dir = "a" if turn_left else "d"
        direction = "left" if turn_left else "right"

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

        print(f"  U-turn: {direction} arc ({n_steps} x "
              f"[fwd {step_fwd:.0f} mm + turn {step_deg:.0f} deg])")
        print(f"  Speeds: fwd={fwd_speed}%  turn={turn_speed}%")

        arc_mm = TURN_90_ARC_MM * step_deg / 90.0
        for step in range(n_steps):
            print(f"  [{step+1}/{n_steps}] fwd {step_fwd:.0f} mm ...")
            robot.pmove("w", fwd_speed, distance_mm=step_fwd)
            r = robot.wait_for_pmove(10)
            if r:
                h = r.get("pos", {}).get("h", 0)
                print(f"        fwd done  h={h:.1f}")
            else:
                print("        fwd timed out!")
                return True

            robot.pmove(turn_dir, turn_speed, distance_mm=arc_mm)
            r = robot.wait_for_pmove(10)
            if r:
                h = r.get("pos", {}).get("h", 0)
                print(f"        turn done h={h:.1f}")
            else:
                print("        turn timed out!")
                return True

        # Right U-turns need extra correction
        if not turn_left and UTURN_RIGHT_EXTRA_DEG > 0:
            extra_arc = TURN_90_ARC_MM * UTURN_RIGHT_EXTRA_DEG / 90.0
            print(f"  Extra right correction: {UTURN_RIGHT_EXTRA_DEG:.0f} deg ...")
            robot.pmove("d", turn_speed, distance_mm=extra_arc)
            r = robot.wait_for_pmove(10)
            if r:
                h = r.get("pos", {}).get("h", 0)
                print(f"        done h={h:.1f}")

        # Extra forward to clear arc / reach next tape
        if extra_fwd > 0:
            print(f"  Extra forward: {extra_fwd:.0f} mm ...")
            robot.pmove("w", fwd_speed, distance_mm=extra_fwd)
            r = robot.wait_for_pmove(10)
            if r:
                h = r.get("pos", {}).get("h", 0)
                print(f"        done h={h:.1f}")

        # Print summary
        robot.get_position()
        time.sleep(0.2)
        print(f"  U-turn complete: pos=({robot.x_mm:.1f}, {robot.y_mm:.1f})  "
              f"h={robot.heading_deg:.1f}")

    elif cmd == "seq":
        # Run semicolon-separated commands
        rest = line[3:].strip()
        subcmds = [s.strip() for s in rest.split(";") if s.strip()]
        for i, sc in enumerate(subcmds):
            print(f"--- seq [{i+1}/{len(subcmds)}]: {sc}")
            execute_command(robot, sc)
            time.sleep(0.1)

    elif cmd == "repeat":
        if len(parts) < 3:
            print("Usage: repeat <n> <command ...>")
            return True
        n = int(parse_float(parts[1], 1))
        subcmd = " ".join(parts[2:])
        for i in range(n):
            print(f"--- repeat [{i+1}/{n}]: {subcmd}")
            execute_command(robot, subcmd)
            time.sleep(0.1)

    else:
        print(f"Unknown command: {cmd}  (type 'help' for list)")

    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)-10s] %(levelname)-5s %(message)s",
        datefmt="%H:%M:%S",
    )
    logger = logging.getLogger("main")

    parser = argparse.ArgumentParser(
        description="Interactive U-turn tuning tool for Pololu 3pi+")
    parser.add_argument("--ip", type=str, default=None,
                        help="Bridge IP (skip auto-discovery)")
    parser.add_argument("--port", type=int, default=DEFAULT_ROBOT_PORT)
    args = parser.parse_args()

    # Connect
    if args.ip:
        ip, port = args.ip, args.port
    else:
        logger.info("No --ip specified, attempting auto-discovery ...")
        result = discover_bridge(timeout=8.0)
        if not result:
            logger.error("Discovery failed. Use --ip to connect manually.")
            sys.exit(1)
        ip, port = result

    logger.info("Connecting to %s:%d", ip, port)
    robot = RobotClient(ip, port)
    robot.start()

    if not robot.wait_for_connection(timeout=5.0):
        logger.error("Bridge not responding.")
        robot.stop()
        sys.exit(1)
    logger.info("Bridge connected")

    # Verify robot UART
    logger.info("Verifying robot UART ...")
    if robot.verify_robot(timeout=3.0):
        logger.info("Robot OK (battery %d mV)", robot.battery_mv)
    else:
        logger.warning("Robot UART not responding -- "
                       "commands may not reach the 3pi+")

    # Interactive loop
    print("\n" + "=" * 60)
    print("  U-Turn Tuner  --  type 'help' for commands")
    print("=" * 60 + "\n")

    try:
        while True:
            try:
                line = input(">> ").strip()
            except EOFError:
                break
            if not execute_command(robot, line):
                break
    except KeyboardInterrupt:
        print("\nInterrupted")

    robot.stop()
    logger.info("Done")


if __name__ == "__main__":
    main()
