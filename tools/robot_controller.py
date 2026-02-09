"""
Pololu 3pi+ Robot - UDP Teleoperation Controller

Sends JSON commands to a Pololu 3pi+ 32U4 robot via an ESP32 WiFi bridge.
Designed for both interactive keyboard control and programmatic automation.

Author: Janis
Version: 1.0.0

Usage:
    Interactive:  python robot_controller.py --ip 192.168.7.210 --port 5005
    Programmatic: Import RobotController class and call methods directly

Controls (interactive mode):
    W/S/A/D     - Movement (forward/backward/left/right)
    Space       - Stop
    1-5         - Speed presets
    P           - Precision move mode
    G/R/I       - Get position / Reset position / IMU status
    Q/ESC       - Quit
"""

import socket
import json
import argparse
import time
import threading
import re
from dataclasses import dataclass
from typing import Optional, Callable

# ============================================================================
# DEFINITIONS
# ============================================================================

# Network Configuration
DEFAULT_IP = "192.168.7.210"
DEFAULT_PORT = 5005
SOCKET_TIMEOUT = 0.1

# Motor Speed Presets (level 1-5)
SPEED_MAP = {1: 80, 2: 150, 3: 200, 4: 300, 5: 400}
DEFAULT_SPEED_LEVEL = 3
MAX_SPEED = 400

# Timing
MOVE_BURST_DURATION_MS = 500
TURN_BURST_DURATION_MS = 300
PRECISION_MOVE_TIMEOUT_S = 30.0

# ============================================================================
# OPTIONAL DEPENDENCY
# ============================================================================

try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False


# ============================================================================
# STATE DATACLASS
# ============================================================================

@dataclass
class RobotState:
    """Current state of the robot controller."""
    forward: bool = False
    backward: bool = False
    turn_left: bool = False
    turn_right: bool = False
    speed_level: int = DEFAULT_SPEED_LEVEL
    running: bool = True
    battery_mv: int = 0
    encoder_left: int = 0
    encoder_right: int = 0
    pos_x: float = 0.0
    pos_y: float = 0.0
    heading: float = 0.0
    imu_available: bool = False
    pmove_pending: bool = False
    pmove_start_time: float = 0.0


# ============================================================================
# ROBOT CONTROLLER CLASS
# ============================================================================

class RobotController:
    """
    UDP controller for Pololu 3pi+ robot via ESP32 bridge.
    
    Can be used interactively (keyboard control) or programmatically
    by calling methods like move(), stop(), precision_move() directly.
    
    Example (programmatic):
        controller = RobotController(target_ip="192.168.7.210")
        controller.move(200, 200, duration=1000)  # Forward 1 second
        time.sleep(1.2)
        controller.stop()
    """
    
    def __init__(self, target_ip: str = DEFAULT_IP, target_port: int = DEFAULT_PORT,
                 on_response: Optional[Callable] = None):
        """
        Initialize the robot controller.
        
        Args:
            target_ip: ESP32 bridge IP address
            target_port: UDP port (default 5005)
            on_response: Optional callback for received messages
        """
        self.target_ip = target_ip
        self.target_port = target_port
        self.on_response = on_response
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(SOCKET_TIMEOUT)
        
        self.state = RobotState()
        self.sequence = 0
    
    # ========================================================================
    # CORE COMMANDS
    # ========================================================================
    
    def send_command(self, cmd: dict) -> bool:
        """Send a JSON command to the robot."""
        try:
            message = json.dumps(cmd)
            self.sock.sendto(message.encode(), (self.target_ip, self.target_port))
            self.sequence += 1
            return True
        except Exception as e:
            print(f"[ERROR] Send failed: {e}")
            return False
    
    def move(self, left: int, right: int, duration: int = 0) -> bool:
        """
        Send move command with motor speeds.
        
        Args:
            left: Left motor speed (-400 to 400)
            right: Right motor speed (-400 to 400)
            duration: Duration in ms (0 = continuous until stop)
        """
        cmd = {"cmd": "move", "left": left, "right": right}
        if duration > 0:
            cmd["duration"] = duration
        return self.send_command(cmd)
    
    def stop(self) -> bool:
        """Stop all motors immediately."""
        return self.send_command({"cmd": "stop"})
    
    def precision_move(self, direction: str, speed: int, 
                       distance: float = 0, duration: int = 0) -> bool:
        """
        Execute precision move with IMU PID heading correction.
        
        Args:
            direction: 'w' (forward), 's' (backward), 'a' (left), 'd' (right)
            speed: Speed percentage (1-100)
            distance: Distance in mm (mutually exclusive with duration)
            duration: Duration in ms (mutually exclusive with distance)
        """
        if direction not in "wasd":
            print(f"[ERROR] Invalid direction: {direction}")
            return False
        
        cmd = {"cmd": "pmove", "dir": direction, "speed": max(1, min(100, speed))}
        
        if distance > 0:
            cmd["dist"] = int(distance)
        elif duration > 0:
            cmd["dur"] = duration
        else:
            print("[ERROR] Must specify distance or duration")
            return False
        
        return self.send_command(cmd)
    
    def turn(self, angle: int, speed: int = 100) -> bool:
        """
        Turn robot by angle (open-loop).
        
        Args:
            angle: Degrees (positive = right, negative = left)
            speed: Turn speed (1-400)
        """
        return self.send_command({"cmd": "turn", "angle": angle, "speed": speed})
    
    # ========================================================================
    # SENSOR & STATUS COMMANDS
    # ========================================================================
    
    def get_position(self) -> bool:
        """Request current position."""
        return self.send_command({"cmd": "getpos"})
    
    def reset_position(self) -> bool:
        """Reset position tracking to origin."""
        return self.send_command({"cmd": "resetpos"})
    
    def get_imu_status(self) -> bool:
        """Request IMU status and readings."""
        return self.send_command({"cmd": "imu"})
    
    def get_status(self) -> bool:
        """Request full robot status."""
        return self.send_command({"cmd": "status"})
    
    # ========================================================================
    # PERIPHERAL COMMANDS
    # ========================================================================
    
    def set_leds(self, red: bool = False, yellow: bool = False, green: bool = False) -> bool:
        """Set LED states."""
        return self.send_command({"cmd": "led", "red": red, "yellow": yellow, "green": green})
    
    def buzzer(self, frequency: int = 440, duration: int = 200) -> bool:
        """Play buzzer tone."""
        return self.send_command({"cmd": "buzzer", "frequency": frequency, "duration": duration})
    
    def calibrate_line_sensors(self) -> bool:
        """Start line sensor calibration (robot will spin)."""
        return self.send_command({"cmd": "calibrate", "sensor": "line"})
    
    # ========================================================================
    # RESPONSE HANDLING
    # ========================================================================
    
    def receive_messages(self) -> Optional[dict]:
        """Check for incoming messages. Returns message dict or None."""
        try:
            data, _ = self.sock.recvfrom(1024)
            msg = json.loads(data.decode())
            self._handle_response(msg)
            if self.on_response:
                self.on_response(msg)
            return msg
        except socket.timeout:
            return None
        except json.JSONDecodeError:
            return None
        except Exception:
            return None
    
    def _handle_response(self, msg: dict):
        """Internal response handler to update state."""
        msg_type = msg.get("type", "")
        
        if msg_type == "status":
            self.state.battery_mv = msg.get("battery", 0)
            enc = msg.get("enc", [0, 0])
            if len(enc) >= 2:
                self.state.encoder_left, self.state.encoder_right = enc[0], enc[1]
            print(f"[Status] Battery: {self.state.battery_mv}mV, Encoders: L={self.state.encoder_left} R={self.state.encoder_right}")
        
        elif msg_type == "ack":
            cmd = msg.get("cmd", "")
            if cmd == "pmove":
                imu = msg.get("imu", False)
                dist = msg.get("dist", 0)
                dur = msg.get("dur", 0)
                print(f"[ACK] Precision move: {'%dmm' % dist if dist else '%dms' % dur} (IMU: {imu})")
            elif cmd not in ["move", "stop"]:
                print(f"[ACK] {cmd}")
        
        elif msg_type == "pmove_done":
            self.state.pmove_pending = False
            pos = msg.get("pos", {})
            self.state.pos_x = pos.get("x", 0)
            self.state.pos_y = pos.get("y", 0)
            self.state.heading = pos.get("h", 0)
            print(f"[PMOVE DONE] Dist:{msg.get('dist', 0):.1f}mm Pos: X={self.state.pos_x:.1f} Y={self.state.pos_y:.1f} H={self.state.heading:.1f}°")
        
        elif msg_type == "position":
            self.state.pos_x = msg.get("x", 0)
            self.state.pos_y = msg.get("y", 0)
            self.state.heading = msg.get("h", 0)
            print(f"[Position] X={self.state.pos_x:.1f}mm Y={self.state.pos_y:.1f}mm H={self.state.heading:.1f}°")
        
        elif msg_type == "imu":
            self.state.imu_available = msg.get("avail", False)
            if self.state.imu_available:
                print(f"[IMU] Available, Heading: {msg.get('heading', 0):.1f}°")
            else:
                print("[IMU] Not available")
        
        elif msg_type == "event":
            event = msg.get("event", "")
            if event == "bump":
                print(f"[EVENT] Bump: {msg.get('side', 'unknown')}")
            elif event == "startup":
                self.state.imu_available = msg.get("imu", False)
                print(f"[EVENT] Robot startup (IMU: {self.state.imu_available})")
            else:
                print(f"[EVENT] {event}")
        
        elif msg_type == "error":
            print(f"[ERROR] {msg.get('error', 'unknown')}")
    
    # ========================================================================
    # PRECISION MOVE PARSER
    # ========================================================================
    
    def parse_precision_move(self, cmd_str: str) -> bool:
        """
        Parse and execute precision move from string like 'w-100-50mm'.
        
        Format: direction-speed%-distance/duration
        Examples:
            w-100-50mm  - Forward 50mm at 100% speed
            s-50-500ms  - Backward 500ms at 50% speed
            a-75-90     - Turn left 90mm arc at 75% speed
        """
        pattern = r'^([wasd])-(\d+)-(\d+)(mm|ms)?$'
        match = re.match(pattern, cmd_str.lower().strip())
        
        if not match:
            print(f"[ERROR] Invalid format: '{cmd_str}' (use: w-100-50mm)")
            return False
        
        direction = match.group(1)
        speed = int(match.group(2))
        value = int(match.group(3))
        unit = match.group(4) or 'mm'
        
        if not 1 <= speed <= 100:
            print(f"[ERROR] Speed must be 1-100, got {speed}")
            return False
        
        if unit == 'mm':
            return self.precision_move(direction, speed, distance=value)
        else:
            return self.precision_move(direction, speed, duration=value)
    
    # ========================================================================
    # INTERACTIVE MODE
    # ========================================================================
    
    def _update_movement(self):
        """Update motors based on current keyboard state."""
        speed = SPEED_MAP[self.state.speed_level]
        left = right = 0
        
        if self.state.forward: left += speed; right += speed
        if self.state.backward: left -= speed; right -= speed
        if self.state.turn_left: left -= speed // 2; right += speed // 2
        if self.state.turn_right: left += speed // 2; right -= speed // 2
        
        self.move(max(-MAX_SPEED, min(MAX_SPEED, left)), 
                  max(-MAX_SPEED, min(MAX_SPEED, right)))
    
    def run_simple(self):
        """Run controller with simple text input (fallback mode)."""
        print("\n" + "=" * 50)
        print("  Pololu 3pi+ Robot Controller (Simple Mode)")
        print("=" * 50)
        print(f"Target: {self.target_ip}:{self.target_port}")
        print("\nCommands: w/s/a/d, stop, pos, reset, imu, status, beep")
        print("Precision: w-100-50mm, s-50-500ms, a-75-90, d-60-1000ms")
        print("Speed: speed 1-5, quit: q\n")
        
        while self.state.running:
            try:
                for _ in range(5): self.receive_messages()
                cmd = input("> ").strip().lower()
                
                if not cmd: continue
                if cmd in ('quit', 'q'): break
                if cmd in ('help', '?'):
                    print("Commands: w, s, a, d, stop, pos, reset, imu, status, beep, speed N")
                    continue
                
                if cmd == 'w': self.move(SPEED_MAP[self.state.speed_level], SPEED_MAP[self.state.speed_level], MOVE_BURST_DURATION_MS)
                elif cmd == 's': self.move(-SPEED_MAP[self.state.speed_level], -SPEED_MAP[self.state.speed_level], MOVE_BURST_DURATION_MS)
                elif cmd == 'a': self.move(-SPEED_MAP[self.state.speed_level], SPEED_MAP[self.state.speed_level], TURN_BURST_DURATION_MS)
                elif cmd == 'd': self.move(SPEED_MAP[self.state.speed_level], -SPEED_MAP[self.state.speed_level], TURN_BURST_DURATION_MS)
                elif cmd == 'stop': self.stop()
                elif cmd == 'pos': self.get_position()
                elif cmd == 'reset': self.reset_position()
                elif cmd == 'imu': self.get_imu_status()
                elif cmd == 'status': self.get_status()
                elif cmd == 'beep': self.buzzer()
                elif cmd.startswith('speed'):
                    parts = cmd.split()
                    if len(parts) >= 2 and parts[1].isdigit():
                        level = int(parts[1])
                        if 1 <= level <= 5:
                            self.state.speed_level = level
                            print(f"Speed: {level} ({SPEED_MAP[level]})")
                elif '-' in cmd and len(cmd) >= 5 and cmd[0] in 'wasd':
                    if self.parse_precision_move(cmd):
                        self.state.pmove_pending = True
                        self.state.pmove_start_time = time.time()
                        print("Waiting for move...")
                        while self.state.pmove_pending:
                            self.receive_messages()
                            if time.time() - self.state.pmove_start_time > PRECISION_MOVE_TIMEOUT_S:
                                print("[TIMEOUT] Move timed out")
                                self.state.pmove_pending = False
                                self.stop()
                                break
                            time.sleep(0.05)
                else:
                    print(f"Unknown: '{cmd}' (type 'help')")
                
                time.sleep(0.1)
                self.receive_messages()
                
            except (EOFError, KeyboardInterrupt):
                break
        
        self.stop()
        print("Stopped.")
    
    def run_with_pynput(self):
        """Run controller with pynput keyboard handling."""
        print("\n" + "=" * 50)
        print("  Pololu 3pi+ Robot Controller")
        print("=" * 50)
        print(f"Target: {self.target_ip}:{self.target_port}")
        print(f"Speed: {self.state.speed_level} ({SPEED_MAP[self.state.speed_level]})")
        print("\nW/S/A/D=Move, Space=Stop, 1-5=Speed, Q/ESC=Quit\n")
        
        def receiver():
            while self.state.running:
                self.receive_messages()
                time.sleep(0.01)
        
        threading.Thread(target=receiver, daemon=True).start()
        self.get_status()
        
        def on_press(key):
            try:
                if hasattr(key, 'char') and key.char:
                    c = key.char.lower()
                    if c == 'w': self.state.forward = True; self._update_movement()
                    elif c == 's': self.state.backward = True; self._update_movement()
                    elif c == 'a': self.state.turn_left = True; self._update_movement()
                    elif c == 'd': self.state.turn_right = True; self._update_movement()
                    elif c == 'q': self.state.running = False; return False
                    elif c in '12345':
                        self.state.speed_level = int(c)
                        print(f"Speed: {c} ({SPEED_MAP[int(c)]})")
                    elif c == 'g': self.get_position()
                    elif c == 'r': self.reset_position()
                    elif c == 'i': self.get_imu_status()
                    elif c == 'b': self.buzzer(880, 100)
                
                if key == keyboard.Key.space:
                    self.stop()
                    self.state.forward = self.state.backward = False
                    self.state.turn_left = self.state.turn_right = False
                elif key == keyboard.Key.esc:
                    self.state.running = False
                    return False
            except: pass
        
        def on_release(key):
            try:
                if hasattr(key, 'char') and key.char:
                    c = key.char.lower()
                    if c == 'w': self.state.forward = False; self._update_movement()
                    elif c == 's': self.state.backward = False; self._update_movement()
                    elif c == 'a': self.state.turn_left = False; self._update_movement()
                    elif c == 'd': self.state.turn_right = False; self._update_movement()
            except: pass
        
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            while self.state.running: time.sleep(0.1)
        
        self.stop()
        print("Stopped.")
    
    def run(self):
        """Run the controller (auto-selects mode based on pynput availability)."""
        if PYNPUT_AVAILABLE:
            self.run_with_pynput()
        else:
            print("Note: Install pynput for real-time keyboard control (pip install pynput)")
            self.run_simple()


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="3pi+ Robot UDP Controller")
    parser.add_argument('--ip', type=str, default=DEFAULT_IP, help=f'ESP32 IP (default: {DEFAULT_IP})')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help=f'UDP port (default: {DEFAULT_PORT})')
    args = parser.parse_args()
    
    controller = RobotController(target_ip=args.ip, target_port=args.port)
    
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop()
        print("\nStopped.")


if __name__ == "__main__":
    main()
