# Pololu 3pi+ Robot - Message Format Definition

Communication protocol between the control server and the 3pi+ robot via ESP32 WiFi bridge.

## Overview

| Property  | Value                  |
| --------- | ---------------------- |
| Transport | JSON over UDP          |
| Port      | 5006                   |
| Direction | Bidirectional          |
| Baud Rate | 115200 (ESP32 ↔ Robot) |

## Architecture

```
Controller (Python/Backend) ←─UDP JSON─→ ESP32 Bridge ←─UART─→ 3pi+ Robot
```

## Hardware Connection

### ESP32-S3-WROOM-1

| 3pi+ 32U4 | Function | ESP32-S3 |
|-----------|----------|----------|
| Pin 0     | TX1      | GPIO18   |
| Pin 1     | RX1      | GPIO17   |
| GND       | Ground   | GND      |

### ESP32-C3 SuperMini

| 3pi+ 32U4 | Function | ESP32-C3 |
|-----------|----------|----------|
| Pin 0     | TX1      | GPIO5    |
| Pin 1     | RX1      | GPIO4    |
| GND       | Ground   | GND      |

**Note:** I2C pins (2/3) are used by the LSM6DS33 IMU. The OLED display (pins 0/1) is disabled during WiFi operation.

---

## Commands (Controller → Robot)

### move

Drive with specified motor speeds.

```json
{"cmd": "move", "left": 200, "right": 200, "duration": 1000}
```

| Field    | Type | Range       | Description                                                        |
| -------- | ---- | ----------- | ------------------------------------------------------------------ |
| left     | int  | -400 to 400 | Left motor speed                                                   |
| right    | int  | -400 to 400 | Right motor speed                                                  |
| duration | int  | 0+          | Duration in ms (0 = continuous)                                    |
| seq      | int  | optional    | Sequence number, echoed in the ack for RTT matching (0 if omitted) |

### stop

Stop all motors immediately.

```json
{"cmd": "stop"}
```

### pmove

Precision move with IMU-based heading correction.

```json
{"cmd": "pmove", "dir": "w", "speed": 100, "dist": 50}
{"cmd": "pmove", "dir": "s", "speed": 50, "dur": 500}
```

| Field | Type | Values | Description |
|-------|------|--------|-------------|
| dir | string | w, s, a, d | Direction (forward/back/left/right) |
| speed | int | 1-100 | Speed percentage |
| dist | int | mm | Distance (mutually exclusive with dur) |
| dur | int | ms | Duration (mutually exclusive with dist) |

### getpos

Request current position.

```json
{"cmd": "getpos"}
```

### resetpos

Reset position to origin (0, 0, 0).

```json
{"cmd": "resetpos"}
```

### setheading

Set the heading without changing x/y. Useful for correcting gyro drift, e.g. after a line-follow segment of known direction.

```json
{"cmd": "setheading", "h": 90}
```

| Field | Type | Description |
|-------|------|-------------|
| h | int | Heading in whole degrees (CCW positive) |

### imu

Request IMU status and readings.

```json
{"cmd": "imu"}
```

### status

Request full robot status.

```json
{"cmd": "status"}
```

### led

Control LEDs.

```json
{"cmd": "led", "red": true, "yellow": false, "green": true}
```

### buzzer

Play buzzer tone.

```json
{"cmd": "buzzer", "frequency": 440, "duration": 200}
```

### calibrate

Calibrate line sensors. The robot spins in place for about 1.6 s (speed 55) so the sensors sweep over the line, then resets position, heading and encoders to zero. Place the robot on the line first. Required before `linefollow`.

```json
{"cmd": "calibrate", "sensor": "line"}
```

### linefollow

Follow a black line using the 5 IR reflectance sensors (integer PID based on the Pololu reference line follower). Ends with an `lf_done` response; `lf_status` is streamed every 250 ms while running.

```json
{"cmd": "linefollow", "speed": 50, "dist": 800}
{"cmd": "linefollow", "speed": 50, "dur": 5000}
{"cmd": "linefollow", "speed": 40, "dist": 600, "search": 150}
```

| Field | Type | Values | Description |
|-------|------|--------|-------------|
| speed | int | 1-100 | Speed percentage (missing or 0 = 50) |
| dist | int | mm | Distance to follow (mutually exclusive with dur) |
| dur | int | ms | Duration to follow (mutually exclusive with dist) |
| search | int | mm | Optional. First drive straight (speed 80, holding heading) up to this distance until the line is detected, then start PID. `dist`/`dur` are counted from when the line is found. |

- Refused with error `line_not_calibrated` (motors untouched) if `calibrate` has not been run since power-up.
- If the line is lost for more than 500 ms the robot stops and sends `lf_done` with `"reason": "line_lost"`.
- `stop` and the bump sensors cancel a running line follow.

### ping

End-to-end latency measurement. The command traverses desktop -> ESP32 bridge -> robot UART and back. The robot echoes with a `pong` response including its local timestamp.

```json
{"cmd": "ping", "seq": 1, "ts": 123456}
```

| Field | Type | Description |
|-------|------|-------------|
| seq | int | Sequence number (for matching responses) |
| ts | int | Client-side timestamp in ms, **0 to 2147483647** |

`ts` must fit in a signed 32-bit integer: the robot and bridge parse it as one, so an epoch-millisecond value (e.g. `time.time() * 1000`) overflows and is echoed back wrong. Use a wrapping monotonic counter such as `int(time.monotonic() * 1000) & 0x7FFFFFFF` and compute RTT as `(now - client_ts) & 0x7FFFFFFF`.

### bridge_ping

Bridge-only latency measurement. Handled entirely by the ESP32 bridge; not forwarded to the robot. Useful for isolating WiFi/UDP latency from UART latency.

**ESP32-C3 bridge only.** The S3 bridge forwards it to the robot, which replies with error `unknown_cmd`.

```json
{"cmd": "bridge_ping", "seq": 1, "ts": 123456}
```

Same fields (and the same 32-bit `ts` rule) as `ping`.

---

## Responses (Robot → Controller)

### ack

Command acknowledgment.

```json
{"type": "ack", "cmd": "stop", "ts": 12345}
```

Move acknowledgment (echoes the command's `seq`, 0 if it had none):

```json
{"type": "ack", "cmd": "move", "seq": 12, "ts": 12345}
```

Precision move acknowledgment:

```json
{"type": "ack", "cmd": "pmove", "dir": "w", "speed": 100, "dist": 50, "imu": true, "ts": 12345}
```

Line-follow acknowledgment (`dist` or `dur`, whichever was given):

```json
{"type": "ack", "cmd": "linefollow", "speed": 50, "dist": 800, "ts": 12345}
```

Set-heading acknowledgment:

```json
{"type": "ack", "cmd": "setheading", "h": 90.0, "ts": 12345}
```

### pmove_done

Precision move completed.

```json
{
  "type": "pmove_done",
  "dir": "w",
  "dist": 48.5,
  "dur": 523,
  "pos": {"x": 48.3, "y": 0.2, "h": -0.5},
  "ts": 12345
}
```

| Field | Description |
|-------|-------------|
| dist | Actual distance traveled (mm) |
| dur | Actual duration (ms) |
| pos.x | X position (mm, forward positive) |
| pos.y | Y position (mm, left positive) |
| pos.h | Heading (degrees, CCW positive) |

### lf_status

Streamed every 250 ms during PID line tracking (not during the search phase).

```json
{
  "type": "lf_status",
  "line": 2050,
  "err": 50,
  "sns": [0, 120, 980, 150, 0],
  "spd": [120, 115],
  "dist": 312.4,
  "pos": {"x": 310.2, "y": 4.1, "h": 1.5},
  "ts": 12345
}
```

| Field | Description |
|-------|-------------|
| line | Line position 0-4000 (2000 = centred under the robot) |
| err | `line - 2000` (positive = line to the right) |
| sns | Calibrated sensor readings, 0-1000 each, left to right |
| spd | Commanded left/right motor speeds |
| dist | Distance tracked so far (mm) |

### lf_done

Line follow finished. Three forms:

```json
{"type": "lf_done", "dist": 800.3, "dur": 4120, "pos": {"x": 798.0, "y": 6.2, "h": 0.8}, "ts": 12345}
{"type": "lf_done", "reason": "line_lost", "dist": 410.0, "dur": 2100, "pos": {"x": 405.1, "y": 9.0, "h": 4.2}, "ts": 12345}
{"type": "lf_done", "reason": "no_line"}
```

| reason | Meaning |
|--------|---------|
| *(absent)* | Target distance or duration reached |
| `line_lost` | No line seen for more than 500 ms |
| `no_line` | Search phase drove `search` mm without finding the line |

### position

Position response.

```json
{
  "type": "position",
  "x": 125.3,
  "y": -10.2,
  "h": 5.8,
  "enc": [450, 455],
  "ts": 12345
}
```

### imu

IMU status response.

```json
{
  "type": "imu",
  "avail": true,
  "gyro": [0, 0, -5],
  "heading": 5.8,
  "ts": 12345
}
```

### status

Full status response.

```json
{
  "type": "status",
  "battery": 4200,
  "enc": [1234, 1230],
  "bump": [0, 0],
  "line": [100, 200, 800, 200, 100],
  "mtr": [0, 0],
  "ts": 12345
}
```

### heartbeat

Periodic heartbeat.

```json
{
  "type": "heartbeat",
  "count": 5,
  "battery": 4200,
  "mtr": [0, 0],
  "ts": 12345
}
```

### event

Event notification.

```json
{"type": "event", "event": "bump", "side": "left", "ts": 12345}
{"type": "event", "event": "startup", "imu": true, "ts": 0}
{"type": "event", "event": "line_lost", "ts": 12345}
{"type": "event", "event": "line_found", "ts": 12345}
{"type": "event", "event": "search_ok", "d": 42.0}
```

| event | Meaning |
|-------|---------|
| `bump` | Bump sensor pressed; motors stopped and any move cancelled |
| `startup` | Robot firmware booted |
| `line_lost` | Line follow: all sensors lost the line (robot keeps driving straight for up to 500 ms) |
| `line_found` | Line follow: line reacquired within the 500 ms window |
| `search_ok` | Line follow search phase found the line after `d` mm; PID tracking starts |

### error

Error response.

```json
{"type": "error", "error": "unknown_cmd", "ts": 12345}
```

Error codes: `no_cmd`, `unknown_cmd`, `no_dir`, `bad_dir`, `no_dist_dur`, `line_not_calibrated`

### pong

End-to-end ping response from the robot.

```json
{"type": "pong", "seq": 1, "client_ts": 123456, "robot_ts": 12345}
```

| Field | Type | Description |
|-------|------|-------------|
| seq | int | Echoed sequence number |
| client_ts | int | Echoed client timestamp |
| robot_ts | int | Robot-side millis() at time of response |

### bridge_pong

Bridge-only ping response from the ESP32.

```json
{"type": "bridge_pong", "seq": 1, "client_ts": 123456, "bridge_ts": 54321}
```

| Field | Type | Description |
|-------|------|-------------|
| seq | int | Echoed sequence number |
| client_ts | int | Echoed client timestamp |
| bridge_ts | int | ESP32-side millis() at time of response |

---

## Bridge Discovery (ESP32-C3 bridge only)

Lets a client find the bridge without knowing its IP. Uses a separate UDP port, **5004**.

1. The bridge broadcasts a beacon to the subnet broadcast address every 2 s (every 10 s once a client has sent on the data port):

   ```json
   {"type": "discovery", "service": "pololu-3pi-bridge", "ip": "192.168.7.210", "port": 5006, "mac": "AA:BB:CC:DD:EE:FF", "board": "ESP32-C3", "rssi": -55, "uptime": 123}
   ```

2. The client replies to the beacon's source address and port:

   ```json
   {"cmd": "discover_ack"}
   ```

3. The bridge confirms:

   ```json
   {"type": "discover_confirm", "ip": "192.168.7.210", "port": 5006, "mac": "AA:BB:CC:DD:EE:FF", "board": "ESP32-C3"}
   ```

The client then sends commands to `ip`:`port` as usual.

---

## Robot Constants

| Parameter | Value | Description |
|-----------|-------|-------------|
| WHEEL_DIAMETER | 32.0 mm | Wheel diameter |
| WHEEL_BASE | 85.0 mm | Distance between wheels |
| GEAR_RATIO | 29.86:1 | Motor gear reduction |
| ENCODER_CPR | 12 | Counts per motor revolution |
| MAX_SPEED | 400 | Maximum motor speed |

## PID Constants (Heading Correction)

| Parameter | Value |
|-----------|-------|
| KP | 0.8 |
| KI | 0.0 |
| KD | 0.1 |

---

## Timing

| Property | Value |
|----------|-------|
| Heartbeat interval | 2000 ms |
| Bridge heartbeat | 5000 ms |
| Max command rate | ~50 Hz |
| Response latency | < 20 ms |
