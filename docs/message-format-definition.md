# Pololu 3pi+ Robot - Message Format Definition

Communication protocol between the control server and the 3pi+ robot via ESP32 WiFi bridge.

## Overview

| Property | Value |
|----------|-------|
| Transport | JSON over UDP |
| Port | 5005 |
| Direction | Bidirectional |
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

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| left | int | -400 to 400 | Left motor speed |
| right | int | -400 to 400 | Right motor speed |
| duration | int | 0+ | Duration in ms (0 = continuous) |

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

Calibrate line sensors (robot will spin).

```json
{"cmd": "calibrate", "sensor": "line"}
```

---

## Responses (Robot → Controller)

### ack

Command acknowledgment.

```json
{"type": "ack", "cmd": "stop", "ts": 12345}
```

Move acknowledgment with details:

```json
{"type": "ack", "cmd": "move", "set_l": 200, "set_r": 200, "ts": 12345}
```

Precision move acknowledgment:

```json
{"type": "ack", "cmd": "pmove", "dir": "w", "speed": 100, "dist": 50, "imu": true, "ts": 12345}
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
```

### error

Error response.

```json
{"type": "error", "error": "unknown_cmd", "ts": 12345}
```

Error codes: `no_cmd`, `unknown_cmd`, `no_dir`, `bad_dir`, `no_dist_dur`

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
