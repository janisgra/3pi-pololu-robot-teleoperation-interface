# Pololu 3pi+ Robot - WiFi Teleoperation Interface

![Status](https://img.shields.io/badge/status-working-brightgreen)
![Tested](https://img.shields.io/badge/tested-yes-brightgreen)
![Maintenance](https://img.shields.io/badge/maintenance-bug%20fixes%20only-yellow)
![License](https://img.shields.io/badge/license-Unlicense-blue)

WiFi teleoperation interface for the Pololu 3pi+ 32U4 OLED robot using an ESP32 bridge.

**Author:** Janis

---

## Important Notice

**The OLED display will NOT work when using WiFi teleoperation.**

The ATmega32U4's Serial1 (pins 0/1) is shared between the display and UART communication. When WiFi control is enabled, the display pins are used for the ESP32 bridge connection instead.

---

## Overview

```
┌──────────────────┐      WiFi/UDP        ┌───────────────┐      UART      ┌──────────────────┐
│  Python Script   │ ◄──────────────────► │  ESP32 Bridge │ ◄────────────► │  3pi+ 32U4 Robot │
│  or your backend │      JSON msgs       │  (S3 or C3)   │   115200 baud  │  ATmega32U4      │
└──────────────────┘                      └───────────────┘                └──────────────────┘
```

The system consists of three components:

1. **Robot Firmware** (`src/main.cpp`) - Runs on the 3pi+ 32U4
2. **ESP32 WiFi Bridge** (`src/esp32/`) - Bridges UDP↔UART
3. **Python Controller** (`tools/robot_controller.py`) - Interactive or programmatic control

**New here?** Follow the step-by-step [tutorial](docs/tutorial.md), from wiring to your first command (`tools/hello_robot.py`) to line following.

## Features

- **Basic movement**: Forward, backward, turn left/right with speed control
- **Precision moves**: IMU-based PID heading correction for straight-line accuracy
- **Position tracking**: X, Y, heading relative to start
- **Line following**: PID line tracking on the IR reflectance sensors, with optional line search
- **Sensor access**: Encoders, bump sensors, line sensors, battery voltage
- **LED & buzzer control**: Visual and audio feedback
- **Heartbeat monitoring**: Connection health tracking
- **Latency measurement**: End-to-end and bridge-only ping
- **Bridge auto-discovery** (ESP32-C3 bridge only): UDP broadcast beacon on port 5004

## Hardware Requirements

- Pololu 3pi+ 32U4 OLED Robot
- ESP32-S3-WROOM-1 **or** ESP32-C3 SuperMini
- 3 jumper wires (TX, RX, GND)

## Wiring

### ESP32-S3-WROOM-1

| 3pi+ 32U4 Pin | Function | ESP32-S3 Pin |
|---------------|----------|--------------|
| Pin 0 (PD2)   | TX1      | GPIO18 (RX)  |
| Pin 1 (PD3)   | RX1      | GPIO17 (TX)  |
| GND           | Ground   | GND          |

### ESP32-C3 SuperMini (Alternative)

| 3pi+ 32U4 Pin | Function | ESP32-C3 Pin |
|---------------|----------|--------------|
| Pin 0 (PD2)   | TX1      | GPIO5 (RX)   |
| Pin 1 (PD3)   | RX1      | GPIO4 (TX)   |
| GND           | Ground   | GND          |

**Power the ESP32 separately via USB.**

## Building & Flashing

### 3pi+ Robot Firmware

```bash
pio run -e pololu-3pi -t upload
```

**After flashing: Power cycle the 3pi+ to ensure motors initialize correctly.**

### ESP32-S3 Bridge

```bash
pio run -e esp32-wifi-bridge -t upload
```

### ESP32-C3 Bridge

```bash
pio run -e esp32c3-wifi-bridge -t upload
```

## Configuration

Edit WiFi credentials in the ESP32 bridge source:

```cpp
// src/esp32/esp32_bridge.cpp or esp32c3_bridge.cpp
#define WIFI_SSID     "YourNetwork"
#define WIFI_PASSWORD "YourPassword"
```

## Python Controller

### Interactive Mode

```bash
cd tools
pip install pynput  # Optional, for real-time keyboard control
python robot_controller.py --ip 192.168.7.210 --port 5006
```

Controls:
- **W/S/A/D** - Movement
- **Space** - Stop
- **1-5** - Speed presets
- **G** - Get position
- **R** - Reset position
- **Q/ESC** - Quit

### Programmatic Use

```python
from robot_controller import RobotController
import time

# Create controller
robot = RobotController(target_ip="192.168.7.210", target_port=5006)

# Basic movement
robot.move(200, 200, duration=1000)  # Forward 1 second
time.sleep(1.2)
robot.stop()

# Precision move with IMU correction
robot.precision_move('w', speed=100, distance=100)  # Forward 100mm at 100%

# Read messages
while True:
    msg = robot.receive_messages()
    if msg and msg.get('type') == 'pmove_done':
        print(f"Moved to: X={msg['pos']['x']}, Y={msg['pos']['y']}")
        break
    time.sleep(0.1)

robot.stop()
```

## JSON Protocol

All communication uses JSON over UDP (port 5006).

### Commands (Controller → Robot)

| Command | Example |
|---------|---------|
| Move | `{"cmd":"move","left":200,"right":200,"duration":1000}` |
| Stop | `{"cmd":"stop"}` |
| Precision Move | `{"cmd":"pmove","dir":"w","speed":100,"dist":50}` |
| Get Position | `{"cmd":"getpos"}` |
| Reset Position | `{"cmd":"resetpos"}` |
| Set Heading | `{"cmd":"setheading","h":90}` |
| Get IMU | `{"cmd":"imu"}` |
| Get Status | `{"cmd":"status"}` |
| LED | `{"cmd":"led","red":true,"yellow":false,"green":true}` |
| Buzzer | `{"cmd":"buzzer","frequency":440,"duration":200}` |
| Calibrate Line Sensors | `{"cmd":"calibrate","sensor":"line"}` |
| Line Follow | `{"cmd":"linefollow","speed":50,"dist":800,"search":150}` |
| Ping (E2E) | `{"cmd":"ping","seq":1,"ts":0}` |
| Bridge Ping (C3 only) | `{"cmd":"bridge_ping","seq":1,"ts":0}` |

### Responses (Robot → Controller)

| Type | Example |
|------|---------|
| Acknowledgment | `{"type":"ack","cmd":"stop","ts":12345}` |
| Move Ack | `{"type":"ack","cmd":"move","seq":12,"ts":12345}` |
| Position | `{"type":"position","x":100.5,"y":0.0,"h":0.0,"enc":[500,500],"ts":12345}` |
| Precision Move Done | `{"type":"pmove_done","dir":"w","dist":50.2,"dur":523,"pos":{"x":50.1,"y":0.2,"h":-0.3},"ts":12345}` |
| Line Follow Status | `{"type":"lf_status","line":2050,"err":50,"sns":[0,120,980,150,0],"spd":[120,115],"dist":312.4,"pos":{...},"ts":12345}` |
| Line Follow Done | `{"type":"lf_done","dist":800.3,"dur":4120,"pos":{...},"ts":12345}` |
| Status | `{"type":"status","battery":4200,"enc":[500,500],"bump":[0,0],"mtr":[0,0],"ts":12345}` |
| Event | `{"type":"event","event":"bump","side":"left","ts":12345}` |
| Error | `{"type":"error","error":"unknown_cmd","ts":12345}` |
| Pong (E2E) | `{"type":"pong","seq":1,"client_ts":0,"robot_ts":12345}` |
| Bridge Pong | `{"type":"bridge_pong","seq":1,"client_ts":0,"bridge_ts":54321}` |

See `docs/message-format-definition.md` for the complete protocol specification.

## Latency Benchmarking

The system supports two levels of latency measurement:

| Target | Command | Measures |
|--------|---------|----------|
| Bridge only (C3 bridge) | `bridge_ping` | Desktop -> ESP32 WiFi UDP round-trip |
| End-to-end | `ping` | Desktop -> ESP32 -> Robot UART -> ESP32 -> Desktop |

Both echo the request's `ts` back as `client_ts`, so RTT is `now - client_ts`. `ts` must fit in a signed 32-bit integer (the robot and bridge parse it as one), so use a wrapping monotonic counter rather than epoch milliseconds:

```python
ts = int(time.monotonic() * 1000) & 0x7FFFFFFF        # send this
rtt = (int(time.monotonic() * 1000) - client_ts) & 0x7FFFFFFF   # on reply
```

`serpentine_drive_test.py` and `uturn_tuner.py` use these pings to check the bridge and robot link at startup; run `serpentine_drive_test.py -v` to log the measured RTTs.

Expected latencies on a local WiFi network:
- Bridge ping: 2-10 ms (WiFi UDP only)
- End-to-end: 5-25 ms (WiFi + UART at 115200 baud)

## Project Structure

```
├── src/
│   ├── main.cpp               # 3pi+ robot firmware
│   └── esp32/
│       ├── esp32_bridge.cpp   # ESP32-S3 WiFi bridge
│       └── esp32c3_bridge.cpp # ESP32-C3 WiFi bridge
├── tools/
│   ├── hello_robot.py           # Minimal example: send one command
│   ├── robot_controller.py      # Python controller
│   ├── serpentine_drive_test.py # Serpentine line-follow run with live plot
│   ├── uturn_tuner.py           # Interactive U-turn tuning prompt
│   ├── pattern_generator.py     # Serpentine track generator (SVG + CSV)
│   ├── serpentine_layout.svg    # Generated tape layout
│   ├── pololu-robot.csv         # Generated ground-truth path
│   └── requirements.txt         # Python dependencies
├── docs/
│   ├── tutorial.md                   # Getting-started guide
│   ├── message-format-definition.md  # Protocol specification
│   └── serpentine_log_*.jsonl        # Recorded serpentine runs
├── ref-code/                  # Pololu 3pi sample code + reference notes
├── platformio.ini             # Build configuration
└── LICENSE                    # Unlicense (public domain)
```

## License

This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or distribute this software, either in source code form or as a compiled binary, for any purpose, commercial or non-commercial, and by any means.

Credits to Janis would be appreciated but are not required.

See LICENSE file for full text.
