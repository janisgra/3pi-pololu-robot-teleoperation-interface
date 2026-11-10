# Tutorial: Getting Started with the 3pi+ WiFi Robot

This guide takes you from a Pololu 3pi+ and an ESP32 board to sending your first command over WiFi, then driving, line following, and the serpentine test.

For every message field and response type, see [message-format-definition.md](message-format-definition.md).

---

## What you need

**Hardware**
- Pololu 3pi+ 32U4 OLED robot with charged batteries
- ESP32-C3 SuperMini (recommended) or ESP32-S3-WROOM-1
- 3 jumper wires
- USB cables for the robot and the ESP32
- For line following: a white surface and black tape (25 mm wide recommended)

**Software**
- VS Code with the PlatformIO extension, or the PlatformIO command line (`pio`)
- Python 3.8 or newer
- A **2.4 GHz** WiFi network that both your computer and the ESP32 can join (the ESP32 does not support 5 GHz)

The C3 bridge is recommended because only it supports auto-discovery (the tools find it without an IP address) and `bridge_ping` (tests WiFi separately from the robot).

---

## 1. How the pieces fit together

```
Your Python script  --UDP JSON, port 5006-->  ESP32 bridge  --UART wire-->  3pi+ robot
                    <--------------------                   <-------------
```

- You send one JSON command per UDP packet to the bridge's IP on port **5006**.
- The bridge passes each command to the robot over the serial wire.
- Everything the robot prints (acks, heartbeats, telemetry) goes back to **whoever sent the most recent packet**.

This means you must send something before you receive anything, and only one controller can be in use at a time.

---

## 2. Wire the bridge to the robot

TX goes to RX and RX goes to TX:

| 3pi+ pin | Function | ESP32-C3 | ESP32-S3 |
| -------- | -------- | -------- | -------- |
| Pin 0    | TX1      | GPIO5    | GPIO18   |
| Pin 1    | RX1      | GPIO4    | GPIO17   |
| GND      | Ground   | GND      | GND      |

Power the ESP32 from USB. The robot runs on its own batteries.

> The robot's OLED display shares these pins, so it stays blank while the bridge is connected. This is expected.

---

## 3. Flash the robot firmware

Connect the robot by USB, switch it on, and from the repository root run:

```bash
pio run -e pololu-3pi -t upload
```

Afterwards, **switch the robot off and on again** so the motors initialise correctly.

---

## 4. Set the WiFi and flash the bridge

Open `src/esp32/esp32c3_bridge.cpp` (or `esp32_bridge.cpp` for the S3) and set your network:

```cpp
#define WIFI_SSID     "YourNetwork"
#define WIFI_PASSWORD "YourPassword"
```

Connect the ESP32 by USB and flash it:

```bash
pio run -e esp32c3-wifi-bridge -t upload     # S3: -e esp32-wifi-bridge
```

Open the serial monitor to find the bridge's IP address:

```bash
pio device monitor -e esp32c3-wifi-bridge
```

Look for:

```
WiFi connected!
IP: 192.168.7.210, RSSI: -52 dBm, MAC: ...
```

Write down the IP. To keep it from changing, reserve it for the ESP32's MAC address in your router's DHCP settings.

---

## 5. Send your first command

[`tools/hello_robot.py`](../tools/hello_robot.py) is the smallest complete client. It makes the robot beep and prints the robot's reply:

```python
import socket

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2)
s.sendto(b'{"cmd":"buzzer"}', ("192.168.7.210", 5006))  # bridge IP; robot beeps
while b'"ack"' not in (reply := s.recv(512)):
    pass  # skip bridge_status etc.
print(reply.decode())
```

Replace `192.168.7.210` with your bridge's IP, then run:

```bash
cd tools
python hello_robot.py
```

The robot beeps (440 Hz, 200 ms) and you see:

```json
{"type":"ack","cmd":"buzzer","ts":53120}
```

`ts` is the robot's time in milliseconds since it was switched on.

**Why the loop?** Your first packet makes the bridge treat you as the client, and it immediately sends you its own `bridge_status` message. The robot's heartbeat (every 2 s) can also arrive at any time. The loop skips everything until the robot's `ack`.

If nothing arrives within 2 seconds, Python raises `TimeoutError` (`socket.timeout`). See [Troubleshooting](#troubleshooting).

---

## 6. Drive it

> **Safety first:** put the robot on the floor. A `move` with `"duration": 0` keeps driving until you send `stop`. The bump sensors stop the robot automatically.

### Keyboard control

```bash
pip install -r requirements.txt   # pynput gives real-time key control
python robot_controller.py --ip 192.168.7.210
```

Keys: **W/A/S/D** to drive, **Space** to stop, **1-5** for speed presets, **G** to get position, **R** to reset position, **Q** to quit.

### Raw commands

You can send any of these exactly like the buzzer command in step 5:

```json
{"cmd":"move","left":150,"right":150,"duration":1000}
{"cmd":"pmove","dir":"w","speed":50,"dist":200}
{"cmd":"getpos"}
{"cmd":"stop"}
```

- `move` sets the raw motor speeds (-400 to 400) for `duration` ms.
- `pmove` drives a set distance in mm, using the gyro to keep straight, and ends with a `pmove_done` message containing the final position.

### From Python

```python
from robot_controller import RobotController

robot = RobotController(target_ip="192.168.7.210")
robot.precision_move('w', speed=50, distance=200)   # forward 200 mm
while (msg := robot.receive_messages()) is None or msg.get("type") != "pmove_done":
    pass
print("Now at", msg["pos"])
```

---

## 7. Follow a line

1. Lay a strip of black tape on a white surface, at least 60 cm long.
2. Place the robot so the sensor bar underneath sits **over the tape**.
3. **Calibrate.** The robot spins left and right for about 1.6 s to learn what black and white look like. Wait for the `calibrate` ack before sending anything else, because the robot does not read commands while spinning. Calibration lasts until the robot is switched off. It also resets the position and heading to zero.
4. **Follow.** Send `linefollow` with a speed (1-100 %) and a distance in mm, or a duration in ms.
5. **Watch.** The robot sends `lf_status` every 250 ms and `lf_done` when it stops.

```python
import json, socket

BRIDGE = ("192.168.7.210", 5006)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(5)

def send(cmd):
    s.sendto(json.dumps(cmd).encode(), BRIDGE)

def wait_for(msg_type, cmd=None):
    while True:
        msg = json.loads(s.recv(512))
        print(msg)
        if msg.get("type") == "error":
            raise RuntimeError(msg["error"])
        if msg.get("type") == msg_type and cmd in (None, msg.get("cmd")):
            return msg

send({"cmd": "calibrate", "sensor": "line"})
wait_for("ack", "calibrate")

send({"cmd": "linefollow", "speed": 40, "dist": 500})
done = wait_for("lf_done")
print("Stopped:", done.get("reason", "distance reached"))
```

How a line follow can end:

| `lf_done`               | Meaning                                        |
| ----------------------- | ---------------------------------------------- |
| no `reason`             | Distance or duration reached                   |
| `"reason": "line_lost"` | No line under the sensors for more than 500 ms |
| `"reason": "no_line"`   | The search phase (below) did not find the line |

If you skip calibration you get `{"type":"error","error":"line_not_calibrated"}` and the robot does not move.

**Starting next to the line:** add `"search": 150` to drive straight up to 150 mm until the sensors find the line. Line tracking, and the distance count, start from there. This is how the serpentine test re-acquires the tape after each U-turn.

---

## 8. Run the serpentine test (optional)

The serpentine test drives the robot along a back-and-forth tape course while plotting its position live. It is used for checking IR camera tracking against a known path.

```bash
cd tools
pip install -r requirements.txt
```

1. **Lay out the track.** The default course for a 122 × 80 cm board is already in `serpentine_layout.svg`, drawn at 1:1 scale in millimetres, so you can use it as a taping template. To change the board size or track spacing, generate a new layout and ground-truth path:

   ```bash
   python pattern_generator.py --svg serpentine_layout.svg --csv pololu-robot.csv
   ```

   Run `python pattern_generator.py --help` for all options.

2. **Preview** the course without a robot:

   ```bash
   python serpentine_drive_test.py --dry-run
   ```

3. **Tune the U-turns** on your surface. In the interactive tuner, type `help` for the full command list:

   ```bash
   python uturn_tuner.py --ip 192.168.7.210
   > cal
   > lf 300
   > uturn 100
   ```

4. **Run it.** Place the robot at the start point shown on the layout, then:

   ```bash
   python serpentine_drive_test.py                       # C3: finds the bridge automatically
   python serpentine_drive_test.py --ip 192.168.7.210    # S3, or if discovery fails
   ```

   The script checks the link, calibrates, drives the course, and writes the trajectory to `serpentine_log_<timestamp>.jsonl` in the current folder. Add `-v` for detailed logs, including ping round-trip times.

---

## Troubleshooting

| Symptom                                                                       | Likely cause and fix                                                                                                                                                                                                                                                    |
| ----------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `TimeoutError` / `socket.timeout`, or `ConnectionResetError [WinError 10054]` | Nothing answered at that IP and port. Check the IP in the serial monitor. Make sure your computer is on the same network (not a guest network). Check that both bridge and tools use port 5006: older firmware used 5005.                                               |
| `bridge_status` arrives but never an `ack`                                    | The bridge works but the robot doesn't answer. Check that the robot is switched on, TX/RX are crossed (not straight), GND is connected, and the robot firmware is flashed. On a C3, if `bridge_ping` works but `ping` doesn't, the problem is between bridge and robot. |
| "Auto-discovery failed"                                                       | The S3 bridge has no discovery, so use `--ip`. On a C3, Windows Firewall may be blocking incoming broadcasts on UDP port 5004: allow Python on private networks, or use `--ip`.                                                                                         |
| `line_not_calibrated`                                                         | Run `calibrate` first, and again after every power cycle.                                                                                                                                                                                                               |
| `lf_done` with `line_lost` right away                                         | The robot isn't over the line, or was calibrated away from it. Recalibrate with the sensors over the tape, or use `search`.                                                                                                                                             |
| Robot stops on its own                                                        | A bump sensor was pressed, or the batteries are low. `{"cmd":"status"}` reports `battery` in mV.                                                                                                                                                                        |
| Motors don't move after flashing                                              | Switch the robot off and on again.                                                                                                                                                                                                                                      |
| A long command is ignored                                                     | The robot reads at most 127 characters per command. Send one short command per packet.                                                                                                                                                                                  |
| OLED display is blank                                                         | Expected: the display pins are used by the bridge.                                                                                                                                                                                                                      |

---

## Where to go next

- [message-format-definition.md](message-format-definition.md): every command, response, and event
- [`tools/robot_controller.py`](../tools/robot_controller.py): the reusable Python controller class
- [`ref-code/POLOLU_3PI_REFERENCE.md`](../ref-code/POLOLU_3PI_REFERENCE.md): notes on the 3pi line-following and maze-solving examples
