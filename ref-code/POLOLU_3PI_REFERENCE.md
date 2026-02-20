# Pololu 3pi Robot Reference Code Documentation

> **Source**: [Pololu 3pi Robot User's Guide](https://www.pololu.com/docs/0J21)
>
> This document contains reference documentation from the Pololu AVR Library for use in the EMG-HCI teleoperation project.

---

## Contents

- [Overview](#overview)
- [Section 7.b: Simple Line Following Algorithm](#section-7b-simple-line-following-algorithm)
- [Section 7.c: PID Line Following](#section-7c-pid-line-following)
- [Section 8.a: Solving a Line Maze](#section-8a-solving-a-line-maze)
- [Section 8.b: Working with Multiple C Files](#section-8b-working-with-multiple-c-files)
- [Section 8.c: Left Hand on the Wall](#section-8c-left-hand-on-the-wall)
- [Section 8.d: The Main Loops](#section-8d-the-main-loops)
- [Section 8.e: Simplifying the Solution](#section-8e-simplifying-the-solution)
- [Section 8.f: Improving the Maze-Solving Code](#section-8f-improving-the-maze-solving-code)
- [Key Functions Reference](#key-functions-reference)

---

## Overview

These examples demonstrate core 3pi functionality:

| Example              | Directory               | Purpose                                       |
| -------------------- | ----------------------- | --------------------------------------------- |
| Simple Line Follower | `3pi-linefollower/`     | Basic bang-bang control line following        |
| PID Line Follower    | `3pi-linefollower-pid/` | Smooth PID-based line following               |
| Maze Solver          | `3pi-mazesolver/`       | Intersection navigation and path optimization |

---

## Section 7.b: Simple Line Following Algorithm

**Location**: `examples/atmegaxx8/3pi-linefollower`

The source code demonstrates a variety of different features of the 3pi, including the line sensors, motors, LCD, battery voltage monitor, and buzzer. The program has two phases.

### Phase 1: Initialization and Calibration

The first phase is handled by the function `initialize()`, called once at the beginning of `main()`:

1. **`pololu_3pi_init(2000)`** - Sets up the 3pi with sensor timeout of 2000 x 0.4us = 800us. Sensor values range from 0 (white) to 2000 (black), where 2000 indicates the capacitor took at least 800us to discharge.

2. **Display battery voltage** - Uses `read_battery_millivolts()`. Important to monitor battery so robot doesn't unexpectedly shut down during competition or programming.

3. **Calibrate sensors** - Turn the 3pi right and left on the line while calling `calibrate_line_sensors()`. Min/max values stored in RAM allow `read_line_sensors_calibrated()` to return values adjusted to 0-1000 for each sensor.

4. **Display calibrated values** - Bar graph using `lcd_load_custom_character()` with `print_character()` to verify sensors before starting.

5. **Wait for button press** - Uses `button_is_pressed()` to wait for button B. Critical so robot doesn't drive off table or out of hands.

### Phase 2: Main Loop

The `while(1)` loop takes sensor readings and sets motor speed:

1. **`read_line()`** - Returns position 0-4000:
   - 0 = line to left of sensor 0
   - 1000 = line under sensor 1
   - 2000 = line under sensor 2
   - etc.

2. **Three-state control**:
   - **0-1000**: Far right of line -> turn left (motors 0, 100)
   - **1000-3000**: Centered -> straight (motors 100, 100)
   - **3000-4000**: Far left of line -> turn right (motors 100, 0)

3. **LED indication** - Corresponding LEDs turned on for debugging.

### Simple Line Follower Source Code

```c
/*
 * 3pi-linefollower - demo code for the Pololu 3pi Robot
 *
 * This code will follow a black line on a white background, using a
 * very simple algorithm. It demonstrates auto-calibration and use of
 * the 3pi IR sensors, motor control, bar graphs using custom
 * characters, and music playback.
 *
 * http://www.pololu.com/docs/0J21
 */

#include <pololu/3pi.h>
#include <avr/pgmspace.h>

// Introductory messages stored in program space
const char welcome_line1[] PROGMEM = " Pololu";
const char welcome_line2[] PROGMEM = "3\xf7 Robot";
const char demo_name_line1[] PROGMEM = "Line";
const char demo_name_line2[] PROGMEM = "follower";

// Simple tunes stored in program space
const char welcome[] PROGMEM = ">g32>>c32";
const char go[] PROGMEM = "L16 cdegreg4";

// Bar graph character data
const char levels[] PROGMEM = {
    0b00000, 0b00000, 0b00000, 0b00000,
    0b00000, 0b00000, 0b00000, 0b11111,
    0b11111, 0b11111, 0b11111, 0b11111,
    0b11111, 0b11111
};

// Load custom characters for bar graph (7 levels)
void load_custom_characters()
{
    lcd_load_custom_character(levels+0, 0);
    lcd_load_custom_character(levels+1, 1);
    lcd_load_custom_character(levels+2, 2);
    lcd_load_custom_character(levels+3, 3);
    lcd_load_custom_character(levels+4, 4);
    lcd_load_custom_character(levels+5, 5);
    lcd_load_custom_character(levels+6, 6);
    clear();
}

// Display sensor readings as bar graph
void display_readings(const unsigned int *calibrated_values)
{
    unsigned char i;
    for (i = 0; i < 5; i++) {
        const char display_characters[10] = {' ',0,0,1,2,3,4,5,6,255};
        char c = display_characters[calibrated_values[i] / 101];
        print_character(c);
    }
}

// Initialize, display welcome, calibrate, play music
void initialize()
{
    unsigned int counter;
    unsigned int sensors[5];

    pololu_3pi_init(2000);
    load_custom_characters();

    // Welcome message and music
    print_from_program_space(welcome_line1);
    lcd_goto_xy(0, 1);
    print_from_program_space(welcome_line2);
    play_from_program_space(welcome);
    delay_ms(1000);

    clear();
    print_from_program_space(demo_name_line1);
    lcd_goto_xy(0, 1);
    print_from_program_space(demo_name_line2);
    delay_ms(1000);

    // Display battery voltage, wait for button
    while (!button_is_pressed(BUTTON_B)) {
        int bat = read_battery_millivolts();
        clear();
        print_long(bat);
        print("mV");
        lcd_goto_xy(0, 1);
        print("Press B");
        delay_ms(100);
    }

    wait_for_button_release(BUTTON_B);
    delay_ms(1000);

    // Auto-calibration: turn right and left
    for (counter = 0; counter < 80; counter++) {
        if (counter < 20 || counter >= 60)
            set_motors(40, -40);
        else
            set_motors(-40, 40);

        calibrate_line_sensors(IR_EMITTERS_ON);
        delay_ms(20);
    }
    set_motors(0, 0);

    // Display calibrated values as bar graph
    while (!button_is_pressed(BUTTON_B)) {
        unsigned int position = read_line(sensors, IR_EMITTERS_ON);
        clear();
        print_long(position);
        lcd_goto_xy(0, 1);
        display_readings(sensors);
        delay_ms(100);
    }
    wait_for_button_release(BUTTON_B);

    clear();
    print("Go!");
    play_from_program_space(go);
    while (is_playing());
}

int main()
{
    unsigned int sensors[5];
    initialize();

    while (1) {
        unsigned int position = read_line(sensors, IR_EMITTERS_ON);

        if (position < 1000) {
            // Far right of line: turn left
            set_motors(0, 100);
            left_led(1);
            right_led(0);
        }
        else if (position < 3000) {
            // Centered: drive straight
            set_motors(100, 100);
            left_led(1);
            right_led(1);
        }
        else {
            // Far left of line: turn right
            set_motors(100, 0);
            left_led(0);
            right_led(1);
        }
    }

    // Never reached - robot should never end its program
    // while(1);
}
```

### Improvement Ideas

- Increase maximum speed beyond 100/255
- Add more intermediate cases for smoother motion
- Implement speed memory (increase after consistent tracking)
- Measure loop speed using timing functions or LED blinks
- Display sensor readings on LCD (few times per second max)
- Add buzzer music using PLAY_CHECK to avoid disrupting sensors

---

## Section 7.c: PID Line Following

**Location**: `examples/atmegaxx8/3pi-linefollower-pid`

PID control addresses the jerkiness of the simple algorithm by using continuous functions for smooth motor response. PID stands for Proportional, Integral, Derivative.

### PID Components

| Term             | Calculation                        | Purpose                                  |
| ---------------- | ---------------------------------- | ---------------------------------------- |
| **Proportional** | `position - 2000`                  | Current position error (0 when centered) |
| **Integral**     | Sum of all proportional values     | Historical motion record                 |
| **Derivative**   | `proportional - last_proportional` | Rate of change                           |

### PID Computation Code

```c
// Get line position (must provide sensors array)
unsigned int position = read_line(sensors, IR_EMITTERS_ON);

// Proportional term: 0 when on line
int proportional = ((int)position) - 2000;

// Derivative (change) and integral (sum)
int derivative = proportional - last_proportional;
integral += proportional;

// Remember last position
last_proportional = proportional;
```

**Note**: Cast `position` to `int` before subtracting 2000. Unsigned int cannot store negative values, which would cause overflow.

### Motor Speed Calculation

```c
// Compute motor power difference
// Positive = turn right, Negative = turn left
int power_difference = proportional/20 + integral/10000 + derivative*3/2;

// Clamp to maximum
const int max = 60;
if (power_difference > max)
    power_difference = max;
if (power_difference < -max)
    power_difference = -max;

// Apply to motors (never negative)
if (power_difference < 0)
    set_motors(max + power_difference, max);
else
    set_motors(max, max - power_difference);
```

### Tuning Parameters

| Parameter    | Default | Effect                       |
| ------------ | ------- | ---------------------------- |
| Proportional | 1/20    | Immediate response strength  |
| Integral     | 1/10000 | Accumulated error correction |
| Derivative   | 3/2     | Oscillation dampening        |
| Max speed    | 60      | Safe starting value          |

**Tuning process**:

1. Start with max speed of 100
2. Adjust PID parameters until stable
3. Gradually increase max speed
4. Re-tune parameters at each speed level
5. Maximum speed of 255 achievable on 6"-radius curves with proper tuning

---

## Section 8.a: Solving a Line Maze

Line mazes are networks of intersecting black lines with a goal circle. Robots travel from start to goal, tracking intersections. Multiple runs allow learning the fastest path.

### Maze Types

The examples solve **non-looped mazes** - no way to revisit a point without retracing steps. This is easier than looped mazes because a simple exploration strategy covers the entire maze.

Mazes are typically straight lines on a regular grid, but the solving strategy doesn't require this.

---

## Section 8.b: Working with Multiple C Files

**Location**: `examples/atmegaxx8/3pi-mazesolver`

The maze solver is split into multiple files for organization.

### turn.c - Turn Execution

```c
#include <pololu/3pi.h>

// Turn based on direction: 'L', 'R', 'S' (straight), 'B' (back)
void turn(char dir)
{
    switch (dir) {
    case 'L':
        set_motors(-80, 80);
        delay_ms(200);
        break;
    case 'R':
        set_motors(80, -80);
        delay_ms(200);
        break;
    case 'B':
        set_motors(80, -80);
        delay_ms(400);
        break;
    case 'S':
        // No action needed
        break;
    }
}
```

### turn.h - Header File

```c
void turn(char dir);
```

Include with `#include "turn.h"` (double quotes for project files, not system headers).

### follow-segment.c - Line Following Until Intersection

```c
void follow_segment()
{
    int last_proportional = 0;
    long integral = 0;

    while (1) {
        unsigned int sensors[5];
        unsigned int position = read_line(sensors, IR_EMITTERS_ON);

        // PID control (max speed 60 for reliability)
        int proportional = ((int)position) - 2000;
        int derivative = proportional - last_proportional;
        integral += proportional;
        last_proportional = proportional;

        int power_difference = proportional/20 + integral/10000 + derivative*3/2;

        const int max = 60;
        if (power_difference > max)
            power_difference = max;
        if (power_difference < -max)
            power_difference = -max;

        if (power_difference < 0)
            set_motors(max + power_difference, max);
        else
            set_motors(max, max - power_difference);

        // Check for dead end (inner sensors 1,2,3)
        if (sensors[1] < 100 && sensors[2] < 100 && sensors[3] < 100) {
            return;  // Dead end
        }
        // Check for intersection (outer sensors 0,4)
        else if (sensors[0] > 200 || sensors[4] > 200) {
            return;  // Found intersection
        }
    }
}
```

---

## Section 8.c: Left Hand on the Wall

The basic strategy for non-looped mazes: keep your left hand on the wall at all times.

- Turn left whenever possible
- Go straight if no left turn
- Turn right only if no other exit
- Turn back (180 deg) at dead ends

This explores every hallway exactly twice and guarantees finding the goal.

### select_turn() Implementation

```c
// Decide turn direction using left-hand-on-wall strategy
char select_turn(unsigned char found_left, unsigned char found_straight,
                 unsigned char found_right)
{
    if (found_left)
        return 'L';
    else if (found_straight)
        return 'S';
    else if (found_right)
        return 'R';
    else
        return 'B';  // Dead end - turn back
}
```

---

## Section 8.d: The Main Loops

The maze solver has two main loops in `maze-solve.c`:

### Path Storage

```c
char path[100] = "";
unsigned char path_length = 0;
```

### maze_solve() Structure

```c
void maze_solve()
{
    // FIRST LOOP: Learning phase
    while (1) {
        // Explore maze, record turns
        // Break when goal found
    }

    // SECOND LOOP: Replay phase (infinite)
    while (1) {
        // Wait for button press
        // Execute optimized path
        for (int i = 0; i < path_length; i++) {
            // Follow segment and turn
        }
        follow_segment();  // Final segment to finish
    }
}
```

### First Main Loop Body (Learning)

```c
follow_segment();

// Drive straight to align with intersection
set_motors(50, 50);
delay_ms(50);

unsigned char found_left = 0;
unsigned char found_straight = 0;
unsigned char found_right = 0;

// Read sensors for intersection type
unsigned int sensors[5];
read_line(sensors, IR_EMITTERS_ON);

// Check left and right exits
if (sensors[0] > 100)
    found_left = 1;
if (sensors[4] > 100)
    found_right = 1;

// Line up wheels with intersection
set_motors(40, 40);
delay_ms(200);

// Check for straight exit
read_line(sensors, IR_EMITTERS_ON);
if (sensors[1] > 200 || sensors[2] > 200 || sensors[3] > 200)
    found_straight = 1;

// Check for goal (all middle sensors dark)
if (sensors[1] > 600 && sensors[2] > 600 && sensors[3] > 600)
    break;

// Select and execute turn
unsigned char dir = select_turn(found_left, found_straight, found_right);
turn(dir);

// Record turn
path[path_length] = dir;
path_length++;

// Prune dead ends
simplify_path();
display_path();
```

### Second Main Loop Body (Replay)

```c
follow_segment();

// Slow down at intersection
set_motors(50, 50);
delay_ms(50);
set_motors(40, 40);
delay_ms(200);

// Execute recorded turn
turn(path[i]);
```

---

## Section 8.e: Simplifying the Solution

Dead ends create `xBx` sequences that can be simplified.

### Examples

| Sequence | Angles          | Total          | Result |
| -------- | --------------- | -------------- | ------ |
| LBL      | 270 + 180 + 270 | 720 -> 0 deg   | S      |
| LBS      | 270 + 180 + 0   | 450 -> 90 deg  | R      |
| RBL      | 90 + 180 + 270  | 540 -> 180 deg | B      |
| SBL      | 0 + 180 + 270   | 450 -> 90 deg  | R      |
| SBS      | 0 + 180 + 0     | 180 deg        | B      |
| RBS      | 90 + 180 + 0    | 270 deg        | L      |

### simplify_path() Implementation

```c
void simplify_path()
{
    // Only simplify if second-to-last turn was 'B'
    if (path_length < 3 || path[path_length - 2] != 'B')
        return;

    int total_angle = 0;
    for (int i = 1; i <= 3; i++) {
        switch (path[path_length - i]) {
        case 'R': total_angle += 90;  break;
        case 'L': total_angle += 270; break;
        case 'B': total_angle += 180; break;
        }
    }

    total_angle = total_angle % 360;

    switch (total_angle) {
    case 0:   path[path_length - 3] = 'S'; break;
    case 90:  path[path_length - 3] = 'R'; break;
    case 180: path[path_length - 3] = 'B'; break;
    case 270: path[path_length - 3] = 'L'; break;
    }

    path_length -= 2;
}
```

### Path Simplification Example

As the robot explores:

```
L
LS
LSB
LSBL => LR     (SBL -> R)
LRB
LRBL => LB    (RBL -> B)
LBL => S      (LBL -> S)
SB
SBL => R      (SBL -> R)
```

Final optimized path: `R`

---

## Section 8.f: Improving the Maze-Solving Code

### Optimization Strategies

1. **Increase line-following speed**
2. **Tune PID constants**
3. **Faster turns**
4. **Detect when robot is lost**
5. **Adjust speed based on upcoming turns** (full speed through 'S')

### Segment Timing Optimization

Record time for each segment during learning:

```
{ L, S, S, R, L, ... }  // Actions
{ 3, 3, 6, 5, 8, ... }  // Segment times
```

**Replay algorithm**:

- If next intersection is straight: drive at high speed
- Otherwise: drive at high speed until time T, then slow down

**Time T** is computed from segment length:

- Short segments: T negative -> drive at normal speed
- Long segments: T positive -> high speed most of segment

### Memory Considerations

- ATmega168: 1024 bytes RAM
- ATmega328: 2048 bytes RAM
- Leave 300-400 bytes for stack and library
- Use `unsigned char` arrays (1 byte each, max 255)

### Timing vs Encoders

The 3pi's regulated motor voltage provides repeatable results, allowing timing-based segment measurement. Traditional unregulated systems would need encoders since motor speed varies with battery discharge.

### Tire Maintenance

High-speed performance depends on tire traction. Clean tires with rubbing alcohol on a paper towel every few runs to prevent fishtailing.

---

## Key Functions Reference

### Initialization

| Function                                 | Description                        |
| ---------------------------------------- | ---------------------------------- |
| `pololu_3pi_init(timeout)`               | Initialize 3pi with sensor timeout |
| `calibrate_line_sensors(IR_EMITTERS_ON)` | Record min/max for calibration     |
| `read_battery_millivolts()`              | Get battery voltage                |

### Sensing

| Function                                | Description                          |
| --------------------------------------- | ------------------------------------ |
| `read_line(sensors, IR_EMITTERS_ON)`    | Returns 0-4000 position estimate     |
| `read_line_sensors_calibrated(sensors)` | Returns calibrated 0-1000 per sensor |

### Motors

| Function                  | Description                    |
| ------------------------- | ------------------------------ |
| `set_motors(left, right)` | Set motor speeds (-255 to 255) |

### Display

| Function                               | Description            |
| -------------------------------------- | ---------------------- |
| `clear()`                              | Clear LCD              |
| `print(str)`                           | Print string           |
| `print_long(val)`                      | Print integer          |
| `print_character(c)`                   | Print single character |
| `lcd_goto_xy(x, y)`                    | Move cursor            |
| `lcd_load_custom_character(data, num)` | Load custom character  |
| `print_from_program_space(str)`        | Print PROGMEM string   |

### Input

| Function                            | Description          |
| ----------------------------------- | -------------------- |
| `button_is_pressed(BUTTON_B)`       | Check button state   |
| `wait_for_button_release(BUTTON_B)` | Block until released |

### Timing

| Function       | Description    |
| -------------- | -------------- |
| `delay_ms(ms)` | Blocking delay |

### Audio

| Function                          | Description              |
| --------------------------------- | ------------------------ |
| `play_from_program_space(melody)` | Play melody from PROGMEM |
| `is_playing()`                    | Check if buzzer active   |

### LEDs

| Function        | Description       |
| --------------- | ----------------- |
| `left_led(on)`  | Control left LED  |
| `right_led(on)` | Control right LED |

---

## License

The original Pololu AVR Library code is provided under its respective license.
See: https://github.com/pololu/libpololu-avr
