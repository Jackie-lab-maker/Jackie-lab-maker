# VEX Robotics Fluid Transportation Robot — C++ Coding Guide

## Project Overview

This guide covers how to code a VEX V5 robot that:
- Moves on XYZ axes (like a 3D printer gantry)
- Uses a color sensor to identify fluid color
- Picks up the correct fluid and transports it to a target container

---

## 1. Environment Setup

### Required Software
| Tool | Purpose | Download |
|------|---------|---------|
| **VEXcode Pro V5** | Official VEX C++ IDE (recommended for beginners) | vex.com/vexcode |
| **PROS** + VS Code | Community SDK with more control | pros.cs.purdue.edu |

> **Recommended for VS Code:** Use the **PROS** extension. It gives you a full C++ toolchain inside VS Code with IntelliSense, debugging, and direct robot uploads.

### Setting Up PROS in VS Code
1. Install VS Code
2. Open Extensions (`Ctrl+Shift+X`) → search **PROS** → install
3. Open the PROS sidebar → click **Create New Project**
4. Select **V5** as the target, choose a template, and name your project
5. Upload to the robot via **PROS: Upload** in the command palette

---

## 2. Hardware You Will Need

| Component | VEX Part | Role |
|-----------|---------|------|
| V5 Brain | 276-4810 | Main controller |
| V5 Smart Motors (×3–4) | 276-4840 | X, Y, Z axis movement |
| V5 Color Sensor | 276-7000 | Detect fluid color |
| Optical Distance Sensor (optional) | 276-7009 | Detect container position |
| Pneumatics / Micro Pump | External | Fluid pickup and drop |
| Linear Slides / Rails | VEX structural | XYZ gantry frame |
| Limit Switches (×3) | 276-2174 | Homing each axis |

---

## 3. Project File Structure (PROS)

```
my_robot/
├── include/
│   ├── robot_config.h     ← motor ports, sensor ports, constants
│   ├── motion.h           ← axis movement declarations
│   └── fluid_system.h     ← color detection + pump declarations
├── src/
│   ├── main.cpp           ← initialize() and opcontrol() / autonomous()
│   ├── motion.cpp         ← XYZ movement implementation
│   └── fluid_system.cpp   ← color sensor + pump implementation
└── Makefile
```

---

## 4. Robot Configuration Header

```cpp
// include/robot_config.h
#pragma once
#include "pros/api.h"

// ── Motor Ports ──────────────────────────────────────────
constexpr int PORT_MOTOR_X  = 1;   // X-axis (horizontal left-right)
constexpr int PORT_MOTOR_Y  = 2;   // Y-axis (horizontal front-back)
constexpr int PORT_MOTOR_Z  = 3;   // Z-axis (vertical up-down)
constexpr int PORT_PUMP     = 4;   // Pump motor (spin to pick up fluid)

// ── Sensor Ports ─────────────────────────────────────────
constexpr int PORT_COLOR    = 5;   // V5 Color Sensor (Smart Port)
constexpr int PORT_LIMIT_X  = 'A'; // Limit switch for X home (3-wire port)
constexpr int PORT_LIMIT_Y  = 'B';
constexpr int PORT_LIMIT_Z  = 'C';

// ── Movement Constants ────────────────────────────────────
constexpr double STEPS_PER_MM = 10.0; // encoder counts per mm of travel
constexpr int    HOME_SPEED   = 30;   // % speed during homing (slow & safe)
constexpr int    MOVE_SPEED   = 80;   // % speed during normal movement
constexpr int    PUMP_SPEED   = 100;  // % speed for pump intake
constexpr int    PUMP_TIME_MS = 2000; // ms to run pump per pickup

// ── Container Positions (in mm from home) ─────────────────
// Adjust these after physically measuring your setup
struct Position { double x, y, z; };

constexpr Position SOURCE_RED    = { 50.0,  0.0, 30.0 };
constexpr Position SOURCE_BLUE   = { 100.0, 0.0, 30.0 };
constexpr Position SOURCE_GREEN  = { 150.0, 0.0, 30.0 };
constexpr Position DROP_CONTAINER= { 200.0, 0.0, 30.0 };
constexpr Position SAFE_Z        = {   0.0, 0.0,  0.0 }; // raised Z before moving
```

---

## 5. Motion System

```cpp
// include/motion.h
#pragma once
#include "robot_config.h"

void home_all_axes();
void move_x(double mm);
void move_y(double mm);
void move_z(double mm);
void move_to(Position pos);
```

```cpp
// src/motion.cpp
#include "motion.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"

pros::Motor motor_x(PORT_MOTOR_X);
pros::Motor motor_y(PORT_MOTOR_Y);
pros::Motor motor_z(PORT_MOTOR_Z);

pros::ADIDigitalIn limit_x(PORT_LIMIT_X);
pros::ADIDigitalIn limit_y(PORT_LIMIT_Y);
pros::ADIDigitalIn limit_z(PORT_LIMIT_Z);

// ── Home a single axis by driving toward the limit switch ──
static void home_axis(pros::Motor& motor, pros::ADIDigitalIn& limit) {
    motor.move(-HOME_SPEED); // drive toward home
    while (!limit.get_value()) {
        pros::delay(10);
    }
    motor.move(0);
    motor.tare_position(); // zero the encoder here
}

void home_all_axes() {
    home_axis(motor_z, limit_z); // always home Z first (lift up)
    pros::delay(200);
    home_axis(motor_x, limit_x);
    pros::delay(200);
    home_axis(motor_y, limit_y);
}

// ── Move an axis by a given distance in mm ─────────────────
static void move_axis(pros::Motor& motor, double mm) {
    double target_counts = mm * STEPS_PER_MM;
    motor.move_absolute(target_counts, MOVE_SPEED);

    // Wait until motor reaches target (within 5 counts)
    while (std::abs(motor.get_position() - target_counts) > 5.0) {
        pros::delay(20);
    }
}

void move_x(double mm) { move_axis(motor_x, mm); }
void move_y(double mm) { move_axis(motor_y, mm); }
void move_z(double mm) { move_axis(motor_z, mm); }

// ── Move to an absolute XYZ position safely ────────────────
void move_to(Position pos) {
    // Raise Z first to avoid collisions, then move X/Y, then lower Z
    move_z(SAFE_Z.z);
    move_x(pos.x);
    move_y(pos.y);
    move_z(pos.z);
}
```

---

## 6. Color Detection & Fluid Pickup

```cpp
// include/fluid_system.h
#pragma once
#include <string>

enum class FluidColor { RED, BLUE, GREEN, UNKNOWN };

FluidColor detect_color();
std::string color_name(FluidColor c);
void pick_up_fluid();
void drop_fluid();
```

```cpp
// src/fluid_system.cpp
#include "fluid_system.h"
#include "robot_config.h"
#include "pros/optical.hpp"
#include "pros/motors.hpp"

pros::Optical color_sensor(PORT_COLOR);
pros::Motor   pump_motor(PORT_PUMP);

// ── Detect color using hue from the optical sensor ─────────
// Hue is 0–360°. Adjust these ranges after testing with your containers.
FluidColor detect_color() {
    color_sensor.set_led_pwm(100); // turn on LED for accurate reading
    pros::delay(100);              // let sensor stabilize

    double hue = color_sensor.get_hue();

    if (hue < 20.0 || hue > 340.0)  return FluidColor::RED;
    if (hue >= 200.0 && hue < 260.0) return FluidColor::BLUE;
    if (hue >= 80.0  && hue < 160.0) return FluidColor::GREEN;

    return FluidColor::UNKNOWN;
}

std::string color_name(FluidColor c) {
    switch (c) {
        case FluidColor::RED:   return "RED";
        case FluidColor::BLUE:  return "BLUE";
        case FluidColor::GREEN: return "GREEN";
        default:                return "UNKNOWN";
    }
}

// ── Run pump to pick up fluid ──────────────────────────────
void pick_up_fluid() {
    pump_motor.move(PUMP_SPEED);
    pros::delay(PUMP_TIME_MS);
    pump_motor.move(0);
}

// ── Run pump in reverse to drop fluid ─────────────────────
void drop_fluid() {
    pump_motor.move(-PUMP_SPEED);
    pros::delay(PUMP_TIME_MS);
    pump_motor.move(0);
}
```

---

## 7. Main Program Logic

```cpp
// src/main.cpp
#include "pros/api.h"
#include "robot_config.h"
#include "motion.h"
#include "fluid_system.h"
#include <string>

// Called once when the program starts
void initialize() {
    pros::lcd::initialize(); // turn on V5 brain screen
    pros::lcd::set_text(0, "Homing axes...");

    home_all_axes();

    pros::lcd::set_text(0, "Ready.");
}

// ── Returns the source Position for a detected color ───────
Position get_source_position(FluidColor color) {
    switch (color) {
        case FluidColor::RED:   return SOURCE_RED;
        case FluidColor::BLUE:  return SOURCE_BLUE;
        case FluidColor::GREEN: return SOURCE_GREEN;
        default:                return SOURCE_RED; // fallback
    }
}

// ── Main autonomous transport routine ──────────────────────
void autonomous() {
    // Step 1: Go to the color sensor scanning position
    Position scan_pos = { 0.0, 50.0, 10.0 }; // above your color sample area
    move_to(scan_pos);

    // Step 2: Detect the fluid color
    FluidColor color = detect_color();
    pros::lcd::set_text(1, "Color: " + color_name(color));

    if (color == FluidColor::UNKNOWN) {
        pros::lcd::set_text(2, "No color detected!");
        return;
    }

    // Step 3: Navigate to the correct source container
    Position source = get_source_position(color);
    pros::lcd::set_text(2, "Moving to source...");
    move_to(source);

    // Step 4: Pick up the fluid
    pros::lcd::set_text(2, "Picking up fluid...");
    pick_up_fluid();

    // Step 5: Transport to the drop container
    pros::lcd::set_text(2, "Transporting...");
    move_to(DROP_CONTAINER);

    // Step 6: Drop the fluid
    pros::lcd::set_text(2, "Dropping fluid...");
    drop_fluid();

    // Step 7: Return home safely
    pros::lcd::set_text(2, "Returning home...");
    home_all_axes();

    pros::lcd::set_text(2, "Cycle complete!");
}

// opcontrol() is for driver-controlled mode — leave empty for autonomous-only
void opcontrol() {}
```

---

## 8. Calibration Steps

After assembling the robot, do these before running the full program:

### 8.1 Calibrate `STEPS_PER_MM`
```cpp
// Temporary test: command 100mm of movement
move_x(100.0);
// Measure actual distance moved with a ruler.
// If actual = 87mm:  STEPS_PER_MM = current_value * (100 / 87)
```

### 8.2 Calibrate Color Hue Ranges
```cpp
// Temporary test: print hue readings to the brain screen
pros::lcd::print(0, "Hue: %.1f", color_sensor.get_hue());
pros::delay(100);
// Hold each colored fluid under the sensor and note the hue.
// Update the if-ranges in detect_color() to match.
```

### 8.3 Calibrate Container Positions
```cpp
// Manually drive the robot to each container using opcontrol(),
// then read motor.get_position() and convert to mm.
// Update SOURCE_RED, SOURCE_BLUE, etc. in robot_config.h.
```

---

## 9. Useful PROS API Reference

| Task | API Call |
|------|---------|
| Move motor at speed | `motor.move(speed)` — speed: -127 to 127 |
| Move motor to position | `motor.move_absolute(position, speed)` |
| Read motor encoder | `motor.get_position()` |
| Zero encoder | `motor.tare_position()` |
| Read limit switch | `limit.get_value()` — returns 1 when pressed |
| Read color hue | `optical.get_hue()` — 0.0 to 359.9 |
| Read color brightness | `optical.get_brightness()` — 0.0 to 1.0 |
| Print to brain screen | `pros::lcd::print(row, "text %d", val)` |
| Delay | `pros::delay(milliseconds)` |

---

## 10. Debugging Tips

- **Motor doesn't stop at limit switch:** Check that the limit switch wiring is correct and the 3-wire port letter matches `PORT_LIMIT_X/Y/Z`.
- **Wrong color detected:** Run the hue calibration step (8.2). Ambient lighting strongly affects readings — run the LED at full power with `set_led_pwm(100)`.
- **Robot overshoots positions:** Lower `MOVE_SPEED` or increase the tolerance in the `while` loop inside `move_axis()`.
- **Pump picks up no fluid:** Increase `PUMP_TIME_MS` or verify the pump wiring polarity (swap wires if it runs backward).
- **Build errors in VS Code:** Run `pros make clean` then `pros make` in the terminal to do a fresh compile.

---

## 11. Recommended Development Order

1. [ ] Wire and test each motor individually (confirm port numbers)
2. [ ] Test homing — each axis should stop exactly at the limit switch
3. [ ] Test `move_to()` with known coordinates and measure accuracy
4. [ ] Test color sensor — log hue values for each fluid
5. [ ] Test pump pickup and drop in isolation
6. [ ] Run full `autonomous()` cycle with one color
7. [ ] Test all three colors and the UNKNOWN fallback case

---

*VEX V5 API docs: https://pros.cs.purdue.edu/v5/api/cpp/index.html*
