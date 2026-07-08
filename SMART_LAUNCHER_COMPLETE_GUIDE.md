# Smart Launcher — Complete Implementation & Wiring Guide

**Project:** Autonomous drone seed release system for mangrove reforestation  
**Hardware:** Freenove ESP32-S3 WROOM + Cube Orange (ArduPilot)  
**Version:** Phase 2 — Firmware Skeleton  
**Last updated:** 2026

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Hardware Bill of Materials](#3-hardware-bill-of-materials)
4. [Wiring Guide](#4-wiring-guide)
5. [Power System](#5-power-system)
6. [Mechanical Integration](#6-mechanical-integration)
7. [Software Environment Setup](#7-software-environment-setup)
8. [Firmware — Phase 2 Skeleton](#8-firmware--phase-2-skeleton)
9. [Servo Tester Tool](#9-servo-tester-tool)
10. [Mission Configuration](#10-mission-configuration)
11. [Cube Orange Setup](#11-cube-orange-setup)
12. [Bench Testing Protocol](#12-bench-testing-protocol)
13. [Calibration Guide](#13-calibration-guide)
14. [State Machine & LED Reference](#14-state-machine--led-reference)
15. [Serial Command Reference](#15-serial-command-reference)
16. [Flight Checklist](#16-flight-checklist)
17. [Troubleshooting](#17-troubleshooting)
18. [Development Roadmap](#18-development-roadmap)

---

## 1. Project Overview

The Smart Launcher is a **self-contained, platform-agnostic payload module** that autonomously releases mangrove propagules (seeds) from a drone at predefined GPS coordinates.

### Key design principles

- **Zero coupling to the flight controller.** The launcher uses its own GNSS or reads MAVLink from the Cube Orange. No DJI OSDK, PSDK, or Channel Expansion Kit required.
- **Power only from the drone.** The drone provides 2S–6S power. All drop logic runs on the ESP32-S3 onboard.
- **Two independent modules, synchronized.** Each module holds 5 seeds. Module A fires first; Module B fires immediately after A's limit switch confirms completion. This preserves balance.
- **Configurable without reflashing.** All mission parameters (waypoints, timing, PWM values) live in `mission.json` on the ESP32-S3 filesystem. Change the file, re-upload the filesystem, fly.
- **Safe.** A hardware fault in the launcher cannot affect the flight controller. Power is the only shared resource.

### Physical mechanism

Each launcher module contains a **sliding cover** over 5 seed compartments (35 mm wide each). A continuous rotation servo drives a rack-and-pinion that retracts the cover progressively:

```
Start: [COVER████████████████████] all 5 closed
WP 1:  [        ████████████████] door 1 open → seed drops
WP 2:  [                ████████] door 2 open → seed drops
WP 3:  [                    ████] door 3 open → seed drops
...
WP 5:  [                        ] all open → limit switch hits → mission done
```

The cover only moves in **one direction** (retract). Seeds fall by gravity. The limit switch at the fully-retracted end signals that all seeds have been dropped from that module.

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         DRONE (host)                            │
│  ┌──────────────────┐          ┌──────────────────────────────┐ │
│  │  Cube Orange     │          │   Smart Launcher             │ │
│  │  ArduCopter 4.6  │ TELEM2   │   Freenove ESP32-S3 WROOM   │ │
│  │  (flight ctrl)   │──UART───▶│                              │ │
│  │                  │          │  ┌─────────────────────────┐ │ │
│  │  GPS/IMU/EKF     │          │  │ Drop Logic Engine        │ │ │
│  │  Flies waypoints │          │  │ Config Manager           │ │ │
│  │  via QGC/UgCS    │          │  │ State Machine            │ │ │
│  └──────────────────┘          │  │ Logger                   │ │ │
│                                │  └──────────┬──────────────┘ │ │
│  ┌──────────────────┐          │             │                 │ │
│  │  UBEC 5V/5A      │          │  ┌──────────▼──────────────┐ │ │
│  │  (power only)    │──5V─────▶│  │ Actuator Module A & B   │ │ │
│  └──────────────────┘          │  │ Servo PWM (LEDC)        │ │ │
│                                │  │ Limit switch read        │ │ │
│  ┌──────────────────┐          │  └──────────┬──────────────┘ │ │
│  │  u-blox M8N GNSS │──UART───▶│             │                 │ │
│  │  (onboard)       │          └─────────────│─────────────────┘ │
│  └──────────────────┘                        │                   │
└─────────────────────────────────────────────│───────────────────┘
                               ┌──────────────▼──────────────────┐
                               │  Module A          Module B      │
                               │  Servo + rack      Servo + rack  │
                               │  5 doors           5 doors       │
                               │  Limit SW A        Limit SW B    │
                               └─────────────────────────────────┘
```

### Position source selection

| Mode | Source | When to use |
|---|---|---|
| `GNSS` | Onboard u-blox M8N on UART1 | Any drone — fully independent |
| `MAVLINK` | Cube Orange TELEM2 on UART2 | DIY ArduPilot drones — higher accuracy |

Set `position_source` in `mission.json`. No reflash needed to switch.

---

## 3. Hardware Bill of Materials

### Core electronics

| Component | Part | Qty | Notes |
|---|---|---|---|
| MCU | Freenove ESP32-S3 WROOM | 1 | 8MB Flash, 8MB PSRAM, built-in Wi-Fi |
| GNSS | u-blox M8N (BN-880 or similar) | 1 | UART, 3.3V, patch antenna included |
| Servo | FS5106R continuous rotation | 2 | One per module. 4.8–6V |
| Limit switch | Micro snap-action switch (SPDT) | 2 | One per module, at fully-retracted end |
| UBEC | 5V / 5A step-down BEC | 1 | Powers servos + ESP32 |
| Capacitor | 100µF 10V electrolytic | 2 | One across each servo power rail |
| Connector | XT30 or JST-PH | 1 set | Drone power input |
| Wire | 22 AWG silicone | — | Signal lines |
| Wire | 18 AWG silicone | — | Power lines (servo) |

### Optional / Phase 4

| Component | Part | Notes |
|---|---|---|
| JST-GH 6-pin cable | Cube Orange TELEM2 cable | For MAVLink position feed |
| USB-C cable | Data-capable (not charge-only) | Firmware upload |

### For bench testing only

| Item | Purpose |
|---|---|
| Desktop 5V / 3A power supply | Power servos without drone battery |
| Bench alligator clips | Temporary connections |
| Multimeter | Verify voltages before first power-on |

---

## 4. Wiring Guide

### 4.1 Complete pin mapping — ESP32-S3 WROOM

| Signal | ESP32-S3 GPIO | Direction | Voltage | Notes |
|---|---|---|---|---|
| GNSS UART RX | GPIO 4 | IN | 3.3V | ← GNSS module TX |
| GNSS UART TX | GPIO 5 | OUT | 3.3V | → GNSS module RX |
| MAVLink RX | GPIO 16 | IN | 3.3V | ← Cube Orange TELEM2 TX (Phase 4) |
| MAVLink TX | GPIO 17 | OUT | 3.3V | → Cube Orange TELEM2 RX (Phase 4) |
| Servo A PWM | GPIO 18 | OUT | 3.3V | LEDC channel 0, 50 Hz |
| Servo B PWM | GPIO 8 | OUT | 3.3V | LEDC channel 1, 50 Hz |
| Limit Switch A | GPIO 9 | IN | 3.3V | INPUT_PULLUP, active LOW |
| Limit Switch B | GPIO 10 | IN | 3.3V | INPUT_PULLUP, active LOW |
| ARM button | GPIO 0 | IN | 3.3V | INPUT_PULLUP, hold 2s to ARM |
| Status LED | GPIO 2 | OUT | 3.3V | Built-in blue LED on Freenove board |
| GND | GND | — | 0V | Common ground — CRITICAL |
| 5V in | 5V pin | PWR IN | 5V | From UBEC output |

> **GPIO 3.3V signal is safe for servo signal wires.** Most servos accept 3.3V logic on the signal pin even when powered at 5V. Tested with FS5106R.

---

### 4.2 Servo wiring

Each servo has 3 wires. Standard colour coding:

```
Servo wire colour → Connect to
─────────────────────────────────────────────────────
Brown / Black     → UBEC GND  AND  ESP32 GND (common)
Red               → UBEC 5V output
Orange / Yellow   → ESP32 GPIO (18 for A, 8 for B)
```

> **Never power servos from the ESP32 5V or 3.3V pins.** Always use the UBEC. Servo stall current (up to 1.5A each) will reset or damage the ESP32 regulator.

**Add a 100µF electrolytic capacitor** across the servo power rail (+ to 5V, – to GND) near each servo connector. This absorbs voltage spikes when the servo starts or stalls.

---

### 4.3 Limit switch wiring

Each micro switch has 3 terminals: COM, NO (Normally Open), NC (Normally Closed).  
**Use COM and NO.**

```
Limit switch terminal → Connect to
──────────────────────────────────
COM                   → GND
NO                    → GPIO 9 (Module A) or GPIO 10 (Module B)
NC                    → not connected
```

The firmware configures these pins as `INPUT_PULLUP`. When the cover reaches the end of the rail and presses the switch, the GPIO reads LOW → servo auto-stops.

---

### 4.4 GNSS module wiring (u-blox M8N / BN-880)

```
GNSS module pin → ESP32-S3 GPIO
────────────────────────────────
VCC (3.3V)      → 3V3 pin on ESP32 board
GND             → GND
TX              → GPIO 4 (ESP32 UART1 RX)
RX              → GPIO 5 (ESP32 UART1 TX)
```

> The BN-880 runs at 3.3V. Do **not** connect VCC to the 5V rail.

---

### 4.5 Cube Orange TELEM2 wiring (Phase 4 — MAVLink)

The TELEM2 port on the Cube Orange carrier board uses a **JST-GH 6-pin connector**.

```
TELEM2 pin (JST-GH) → Signal    → ESP32-S3 GPIO
─────────────────────────────────────────────────
Pin 1               → +5V       → NOT connected (ESP32 already powered)
Pin 2               → TX (out)  → GPIO 16 (ESP32 UART2 RX)
Pin 3               → RX (in)   → GPIO 17 (ESP32 UART2 TX)
Pin 4               → CTS       → NOT connected
Pin 5               → RTS       → NOT connected
Pin 6               → GND       → GND (common ground)
```

> Both TX/RX are 3.3V TTL — directly compatible with ESP32. No level shifter needed.

---

### 4.6 Complete wiring diagram (text schematic)

```
                    ┌─────────────────────────────┐
  DRONE BATTERY     │    UBEC  5V / 5A            │
  2S–6S ──────────▶ │  IN: 7.4–25.2V              │──── 5V out ──┬── Servo A red
                    │  OUT: 5V regulated           │              ├── Servo B red
                    └─────────────────────────────┘              └── ESP32 5V pin

  GND bus ──────────────────────────────────────────────────────────────────────
           │         │          │          │          │          │
         UBEC GND  Servo A    Servo B   Limit SW A  Limit SW B  GNSS GND
                   brown/blk  brown/blk  COM         COM

  ┌──────────────────────────────────────────────────────────────────────┐
  │                    Freenove ESP32-S3 WROOM                           │
  │                                                                      │
  │  GPIO 4  ◀──────────────────────────────────── GNSS TX              │
  │  GPIO 5  ──────────────────────────────────▶   GNSS RX              │
  │  GPIO 16 ◀──────────────────────────────────── Cube TELEM2 TX (Ph4) │
  │  GPIO 17 ──────────────────────────────────▶   Cube TELEM2 RX (Ph4) │
  │  GPIO 18 ──────────────────────────────────▶   Servo A signal (org) │
  │  GPIO 8  ──────────────────────────────────▶   Servo B signal (org) │
  │  GPIO 9  ◀──── 10kΩ pullup ──── Limit SW A NO ──── GND             │
  │  GPIO 10 ◀──── 10kΩ pullup ──── Limit SW B NO ──── GND             │
  │  GPIO 0  ◀──── 10kΩ pullup ──── ARM button ──── GND                │
  │  GPIO 2  ──── [built-in LED]                                        │
  │  GND     ──────────────────────────────── Common GND bus            │
  │  5V      ◀──────────────────────────────── UBEC 5V out              │
  └──────────────────────────────────────────────────────────────────────┘

  Note: 10kΩ pullup resistors are internal (INPUT_PULLUP). 
  External resistors not required.
```

---

## 5. Power System

### 5.1 Power budget

| Component | Voltage | Typical current | Peak (stall) |
|---|---|---|---|
| ESP32-S3 WROOM | 5V → 3.3V internal | 80 mA | 250 mA (Wi-Fi TX) |
| GNSS M8N | 3.3V | 25 mA | 35 mA |
| Servo A (FS5106R) | 5V | 150 mA running | 1500 mA stall |
| Servo B (FS5106R) | 5V | 150 mA running | 1500 mA stall |
| **Total (running)** | **5V** | **~400 mA** | **~3300 mA** |

**Required UBEC: 5V / 5A minimum.** Recommended: 5V / 5A with short-circuit protection.

### 5.2 Recommended UBEC options

| Product | Rating | Notes |
|---|---|---|
| Matek FCHUB-6S | 5V / 3A + 9V / 3A | Good for light servos |
| HobbyWing 5A BEC | 5V / 5A | Best option for FS5106R |
| Pololu D36V28F5 | 5V / 3.8A | Compact, high efficiency |

### 5.3 Power sequencing

Power-on order does not matter — the ESP32 boots from 3.3V (internal LDO from 5V input). Servos are always held at 1500 µs (stop) until the ARM command is received.

---

## 6. Mechanical Integration

### 6.1 Rack and pinion specs

| Parameter | Value | Derivation |
|---|---|---|
| Gear diameter | 38 mm | Measured |
| Teeth count | 24 | Measured |
| Circumference | π × 38 = 119.4 mm | — |
| Travel per tooth | 119.4 ÷ 24 = **4.97 mm** | — |
| Teeth for 35 mm | 35 ÷ 4.97 = **7.04 teeth** | — |
| Revolutions for 35 mm | 7.04 ÷ 24 = **0.293 rev** | — |
| FS5106R speed at 5V | ~60 RPM = 1 rev/sec | Datasheet |
| **Theoretical travel time** | 0.293 sec = **~300 ms** | — |
| **Starting firmware value** | **350 ms** | +20% margin |

> **Calibrate this value on the bench before flying.** See Section 13.

### 6.2 Limit switch placement

The limit switch must be positioned so the switch actuator is **fully depressed** when the sliding cover reaches the end of its retraction travel (all 5 doors open). Mount it at the retracted end of the rail with the switch body fixed to the frame and the actuator contacting a tab on the sliding cover.

### 6.3 Module mounting on drone

- Mount both modules **symmetrically** on the drone frame, equidistant from the centre of gravity.
- Use vibration-dampening foam or rubber grommets between the launcher frame and drone frame.
- Run all wiring through cable sleeves and secure with zip ties — no loose wires near propellers.
- The entire launcher assembly should be detachable via a single multi-pin connector for easy module swap between missions.

---

## 7. Software Environment Setup

### 7.1 Arduino IDE (recommended for servo tester)

1. Download Arduino IDE 2.x from [arduino.cc](https://www.arduino.cc/en/software)
2. Open `File → Preferences`
3. Paste into "Additional boards manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. `Tools → Board → Boards Manager` → search "esp32" → install **esp32 by Espressif Systems** (version 3.x)
5. Select board: `Tools → Board → ESP32 Arduino → ESP32S3 Dev Module`
6. Set these options:
   ```
   USB CDC On Boot  : Enabled      ← CRITICAL for Freenove WROOM
   Upload Speed     : 921600
   Flash Size       : 8MB
   Partition Scheme : 8M with spiffs (3MB APP/1.5MB SPIFFS)
   ```

### 7.2 PlatformIO (for main firmware)

1. Install [VS Code](https://code.visualstudio.com/)
2. Install the PlatformIO extension from the VS Code marketplace
3. Open the `smart_launcher/` project folder
4. PlatformIO auto-detects `platformio.ini` and installs all dependencies

**Build commands:**
```bash
# Build firmware only
pio run

# Upload firmware to ESP32
pio run --target upload

# Upload filesystem (mission.json to LittleFS)
pio run --target uploadfs

# Open serial monitor at 115200
pio device monitor

# Build + upload + monitor in one step
pio run --target upload && pio device monitor
```

### 7.3 Required libraries

| Library | Version | Purpose |
|---|---|---|
| TinyGPSPlus | ^1.0.3 | NMEA sentence parsing |
| ArduinoJson | ^7.0.0 | JSON config parsing |
| FastLED | ^3.6.0 | WS2812 RGB LED |
| Arduino core for ESP32 | ^3.x | Board support |

All installed automatically by PlatformIO from `platformio.ini`.

---

## 8. Firmware — Phase 2 Skeleton

### 8.1 Project structure

```
smart_launcher/
├── platformio.ini              ← Board config, pins, libraries
├── partitions_custom.csv       ← 3MB firmware + 960KB LittleFS
├── data/
│   └── mission.json            ← Mission config (upload to LittleFS)
└── src/
    ├── main.cpp                ← Setup, loop, serial command handler
    ├── config/
    │   ├── ConfigManager.h
    │   └── ConfigManager.cpp   ← Load/parse mission.json from LittleFS
    ├── position/
    │   ├── IPositionSource.h   ← Abstract interface + Haversine()
    │   ├── GNSSPositionSource.h
    │   └── GNSSPositionSource.cpp  ← NMEA parser via TinyGPS++
    ├── actuator/
    │   ├── ActuatorModule.h
    │   └── ActuatorModule.cpp  ← Servo LEDC PWM + limit switch logic
    ├── safety/
    │   ├── StateMachine.h
    │   └── StateMachine.cpp    ← State machine + ARM button + LED
    ├── logging/
    │   ├── Logger.h
    │   └── Logger.cpp          ← Serial + LittleFS flash log
    └── logic/
        ├── DropLogicEngine.h
        └── DropLogicEngine.cpp ← Haversine check, sequential A→B firing
```

### 8.2 Module responsibilities

#### ConfigManager
- Mounts LittleFS on boot
- Parses `mission.json` into typed structs (`MissionConfig`, `DropPoint`, `ModuleConfig`)
- Provides `reload()` for hot-reload after Wi-Fi upload (Phase 3)
- Prints config summary to serial for verification

#### IPositionSource
- Pure abstract interface: `begin()`, `update()`, `read()`, `hasFix()`
- `haversineDistance()` inline function — returns distance in metres between two GPS coordinates
- Swap GNSS for MAVLink by changing one pointer in `main.cpp` — no other code changes

#### GNSSPositionSource
- Opens UART1 on GPIO 4/5 at 9600 baud
- Feeds bytes to TinyGPS++ decoder
- Updates internal `Position` struct on each valid NMEA sentence
- `hasFix()` returns true only when HDOP < 5.0 and satellites ≥ 4

#### ActuatorModule
- Configures LEDC at 50 Hz, 16-bit resolution
- `dropNext()`:
  - Doors 1–4: sends open PWM, waits exactly `travel_ms`, sends stop PWM
  - Door 5 (last): sends open PWM, polls limit switch until triggered OR timeout
  - Returns `DropResult::OK` or `DropResult::TIMEOUT`
- `stop()`: immediately sends 1500 µs (safe in any state)
- `reset()`: clears door counter (for bench testing and module swap)

#### StateMachine
```
BOOT ──(self-check OK)──▶ IDLE
IDLE ──(hold ARM 2s)────▶ ARMED
ARMED ──(drop executing)─▶ DROPPING
DROPPING ──(drop done)───▶ ARMED
ANY ──(fault detected)───▶ FAULT
ARMED ──(hold ARM 2s)────▶ IDLE  (disarm)
```

#### DropLogicEngine
- Called every loop iteration (20 Hz)
- Reads current position and computes Haversine distance to next pending waypoint
- When distance < `radius_m` AND system is ARMED:
  1. Sets state → DROPPING
  2. Calls `modA.dropNext()` — blocks until complete
  3. Immediately calls `modB.dropNext()` — sequential confirm
  4. Sets state → ARMED
  5. Advances to next waypoint
- Logs every drop event to serial and LittleFS

#### Logger
- Writes human-readable CSV to `/logs/flight.log` on LittleFS
- Format: `timestamp_ms, drop_id, module, door_num, lat, lon, alt, result`
- `dumpLog()` prints entire log to serial (use after landing for review)

### 8.3 Drop sequence — detailed timing

```
Timeline (one waypoint, both modules):

T+0 ms    Enter radius → state = DROPPING
T+0 ms    Module A servo: 1700 µs (OPEN)
T+350 ms  Module A servo: 1500 µs (STOP) [timed, doors 1-4]
           OR limit switch triggers [door 5]
T+351 ms  Log Module A drop → Serial + LittleFS
T+351 ms  Module B servo: 1300 µs (OPEN)  ← fires immediately
T+701 ms  Module B servo: 1500 µs (STOP)
T+702 ms  Log Module B drop
T+702 ms  State = ARMED, advance to next waypoint
```

Total time per waypoint: **~700 ms** (at 350 ms travel_ms)

At typical drone mission speed of 3 m/s, the drone travels ~2.1 m during this window. Tune `radius_m` accordingly (3–5 m recommended).

---

## 9. Servo Tester Tool

A standalone Arduino sketch (`servo_tester.ino`) for bench testing servos before integrating with the full firmware.

### 9.1 Features
- Wi-Fi access point mode — no router needed
- Web interface at `http://192.168.4.1`
- Individual control of Module A and Module B
- OPEN / STOP / CLOSE buttons with correct preset PWM per module
- Fine-tuning slider (1000–2000 µs, 10 µs steps)
- Live limit switch indicators (auto-stops servo on trigger)
- Event log with timestamps
- Heartbeat LED blink on GPIO 2

### 9.2 How to use

1. Flash `servo_tester.ino` with Arduino IDE
2. Connect phone/laptop to Wi-Fi: **"SmartLauncher"** (no password)
3. Open browser → `http://192.168.4.1`
4. Blue LED blinks every 2 seconds = server healthy

### 9.3 PWM reference for FS5106R

| PWM (µs) | Module A direction | Module B direction |
|---|---|---|
| 1000 | Full speed reverse (CLOSE) | Full speed forward |
| 1300 | Slow reverse | Slow forward |
| 1500 | **STOP (neutral)** | **STOP (neutral)** |
| 1700 | Slow forward (OPEN) | Slow reverse |
| 2000 | Full speed forward | Full speed reverse |

> The two modules rotate in **opposite directions** because they retract toward each other for balance. This is reflected in the firmware (`module_a.servo_open_pwm_us = 1700`, `module_b.servo_open_pwm_us = 1300`).

> **Important:** The exact neutral point (true stop) may not be exactly 1500 µs. Use the slider to find the value where the servo is completely still. Record this as `servo_stop_pwm_us` in `mission.json`.

---

## 10. Mission Configuration

### 10.1 File location

The mission config lives at `/mission.json` on the ESP32-S3 LittleFS filesystem.  
Edit `smart_launcher/data/mission.json` on your PC, then upload:
```bash
pio run --target uploadfs
```

### 10.2 Full annotated config

```json
{
  "mission_name": "mangrove_plot_A",

  "min_gps_fix_duration_s": 10,
  // How many seconds of stable GPS fix required before ARM is allowed.
  // Prevents arming in poor reception. Range: 5–30s recommended.

  "position_source": "GNSS",
  // "GNSS" = use onboard u-blox M8N (default, works on any drone)
  // "MAVLINK" = use Cube Orange TELEM2 (Phase 4, higher accuracy)

  "modules": {
    "A": {
      "servo_open_pwm_us": 1700,
      // PWM to run servo in OPEN direction. 
      // Calibrate: should move rack ~35mm in travel_ms milliseconds.
      // Increase toward 2000 for faster travel, decrease toward 1500 for slower.

      "servo_stop_pwm_us": 1500,
      // PWM to hold servo stopped. Find true neutral with servo tester slider.
      // Common range: 1480–1520 µs depending on individual servo.

      "travel_ms": 350,
      // Time in milliseconds to travel exactly 35mm (one door width).
      // Theoretical: 293ms. Starting value: 350ms. Calibrate on bench.
      // If rack travels MORE than 35mm, reduce this value.
      // If rack travels LESS than 35mm, increase this value.

      "door_count": 5
      // Total seeds in this module. Do not change unless redesigning.
    },
    "B": {
      "servo_open_pwm_us": 1300,
      // Module B rotates opposite to A (1300 = open direction for B)
      "servo_stop_pwm_us": 1500,
      "travel_ms": 350,
      "door_count": 5
    }
  },

  "drops": [
    // One entry per seed drop. Waypoints 1-5 use Module A door 1-5.
    // Waypoints 6-10 use Module B door 1-5 (synchronized with A).
    // Actually: each drop entry fires BOTH modules simultaneously (one door each).
    // 10 entries = 5 drops × 2 modules = 10 seeds total.

    { "id": 1, "lat_deg": 9.345678, "lon_deg": -79.876543, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 2, "lat_deg": 9.345700, "lon_deg": -79.876520, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 3, "lat_deg": 9.345722, "lon_deg": -79.876497, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 4, "lat_deg": 9.345744, "lon_deg": -79.876474, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 5, "lat_deg": 9.345766, "lon_deg": -79.876451, "alt_m": 8.0, "radius_m": 3.0 }

    // radius_m: trigger distance in metres. 
    // Recommended 3.0–5.0m for 3 m/s drone speed.
    // Increase if drops are being missed; decrease for tighter placement.
  ]
}
```

### 10.3 Generating waypoints from QGroundControl

1. Plan mission in QGC with drop points as waypoints
2. Export as `.plan` file
3. Open `.plan` (it is JSON) and extract `lat`, `lon` from each waypoint item
4. Paste coordinates into `mission.json` drops array
5. Re-upload filesystem

Python helper script (coming in Phase 5):
```bash
python tools/convert_mission.py --input mission.plan --output data/mission.json
```

---

## 11. Cube Orange Setup

### 11.1 Firmware

- **Firmware:** ArduCopter V4.6.3 OFFICIAL (or latest stable)
- **Board:** CubeOrangePlus
- **Frame:** Hexa X (`FRAME_CLASS=2`, `FRAME_TYPE=1`)

### 11.2 Initial setup sequence (Mission Planner)

1. Connect via USB → COM port at 115200 baud
2. `Setup → Install Firmware` → select CubeOrangePlus → Copter V4.6.3
3. `Config → Mandatory Hardware → Frame Type` → Hexa X
4. `Config → Mandatory Hardware → Accel Calibration` → follow wizard (6 positions)
5. `Config → Mandatory Hardware → Compass` → calibrate (rotate drone on all axes)
6. `Config → Mandatory Hardware → Radio Calibration` → calibrate RC transmitter
7. `Config → Mandatory Hardware → ESC Calibration` → calibrate all 6 ESCs
8. Motor test (no props): `Setup → Optional Hardware → Motor Test` → verify correct order

### 11.3 TELEM2 parameters for Smart Launcher (Phase 4)

Set these in `Config → Full Parameter List`:

| Parameter | Value | Description |
|---|---|---|
| `SERIAL2_PROTOCOL` | `2` | MAVLink 2.0 on TELEM2 |
| `SERIAL2_BAUD` | `57` | 57600 baud |
| `SR2_POSITION` | `4` | GLOBAL_POSITION_INT at 4 Hz |
| `SR2_EXT_STAT` | `2` | Battery and GPS status |
| `SR2_EXTRA1` | `0` | Disable unnecessary messages |
| `SR2_EXTRA2` | `0` | Disable unnecessary messages |

Write parameters → reboot Cube Orange → TELEM2 will output MAVLink immediately on power-up.

### 11.4 TELEM2 wiring to ESP32 (Phase 4)

```
Cube Orange TELEM2       ESP32-S3 WROOM
(JST-GH 6-pin)
─────────────────────────────────────────
Pin 1 (+5V)    →    NOT CONNECTED
Pin 2 (TX)     →    GPIO 16 (UART2 RX)
Pin 3 (RX)     →    GPIO 17 (UART2 TX)
Pin 4 (CTS)    →    NOT CONNECTED
Pin 5 (RTS)    →    NOT CONNECTED
Pin 6 (GND)    →    GND (common ground)
```

---

## 12. Bench Testing Protocol

Perform all these tests **before mounting on the drone** and **before any flight**.

### Test 1 — Power-on and boot

```
PASS criteria:
□ Blue LED blinks 3 times fast → boot sequence started
□ Serial monitor shows:
    === Smart Launcher booting ===
    [CFG] LittleFS mounted OK
    [CFG] Loaded 'mangrove_plot_A' — 5 drop points
    [HW] Servos initialised — both at 1500µs (STOP)
    [SM] State machine started — BOOT
    [SM] Disarmed → IDLE
    [MAIN] Boot complete — IDLE
□ LED solid YELLOW = IDLE state
□ No "FAULT" messages in serial output
```

### Test 2 — Servo neutral verification

```
Procedure:
1. Type 'dropa' in serial monitor
2. Observe servo A — it should NOT move (neutral test)
   If it moves: adjust servo_stop_pwm_us in mission.json
3. Repeat with 'dropb' for servo B

PASS criteria:
□ Servo A stationary at 1500µs (or calibrated neutral)
□ Servo B stationary at 1500µs (or calibrated neutral)
```

### Test 3 — Servo tester web interface

```
Procedure:
1. Flash servo_tester.ino
2. Connect to "SmartLauncher" Wi-Fi
3. Open http://192.168.4.1
4. Click OPEN for Module A — verify servo runs
5. Click STOP — verify servo stops
6. Click CLOSE — verify servo reverses
7. Manually press limit switch — verify:
   - Dot turns purple in interface
   - Servo auto-stops
8. Repeat for Module B

PASS criteria:
□ Both servos respond to commands
□ Both limit switches trigger auto-stop
□ Servo directions match OPEN/CLOSE labels
```

### Test 4 — Single door drop timing

```
Procedure:
1. Flash main firmware
2. Mark start position on rack with marker pen
3. Type 'arm' in serial monitor → LED blinks GREEN
4. Type 'dropa' → servo runs for travel_ms, stops
5. Measure how far rack moved
6. Adjust travel_ms in mission.json if needed
7. Re-upload filesystem: pio run --target uploadfs
8. Repeat until rack moves exactly 35mm

PASS criteria:
□ Module A rack moves 35mm ± 2mm per 'dropa' command
□ Module B rack moves 35mm ± 2mm per 'dropb' command
□ 'reset' command resets door counter
□ 5 consecutive drops complete, limit switch triggers on 5th
```

### Test 5 — GPS fix and arming

```
Procedure:
1. Take ESP32 outside (or near window) with GNSS connected
2. Power on — wait for GPS fix
3. Serial output should show:
    [GPS] UART1 started on RX=4 TX=5 @9600 baud
   After satellite acquisition (~60s):
    [MAIN] GPS stable for 10 s — ready to ARM
4. Hold ARM button for 2+ seconds → LED blinks GREEN

PASS criteria:
□ GPS acquires fix within 90s (cold start)
□ HDOP reported < 5.0
□ Satellites ≥ 4
□ ARM button hold triggers ARMED state
□ Serial shows: [SM] >>> ARMED <<<
```

### Test 6 — Full mission simulation (NMEA replay)

```
Procedure:
1. Record real NMEA sentences from your field location
   (or use gpsfake / miniterm to replay a .nmea file)
2. Feed NMEA over UART to ESP32 GNSS port
3. Arm the system
4. Verify drops trigger at correct waypoints
5. Verify A fires before B at each waypoint
6. After 5th waypoint: verify limit switches hit, mission complete

PASS criteria:
□ Each waypoint triggers exactly one drop
□ Module A fires first, B follows
□ Mission complete logged after waypoint 5
□ System disarms automatically
□ Serial log matches expected drop sequence
```

---

## 13. Calibration Guide

### 13.1 Finding servo neutral (true stop)

The FS5106R continuous rotation servo may not stop exactly at 1500 µs.  
Individual units vary ±20 µs.

**Using servo tester:**
1. Open `http://192.168.4.1`
2. Click STOP → note if servo is still moving slightly
3. Move slider slowly toward 1490 or 1510 until servo is completely still
4. Record the value (e.g. `1508`)
5. Update `servo_stop_pwm_us` in `mission.json`
6. Re-upload filesystem

### 13.2 Calibrating travel_ms (35mm per door)

**Method:**
1. Mark a reference line on the rack with a marker
2. Type `dropa` in serial monitor
3. Measure displacement with ruler
4. Calculate correction:
   ```
   new_travel_ms = current_travel_ms × (35 / measured_mm)
   Example: measured 40mm with 350ms
   new_travel_ms = 350 × (35/40) = 306 ms
   ```
5. Update `travel_ms` in `mission.json`, re-upload filesystem
6. Repeat until displacement is 35 ± 1 mm

### 13.3 Verifying balance (A vs B timing)

After calibrating both modules individually:
1. Weigh both modules with seeds loaded — should be equal
2. Run `dropa` and `dropb` with stopwatch
3. Total time for 5 doors should be equal between modules
4. If one module is faster: reduce its `travel_ms` slightly

---

## 14. State Machine & LED Reference

```
                    ┌──────┐
                    │ BOOT │  Blue — startup, self-check
                    └──┬───┘
                       │ config OK
                    ┌──▼───┐
              ┌────▶│ IDLE │  Yellow — ready, drop logic off
              │     └──┬───┘
              │        │ ARM button held 2s
              │     ┌──▼────┐
              │     │ ARMED │  Green blink — drop logic active
              │     └──┬────┘
              │        │ in radius + armed
              │     ┌──▼──────────┐
              │     │  DROPPING   │  Purple — executing drop
              │     └──┬──────────┘
              │        │ drop complete
              └────────┘ back to ARMED

         Any state ──(error)──▶ FAULT  Red — servos stopped
```

| LED colour | State | Action required |
|---|---|---|
| Blue steady | BOOT | Wait for boot to complete |
| Yellow steady | IDLE | Normal — hold button 2s to arm |
| Green blink | ARMED | Normal — ready for flight |
| Purple steady | DROPPING | Normal — drop in progress |
| Red steady | FAULT | Check serial output for error |

---

## 15. Serial Command Reference

Connect at **115200 baud** (USB CDC, no FTDI adapter needed on Freenove WROOM).

| Command | Action |
|---|---|
| `help` | Print all available commands |
| `arm` | Force ARM state (bypasses GPS stability check — bench only) |
| `disarm` | Return to IDLE state |
| `status` | Print current state, GPS position, door counts |
| `dropa` | Manually trigger Module A — one door drop |
| `dropb` | Manually trigger Module B — one door drop |
| `reset` | Reset door counters on both modules to 0 |
| `log` | Dump entire flight log to serial |

**Example status output:**
```
[CMD] State=2 Armed=1 GPS valid=1 lat=9.345678 lon=-79.876543
```

States: 0=BOOT 1=IDLE 2=ARMED 3=DROPPING 4=FAULT

---

## 16. Flight Checklist

### Pre-flight (ground)

```
□ mission.json updated with correct waypoints for today's field site
□ Filesystem uploaded: pio run --target uploadfs
□ Firmware uploaded: pio run --target upload
□ Config summary verified in serial monitor (correct coordinates, radius)
□ Both modules loaded with seeds
□ Limit switches verified functional (test bench)
□ Travel_ms calibrated (35mm per door confirmed)
□ Servo neutral calibrated
□ All wiring secured, no loose connections
□ UBEC output verified: 5.0V ± 0.1V with multimeter
□ Common ground confirmed between UBEC, servos, ESP32
□ SD card inserted in Cube Orange
□ Cube Orange compass calibrated (if moved since last calibration)
□ Mission uploaded to Cube Orange via QGC
□ Geofence configured in Mission Planner (recommended)
```

### Arming sequence (field)

```
□ Power on launcher (UBEC) → LED = YELLOW (IDLE)
□ Wait for GPS fix → serial: "[MAIN] GPS stable for 10s"
□ LED still YELLOW — GPS acquired
□ Hold ARM button 2 seconds → LED blinks GREEN (ARMED)
□ Confirm: serial shows [SM] >>> ARMED <<<
□ Power on drone
□ Arm drone via RC transmitter (normal ArduPilot arm procedure)
□ Safety check: do NOT approach propellers after arming
□ Execute mission via QGC or RC
```

### Post-flight

```
□ Land drone
□ Disarm drone (RC transmitter)
□ Hold ARM button 2s → launcher LED = YELLOW (disarmed)
□ Power off drone + launcher
□ Connect ESP32 via USB
□ Open serial monitor → type 'log' → review drop events
□ Verify all 5 waypoints show OK drops for both modules
□ Note any TIMEOUT events for investigation
□ Remove empty modules, reload seeds for next mission
□ Type 'reset' before next mission if not reflashing
```

---

## 17. Troubleshooting

### Servo does not move

| Check | Action |
|---|---|
| Is system ARMED? | Type `arm` in serial, check LED is green |
| Is power supply on? | Verify UBEC 5V with multimeter |
| Is PWM wire connected? | Check GPIO 18 (A) or GPIO 8 (B) |
| Is common ground connected? | UBEC GND and ESP32 GND must share ground |
| Is servo signal 3.3V compatible? | FS5106R accepts 3.3V signal — should be fine |

### Servo does not stop

| Symptom | Cause | Fix |
|---|---|---|
| Servo creeps slowly | Neutral not calibrated | Use servo tester slider to find true stop |
| Servo runs continuously | Limit switch wired incorrectly | Verify COM→GND, NO→GPIO |
| Servo runs past door | travel_ms too high | Reduce by 20ms, re-test |

### GPS not acquiring fix

| Check | Action |
|---|---|
| Is antenna outdoors or near window? | GNSS needs sky view |
| Are UART wires swapped? | Verify GNSS TX → GPIO 4, GNSS RX → GPIO 5 |
| Is GNSS powered correctly? | 3.3V only — never connect to 5V |
| Cold start? | Wait 90s for first fix, 10s for warm start |
| Is baud rate correct? | Default M8N is 9600 — check platformio.ini |

### LittleFS mount fails

```
[CFG] LittleFS mount FAILED
```
| Check | Action |
|---|---|
| Was filesystem uploaded? | Run: `pio run --target uploadfs` |
| Wrong partition table? | Verify `partitions_custom.csv` is in project root |
| Board not selected correctly? | Must be ESP32S3 Dev Module in PlatformIO |

### Mission Planner cannot connect to Cube Orange

| Symptom | Fix |
|---|---|
| "USB Serial Device" in Device Manager | Install CubePilot driver from ardupilot.org |
| Two COM ports appear | Use the MAVLink port (not SLCAN) |
| Connected but no telemetry | Check baud rate = 115200 |
| PreArm: Check frame class | Set FRAME_CLASS=2, FRAME_TYPE=1 in Full Parameter List |

---

## 18. Development Roadmap

| Phase | Description | Status |
|---|---|---|
| **Phase 1** | Electronics, schematic, pin mapping, power budget | ✅ Complete |
| **Phase 2** | Firmware skeleton: GNSS, servo, config, state machine, logging | ✅ Complete |
| **Phase 3** | Wi-Fi AP config upload, NMEA bench replay tool | 🔲 Next |
| **Phase 4** | MAVLink position source (Cube Orange TELEM2) | 🔲 Planned |
| **Phase 5** | Python config tool: import QGC/UgCS missions, upload via serial | 🔲 Planned |
| **Phase 6** | Full flight testing protocol | 🔲 Planned |

### Phase 3 details (next sprint)

- **Wi-Fi AP config server:** Browse to `http://192.168.4.1/config` to upload new `mission.json` without USB
- **NMEA replay script:** Python script feeds recorded `.nmea` file over serial to simulate GPS flight path — allows full system test on bench without GPS signal
- **Watchdog timer:** Hardware watchdog to reset ESP32 if firmware hangs during flight

### Phase 4 details

- Implement `MavlinkPositionSource` parsing `GLOBAL_POSITION_INT` (MAVLink msg #33)
- Runtime selection: if `position_source = "MAVLINK"` in config, use TELEM2; else use GNSS
- Fallback: if MAVLink stops updating for 2 seconds, fall back to GNSS automatically

---

*Smart Launcher Project — Mangrove Reforestation Initiative*  
*Hardware: Freenove ESP32-S3 WROOM + Cube Orange Standard Set*  
*Firmware: ArduCopter V4.6.3 + Custom PlatformIO C++ (Arduino framework)*

