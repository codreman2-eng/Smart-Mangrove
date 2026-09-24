# Smart Launcher — Complete Implementation & Wiring Guide

**Project:** Autonomous drone seed release system for mangrove reforestation  
**Hardware:** Freenove ESP32-WROVER-E + DJI M600 / A3 Pro (Cube Orange+ = Phase 4)  
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

- **Zero *control* coupling to the flight controller.** The launcher reads position — from its own GNSS, or from the DJI A3's unprompted OSDK broadcast (Phase 3d) — and **never transmits** to the aircraft. No PSDK or Channel Expansion Kit, and no commanded flight. The original "no OSDK at all" rule was narrowed to receive-only, not dropped.
- **Power only from the drone.** The drone provides 2S–6S power. All drop logic runs on the ESP32 onboard.
- **Two independent modules, synchronized.** Each module holds 5 seeds. Module A fires first; Module B fires immediately after A's limit switch confirms completion. This preserves balance.
- **Configurable without reflashing.** All mission parameters (waypoints, timing, PWM values) live in `mission.json` on the ESP32 filesystem. Change the file, re-upload the filesystem, fly.
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
│  │  ArduCopter 4.6  │ TELEM2   │   Freenove ESP32-WROVER-E   │ │
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
| MCU | Freenove ESP32-WROVER-E | 1 | PSRAM, built-in Wi-Fi. Arduino IDE board: **ESP32 Wrover Module** |
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
| JST-GH 6-pin cable | Cube Orange+ TELEM2 cable | Phase 4 MAVLink feed — pins unassigned, see 4.6 |
| USB-C cable | Data-capable (not charge-only) | Firmware upload |

### For bench testing only

| Item | Purpose |
|---|---|
| Desktop 5V / 3A power supply | Power servos without drone battery |
| Bench alligator clips | Temporary connections |
| Multimeter | Verify voltages before first power-on |

---

## 4. Wiring Guide

### 4.1 Complete pin mapping — ESP32-WROVER-E

> **`Servo_Web_Calibrator_WROVER/Pins.h` is the authority.** These values were
> validated on real hardware by serial-checkpoint bisection. If this table and
> `Pins.h` ever disagree, `Pins.h` is right and this table is stale.

| Signal | GPIO | Direction | Voltage | Notes |
|---|---|---|---|---|
| Servo A PWM | GPIO 12 | OUT | 3.3V | `ledcAttach`, 50 Hz, 16-bit |
| Servo B PWM | GPIO 13 | OUT | 3.3V | `ledcAttach`, 50 Hz, 16-bit |
| Limit Switch A | GPIO 32 | IN | 3.3V | INPUT_PULLUP, active LOW |
| Limit Switch B | GPIO 33 | IN | 3.3V | INPUT_PULLUP, active LOW |
| Status LED | GPIO 22 | OUT | 3.3V | Plain 2-pin LED + 330R. **Not** the board's built-in |
| Wait LED (yellow) | GPIO 27 | OUT | 3.3V | 330R to GND. Steady ON = ARMED |
| Drop announce LED | GPIO 14 | OUT | 3.3V | 330R to GND. Lit for the servo drive window |
| RC pilot trigger | GPIO 35 | IN | 3.3V | Input-only pin — **external 10k pull-down required** |
| OSDK RX | GPIO 25 | IN | 3.3V | ← DJI A3 Pro API port TX, 921600 baud (UART2) |
| OSDK TX | GPIO 26 | OUT | 3.3V | Wired for future use; never driven — the link is receive-only |
| GNSS UART RX | GPIO 4 | IN | 3.3V | ← GNSS module TX (UART1) |
| GNSS UART TX | GPIO 5 | OUT | 3.3V | → GNSS module RX (UART1) |
| ARM button | GPIO 0 | IN | 3.3V | Onboard BOOT button, hold 2s **after** boot. Secondary path — the dashboard ARM button is primary |
| GND | GND | — | 0V | Common ground — CRITICAL |
| 5V in | 5V pin | PWR IN | 5V | From UBEC output |

#### ⛔ Pins that must never be used on this board

| Pin | Why | How it was found |
|---|---|---|
| GPIO 2 | Strapping pin. Driven as OUTPUT during/around boot it boot-loops (`rst:0x8 TG1WDT_SYS_RESET`) | Cost a debugging session |
| GPIO 9, 10 | Permanently wired to the shared flash+PSRAM SPI bus on WROVER modules, regardless of flash mode. `pinMode(9, INPUT_PULLUP)` crashes the board instantly | Live serial bisection |
| GPIO 16, 17 | Same PSRAM bus. These are `Serial2`'s **default** pins, so UART2 pins must always be passed explicitly to `Serial2.begin()` | Caught before it cost anything |

Free and safe if you need a spare: **18, 19, 21, 23**.

> **A 3.3V GPIO signal is safe for servo signal wires.** Most servos accept 3.3V logic on the signal pin even when powered at 5V. Tested with FS5106R.

> **Note on GPIO 14:** the boot ROM emits a brief PWM burst on this pin, so a
> flicker of the drop announce LED at power-on — before checkpoint `[1]` prints —
> is expected, not a fault.

---

### 4.2 Servo wiring

Each servo has 3 wires. Standard colour coding:

```
Servo wire colour → Connect to
─────────────────────────────────────────────────────
Brown / Black     → UBEC GND  AND  ESP32 GND (common)
Red               → UBEC 5V output
Orange / Yellow   → ESP32 GPIO (12 for A, 13 for B)
```

> **Never power servos from the ESP32 5V or 3.3V pins.** Always use the UBEC. Servo stall current (up to 1.5A each) will reset or damage the ESP32 regulator.

**Add a 100µF electrolytic capacitor** across the servo power rail (+ to 5V, – to GND) near each servo connector. This absorbs voltage spikes when the servo starts or stalls.

> **`SIM_MODE` in `Pins.h` decides whether these pins are driven at all.** With
> `SIM_MODE 1`, `ledcAttach()` is never called on GPIO 12/13 — the signal lines
> stay high-impedance and no PWM is generated. **Also unplug the servo UBEC for
> sim runs.** Set it to `0` only for a flight build.

---

### 4.3 Limit switch wiring

Each micro switch has 3 terminals: COM, NO (Normally Open), NC (Normally Closed).  
**Use COM and NO.**

```
Limit switch terminal → Connect to
──────────────────────────────────
COM                   → GND
NO                    → GPIO 32 (Module A) or GPIO 33 (Module B)
NC                    → not connected
```

The firmware configures these pins as `INPUT_PULLUP`. When the cover reaches the end of the rail and presses the switch, the GPIO reads LOW → servo auto-stops.

> ⚠️ **Wire these before `SIM_MODE` goes to 0.** With the switches unwired the pin
> floats high and the last door of each module runs to its timeout instead of
> stopping: module A drove **1740 ms against a 580 ms target** — a 3x overdrive.
> Harmless in SIM, mechanically dangerous with real servos.
>
> Only the **last** door of a module reads the switch (`_is_last_door`); doors 1–4
> use `travel_ms` and never read the pin, so they are safe to fire un-switched.

---

### 4.4 GNSS module wiring (u-blox M8N / BN-880)

```
GNSS module pin → ESP32 GPIO
────────────────────────────────
VCC (3.3V)      → 3V3 pin on ESP32 board
GND             → GND
TX              → GPIO 4 (ESP32 UART1 RX)
RX              → GPIO 5 (ESP32 UART1 TX)
```

> The BN-880 runs at 3.3V. Do **not** connect VCC to the 5V rail.

---

### 4.5 DJI M600 / A3 Pro wiring (primary position + pilot trigger)

This is the link that is actually flown today; the M8N above is the fallback.

**API port → OSDK position (receive-only):**

```
A3 "API" port pin → ESP32
──────────────────────────────────────────────────
TX                → GPIO 25   (330R series optional)
GND               → GND       ← MANDATORY
RX                → NOT CONNECTED
VCC               → NOT CONNECTED
```

Both sides are 3.3V — no level shifter. 921600 baud, as set in DJI Assistant 2.
**The launcher never transmits on this link** — that is a design rule, not a
limitation. Trimming the broadcast in Assistant 2 to POSITION+GPS+RC+STATUS cuts
the frame from 231 to ~60 bytes, which is safe: the decoder computes field
offsets from the `passFlag`.

**F-channel → pilot RC trigger:**

```
A3 F-channel signal → ESP32 GPIO 35
                    → 10k pull-down from GPIO 35 to GND   ← REQUIRED
A3 GND              → ESP32 GND                            ← MANDATORY
```

GPIO 35 is input-only (34–39 are), so no firmware bug can ever back-drive the
flight controller — that is the reason for the choice, not convenience.
Input-only pins have **no internal pull resistor**, so without the external 10k
pull-down a disconnected signal wire floats and reads as noise instead of
"no signal".

> **MEASURE THE F-CHANNEL VOLTAGE BEFORE CONNECTING.** It should be 3.3V logic.
> If it reads 5V, fit a 10k/20k divider or a level shifter — the ESP32 is not 5V
> tolerant.

Bring-up: use `RC_Input_Calibration_Tool.ino` (repo root) before trusting this
path in the dashboard firmware.

---

### 4.6 Cube Orange+ TELEM2 wiring (Phase 4 — MAVLink, NOT YET WIRED)

Phase 4 would take position from the Cube Orange+ over MAVLink as a cross-check
and mode/abort detector. **No pins are assigned for this yet**, and the obvious
choice is a trap:

- **Not GPIO 16/17.** Those are `Serial2`'s defaults *and* the WROVER PSRAM bus.
- UART0 is the console, UART1 is the GNSS (GPIO 4/5), and **UART2 is already
  taken by the DJI OSDK link** (GPIO 25/26). A third UART has to be resolved
  before this is wired — pick free pins (18/19/21/23) and pass them explicitly
  to `begin()`.

For reference, the Cube Orange carrier's TELEM2 port is a **JST-GH 6-pin**:

```
TELEM2 pin (JST-GH) → Signal    → ESP32
─────────────────────────────────────────────────
Pin 1               → +5V       → NOT connected (ESP32 already powered)
Pin 2               → TX (out)  → ESP32 UART RX   (pin TBD — NOT 16)
Pin 3               → RX (in)   → ESP32 UART TX   (pin TBD — NOT 17)
Pin 4               → CTS       → NOT connected
Pin 5               → RTS       → NOT connected
Pin 6               → GND       → GND (common ground)
```

> Both TX/RX are 3.3V TTL — directly compatible with ESP32. No level shifter needed.

---

### 4.7 Complete wiring diagram (text schematic)

```
                    ┌─────────────────────────────┐
  DRONE BATTERY     │    UBEC  5V / 5A            │
  2S–6S ──────────▶ │  IN: 7.4–25.2V              │──── 5V out ──┬── Servo A red
                    │  OUT: 5V regulated          │              ├── Servo B red
                    └─────────────────────────────┘              └── ESP32 5V pin

  GND bus ──────────────────────────────────────────────────────────────────────
           │         │          │          │          │          │        │
         UBEC GND  Servo A    Servo B   Limit SW A  Limit SW B  GNSS GND  A3 GND
                   brown/blk  brown/blk  COM         COM

  ┌──────────────────────────────────────────────────────────────────────┐
  │                   Freenove ESP32-WROVER-E                            │
  │                                                                      │
  │  GPIO 4  ◀──────────────────────────────────── GNSS TX              │
  │  GPIO 5  ──────────────────────────────────▶   GNSS RX              │
  │  GPIO 25 ◀──────────────────────────────────── A3 API TX (OSDK)     │
  │  GPIO 26 ──────────────────────────────────▶   A3 API RX (unused)   │
  │  GPIO 35 ◀──── 10kΩ pull-down to GND ────────  A3 F-channel (RC)    │
  │  GPIO 12 ──────────────────────────────────▶   Servo A signal (org) │
  │  GPIO 13 ──────────────────────────────────▶   Servo B signal (org) │
  │  GPIO 32 ◀──── internal pullup ──── Limit SW A NO ──── GND          │
  │  GPIO 33 ◀──── internal pullup ──── Limit SW B NO ──── GND          │
  │  GPIO 0  ◀──── onboard BOOT button                                  │
  │  GPIO 22 ──── [330R] ──── Status LED ──── GND                       │
  │  GPIO 27 ──── [330R] ──── Wait LED (yellow) ──── GND                │
  │  GPIO 14 ──── [330R] ──── Drop announce LED ──── GND                │
  │  GND     ──────────────────────────────── Common GND bus            │
  │  5V      ◀──────────────────────────────── UBEC 5V out              │
  └──────────────────────────────────────────────────────────────────────┘

  Note: the limit-switch 10kΩ pullups are INTERNAL (INPUT_PULLUP) — no external
  resistors needed. The GPIO 35 pull-DOWN is EXTERNAL and IS required.
  Never use GPIO 2, 9, 10, 16 or 17 on this board — see 4.1.
```

---

## 5. Power System

### 5.1 Power budget

| Component | Voltage | Typical current | Peak (stall) |
|---|---|---|---|
| ESP32-WROVER-E | 5V → 3.3V internal | 80 mA | 250 mA (Wi-Fi TX) |
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

> **The firmware is built with the Arduino IDE, not PlatformIO.** The live
> sketch is `Servo_Web_Calibrator_WROVER/Servo_Web_Calibrator_WROVER.ino`.
> The PlatformIO project under `smart_launcher_phase2/` is the abandoned Phase 2
> skeleton — see 7.6 before touching it.

### 7.1 Arduino IDE — board and core

1. Download Arduino IDE 2.x from [arduino.cc](https://www.arduino.cc/en/software)
2. `File → Preferences` → paste into "Additional boards manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. `Tools → Board → Boards Manager` → search "esp32" → install **esp32 by
   Espressif Systems**. The project machine has **3.3.11** installed (verified
   via `arduino-cli board details`, 2026-09-14); match it if you can. Note
   `START_HERE.md` still says 3.3.8 — it has drifted.
4. Select board: `Tools → Board → ESP32 Arduino → **ESP32 Wrover Module**`

| Setting | Value |
|---|---|
| Board | **ESP32 Wrover Module** |
| Upload Speed | 921600 |
| Flash Frequency | 80 MHz |
| Partition Scheme | Default 4MB with spiffs |

> ⛔ **"ESP32 Wrover Module" is not optional.** Choosing "ESP32 Dev Module"
> misconfigures PSRAM and the board crashes on boot. This already happened once.
> "ESP32S3 Dev Module" is wrong for a different reason — this is not an S3.

### 7.2 Turn compiler warnings on

`File → Preferences → Compiler warnings → **Default**` (or higher).

The sketch carries a deliberate `#warning` that fires on a flight build,
restating which defects were confirmed on hardware, which were closed by code
reading only, and that the servos have never been driven by this firmware. At
the IDE's default setting of "None" you would never see it.

### 7.3 Uploading

1. Open `Servo_Web_Calibrator_WROVER/Servo_Web_Calibrator_WROVER.ino`
2. Check `#define SIM_MODE` in `Pins.h` — for bench work **it must be `1`**
3. Set board (7.1) and port
4. Upload. Wait for `Hash of data verified` and `Hard resetting`

**The COM port moves.** The launcher enumerates as a **CH340** and has been seen
on COM6 and COM10; unplugging and replugging can move it. Never assume — plug
in, check `Tools → Port`, unplug, look again; the one that disappeared is yours.
It is a different device from the Cube Orange+.

**Only one program can hold a port.** If upload fails with "Access denied" or
"port busy", the Serial Monitor is almost certainly still open. Close it. This
has caught us three times.

### 7.4 There is no filesystem upload step

The old `uploadfs` instruction does not apply to this firmware. `ConfigManager`
calls `LittleFS.begin(true)`, which formats on first mount if needed, and
`/mission.json` **self-seeds from a PROGMEM default** (`MissionDefault.h`) when
it is missing. Waypoints and calibration are then written back to `mission.json`
from the device itself — the dashboard's Save button, or the SIM-only serial
commands `wpadd` / `wpradius` / `wpclear` / `wpsave`.

Nothing needs to be pushed to flash from the PC except the sketch.

### 7.5 Required libraries

Install via `Tools → Manage Libraries`:

| Library | Version | Purpose |
|---|---|---|
| TinyGPSPlus | ^1.0.3 | NMEA sentence parsing |
| ArduinoJson | ^7.0.0 | JSON config parsing |

`WiFi`, `WebServer`, `LittleFS` and `HardwareSerial` ship with the ESP32 core —
nothing to install.

### 7.6 Headless builds (arduino-cli)

`arduino-cli` is **not on PATH**. It ships inside the IDE install:

```bash
ARDUINO_CLI="C:/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"
export ARDUINO_DIRECTORIES_DATA="C:/Users/carli/AppData/Local/Arduino15"

"$ARDUINO_CLI" compile --fqbn esp32:esp32:esp32wrover Servo_Web_Calibrator_WROVER
```

Using the IDE's own data directory is what makes the CLI see the same installed
core and libraries as the IDE. `esp32:esp32:esp32wrover` is the same board
profile as "ESP32 Wrover Module" in 7.1.

### 7.7 PlatformIO — historical only

`smart_launcher_phase2/smart_launcher/platformio.ini` targets
`freenove_esp32_wrover` and was the Phase 2 build. **That tree is not the flight
firmware and has not been touched since Phase 3 landed.** Its pin `build_flags`
were partly corrected during the WROVER port, but they still carry
`-DPIN_MAVLINK_RX=16 -DPIN_MAVLINK_TX=17` — the PSRAM-bus trap described in 4.1.
Do not copy pins out of it; `Pins.h` is the authority.

---

## 8. Firmware — Phase 2 Skeleton

### 8.1 Project structure

> ⚠️ **This is the Phase 2 skeleton's layout, not the flight firmware.** The
> live code is a flat Arduino sketch folder, `Servo_Web_Calibrator_WROVER/`,
> holding `Servo_Web_Calibrator_WROVER.ino` plus `Pins.h`, `ConfigManager`,
> `PositionSource` / `GNSSPositionSource` / `OsdkPositionSource` /
> `SimPositionSource` / `FallbackPositionSource`, `ActuatorModule`,
> `StateMachine`, `DropLogicEngine`, `RcPwmInput`, `Ballistics.h`,
> `MissionDefault.h` and `Logger`. The module responsibilities in 8.2 still
> describe it; the directory tree below does not. See 7.7.

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
- Heartbeat LED blink on GPIO 22

### 9.2 How to use

1. Flash `servo_tester.ino` with Arduino IDE
2. Connect phone/laptop to Wi-Fi: **"SmartLauncher"** (no password)
3. Open browser → `http://192.168.4.1`
4. Status LED (GPIO 22) blinking = server healthy

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

The mission config lives at `/mission.json` on the ESP32 LittleFS filesystem.

**It is not uploaded from the PC.** On first boot it self-seeds from a PROGMEM
default (`MissionDefault.h`), and it is edited on the device thereafter:

- the Wi-Fi dashboard's **Save** button (edits live in RAM until you save), or
- the SIM-only serial commands `wpadd` / `wpradius` / `wpclear` / `wpsave`.

See 7.4. The annotated structure below documents the file's schema.

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
Cube Orange+ TELEM2      ESP32-WROVER-E
(JST-GH 6-pin)
─────────────────────────────────────────
Pin 1 (+5V)    →    NOT CONNECTED
Pin 2 (TX)     →    ESP32 UART RX  (pin TBD — NOT 16, see 4.6)
Pin 3 (RX)     →    ESP32 UART TX  (pin TBD — NOT 17, see 4.6)
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
□ Status LED (GPIO 22) solid on → BOOT state; a brief GPIO 14 flicker at power-on is the boot ROM, not a fault
□ Serial monitor shows:
    === Smart Launcher booting ===
    [CFG] LittleFS mounted OK
    [CFG] Loaded 'mangrove_plot_A' — 5 drop points
    [HW] Servos initialised — both at 1500µs (STOP)
    [SM] State machine started — BOOT (state LED GPIO22, wait LED GPIO27)
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
3. Type 'arm' in serial monitor → status LED goes to a 2 Hz blink, wait LED steady on (§14)
4. Type 'dropa' → servo runs for travel_ms, stops
5. Measure how far rack moved
6. Adjust travel_ms via the dashboard's calibration Save (writes mission.json)
7. Reboot or reload config — no filesystem upload, see 7.4
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
4. Hold ARM button for 2+ seconds → status LED goes to a 2 Hz blink, wait LED steady on (§14)

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
                    │ BOOT │  startup, self-check
                    └──┬───┘
                       │ config OK
                    ┌──▼───┐
              ┌────▶│ IDLE │  ready, drop logic off
              │     └──┬───┘
              │        │ ARM (dashboard button, or BOOT button held 2s)
              │     ┌──▼────┐
              │     │ ARMED │  drop logic active
              │     └──┬────┘
              │        │ in radius + armed
              │     ┌──▼──────────┐
              │     │  DROPPING   │  executing drop
              │     └──┬──────────┘
              │        │ drop complete
              └────────┘ back to ARMED

         Any state ──(error)──▶ FAULT  servos stopped
```

### There is no RGB LED on this board

State is signalled by **blink cadence across three single-colour LEDs**, not by
colour — `PIN_LED` is a plain 2-pin LED, not a WS2812. See
`StateMachine::_updateLED()` / `_updateWaitLED()`.

| State | Status LED (GPIO 22) | Wait LED, yellow (GPIO 27) |
|---|---|---|
| BOOT | Solid on | Dark until IDLE |
| IDLE | Slow single blink (150 ms on / 2 s period) | Slow blink (120 ms on / 1.5 s period) |
| ARMED | Steady 2 Hz blink (250/500 ms) | **STEADY ON** — the "waiting for trigger" state |
| DROPPING | Rapid blink (50/100 ms) | **Dark** — the announce LED has the floor |
| FAULT | Slow 50% on/off (500/1000 ms) | Rapid blink (100/200 ms) |

The wait LED is deliberately the **inverse** of the announce LED during DROPPING:
yellow steady = holding, yellow dark + announce lit = firing. That makes the
A→B handoff readable at a glance, and makes an unexpected re-fire after an
emergency stop obvious.

### Drop announce LED (GPIO 14)

Active in **both** SIM and flight builds, so bench behaviour matches the aircraft.
It is lit for exactly the window the servo is (or would be) driven, then — after
a 200 ms dark gap — blips out which module just dropped:

| Module | Confirm pattern |
|---|---|
| A | Short blips — 80 ms on / 120 ms off |
| B | Long blips — 400 ms on / 200 ms off |

> A brief flicker on GPIO 14 at power-on, before checkpoint `[1]` prints, is the
> boot ROM's PWM burst — expected, not a fault.

---

## 15. Serial Command Reference

Connect at **115200 baud**. The board enumerates as a **CH340** — no FTDI
adapter needed, but the port moves; see 7.3. Opening the port does *not* reset
the board (only `tools/bench_serial.py`'s explicit DTR/RTS sequence does), and
serial works at the same time as the Wi-Fi dashboard.

> Timing-critical sequences should be scripted with `tools/bench_serial.py`, not
> hand-typed.

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
□ Firmware uploaded from the Arduino IDE, board "ESP32 Wrover Module" (7.1)
□ SIM_MODE set deliberately in Pins.h — and limit switches wired if it is 0 (4.3)
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
□ Hold ARM button 2 seconds → status LED 2 Hz blink + wait LED steady on (ARMED)
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
| Is PWM wire connected? | Check GPIO 12 (A) or GPIO 13 (B) |
| Is common ground connected? | UBEC GND and ESP32 GND must share ground |
| Is servo signal 3.3V compatible? | FS5106R accepts 3.3V signal — should be fine |

### Servo does not stop

| Symptom | Cause | Fix |
|---|---|---|
| Servo creeps slowly | Neutral not calibrated | Use servo tester slider to find true stop |
| Servo runs continuously | Limit switch wired incorrectly or unwired | Verify COM→GND, NO→GPIO 32 (A) / 33 (B). Only the LAST door of a module reads the switch |
| Servo runs past door | travel_ms too high | Reduce by 20ms, re-test |

### GPS not acquiring fix

| Check | Action |
|---|---|
| Is antenna outdoors or near window? | GNSS needs sky view |
| Are UART wires swapped? | Verify GNSS TX → GPIO 4, GNSS RX → GPIO 5 |
| Is GNSS powered correctly? | 3.3V only — never connect to 5V |
| Cold start? | Wait 90s for first fix, 10s for warm start |
| Is baud rate correct? | Default M8N is 9600 — check `GNSSPositionSource` |

### LittleFS mount fails

```
[CFG] LittleFS mount FAILED
```
| Check | Action |
|---|---|
| Nothing to upload — is the partition scheme right? | Must be "Default 4MB with spiffs" (7.1). `LittleFS.begin(true)` formats on first mount, so a genuine mount failure is a partition or flash problem, not a missing upload |
| Board not selected correctly? | Must be **ESP32 Wrover Module** — "ESP32 Dev Module" misconfigures PSRAM (7.1) |

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
*Hardware: Freenove ESP32-WROVER-E + DJI M600 / A3 Pro (Cube Orange+ Standard Set = Phase 4)*  
*Firmware: ArduCopter V4.6.3 (Cube Orange+) + custom Arduino C++ built with the Arduino IDE*

