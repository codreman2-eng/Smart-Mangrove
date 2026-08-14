# Smart-Mangrove

Smart mangrove launcher for reforestation — a self-contained drone payload
that autonomously drops mangrove propagules at GPS waypoints.

The launcher does all drop logic on its own microcontroller. The host drone
only supplies power and flies. Works on a DJI M600 or a DIY ArduPilot
aircraft by changing configuration, not code.

---

## 👉 New to this project? Read [`START_HERE.md`](START_HERE.md)

It covers safety, the hardware, setting up the tools, flashing, using the
dashboard, and what to work on next — written for someone with no prior
knowledge of this project.

---

## Current status

| | |
|---|---|
| Firmware | Mission Dashboard, running on ESP32 WROVER-E |
| Mode | **SIM — servos electrically disconnected, LED fires instead** |
| Position | DJI M600 OSDK at ~47 Hz, verified outdoors with a real fix |
| Proven | Waypoint triggering, drop sequencing, flight logging |
| Not proven | Pilot RC trigger end-to-end; servos never driven; nothing flown |
| Open | 3 known bugs left in deliberately, awaiting confirmation |

**Do not connect servos.** See `START_HERE.md` §1 and §9.

## Documents

| File | Purpose |
|---|---|
| [`START_HERE.md`](START_HERE.md) | Beginner's guide — start here |
| [`SIM_MODE_VALIDATION_PLAN.md`](SIM_MODE_VALIDATION_PLAN.md) | The test script — this is the next work |
| [`PROJECT_HANDOFF.md`](PROJECT_HANDOFF.md) | Full history and reasoning |
| [`DESIGN_RATIONALE.md`](DESIGN_RATIONALE.md) | Why the launcher runs its own MCU |
| [`SMART_LAUNCHER_COMPLETE_GUIDE.md`](SMART_LAUNCHER_COMPLETE_GUIDE.md) | Wiring, BOM, mechanical build |

## Layout

```
Servo_Web_Calibrator_WROVER/   main firmware (ESP32) — Pins.h is the pin authority
RC_Input_Calibration_Tool.ino  test tool: measure an RC signal (ESP32)
RC_Pulse_Generator_Mega2560.ino test tool: generate an RC signal (Mega 2560)
tools/osdk_sniff.py            PC tool: decode DJI OSDK telemetry
```

Build with the **Arduino IDE**, board **ESP32 Wrover Module**. Not
PlatformIO — see `PROJECT_HANDOFF.md` for why that matters here.
