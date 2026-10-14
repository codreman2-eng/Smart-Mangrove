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
| Committed build | **SIM (`SIM_MODE 1` in `Pins.h`) — servo pins never attached, an LED fires instead** |
| Bench board | Was flashed 2026-09-30 with a **flight build** (`SIM_MODE 0`). Reflash from this repo before bench work and check boot line `[5]` |
| Position | DJI M600 OSDK at ~47 Hz, verified outdoors with a real fix |
| Release logic | Predicts the impact point; fires only inside the radius **and** at ≤ 2.0 m/s horizontal speed (OSDK only — GNSS fallback is radius-only) |
| Proven | Waypoint triggering, drop sequencing, flight logging; all four planted defects closed |
| Not proven | Pilot RC trigger end-to-end; no drop by this firmware on real servos; the predictor has never seen aircraft data; nothing flown |
| Open before flying | `T_EXIT_S` unmeasured (0.0); limit switches unwired; Module B uncalibrated; DJI vertical-velocity sign unverified |

**Do not connect servos unless someone supervising has said so.** See
`START_HERE.md` §1 and §9.

## Documents

| File | Purpose |
|---|---|
| [`START_HERE.md`](START_HERE.md) | Beginner's guide — start here |
| [`SIM_MODE_VALIDATION_PLAN.md`](SIM_MODE_VALIDATION_PLAN.md) | The test script — this is the next work |
| [`PROJECT_HANDOFF.md`](PROJECT_HANDOFF.md) | Full history and reasoning |
| [`DESIGN_RATIONALE.md`](DESIGN_RATIONALE.md) | Why the launcher runs its own MCU |
| [`SMART_LAUNCHER_COMPLETE_GUIDE.md`](SMART_LAUNCHER_COMPLETE_GUIDE.md) | Wiring, BOM, mechanical build |
| [`docs/T_EXIT_MEASUREMENT.md`](docs/T_EXIT_MEASUREMENT.md) | Bench procedure for `T_EXIT_S` — the next measurement |

## Layout

```
Servo_Web_Calibrator_WROVER/   main firmware (ESP32) — Pins.h is the pin authority
RC_Input_Calibration_Tool.ino  test tool: measure an RC signal (ESP32)
RC_Pulse_Generator_Mega2560.ino test tool: generate an RC signal (Mega 2560)
tools/osdk_sniff.py            PC tool: decode DJI OSDK telemetry
tools/bench_serial.py          PC tool: scripted serial bench commands
tools/t_exit_*.py              PC tools: T_EXIT_S trial driver and reduction
tools/test_ballistics.py       PC tool: independent check of Ballistics.h
```

Build with the **Arduino IDE**, board **ESP32 Wrover Module**. Not
PlatformIO — see `PROJECT_HANDOFF.md` for why that matters here.
