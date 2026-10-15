# Smart-Mangrove

Smart mangrove launcher for reforestation — a self-contained drone payload
that autonomously drops mangrove propagules at GPS waypoints.

The launcher does all drop logic on its own microcontroller. The host drone
only supplies power and flies. Works on a DJI M600 or a DIY ArduPilot
aircraft by changing configuration, not code.

---

> **Current work is on branch `sim-mode-and-rc-trigger`, not `main`** (`main`
> stops at 2026-07-10). Clone it with:
> `git clone -b sim-mode-and-rc-trigger https://github.com/codreman2-eng/Smart-Mangrove.git`

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
| Bench board | Was flashed 2026-09-30 with a **flight build** (`SIM_MODE 0`) from `22d753a`. It does **not** have the 2026-10-14/15 changes. Reflash from this repo before bench work and check boot line `[5]` |
| Position | DJI M600 OSDK at ~47 Hz, verified outdoors with a real fix. One receive-only wire (A3 API TX → GPIO 25); keep VELOCITY on in DJI Assistant 2 |
| Release logic | Predicts the impact point; fires when it is inside the waypoint radius, horizontal speed ≤ max speed, held for the dwell time. **Max speed (2.0 m/s), dwell (0 s) and default radius (5 m) are set on the dashboard** (Waypoints → RELEASE SETTINGS). Speed gate and prediction need OSDK velocity; GNSS fallback is radius + dwell |
| Release settings | Host-tested against the real drop-logic code (2026-10-15); **not yet run on the board** — plan steps 21–26 |
| Pilot RC trigger | Not wired (F-channel removed from the guides); ships disabled |
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
| [`reports/Estado de waypoints en el M600.md`](reports/Estado%20de%20waypoints%20en%20el%20M600.md) | Research: can the M600 tell the launcher a waypoint was reached? (Spanish) |

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
