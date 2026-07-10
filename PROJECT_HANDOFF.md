# Smart Launcher — Project Handoff / Context Resume

Paste this whole document into a new chat to continue seamlessly.

---

## WHAT THE PROJECT IS

A self-contained, platform-agnostic drone payload module ("Smart Launcher") that autonomously
releases mangrove propagules (seeds) at predefined GPS coordinates. For reforestation.

Core principle: the launcher does ALL drop logic onboard. The host drone only provides power
and flies a waypoint mission. No DJI OSDK/PSDK. Works on DJI M600 or DIY ArduPilot drones by
changing config only.

The user is comfortable with C++/Arduino, Python, ROS, embedded work, and 3D printing.
This is a legitimate environmental/academic project (mangrove reforestation).

---

## PHYSICAL MECHANISM (CONFIRMED)

- TWO independent launcher modules, mounted symmetrically for balance.
- Each module: 1 continuous rotation servo + rack-and-pinion + 5 seed doors (35mm wide each).
- A sliding COVER retracts progressively. Each waypoint the servo advances the cover ~35mm,
  exposing the next door. Seeds fall by gravity. Cover only moves ONE direction (retract).
- 1 limit switch per module at the fully-retracted end (signals all 5 seeds dropped).
- Firing is SEQUENTIAL CONFIRM: Module A fires, its limit switch confirms, then Module B fires.
  This preserves balance.

### Gear math (for calibration)
- Pinion: 38mm diameter, 24 teeth.
- Circumference = pi x 38 = 119.4mm. Travel per tooth = 4.97mm.
- 35mm travel = ~0.293 revolutions.
- FS5106R at 5V ~= 60 RPM = 1 rev/sec. So 35mm ~= 293ms. Starting firmware value: 350ms.
- travel_ms MUST be calibrated empirically on the bench (mark rack, run timed, measure, adjust).

---

## HARDWARE

### Current MCU (CHANGED during project)
- Now using: ESP32 WROVER-E (classic ESP32, has 8MB PSRAM).
- Originally spec'd ESP32-S3; then Freenove ESP32-S3 WROOM; now WROVER-E.
- Servo confirmed working on GPIO 12 (user tested, spins fine).

### Servos
- FS5106R continuous rotation (also mentioned Futaba/Parallax CR). 4.8-6V.
- Continuous rotation: 1500us = STOP, >1500 = one direction, <1500 = other.
- Module A open = 1700us. Module B open = 1300us (opposite because they retract toward
  each other for balance).

### Pin assignments (WROVER-E)
- Servo A signal -> GPIO 12  (CONFIRMED WORKING)
- Servo B signal -> GPIO 13
- Limit switch A -> GPIO 9  (INPUT_PULLUP, active LOW, COM->GND, NO->GPIO)
- Limit switch B -> GPIO 10
- DO NOT USE GPIO 2 — it is a strapping pin on WROVER-E and caused boot crashes.

### Power (CRITICAL - this was a real bug source)
- Servos MUST be powered from external 5V (UBEC), never from the ESP32.
- Recommended: 5V/5A UBEC (2x FS5106R stall ~= 3A + margin).
- COMMON GROUND REQUIRED: servo GND + external supply GND + ESP32 GND all tied together.
  (User initially had servo ground only to external supply -> servo dead. Fixed.)

### GNSS (for later phases)
- u-blox M8N (BN-880 etc), UART, 3.3V. Onboard position source.

---

## SERVO CALIBRATION FINDINGS (in progress)

Using `Servo_Calibration_Tool_A.ino` / `Servo_Calibration_Tool_B.ino` (root of repo —
single-servo identification tools, ESP32Servo library, wide 500-2500us range,
`writeMicroseconds()` sends the pulse directly with no rescaling, so these numbers
transfer 1:1 into `mission.json` and the raw-`ledcWrite` sketches).

- **Module A (GPIO 12): true stop = 1650us.** Confirmed via slider/step sweep —
  a genuine flat stop, not slow creep. This is 150us off the textbook 1500 center,
  much larger than the ±20us the guide assumed. Written into
  `smart_launcher_phase2/smart_launcher/data/mission.json` (`modules.A.servo_stop_pwm_us`).
- **Module B (GPIO 13): NOT YET MEASURED.** Do not assume it mirrors A's offset —
  it's a physically different unit. Use `Servo_Calibration_Tool_B.ino` the same way.

### Why this invalidates prior open/close and travel_ms numbers
Every calibrator/firmware in this repo hardcodes open/close pulses as fixed offsets
from 1500 (1700/1300 for A, 1300/1700 for B), not from the real measured stop.
At true stop 1650, an "open" command of 1700 is only a 50us push (near-stall,
crawling) while "close" at 1300 is a 350us push (fast) — a big asymmetry that
means the servo was NOT running at the speed any earlier `travel_ms` figure
assumed. Do not trust the current `travel_ms: 350` value for Module A.

### Next steps (in order)
1. Measure Module B's true stop with `Servo_Calibration_Tool_B.ino`.
2. Use `Servo_Web_Calibrator_WROVER.ino` (see below) to pick working
   open/close offsets and run the mark-run-measure-adjust travel_ms loop
   for both modules.
3. Update `mission.json` with final stop/open/travel_ms for both A and B.
4. Move to bench Test 4 (single/5-door drop timing + limit switch) in
   `SMART_LAUNCHER_COMPLETE_GUIDE.md`.

### Web UI calibrator — root cause of earlier boot failures found

The two earlier web-UI attempts (`servo_calibrator.ino`, `servo_cal_final.ino`)
never got confirmed working. Root cause: both still define `LED_PIN = 2` and
drive it as `OUTPUT`, directly contradicting the already-confirmed lesson
above ("DO NOT USE GPIO 2" — boot loops). The chunked-HTML change in
`servo_cal_final.ino` fixed a different suspected issue (large PROGMEM block)
but never removed the GPIO 2 regression, so the underlying crash cause was
likely still present in "the last working" web attempt.

**`Servo_Web_Calibrator_WROVER.ino`** (new, repo root) retries the web UI with:
- Heartbeat LED moved to GPIO 22 (ordinary pin, not touched by anything else) — GPIO 2 is never referenced.
- Same chunked HTML (3 parts) and non-blocking timed runs already validated as good practice.
- Editable "Stop (us)" + "Offset (us)" per module instead of hardcoded
  1700/1300 — defaults to A=1650 (measured), B=1500 (placeholder, update
  once B is measured). OPEN/CLOSE compute live from these and mirror
  correctly for B (opposite retract direction from A).
- Serial checkpoint prints through `setup()` (same debugging pattern as
  `debug_boot.ino`) so if it still crashes, the last checkpoint printed
  pinpoints where.

### CONFIRMED WORKING on real hardware (flashed via arduino-cli, COM10)

Flashing revealed a SECOND bug beyond the GPIO 2 fix: the board still
boot-looped with `rst:0x8 (TG1WDT_SYS_RESET)`, this time crashing the
instant `pinMode(9, INPUT_PULLUP)` ran (limit switch A). Bisected step by
step on real hardware:
  - LED (GPIO22) only                          -> boots clean
  - + limit switches on GPIO 9/10               -> crashes on GPIO 9
  - + limit switches on GPIO 32/33 instead       -> boots clean
  - + LEDC servo attach (12/13)                  -> boots clean
  - + WiFi AP + HTTP server                      -> boots clean, full checkpoint log

Switching FlashMode QIO->DIO did NOT fix it (ruled that theory out).

**Root cause: this is a WROVER module (in-package PSRAM). GPIO 6-11 are
wired internally to the shared flash+PSRAM SPI bus UNCONDITIONALLY on
WROVER boards — not just in QIO mode like a plain WROOM without PSRAM.
GPIO 9/10 can never be regular GPIO on this board, no firmware fix
possible.** This means every earlier sketch in this repo that put limit
switches on GPIO 9/10 (`servo_serial.ino`, `servo_calibrator.ino`,
`servo_cal_final.ino`) was silently broken from the start — they just
never got far enough to prove it because the GPIO 2 bug crashed first.

`Servo_Web_Calibrator_WROVER.ino` now uses **GPIO 32/33** for the limit
switches (updated in both the header comment and the HTML). Confirmed
full boot log on hardware:
```
[1] GPIO init (LED on 22, NOT 2)
[2] LEDC attach (v3.x API)
[3] WiFi AP
[4] IP: 192.168.4.1
[5] HTTP server
[6] Ready — connect to SmartLauncher -> http://192.168.4.1
[7] setup() complete — loop starting
```

**ACTION REQUIRED before using it physically:** if the limit switches are
currently wired to GPIO 9/10 on the actual launcher hardware, move both
wires to GPIO 32/33 before relying on this tool's limit-switch readout.

This WROVER-specific GPIO 6-11 constraint is unrelated to the ESP32-S3
used in the Phase 2 firmware (`smart_launcher_phase2/`) — that chip has a
different flash/PSRAM pin layout, so its GPIO 9/10 use for limit switches
is not automatically invalidated by this finding, but is worth
double-checking separately whenever that firmware gets ported/tested.

### Round 2 fixes (usability, after first hands-on test)

User tested the flashed calibrator and found OPEN/CLOSE were swapped on
the real servo (the code's assumed direction convention — A opens by
going above stop, B opens by going below — didn't match actual wiring).
Rather than guess and reflash again, added a **"Reverse" checkbox per
module** in the web UI: flips OPEN/CLOSE instantly, client-side, no
reflash needed. Check it for whichever module(s) actually came out
backwards.

Also added an **"Apply Correction" helper** for the travel_ms calibration
loop: type the ruler-measured mm from the last TIMED RUN into a "measured
mm" field and press Apply — it computes
`new_ms = current_ms × (door_width / measured_mm)` and fills in the
corrected ms automatically (door width defaults to 35mm, editable at the
top of the page). Removes the manual arithmetic from the mark-run-measure
loop.

Reflashed and reconfirmed clean boot (same checkpoint sequence as above)
after both changes.

### Round 3 fixes (after first physical OPEN/CLOSE/TIMED RUN test)

User confirmed OPEN/CLOSE work correctly, but only with Reverse checked —
meaning the DEFAULT direction was backwards on the real hardware (not
just occasionally). Flipped the base `opensHigh()` formula so unchecked
is now the confirmed-correct direction; Reverse checkbox still available
per-module for whichever unit needs the opposite.

Also: **TIMED RUN never actually stopped the servo** ("ran
indefinitely"). Root cause: every auto-stop code path (timed-run
completion, both limit-switch triggers, in both `loop()` and
`handleStatus()`) hardcoded `servoWrite(pin, 1500)` to mean "stopped".
Module A's real measured stop is 1650us, not 1500 — writing 1500 to a
servo whose true neutral is 1650 doesn't stop it, it commands a
~150us-off-center creep, which reads as "running forever". Fixed by
having the browser send the module's live "Stop (us)" field as a `stop`
query param on every `/cmd` and `/timed` request; firmware now tracks
`stopPwmA`/`stopPwmB` from that and uses them everywhere it used to
hardcode 1500.

Reflashed and reconfirmed clean boot. **Board also re-enumerated from
COM10 to COM6** (CH340 USB-serial adapter) at some point during this
session — likely unplugged/replugged. Check both if the board isn't
found.

### KNOWN ISSUE (low priority for now): door-to-door travel deviation

At travel_ms=580 (Module A), the 1st timed run measured exactly 35mm.
The 2nd timed run (same 580ms, same OPEN pulse) measured 41mm — a 6mm /
~17% overshoot. Ruled out servo creep at the stop PWM as the cause: user
watched the rack for 15-20s after STOP and it did not move. So this is
NOT the 1650us-stop-isn't-perfectly-centered theory.

Not yet root-caused. Candidate explanations, untested:
  - Backlash/slack in the rack-and-pinion gets taken up differently on a
    "cold start" (1st run after sitting) vs a "warm" run right after a
    previous run — i.e. door 1 might be the anomalous (short) one, not
    door 2 the anomalous (long) one. Test: run the SAME ms across all 5
    door positions in sequence and see if door 1 is the outlier or if
    it's a progressive drift across all doors.
  - Mechanical non-uniformity along the rack (uneven teeth, binding at a
    specific position) — would show up as position-dependent, not
    run-order-dependent. Same test (all 5 doors) distinguishes this from
    the backlash theory: position-dependent would NOT reproduce if you
    re-run door 2 multiple times back-to-back; run-order/backlash theory
    WOULD show door 1 low and doors 2+ consistent with each other.
  - Voltage sag / motor warm-up changing effective speed run-to-run —
    considered less likely since sag would slow the servo (less travel),
    not speed it up, and the deviation is toward MORE travel.

**Why this matters more than "low priority" might suggest:** the cover
retracts progressively and cumulatively across all 5 doors for a given
module (each waypoint just advances it ~35mm further). If every door
after the first overshoots by a similar margin, the error COMPOUNDS —
by door 5 the cumulative overshoot could be large enough to prematurely
uncover a later door's compartment before its own waypoint fires,
dropping that seed early. Worth fully characterizing (not just accepting
as noise) before this module is trusted for real seed drops, even though
it's not blocking further calibration tool work right now.

**Suggested next step (not yet done):** with the web calibrator, run the
current 580ms setting across all 5 door positions on Module A in one
sequence (marking/measuring each), to see whether door 1 is uniquely
short, or whether deviation grows/accumulates door over door. That result
determines whether a single global travel_ms is usable as-is, needs a
one-time "first door" exception, or needs the mechanism itself inspected
for binding/backlash.

---

## FLIGHT CONTROLLER (Cube Orange — separate from launcher)

- User has Cube Orange Standard Set. Successfully flashed ArduCopter V4.6.3, CubeOrangePlus.
- Frame: needs Hexa X (FRAME_CLASS=2, FRAME_TYPE=1) — user's final drone is a 6-rotor.
  (Currently user only has a DIY quadcopter with insufficient lift; drone not final yet.)
- Driver issue was resolved (CubePilot driver + reboot).
- For later MAVLink position source (Phase 4): TELEM2 = SERIAL2.
  Set SERIAL2_PROTOCOL=2, SERIAL2_BAUD=57, SR2_POSITION=4.
  TELEM2 JST-GH 6pin: pin2 TX -> ESP32 GPIO16, pin3 RX -> ESP32 GPIO17, pin6 GND. 3.3V, no shifter.
- The launcher is INDEPENDENT of the Cube. They only share nothing but the user may later
  feed MAVLink position. Default is onboard GNSS.

---

## SOFTWARE ARCHITECTURE (Phase 2 firmware, already built)

Full PlatformIO project exists (was delivered as smart_launcher_phase2.zip). Modules:
- ConfigManager    - loads mission.json from LittleFS (ArduinoJson v7)
- IPositionSource  - abstract interface + haversineDistance() helper
- GNSSPositionSource - TinyGPS++ NMEA parser
- ActuatorModule   - continuous servo via LEDC, limit switch, timed drops
- StateMachine     - BOOT/IDLE/ARMED/DROPPING/FAULT + ARM button + RGB LED
- DropLogicEngine  - Haversine radius check, sequential A->B firing, logging
- Logger           - serial + LittleFS flight log
- main.cpp         - setup/loop + serial command handler

mission.json holds: waypoints (lat/lon/radius), per-module servo PWM values, travel_ms,
position_source ("GNSS" or "MAVLINK"), min_gps_fix_duration_s.

NOTE: The full Phase 2 firmware was written for ESP32-S3 with the RGB LED on GPIO 48.
It needs pin updates for WROVER-E and to REMOVE GPIO 2 usage before running on current hardware.

---

## CRITICAL LESSONS LEARNED (bugs already hit and fixed)

1. LEDC API CHANGED in ESP32 Arduino core v3.x:
   - OLD (v2.x): ledcSetup(ch,freq,res) + ledcAttachPin(pin,ch) + ledcWrite(ch,duty)
   - NEW (v3.x): ledcAttach(pin,freq,res) + ledcWrite(pin,duty)   <- USE THIS
   User is on core v3.x.

2. Servo control that WORKS on this board (confirmed):
   ledcAttach(pin, 50, 16); then ledcWrite(pin, (us*65535)/20000);
   A bare forward-only sketch spun the servo perfectly on GPIO 12.

3. GPIO 2 = strapping pin on WROVER-E. Using it as OUTPUT (for an LED) during boot caused
   rst:0x8 (TG1WDT_SYS_RESET) boot loops. DO NOT USE GPIO 2.

4. Board MUST be set to "ESP32 Wrover Module" in Arduino IDE (not "ESP32 Dev Module").
   Wrong board = PSRAM misconfigured = boot crash. This was a real bug; now fixed.

5. Large PROGMEM HTML strings (~9KB) for the web UI were suspected of causing instability
   on WROVER; web UI was being split into chunks / removed to isolate.

6. Watchdog resets (TG1WDT_SYS_RESET) also happened when a web handler blocked for seconds.
   Fix: make timed servo runs NON-BLOCKING (record start time, stop in loop()).

7. Common ground between servo external supply and ESP32 is mandatory or servo won't move.

8. VCRUNTIME140.dll error in Arduino IDE = Windows issue, fix with VC++ redistributable
   (aka.ms/vs/17/release/vc_redist.x64.exe).

---

## CURRENT EXACT STATE / WHERE WE ARE

- Board settings correct: ESP32 Wrover Module, COM10, 8MB flash, QIO, 80MHz,
  Default 4MB with spiffs, upload 921600.
- servo_minimal.ino boot instability is resolved — base is solid.
- Moved on to servo identification/calibration using `Servo_Calibration_Tool_A.ino`
  (ESP32Servo library, GPIO 12 only, no WiFi/LED/limit switches — same minimal
  philosophy as servo_minimal.ino).
- Module A true stop measured: **1650us** (see SERVO CALIBRATION FINDINGS above).
  Written into `mission.json`. Open/close pulses and travel_ms still need
  recalibrating around this real stop.
- Module B not yet calibrated — `Servo_Calibration_Tool_B.ino` created and ready
  to run (GPIO 13).

---

## IMMEDIATE NEXT GOAL

**Module B hardware is not ready/assembled yet.** Decision: proceed with
Module A alone for now rather than blocking on B — the calibrator tool
already fully supports B (per-module Stop/Offset/Reverse fields, and
`Servo_Calibration_Tool_B.ino` is written and ready) so no more tool work
is needed before B can be picked up whenever the hardware exists.

Current sequence:
1. **Characterize the door-to-door travel deviation on Module A** (see
   KNOWN ISSUE above) — run the current 580ms setting across all 5 door
   positions in one sequence, marking/measuring each, to see whether
   door 1 is uniquely short or whether the deviation grows/accumulates.
   This determines whether a single global travel_ms is usable, needs a
   first-door exception, or points to a mechanical problem worth
   inspecting before trusting this module with real seeds.
2. Once travel is acceptably consistent (or compensated for), run bench
   Test 4 from `SMART_LAUNCHER_COMPLETE_GUIDE.md` for Module A alone —
   5 consecutive door drops ending in a confirmed limit-switch stop.
3. Record Module A's final stop/offset/open/close/travel_ms into
   `mission.json`. Leave Module B's fields as placeholders.
4. When Module B hardware exists: `Servo_Calibration_Tool_B.ino` for its
   true stop, then the same web calibrator workflow (already supports it)
   for open/close/travel_ms, then repeat the 5-door consistency check.
5. Once both modules are calibrated: full two-module bench Test
   (sequential A-then-B confirm), then move to Phase 3 in the roadmap
   below (Wi-Fi config upload, NMEA bench replay).

---

## DEVELOPMENT ROADMAP

- Phase 1 Electronics/schematic/pins/power  - DONE (spec'd)
- Phase 2 Firmware skeleton                 - DONE (needs WROVER pin port)
- Phase 3 Config + drop logic + Wi-Fi upload + NMEA bench replay - pending
- Phase 4 MAVLink position (Cube Orange TELEM2) - pending
- Phase 5 Python config tool (import QGC/UgCS missions -> mission.json) - pending
- Phase 6 Testing (bench NMEA -> static drops -> low altitude -> full mission) - pending

There is also a full SMART_LAUNCHER_COMPLETE_GUIDE.md (wiring, BOM, calibration, checklists)
already produced earlier in the project.
