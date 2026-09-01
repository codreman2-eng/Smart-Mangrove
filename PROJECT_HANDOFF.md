# Smart Launcher — Project Handoff / Context Resume

Paste this whole document into a new chat to continue seamlessly.

---

## WHAT THE PROJECT IS

A self-contained, platform-agnostic drone payload module ("Smart Launcher") that autonomously
releases mangrove propagules (seeds) at predefined GPS coordinates. For reforestation.

Core principle: the launcher does ALL drop logic onboard. The host drone only provides power
and flies a waypoint mission. Works on DJI M600 or DIY ArduPilot drones by changing config only.

This once read "No DJI OSDK/PSDK". Phase 3d (2026-08-12) narrowed that rule rather than broke
it: the launcher now READS the A3's unprompted OSDK broadcast for position. It never transmits,
never activates, and never requests control authority - the adapter's TX is physically
disconnected, and that was proven on hardware. Consuming host data is not commanding through
the host; `DropLogicEngine` is untouched and still owns every drop decision.

Why the launcher runs on its own MCU instead of on the Cube Orange+ flight controller (fault
isolation, host-platform independence, bench testing without arming an aircraft, field
calibration over Wi-Fi): see **`DESIGN_RATIONALE.md`**. That document is the reference answer
for this question — it is argued from the real defects logged below, so keep it in sync if new
incidents change the picture.

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
- Limit switch A -> GPIO 32 (INPUT_PULLUP, active LOW, COM->GND, NO->GPIO)
- Limit switch B -> GPIO 33
- State LED      -> GPIO 22 (blink cadence per state)
- Drop announce LED -> GPIO 14 (lit while the servo is/would be driven, then
  blips the door number. ACTIVE IN BOTH SIM AND FLIGHT BUILDS — 330R to GND)
- Yellow WAIT LED  -> GPIO 27 (steady = ARMED and waiting — 330R to GND)
- RC pilot trigger -> GPIO 35 (input-only, NEEDS external 10k pull-down)
- GNSS RX/TX     -> GPIO 4 / 5
- DO NOT USE GPIO 2 — it is a strapping pin on WROVER-E and caused boot crashes.
- DO NOT USE GPIO 9/10 — permanently on the WROVER flash+PSRAM SPI bus.
  (The limit switches were originally spec'd there; they were moved to 32/33.)
- Full pin table with rationale lives in `Servo_Web_Calibrator_WROVER/Pins.h`.

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

### Phase 2 firmware ported to WROVER-E (build-verified, NOT yet flashed)

Ported `smart_launcher_phase2/smart_launcher/` from its original ESP32-S3
target to the real WROVER-E hardware this project actually uses:

- `platformio.ini`: board changed to `freenove_esp32_wrover` (matches the
  actual physical board — confirmed by cross-referencing the "Freenove
  ESP32-WROVER" label already in several bench-test sketch headers).
  Removed the two ESP32-S3-only USB-CDC build flags (native USB doesn't
  apply to classic ESP32, which uses a CH340 UART bridge). Removed
  FastLED from lib_deps (see below). Pins in build_flags updated to the
  validated WROVER-E values: servos 12/13, limits 32/33, LED 22.
- `StateMachine.h/.cpp`: removed FastLED/WS2812 entirely — the real
  hardware has a plain 2-pin LED (confirmed with user), not an
  addressable RGB LED. States are now distinguished by blink cadence
  (BOOT=solid, IDLE=slow blink, ARMED=steady 2Hz, DROPPING=rapid,
  FAULT=slow 50% on/off) instead of colour. Same non-blocking
  millis()-based approach as before.
- `main.cpp`: pin header comment updated; added numbered boot checkpoint
  prints through setup() (the exact debugging pattern that caught both
  the GPIO 2 and GPIO 9/10 bugs on this board this session) — worth
  keeping for any future first-flash-after-changes on this hardware.
- `ActuatorModule.h/.cpp`: **attempted** to port to the same
  ledcAttach()/ledcWrite(pin,...) API used in the calibrator, but this
  project builds with PlatformIO's official `espressif32` platform
  (v6.10.0 as resolved on this machine), which bundles an Arduino-ESP32
  core that predates that API (compile error: "ledcAttach was not
  declared"). Reverted to the classic ledcSetup()+ledcAttachPin()+
  ledcWrite(channel,...) API instead — same underlying LEDC hardware and
  PWM math, just an older wrapper. **This means the calibrator (.ino,
  built via Arduino IDE's bundled arduino-cli, core 3.3.8) and the main
  firmware (PlatformIO, older core) are on two different Arduino-ESP32
  core versions on this machine.** Not a functional problem for LEDC
  itself, but worth knowing if other core-version-dependent behavior
  differences show up later. If full parity is ever wanted, the
  community "pioarduino" platform fork tracks newer cores faster than
  PlatformIO's official registry entry.

**Build verified clean** via `pio run` (PlatformIO CLI at
`~/.platformio/penv/Scripts/pio.exe`, not on PATH by default on this
machine): 0 warnings, 0 errors. RAM 6.9% (22524/327680 bytes), Flash
11.3% (355957/3145728 bytes).

**NOT flashed to hardware yet** — deliberately held off because:
1. Flashing would overwrite the calibrator currently on the physical board.
2. `mission.json` module A `travel_ms` was stale (350) — updated to the
   confirmed working value (**580**).
3. `mission.json` module A `servo_open_pwm_us` is STILL STALE (1700) —
   this predates the stop-value fix (1650) and the direction-reversal
   fix. Need to read the exact "OPEN=" value the web calibrator currently
   shows for Module A (with whatever Offset value is actually set — not
   necessarily the default 200) and write that in before this firmware
   should be trusted to drive the real servo. Do NOT guess this number.

**Next step:** get the confirmed Module A OPEN pwm value from the
calibrator UI, update `mission.json`, then flash + test via the serial
bench commands (`arm`, `dropa`, `status`, `log`) — same live-hardware
verification discipline used for the calibrator all session.

### Phase 2 firmware: flashed, two more real bugs caught and fixed

Got Module A's confirmed OPEN value (1450) from the calibrator UI,
updated `mission.json` (`open=1450 stop=1650 travel=580ms`), flashed both
firmware and filesystem to the board (COM6) via `pio run --target upload`
/ `--target uploadfs`. Two previously-latent bugs surfaced immediately
because this was the first time this firmware had ever actually been
flashed and driven over serial:

1. **Partition name mismatch.** `partitions_custom.csv` named the
   filesystem partition `littlefs`, but Arduino-ESP32's
   `LittleFS.begin()` defaults to looking for a partition literally named
   `spiffs`. Result: `LittleFS mount FAILED` on first boot, config never
   loaded. Fixed by renaming the partition table entry to `spiffs`
   (SubType was already `spiffs`, just the Name column was wrong).
   Pre-existing in the original ESP32-S3 version of this firmware too —
   never caught because it was never flashed before this session.

2. **Boot could never leave the BOOT state.** `setup()` called
   `sm.requestDisarm()` to trigger the BOOT->IDLE transition, but
   `requestDisarm()` is guarded to only fire from `ARMED` (it's meant for
   the normal disarm-during-flight path). Called from `BOOT`, it silently
   no-ops. Result: `status` always reported `State=0` (BOOT) forever, and
   `arm` also silently no-op'd (its guard requires `IDLE`). The system
   was permanently stuck. Fixed by adding a dedicated
   `StateMachine::completeBoot()` method (BOOT->IDLE, one-time, called
   once at the end of setup() instead of the mismatched requestDisarm()
   call). Also pre-existing in the original firmware — same story, never
   actually run before.

**Confirmed working after both fixes** (live serial, via `status`/`arm`):
```
status -> State=1 (IDLE), Armed=0
arm    -> [SM] >>> ARMED <<<
status -> State=2 (ARMED), Armed=1
```

**Deliberately stopped before `dropa`** — that's the first command that
actually moves the real servo/rack (retracts door 1, one-way). Board is
currently sitting ARMED, LED blinking 2Hz on GPIO22, waiting for a live
`dropa` test with the user watching.

### `dropa` test result: found a real servo-runaway incident, root-caused, led to Phase 3

Live `dropa` ran cleanly through the firmware ("timed drop OK, 580ms"), but
the servo did NOT stop — kept slowly, continuously rotating even after a
full reboot re-wrote the same stop command (1650us) fresh during
`ActuatorModule::begin()`. **User had to physically cut servo power.**
Traced the application code line by line (config load -> constructor ->
every `_writePWM(_stop_pwm_us)` call site) and confirmed it was sending
exactly the right number, with duty-cycle math numerically identical to
the calibrator's. Re-measured Module A's true stop via the calibrator
afterward: **1670us, not 1650**.

Conclusion: the same requested (50Hz, 16-bit, duty) LEDC config produces
different real servo behavior depending on which Arduino-ESP32 core
generates it — PlatformIO's official `espressif32` platform (v6.10.0)
bundles an older core (pre-3.x, no `ledcAttach(pin,...)` at all, forced
the classic channel API) with a different underlying LEDC driver (IDF v4)
than Arduino IDE's bundled arduino-cli (core 3.3.8, IDF v5's LEDC driver).
Not proven with certainty (two data points), but the evidence trail
strongly favors the Arduino IDE toolchain — every calibration number
measured through it has held up on re-verification; the PlatformIO path's
first live drop test caused a real safety incident on its very first try.

Also added, in response: a `stop` serial command (emergency stop — writes
each module's calibrated stop PWM immediately, doesn't touch door
counters or arm state) since none existed before this incident.

---

## PHASE 3: Mission Dashboard (built on the calibrator's proven toolchain, not PlatformIO)

Following the incident above, decided to build the Phase 3 web dashboard
(originally on the roadmap as "Wi-Fi config upload") by **extending the
calibrator's Arduino-IDE codebase**, not the PlatformIO project — full
reasoning captured in the approved plan (was written to
`C:\Users\carli\.claude\plans\rosy-frolicking-tide.md` during this
session's planning phase). Short version: build on the toolchain with the
actual reliability track record, not the one that just caused a runaway
servo.

### What was built

Restructured `Servo_Web_Calibrator_WROVER.ino` into its own folder
(`Servo_Web_Calibrator_WROVER/`) with sibling files — Arduino requires the
sketch folder name to match the `.ino` filename, and multi-file sketches
need flat siblings (not PlatformIO-style `src/` subdirectories):

- **`Pins.h`** — plain `#define`s replacing PlatformIO's `build_flags`.
  Same validated pins throughout: servos 12/13, limits 32/33, LED 22, GNSS
  4/5. No external ARM button exists (only onboard EN/RST and BOOT/IO0) —
  kept the GPIO0-hold-2s ARM logic as secondary/best-effort per user
  confirmation, dashboard ARM is primary.
- **`ConfigManager.h/.cpp`** — ported from `smart_launcher_phase2/`, plus:
  `reverse`/`offset_us` fields added to `ModuleConfig` (matching the
  calibrator's UI concepts), a `save()` method (ArduinoJson serialize back
  to `/mission.json`, used by `/savecal`), `setModuleA/B()` mutators, and
  `begin()` now takes an optional PROGMEM default JSON to self-seed the
  file on first boot if it doesn't exist (see `MissionDefault.h` — Arduino
  IDE has no bundled equivalent of `pio run --target uploadfs`).
  `computeOpenPwm()`/`computeClosePwm()`/`opensHigh()` free functions added
  mirroring the calibrator's proven client-side JS formula exactly, so
  server-side automatic drops (GPS-triggered, no browser involved) compute
  the identical pulse the dashboard UI would show.
- **`PositionSource.h`** (renamed `IPositionSource.h`) + **`GNSSPositionSource.h/.cpp`** — ported verbatim.
- **`Logger.h/.cpp`** — ported near-verbatim, added `streamToHttp()` (chunked
  send, same pattern as the HTML chunking, avoids loading a long flight
  log into one `String`).
- **`StateMachine.h/.cpp`** — ported verbatim (already plain-LED, already has
  the `completeBoot()` fix from earlier this session). Removed the
  calibrator's own standalone GPIO22 heartbeat-blink code from the `.ino`
  since `StateMachine::_updateLED()` now owns that pin.
- **`ActuatorModule.h/.cpp`** — real rewrite, not a mechanical port:
  (1) PWM writes via `ledcAttach(pin,...)`/`ledcWrite(pin,...)`, matching
  the calibrator's own proven `servoWrite()` — no channel parameter, unlike
  the PlatformIO version forced onto the classic API; (2) **converted from
  blocking to non-blocking** — `startDrop()`+`poll()` instead of a
  `while(millis()...){delay(1);}` busy-loop, reusing the calibrator's
  already-proven timed-run shape. This matters because this sketch's
  `loop()` also runs `server.handleClient()` — a blocking multi-second
  drop would freeze the live dashboard during exactly the moment you'd
  want to watch it, the same "blocking web handlers cause watchdog
  resets" lesson from earlier this session.
- **`DropLogicEngine.h/.cpp`** — ported logic, restructured as a
  `IDLE/WAITING_A/WAITING_B` phase state machine instead of one blocking
  `_executeDrop()` call, driven by `modA.poll()`/`modB.poll()` being
  called every `loop()` iteration *before* `engine->update()` (documented
  in the header — order matters, the engine reacts to `isDropping()`
  transitioning to false).
- **`MissionDefault.h`** — PROGMEM default `mission.json` content (Module A
  seeded with the last measured values — stop=1670, open=1470, travel=580
  — clearly commented that these still need fresh re-verification before
  trusting a real drop; Module B still placeholder/theoretical, never
  calibrated).

### New HTTP routes / UI

Two tabs (Mission Control default, Calibration) per user preference, a
sticky always-visible **EMERGENCY STOP** button outside both tabs (no
confirm — friction only belongs on actions that cause motion). New routes:
`/arm`, `/disarm`, `/dropa`, `/dropb` (both require ARMED + `confirm=1`,
tightening the old serial commands which had no state guard at all),
`/stop` (no confirm, idempotent), `/log` (streamed), `/clearlog`
(confirm), `/savecal` (persists tuned calibration to `mission.json`).
Existing `/cmd` and `/timed` (calibration) now reject with 409 while
armed/dropping — a stale calibration click during a real mission is a
real hazard now that both feature sets share one servo driver. Manual
DROP A/DROP B buttons use a two-tap confirm (tap turns into "CONFIRM
DROP A?", second tap within 5s fires it). ARM requires one confirm tap
too; DISARM and STOP need none.

### Build + flash: clean

Installed `ArduinoJson`+`TinyGPSPlus` via `arduino-cli lib install`
(previously only used in the PlatformIO project). Compiled with
`--warnings all`: **zero warnings, zero errors**. Flash 80%
(1,056,599/1,310,720 bytes — has headroom but worth watching if more
features get added), RAM 14%. Flashed to COM6, forced reset, captured
full boot log: all 12 checkpoints hit cleanly, `LittleFS mounted OK`,
self-seed logic confirmed working (mission.json didn't exist under this
sketch's partition layout, seeded fresh from `MissionDefault.h`, loaded
correctly), GNSS UART started, both actuators initialized, WiFi AP up,
HTTP server started, BOOT->IDLE transitioned correctly.

### NOT yet done — needs the user physically present (real hardware motion)

Per the plan's verification checklist, in order:
1. **Recalibrate Module A's stop value fresh** on the Calibration tab —
   do not trust 1670 (or 1650) carried forward from memory/a different
   toolchain, even though this is now the same toolchain that produced
   1670. Re-verify.
2. Confirm Mission Control tab renders correctly (state, GPS status even
   with no fix, door counters) — this needs a browser connected to the
   `SmartLauncher` AP, which the agent cannot do (joining that network
   would disconnect the agent's own internet access).
3. ARM (confirm) -> verify dashboard reflects ARMED.
4. Manual DROP A (confirm), human watching — specifically verify the new
   non-blocking actuator code stops correctly on both travel_ms timeout
   and (for the last door) the limit switch.
5. Verify `/status` polling keeps responding *during* an active drop —
   the specific defect class the non-blocking rewrite exists to prevent.
6. Verify EMERGENCY STOP instantly halts both a running TIMED RUN and a
   running DROP.
7. Only after 1-6 pass: consider GPS-triggered auto-fire via
   `DropLogicEngine` for anything beyond a bench test.
8. Module B still has never been calibrated at all — needs
   `Servo_Calibration_Tool_B.ino` first, then the dashboard's Calibration
   tab for open/close/travel_ms, same as Module A went through.

### Round 2 (first hands-on dashboard test): silent-failure bug found and fixed

User reported two symptoms: "calibration mode doesn't work when armed"
and "in mission planning it's random when it works despite having it
armed." Root cause (found by reading the JS, not by reproducing — could
not test the actual browser UI, only reasoned from code): **the dashboard
JS never checked HTTP response status.** Every server-side guard added
this session (`/cmd`/`/timed` reject 409 while armed/dropping, `/dropa`/
`/dropb` reject without `confirm=1` or while not armed) was working
exactly as designed — but the JS treated any completed fetch as success
regardless of status code, updating the UI optimistically either way. So
every intentional block looked like random, silent failure with zero
indication of why.

Fixed: added a `toast()` banner (visible on both tabs, unlike the event
log which only lives on the Calibration tab) plus response-status
checking in `doFetch()` (used by arm/disarm/drop/stop/clearlog),
`sendPwm()` (calibration OPEN/STOP/CLOSE/slider), and `timed()`
(calibration TIMED RUN) — all now surface the exact HTTP status + body on
rejection instead of silently proceeding. `sendPwm()`/`timed()` also no
longer update the UI optimistically before confirming the server actually
accepted the command.

Recompiled clean (0 warnings), reflashed, reconfirmed clean boot
(identical 12-checkpoint sequence, config persisted correctly across the
reflash — no partition change this time, so no re-seed).

**Caveat:** this fix is based on code reading, not reproduction — I
cannot open the dashboard in a browser myself (joining the `SmartLauncher`
AP would disconnect this environment's own network access). It's the
best-evidenced explanation for both symptoms, but if random-seeming
behavior persists after this fix, the toast will now show the real HTTP
status/reason on every blocked action, which should make root-causing
whatever's left much faster next session.

---

## PHASE 3b: SIM MODE — validate everything on an LED, not the servos

**Status: RUN ON HARDWARE, and it did its job — it caught real defects.**
See PHASE 3d and 3e below. The GPS/waypoint half of the plan is validated; the
RC half (Phase 3c) still has never seen a real signal.

**Defects (a), (b) and (e) were CONFIRMED on hardware (2026-08-25) and are
now FIXED and re-verified** — see the E-STOP RESULT and THE FIX in Phase 3e.
(c) is still OPEN and still unconfirmed: it needs more waypoints than the
device currently has saved.

The Round 2 caveat above ("this fix is based on code reading, not
reproduction") plus a checklist of seven items all marked "needs the user
physically present" was the blocker. The insight that unblocked it: every
one of those items is timing, state or HTTP behaviour. The servo is only
the output stage. Swap it for an LED and the whole chain validates with no
one-way rack retraction, no risk of repeating the runaway incident, and no
dependency on Module B hardware existing.

`#define SIM_MODE 1` in `Pins.h`:
- The sim branch lives in `ActuatorModule::_writePWM()` **and nowhere
  else**. Drop timing, door counting, limit switches, `DropLogicEngine`,
  `StateMachine` and every HTTP handler run byte-identically to the flight
  build. A separate `SimActuator` class was explicitly rejected — it would
  have meant validating code that never flies.
- The LED is lit for exactly the window the servo would be driven. No
  stretching, no speed multiplier: the test must not distort the timing it
  exists to verify. A 580ms drop is a 580ms flash (measurable to ±4ms on a
  240fps phone slo-mo). Post-drop confirm blips (A short 80ms, B long
  400ms) make the event readable on one shared LED without touching the
  drive window.
- Compile-time, not a runtime toggle: `ledcAttach()` is never called on
  GPIO 12/13, so the servo lines stay high-impedance rather than carrying
  "PWM we believe equals the stop value". **Still unplug the servo UBEC
  for sim runs** — that removes the risk class instead of reasoning about it.
- `SimPositionSource` + `/simpos?lat=..&lon=..` exercise the GPS-radius
  auto-fire path indoors with no sky view. Both compile out entirely in
  flight builds, so no position-injection surface exists by construction.
- Yellow WAIT LED on GPIO 27: steady when ARMED and waiting, dark during
  DROPPING so the announce LED has the floor. The pair reads as a
  semaphore, which makes an unexpected re-fire after an e-stop obvious.

### Four candidate defects found by reading the code

Three are left in DELIBERATELY so the test run can confirm them safely
rather than have them fixed on a hunch. A `#warning` fires on flight
builds until they are resolved.

1. **(a) E-stop may fire Module B mid-sequence.** `handleStop()` clears
   `modA._active` but not `DropLogicEngine::_phase`. From `WAITING_A` the
   engine's next tick reads that as "A completed OK" and starts B. *The
   stop button would start the other module.*
2. **(b) E-stop does not actually disarm.** `requestDisarm()` is guarded to
   `ARMED`, but the state is `DROPPING`, so it no-ops. Observable symptom
   is "e-stop leaves you ARMED", not "stuck in DROPPING".
3. **(c) 10 waypoints vs 5 doors.** `startDrop()`'s `false` return is
   ignored at `DropLogicEngine.cpp:76`, so drops 6–10 complete instantly
   and log as successful with no motion at all.
4. **(d) Manual drops never entered DROPPING — FIXED.** Had to be, or the
   yellow LED would lie. Side effect, consistent with the sequential-confirm
   principle: a second manual drop can no longer start while one is running.

**`SIM_MODE_VALIDATION_PLAN.md` is the bench script** — 12 steps for the LED
dry-run plus B1–B4 and 13–20 for the RC path, each with a pass criterion and
a fill-in results table. Several have deliberately INVERTED criteria, where
"must NOT fire" is the thing being proven.

---

## PHASE 3c: PILOT RC TRIGGER (DJI M600 A3 Pro F-channel)

**Status: written and compile-verified, NEVER RUN ON HARDWARE. No signal
from a real A3 has ever been read.**

Standard RC PWM (1000–2000us at ~50Hz) from an A3 F-channel mapped to an RC
channel in DJI Assistant 2. Lets the pilot command a drop. **Supplements**
the GPS waypoint path, does not replace it.

Architecturally this is consuming data from the host, not commanding through
it — consistent with `DESIGN_RATIONALE.md`. Drop logic stays on the launcher.

- **Read with an `IRAM_ATTR` edge ISR + `micros()`, never `pulseIn()`** —
  that blocks, which is the exact defect class behind lesson #6 below.
- **GPIO 35 because it is input-only.** It cannot be configured as an
  output, so no firmware bug can back-drive the flight controller. Catch:
  GPIO 34–39 have no internal pull resistors, so an **external 10k
  pull-down is REQUIRED** or a disconnected wire floats and reads as noise.
- Safety rules, all in `RcPwmInput::update()`: range gate 800–2200us;
  5-frame debounce (~100ms); 500ms signal-loss timeout; fail-safe sense
  (1000us = SAFE, and signal loss NEVER triggers); edge-triggered so
  holding the switch cannot dump every door; no fire on `NONE -> FIRE`, so
  powering up with the switch already on is safe.
- **It cannot arm.** The dashboard still owns arming, so two independent
  authorities are needed before anything drops.
- `DropLogicEngine::startManualPair()` reuses the proven
  `WAITING_A`/`WAITING_B` sequencing, so B still fires only after A reports
  done (balance preserved). Logs with `drop_id 0` and does NOT advance the
  waypoint index — a pilot drop must not silently consume a planned GPS
  waypoint.

### Calibration — two tools, use them in this order

1. **`RC_Input_Calibration_Tool.ino`** (repo root) — standalone, serial
   only, no WiFi/servo/LittleFS, zero dependency on the dashboard sources.
   Same philosophy as `servo_minimal.ino`: prove an unknown hardware path
   in isolation first. Live width, min/max sweep, frame rate, glitch count,
   and `g` to suggest thresholds from the travel actually observed.
2. **RC card on the dashboard's Calibration tab** — same workflow (Reset
   sweep -> Suggest -> Apply + Save) on the board that will actually fly it.
   Persists to `mission.json`'s `rc_trigger` block via `/rccal`.

### BEFORE CONNECTING ANYTHING — unverified assumptions

- **MEASURE THE A3 F-CHANNEL OUTPUT VOLTAGE.** It should be 3.3V and
  connect directly, but this has NOT been verified against a datasheet or
  hardware. If it reads 5V, fit a 10k/20k divider or a level shifter — the
  ESP32 is not 5V tolerant.
- Common ground between the M600 and the launcher is mandatory.
- The 1300/1700 band defaults in `mission.json` are nominal RC values, NOT
  measured against a real A3. Treat them exactly like Module B's servo
  values: a starting point, not a promise.

---

## PHASE 3d: OSDK POSITION, WAYPOINTS, FLIGHT LOG — AND THE FIRST REAL DROP SEQUENCE

**Status: RUN ON HARDWARE (2026-08-12 / 2026-08-13). Flashed to COM6, verified
outdoors with a real GPS fix, full stage-1 drop sequence observed end to end.**

This is the phase that turned "written and compiling" into "observed working",
and it moved the position source off the M8N onto the aircraft itself. Servos
were electrically disconnected throughout - every result below was produced by
a `SIM_MODE 1` build driving LEDs.

### Position comes from the M600's A3 broadcast (`OsdkPositionSource`)

- **Read-only by construction.** The A3 broadcasts unprompted, proven with the
  adapter's TX physically disconnected. Never transmits, never activates, never
  requests control authority - which also keeps DJI's SDK Return-Home failsafe
  out of reach, since that is tied to holding authority.
- **50Hz fused position vs the M8N's 1-10Hz.** At 5 m/s a 1Hz fix means 5m of
  travel between samples; through a 3m radius you might get one sample or none.
  At 50Hz it is 10cm. Measured in the field: **46.8 frames/sec**.
- **Field offsets are COMPUTED from the passFlag, never hardcoded.** This paid
  off immediately: the aircraft's broadcast had been trimmed since the original
  capture (passFlag 0x067F -> 0x0A79), which slides POSITION from payload offset
  72 to 44. The decoder computed the new offset and read the coordinates
  correctly on the first try. A hardcoded 72 would have produced garbage.
- **A FRAME IS NOT A FIX.** Indoors the A3 broadcasts happily with lat/lon at
  exactly 0.0 and health 0. Accepting that hands drop logic a "valid" position
  off the coast of Africa. The gate requires health >= 3 AND non-zero
  coordinates; 0/602 frames from the indoor capture pass it.
- **CRC constants had to be recovered from a live capture.** The first run
  against the real aircraft scored 0/601 on header CRC. The CRC16 seed was
  brute-forced over the 16-bit space per candidate polynomial; the CRC32 seed
  was solved algebraically (CRC is linear over GF(2), so init is recoverable
  from one known frame instead of searched through 2^32).

      CRC16: poly 0xA001 (reflected IBM), init 0x3AA3
      CRC32: poly 0xEDB88320,             init 0x00003AA3

  The earlier guess had the right CRC16 seed but the wrong polynomial. **This is
  exactly the case the sync design was built for:** had CRC gated frame
  acceptance rather than merely reporting on it, a perfectly healthy link would
  have been reported as dead and the afternoon spent re-checking wiring.
- `FallbackPositionSource` prefers OSDK and degrades to the M8N after 500ms of
  staleness, logging the switchover. **A stale position is more dangerous than
  an absent one, because it still looks valid.**
- `loop()` now calls `posSource->update()` EVERY iteration. `DropLogicEngine`
  only called it on its armed-and-idle path - nowhere near often enough for
  231-byte frames every 20ms into a 128-byte FIFO. `setRxBufferSize(2048)`
  before `begin()` buys ~180ms of slack.
- **PIN_OSDK_RX 25 / TX 26**, because `Serial2` DEFAULTS to GPIO 16/17, which
  are PSRAM on the WROVER. Same trap as GPIO 9/10.
- `tools/osdk_sniff.py` is the PC-side decoder and the `--diff` field locator:
  capture with a switch in one position, capture again with it moved, and the
  bytes that are stable WITHIN each capture but different BETWEEN them are that
  field. No SDK headers needed.

### `SimPositionSource::begin()` never started the fallback - the "no fix" bug

Reported as "no fix" outdoors, and it was not wiring. `update()`, `read()` and
`hasFix()` all delegated to the fallback source; `begin()` did not. **UART2 was
never initialised, so the OSDK link had never received a single byte, and the
M8N was never started either - in every SIM build since the fallback was added.**
The boot log still looked healthy. The only symptom was the ABSENCE of the
`[OSDK]` and `[GNSS]` init lines at checkpoint [4], which is easy to miss when
nothing is obviously wrong.

Fixed, plus a 5-second OSDK health line on serial: frames, CRC errors, resyncs,
passFlag, raw lat/lon, health byte, whether the fix gate passes, and an explicit
"no frames at all, check GPIO25 wiring" hint at zero frames. Field diagnosis has
to separate "nothing on the wire" from "frames but no fix" from "fix but stale",
and those three are indistinguishable from the LEDs or from a browser that may
be out of range.

### Waypoints tab - a mission could not be planned at all

Waypoints lived in `mission.json`, seeded from `MissionDefault.h` ONLY when the
file was absent. Editing the header and reflashing changed nothing on a board
that already had one, and there was no upload route or editor. The shipped
waypoints are placeholders in Panama, which makes a walk test impossible.

Built around one control: **ADD WAYPOINT HERE** captures the current OSDK/GNSS
position on a single tap. No coordinates typed, no laptop, no import format -
the only workflow that works in a field. It refuses when there is no fix rather
than storing 0,0, which would be a waypoint in the Atlantic that never triggers
and would present as a drop-logic bug rather than a missing fix.

Two correctness details worth keeping:
- Every mutation calls `engine->resetProgress()`. `_next_drop_idx` is a position
  into `drops[]`, so editing the list underneath it would leave the engine aimed
  at a different waypoint than the operator believes, or past the end.
- The tab surfaces candidate defect (c) directly: when waypoints exceed doors it
  warns that drops past the door count cannot fire and would log as successful
  with no motion. **The shipped default is exactly that case - 10 waypoints
  against 5 doors.**

Mutations are blocked while armed, same guard as the calibration controls.

### JSONL flight logger, and the three ways it stalled `loop()`

`/logs/flight.jsonl`, one JSON object per line: 1Hz telemetry rows (state,
armed, lat/lon/sats/hdop, RC width/band/Hz/glitches, doors A/B, drop index, both
limit switches) plus event rows on change (boot, state transitions,
`rc_acquired`, `rc_lost`, `rc_band`, `rc_fire` INCLUDING refusals with the
reason, `drop`, `estop`). JSON Lines rather than one JSON array, deliberately:
appendable with no closing bracket, so the file is valid at every instant. Lose
a battery mid-write and you lose one line, not the file - and that is precisely
the flight whose log you most need. Reads with `pd.read_json(path, lines=True)`.

Then it was reported as "a lot of lag to deploy" after a successful walk test.
The cause was the logger, three ways:

1. **The log fetch.** The events-only filter read the ENTIRE file one byte at a
   time, every 5 seconds, inside an HTTP handler. The file had reached 752KB -
   not the 80KB assumed - because 1Hz telemetry ran through hours of idle bench
   time. That is seconds of stall, growing every session, and while `loop()` is
   blocked, position is not read and the drop radius is not evaluated. It landed
   directly on drop timing. Now: seek to the last ~25KB, block reads, output
   accumulated into 1KB chunks. `/log?tail=0` still fetches everything, on the
   ground.
2. **The telemetry write.** Every row did open/append/close. LittleFS walks
   metadata on each open, which costs 80-200ms once the file is large. The
   handle is now held open and committed with `flush()`, which calls
   `lfs_file_sync()` - so each row still reaches flash before the call returns.
   **The crash-safety argument for closing was satisfied by `flush()` all along;
   closing bought nothing and cost the latency.**
3. **The idle rate.** Telemetry now logs at 1Hz only while ARMED or DROPPING,
   and 1/10Hz otherwise. Events stay at full fidelity in any state.

A fourth, found the next day: `DropLogicEngine` printed `[DLE] WP1 dist=...` on
EVERY loop iteration while armed. ~30 characters at 115200 baud is ~2.6ms of
blocking, thousands of times per second, and `Serial.printf()` stalls once the
TX buffer fills. **The distance readout was itself the largest source of drop
latency - and it only ran while ARMED, which is exactly when the lag was
reported.** Now rate-limited to 1Hz, or whenever the distance moves more than a
metre so an approach stays visible in real time.

Instrumentation added: `loop()` tracks its worst iteration per 5s window,
reported on the dashboard and printed to serial above 50ms. **That figure IS the
added drop latency, so it can be measured instead of guessed at.**

The dashboard also stops auto-fetching the log while armed - the most expensive
request the board serves should not run during the moment that matters - and the
log viewer got a server-side events-only filter (`/log?ev=1`) plus auto-scroll,
because telemetry writes one row per second and a drop writes two, so a few
minutes of running buried the drop lines under hundreds of `tlm` rows in a 180px
box that rendered from the top and never scrolled.

### Serial commands restored

The Phase 2 firmware had them; the dashboard replaced them with HTTP and lost
them. That was a mistake - the browser needs the `SmartLauncher` AP, which means
no internet and a second device, and it is useless on a bench tethered to a
laptop.

    status | clearlog | arm | disarm | stop | wp | simpos <lat> <lon> | simclear

**Deliberately no servo-moving commands:** motion stays behind the dashboard's
two-tap confirm.

### THE RESULT - first full drop sequence

Using those commands to clear the full log and re-run the stage-1 test end to
end: **waypoint detected at 0.0m, Module A fired 580ms, Module B 350ms, both
drops logged, mission complete, auto-disarm. doorsA=1 doorsB=1.** Servos
disconnected; the announce LED is what fired.

### Open items this phase left behind

- **Worst-loop still spikes 60-200ms clustered around the drop**, when
  `[ACT]`/`[DROP]`/`[EVT]` all print together. Evidence points at serial output
  volume generally. Recorded as OPEN, not claimed as fixed.
- **The saved waypoint radius was 1.0m.** GPS wanders 0.5-1m while stationary,
  so that radius is narrower than the position noise, and it likely explains
  much of the perceived lag. **10m is the right figure for testing.** Not a
  software fault.
- **The browser UI is still unverified from this environment** - joining the
  `SmartLauncher` AP cuts this machine's own network access. Everything above
  was confirmed over serial.
- Defects (a), (b) and (c) remain unconfirmed. The run exercised the GPS
  auto-fire path, not the e-stop path.

### Documentation for a handover

The project was handed to another student for a week, which exposed that every
existing document assumes prior context: this one is written for someone already
up to speed, and `SIM_MODE_VALIDATION_PLAN.md` assumes you know what SIM mode is
and why it exists.

`START_HERE.md` assumes none of it: safety first, then the hardware, the
never-use pins, Arduino IDE setup, finding a COM port that moves between
sessions, flashing, reading boot output, the dashboard, and a troubleshooting
table built from the failures actually hit here. It states plainly what is proven
and what is not, and it asks the reader NOT to fix the three known bugs before
the test run confirms them - **without that note, the obvious thing for a capable
student to do is fix them, which would destroy the experiment they are there to
run.**

`docs/GUIA_INICIO.pdf` (+ `.html`) is the same content in Spanish, 15 pages A4,
generated with Edge headless.

---

## PHASE 3e: THE DROP-LATENCY SPIKES — DIAGNOSED, NOT MIGRATED

**Status: RUN ON HARDWARE (2026-08-25, COM10). The drive window measures
+0ms overshoot. The residual stall was traced to `lfs_file_sync()`, not
serial and not the log size.**

The 60-200ms worst-loop spikes left open by Phase 3d prompted the question of
whether the ESP32 was simply too slow and the project should move to a
Raspberry Pi Zero 2 W. **It is not, and it should not.** Recording the
reasoning here because it will be asked again.

### The evidence says I/O blocking, not compute

The decisive datum is already in the Phase 3d field capture: `frames=368
crcErr=2 resync=1`, with the CRC and resync counts **flat between reports**.
A board that could not keep up with its input would show those climbing
continuously — that is what buffer overrun looks like. The ESP32 is decoding
100% of the A3 stream in steady state.

The actual per-frame compute at 240MHz, against a 50Hz frame rate:

| Work | Cost | % of one core |
|---|---|---|
| CRC16 + CRC32 over 231 bytes | ~5-50us | 0.03-0.25% |
| Haversine (float, hardware FPU) | ~5us | 0.03% |
| Servo PWM (LEDC) | 0 — hardware peripheral | 0% |
| RC pulse (edge ISR + micros()) | ~us | ~0% |

Under 1% of one core — and `grep` confirms there is no `xTaskCreatePinnedToCore`
anywhere in the firmware, so the second core is entirely idle.

Meanwhile at 115200 8N1 a byte costs **86.8us of transmit time**, and
`Serial.printf()` blocks once the TX FIFO fills. A drop fires `[ACT]`,
`[DROP]`, `[EVT]` and a telemetry row together: 690 bytes is 60ms, 2300 bytes
is 200ms. **That arithmetic reproduces the observed spike range exactly, and
no compute explanation comes within three orders of magnitude of it.** The
`Logger` also wrote every event to serial AND flash, so each one cost double.

This is the fourth instance of the same disease on this project (the `[DLE]
dist=` print, the 752KB log read, the open/append/close, now this). **It is
not slow, it is blocked.** Different disease, different cure — and swapping
the CPU cures neither.

### Why not the Pi Zero 2 W

It is ~50x the compute, which is irrelevant when compute is under 1%. What it
would actually cost:
- **Tail latency gets worse.** Linux is not an RTOS; the metric here is
  worst-case, not average. SD `fsync` and wear-levelling routinely block
  100ms-1s. That trades a bounded, understood stall for an unbounded one.
- **Boot 300ms -> 20-30s.** A brown-out mid-mission takes the launcher out
  for most of a flight.
- **SD corruption on unclean power-down** is the classic Pi-on-a-drone
  failure, and it fights the JSONL logger directly.
- **Two hardware-enforced safety properties are lost.** GPIO 35 is safe
  because it is *physically* input-only; SIM mode is safe because
  `ledcAttach()` is never called. On Linux every GPIO is bidirectional under
  root and both guarantees become runtime ones. After a servo-runaway
  incident, that is the wrong direction.
- **150mA -> 350-600mA**, against an airframe where lift is the binding
  constraint.
- The three planted defects are **state-machine logic errors, not ESP32
  errors**. They would port faithfully into Python, minus the experiment that
  was going to catch them.

The one genuine ESP32 pressure is flash at 83% — and that is a partition
setting, not a migration: the module has 8MB and the build uses the 1.31MB
"Default 4MB with spiffs" app partition.

**If a camera / CV / ROS requirement ever appears, the answer changes** — but
toward a Pi as a *companion* alongside the ESP32, never as a replacement for
the thing that drives the servos.

### What was actually changed

1. **`Serial.setTxBufferSize(4096)` before `Serial.begin()`.** The core's
   default TX buffer is 256 bytes, so a drop burst overruns it immediately.
   At 4KB the burst is memcpy'd and `printf()` returns; the UART drains it in
   the background while `loop()` keeps running.
   **The baud rate was deliberately NOT raised.** 921600 would also work —
   the OSDK link already runs at it — but the ROM bootloader prints at 115200
   regardless, so `rst:0x8` boot-loop diagnostics would arrive as garbage, and
   `START_HERE.md` sends a newcomer looking for exactly that string.
   Debuggability for the person about to run the validation is worth more than
   wire speed that buffering already bought.
2. **`Logger::setDropWindow(bool)`**, driven from `loop()` off
   `modA.isDropping() || modB.isDropping()`. For exactly the span a servo is
   driven, the logger stops echoing events to serial (the row is already going
   to flash — the echo was a duplicate) and stops `flush()`ing each row.
   **Nothing is dropped or deferred**; the sync happens on the falling edge.
   The only thing relaxed is crash-safety, and only for the sub-second span of
   a drop. Correct trade: a stall inside the drive window overshoots the servo
   mechanically, whereas losing the last log row to a brown-out landing in
   that same sub-second is cosmetic.
3. **`ActuatorModule` now measures the drive window directly.**
   `lastDriveMs()` / `lastOvershootMs()`, captured immediately after the stop
   write, printed on the `[ACT]` completion line, and flagged
   `*** OVER 10% - LOOP STALL ***` past the plan's own tolerance. Worst-loop
   was always a *proxy* for drop latency; this is the quantity itself, in the
   same units as the pass criterion.
4. **The `[PERF]` warning threshold dropped from 50ms to 10ms.** Against a
   580ms window, 50ms is already 8.6% — nearly the whole budget.

### BENCH RESULT (2026-08-25, COM10, SIM build, servos disconnected)

**The drive window is now exact. Three consecutive runs, both modules:**

    [ACT] Door 1 — timed drop OK (580 ms target, 580 ms actual, +0 ms)
    [ACT] Door 1 — timed drop OK (350 ms target, 350 ms actual, +0 ms)

+0ms against a +/-58ms budget. The plan's step 4 criterion passes with the
whole tolerance to spare, and it is now read off the line instead of measured
in slo-mo.

**`[PERF] TX buffer = 4096 bytes`** — the resize is confirmed live. Worth
printing: `setTxBufferSize()` returns 0 and does nothing if the UART is
already running, and the core's `log_e` about it is invisible at
`CORE_DEBUG_LEVEL 0`. Assuming it took effect would have been a guess.

### What the residual stall turned out to be — NOT what was predicted

worst-loop did NOT go to zero. It still reports 38-185ms. Two hypotheses were
tested and one survived:

- **"It is the log size"** — REFUTED. Cleared the log to ~1KB and re-ran:
  148-153ms, unchanged. `lfs_file_sync()` cost is metadata and block commits,
  not file length.
- **"It is serial"** — REFUTED for the residual, though it was correct for
  the drive window. TX buffering is live and the drive window is clean.

So the per-section timer was added, and it named the culprit immediately:

    [PERF] worst loop 38 ms ... (worst section: telemetry 37 ms)
    [PERF] worst loop 84 ms ... (worst section: events 71 ms)
    [PERF] worst loop 185 ms ... (worst section: telemetry 65 ms)

**It is `flush()` — `lfs_file_sync()` — costing 37-71ms per row written.**
That is why deferring the flush inside the drive window works so completely:
the stall is real and it is large, and the fix moves it to where nothing
mechanical depends on it.

### What this means for the deferred work

The gate set before the run resolves cleanly:

- **The `esp_timer` drive-window close is NOT needed.** It was aimed at
  overshoot, and overshoot is +0ms. Do not add concurrency to servo code to
  fix a problem that measures zero.
- **The core-0 split IS the right target if the residual matters.** It is now
  aimed rather than guessed: move the *logger* to the idle core so
  `lfs_file_sync()` stalls a task that nothing is waiting on, instead of
  `loop()`. Cheaper alternative first: stop flushing every row (flush every
  N rows or every N seconds), which trades crash-safety granularity for the
  stall directly.

Severity has changed, though, and it is worth being clear about it. The
residual is no longer *servo overshoot* (mechanical, permanent, accumulates
across doors). It is a *position-sampling blind spot*: at 1 Hz telemetry
while armed, ~40-70ms per second where position is not read — about 0.2-0.35m
of unevaluated travel at 5 m/s. Against a sane 10m radius that is noise.
Against the 1.0m radius currently saved on the device it is not, which is one
more reason that radius has to go up.

### Steps actually validated in this run

Over the serial command interface, no browser, no eyes on the hardware:

| Step | | Result |
|---|---|---|
| 1 | Boot, 12 checkpoints, no reset | **PASS** — all 12, `rst:0x1 POWERON`, SIM confirms GPIO 12/13 not attached |
| 3 | ARM | **PASS** (via the `arm` serial command, not the button or dashboard) |
| 4 | Drop A door 1, 580ms +/-10% | **PASS** — +0ms, three times |
| 11 | Auto-fire A->B full sequence | **PASS** — A 580ms, B 350ms, both logged, mission complete, auto-disarm |

**NOT validated, and not claimable from this run:** every LED criterion (the
blips, the yellow WAIT semaphore) — nobody was looking at the board; steps 2
and 8 (browser); 6, 7a, 7b (multi-door and the limit switch); and **9, 10, 12,
the three that confirm defects (a), (b) and (c)**. Those still need a human at
the bench. Note also that the device's saved mission has **one** waypoint, so
step 12 needs waypoints added before it can run at all.

### E-STOP RESULT (2026-08-25) — (a) and (b) CONFIRMED, plus a third

Run over serial with the stop scheduled 193-218ms into module A's 580ms
window — a human typing into a serial monitor cannot reliably hit that, which
is why it was scripted. Timestamps are relative to the `simpos` that triggered
the drop.

    [ +0.040s] [ACT] Dropping door 1/5 ...          <- module A starts
    [ +0.258s] >>> stop
    [ +0.304s] [CMD] *** EMERGENCY STOP ***
    [ +0.304s] [DROP] id=1 mod=A door=0 ... OK      <- aborted drop logged OK (!)
    [ +0.304s] [DLE] Module B — drop 1
    [ +0.304s] [ACT] Dropping door 1/5 ...          <- *** B STARTED BY THE STOP ***
    [ +0.372s] state=3 armed=0 dropping=1           <- still DROPPING after e-stop
    [ +0.651s] [ACT] Door 1 — timed drop OK (350 ms target, 350 ms actual, +0 ms)
    [ +0.708s] [EVT] {"ev":"state","from":3,"to":2} <- lands in ARMED
    [ +0.816s] [SM] Disarmed → IDLE

**(a) CONFIRMED, and worse than described.** Module B did not merely start —
it started *in the same loop tick as the stop* and ran a **complete 350ms
drive window to completion**, `doorsB=1`. On a flight build that is a real
door opening, immediately after the operator hit emergency stop. Root cause is
as predicted: `handleStop()` clears `modA._active` but not
`DropLogicEngine::_phase`, so the `WAITING_A` branch reads `!isDropping()` as
"A finished" on its very next tick.

**(b) CONFIRMED at the mechanism, symptom partly masked.** The probe 68ms
after the stop reads `state=3` (DROPPING) — `requestDisarm()` did nothing,
exactly as its `if (_state == ARMED)` guard predicts. The system then
transitioned **3 -> 2, i.e. INTO ARMED**, after an emergency stop. It reached
IDLE only because this mission has a single waypoint and completed.
**With two or more waypoints it would have been left sitting ARMED, and would
have fired again at the next waypoint.** The single-waypoint mission is
hiding the full severity here; do not read the final `armed=0` as reassuring.

**NEW — defect (e): an aborted drop is logged as a SUCCESS.**
`[DROP] id=1 mod=A door=0 ... OK` was written for a drop that was killed
193ms into a 580ms window. `ActuatorModule::stop()` never touches
`_lastResult`, so it keeps its initialiser `DropResult::OK`, and
`_recordDrop()` writes that verbatim. Meanwhile `doorsA=0`, because the door
counter is only advanced by `poll()` on real completion — so the flight log
and the door counter now disagree, and the log is the one that is wrong.
**A flight log that reports a successful drop that never happened is worse
than no log**, and this was not on anyone's list before the run.

### These must be fixed together, not one at a time

Fixing (a) alone strands `_phase` in `WAITING_A` with nothing left to advance
it — the state machine would sit in DROPPING forever, which is a worse
failure than the one being fixed. `handleStop()` needs to reset the engine
phase AND force the state out of DROPPING, and `stop()` needs to record an
ABORTED result, in one change.

### THE FIX (2026-08-25) — (a), (b), (e) closed and re-verified

Fixed as ONE change, because fixing (a) alone would have stranded `_phase` in
`WAITING_A` with nothing to advance it — DROPPING forever, worse than the bug.

- **`DropLogicEngine::abort()`** — tears down the in-flight sequence and resets
  `_phase` to IDLE. Deliberately does **not** advance `_next_drop_idx`: an
  aborted waypoint was not serviced and must still be pending on re-arm.
  Records the interrupted door truthfully before resetting.
- **`StateMachine::emergencyStop()`** — IDLE from any state except FAULT.
  `requestDisarm()`'s `ARMED` guard was **not** loosened: that guard is right
  for the normal disarm path (you should not be able to casually disarm
  mid-drop). An emergency stop gets its own authority instead of weakening a
  correct rule. FAULT is excluded so an e-stop can never launder a faulted
  system back into an armable state.
- **`DropResult::ABORTED`**, set by `ActuatorModule::stop()`, which also
  records how far into the window it got — the cover is now parked partway
  between doors and that distance is the one number that says how far.
  `_recordDrop()`'s OK/TIMEOUT ternary became a proper mapping, so an abort is
  no longer relabelled as a TIMEOUT (a different failure with a different
  cause, and the log is all anyone gets after a flight).

### The fix did not work the first time — and that is the interesting part

First flash of the fix, re-running the identical script: **module B fired
anyway.** The fix was correct; it was in the wrong place.

`handleStop()` is the HTTP route. The serial `stop` command was a **second,
independent copy** of the same body at line 1100 — `modA.stop(); modB.stop();
sm.requestDisarm(); ...` — which never called `handleStop()`. The fix landed on
one copy and silently missed the other. The regression test drives the serial
path, so it caught it immediately.

Both entry points now call one `emergencyStopAll()`. **Two implementations of
an emergency stop is one too many.** Note the near-miss: had the test been run
through the browser instead, it would have PASSED, and the serial path — the
one used on the bench, tethered to a laptop, with no dashboard — would have
stayed broken.

### Re-verified on hardware, same script, same timing

    [ +0.272s] >>> stop
    [ +0.303s] [DROP] id=1 mod=A door=1 ... ABORTED
    [ +0.367s] [DLE] *** ABORTED during module A — sequence torn down,
                     waypoint 1 still pending ***
    [ +0.367s] [ACT] Module A ABORTED after 268 ms of a 580 ms window
    [ +0.367s] [SM] *** EMERGENCY STOP -> IDLE (disarmed) ***
    [ +0.445s] [EVT] {"ev":"state","from":3,"to":1}
    [ +0.445s] state=1 armed=0 dropping=0
    [ +0.445s] doorsA=0 doorsB=0 dropIdx=0 of 1

- **(a)** B never starts. No second drive window. `doorsB=0`.
- **(b)** DROPPING -> IDLE directly, disarmed.
- **(e)** the row says `ABORTED`, and `dropIdx=0` — the waypoint was not consumed.

**Happy path re-checked for regression** (same waypoint, no stop): A 580ms
+0ms, B 350ms +0ms, both `OK`, `doorsA=1 doorsB=1`, `dropIdx` advances,
auto-disarm. Unchanged.

**(c) remains OPEN and unconfirmed** — it needs more waypoints than the device
has saved. The flight-build `#warning` now names only (c).

### Defect (c) — FIXED; symptoms proven unreachable 2026-09-01 (see PHASE 3g)

**The "never reproduced" warning below is now RESOLVED — but not the way it
expected.** The run confirmed none of (c)'s symptoms can occur, while showing
the inner guard is unreachable from the GPS path by construction. Read PHASE 3g
before relying on either half. The original reasoning is kept below as written.

`_modA.startDrop()`'s false return was ignored at the GPS trigger, and
`_phase` was set to `WAITING_A` regardless. With no drop actually started, the
`WAITING_A` branch fired on the next tick, recorded a drop using whatever
`_lastResult` happened to hold, started module B, and logged a full SUCCESS
with zero motion. The shipped default mission is exactly that case: 10
waypoints, 5 doors.

**`startManualPair()` has always guarded this correctly** — it checks
`_modA.isEmpty()` and refuses. The GPS path simply never got the same check.
Two trigger paths that disagreed: the same shape of bug as the two e-stop
paths found an hour earlier, and worth noticing as a pattern in this codebase.

The fix:
- the GPS trigger now honours `startDrop()`'s return, logs a real `EMPTY` row
  and leaves `_phase` at IDLE;
- module B's start in the `WAITING_A` branch is checked the same way — B can
  run out independently of A, so the trigger-path check does not cover it;
- `missionComplete()` now also returns true when `launcherEmpty()`. With more
  waypoints than doors the run must END when the doors run out, rather than
  re-triggering at the same waypoint every loop and spamming failure rows;
- `DropResult::ALREADY_DONE` renders as `EMPTY` in the log instead of being
  relabelled `TIMEOUT`.

**It has NEVER been reproduced.** The device has ONE waypoint saved, and the
defect needs more waypoints than doors. Everything above is reasoning from the
code, which is exactly the standard of evidence that (a), (b) and (e) had
before the bench run — and that run showed (a) was *worse* than the reading
suggested, and turned up (e), which no reading had found at all.

**So do not treat (c) as closed.** Run plan step 12. To make that possible,
either add waypoints from a phone on the `SmartLauncher` AP, or reseed
`mission.json` by erasing LittleFS — in which case put the measured values
(A open=1470 stop=1670 travel=580, B open=1300 stop=1500 travel=350, RC
SAFE<=1300 FIRE>=1700) into `MissionDefault.h` FIRST, or they are lost.

### Deliberately NOT done, pending measurement

Moving position ingest and logging to the idle core
(`xTaskCreatePinnedToCore`) and closing the drive window from an `esp_timer`
callback — which would preempt a blocked `loop()` and make drop timing
structurally immune to stall — are both real improvements. **Both were held
back on purpose.** They add concurrency to servo-critical code that has never
driven a servo, and the changes above should be enough on their own. Doing
them now would also mean never learning which fix was the one that worked.

**Decision gate: flash, drop five doors, read the `[ACT]` overshoot figures.**
If overshoot is inside +/-10% of travel_ms, stop here. If it is not, do the
esp_timer change first — it targets the drive window directly — and the core
split only if that still is not enough.

---

## PHASE 3g (2026-09-01): DEFECT (c) CLOSED, AND A LATENT SERVO HAZARD

**Status: RUN ON HARDWARE (COM10, SIM build). Defect (c)'s symptoms are proven
unreachable. Plan steps 6, 7a and 12 now have observed results. Housekeeping
done. One item turned out to be a false alarm.**

### Getting waypoints onto the device \u2014 why serial, not HTTP

Step 12 needs more waypoints than doors, and the device had ONE. Neither the
radius nor the waypoint list can be written over serial: `wp` is read-only. The
`/mission` HTTP API does all of it, but reaching it means joining the
`SmartLauncher` AP \u2014 and this laptop has a single Wi-Fi radio, so it must
abandon its own network, and its internet, to get there.

Two things went wrong, and the second one matters:

1. **Windows roams back.** `FABLAB5Ghz` is an auto-connect profile and kept
   reclaiming the radio. Fixable with `netsh wlan set profileparameter
   name=FABLAB5Ghz connectionmode=manual`; restored afterwards.
2. **The HTTP API is not reliable enough to script against.** With roaming
   disabled and the AP associated at 99% signal, `curl` returned
   `connection refused` more often than `200`, and ping lost 20-30% of packets.
   `server.handleClient()` runs inside `loop()`, and `loop()` is stalling
   **100-182ms** \u2014 the Phase 3e residual stall, now attributed by section to
   `telemetry` and `events`. A web server starved that long drops TCP
   handshakes.

**That is a second, previously unrecorded cost of the Phase 3e residual
stall.** It was understood as a drop-timing risk; it is also making the
dashboard unusable, which is part of why the browser UI has never been
exercised. It strengthens the case for the deferred `xTaskCreatePinnedToCore`
work: that stall now hurts two things, not one.

**A correction worth recording:** opening the serial port does **not** reset the
board. That was assumed during this session and it is wrong \u2014 `resync` counters
climb continuously across separate script invocations, which cannot happen
across a reset. Only the explicit DTR/RTS sequence in `bench_serial.py`'s
`reset()` resets it. Serial and the HTTP dashboard can be used simultaneously.

### The fix: SIM_MODE-gated waypoint commands over serial

`wpadd <lat> <lon> [r] [alt]` | `wpradius <idx|all> <r>` | `wpclear` | `wpsave`

Guarded by `calibrationBlocked()` \u2014 the same armed/dropping rule the HTTP route
enforces, because the engine indexes into the drop list while armed. Gated
`#if SIM_MODE` so a flight build cannot carry a waypoint editor that bypasses
the dashboard's two-tap confirm. `wpradius` takes a **1-based** index to match
what `wp` prints; index 0 underflows to 255 and is rejected by
`setDropRadius`'s bounds check rather than wrapping.

### Housekeeping \u2014 one item was a false alarm

- **The flight log was NOT full.** 460KB against a 1.1MB cap, no `FULL` flag.
  The standing "log is full, nothing is being recorded" note was stale. It was
  deliberately left intact: it holds the 2026-08-25/26 e-stop verification runs,
  and clearing it to fix a problem that did not exist would have destroyed
  evidence.
- **Radius 1.0m -> 5.0m** on all 10 waypoints, saved.
  **5m and not 10m, on purpose.** The trigger fires on circle ENTRY
  (`DropLogicEngine.cpp:101`), not at closest approach, so it fires
  `sqrt(r^2 - d^2)` metres early where `d` is cross-track error. Bigger radius
  = earlier drop. The floor is set by needing the chord `2*sqrt(r^2 - d^2)` to
  exceed one sample step: negligible on the 50Hz OSDK link, but 5m per sample
  on the 1Hz GNSS fallback at 5 m/s. 5m is about the smallest value that
  survives that fallback; 10m would have fired 10m early for no benefit.
  Revisit once the impact predictor lands \u2014 see PHASE 4 RESEARCH, which is
  exactly the change that turns `radius` from a fudge factor into a real
  accuracy tolerance.

### RESULT \u2014 defect (c): symptoms unreachable, inner guard never executed

10 waypoints against 5 doors, driven by `tools/bench_defect_c.py`:

    WP1..WP4   A 580ms +0ms, B 350ms +0ms, doorsA/B -> 1,2,3,4
    WP5        [ACT] Door 5 - limit switch: TIMEOUT (drove 1740 ms)   module A
               [ACT] Door 5 - limit switch: TIMEOUT (drove 1050 ms)   module B
               [MAIN] *** MISSION COMPLETE - all drops done ***
               doorsA=5 doorsB=5 dropIdx=5 of 10
    WP6        simpos injected - NO [DLE] output at all.
               No evaluation, no drop, no module B start.

**None of (c)'s symptoms occur.** No zero-motion SUCCESS row, no spurious
module B start, and the five unserviced waypoints are correctly left pending.

**But the inner guard was never executed, and cannot be.** `missionComplete()`
is checked at `DropLogicEngine.cpp:73`, before any waypoint evaluation, and
`launcherEmpty()` is `_modA.isEmpty() || _modB.isEmpty()` \u2014 so the instant
EITHER module empties, the run ends. The GPS-trigger fix at
`DropLogicEngine.cpp:123` (honouring `startDrop()`'s false return) is therefore
unreachable from the GPS path **by construction**. No door-count configuration
reaches it, because emptying either module ends the mission first.

So (c) closes differently from what was expected: not "the guard fired
correctly" but "the outer guard makes the defect unreachable, and the inner one
is defence-in-depth that this path cannot exercise." Record it that way. Note
also that no `EMPTY` row is ever written \u2014 the earlier prediction that one would
appear was wrong for the same reason.

### LATENT HAZARD found by the same run \u2014 read before SIM_MODE goes to 0

Door 5 of **both** modules ended in `limit switch: TIMEOUT`: module A drove
**1740ms against a 580ms target**, module B **1050ms against 350ms**. Exactly 3x
in both cases. The last door waits for limit-switch confirmation, and the
switches on GPIO 32/33 are not wired on the bench, so it falls through to a
3 x travel_ms timeout.

Harmless in SIM. **Not harmless with servos connected** \u2014 the final door of
every module would drive three times its intended travel, overshooting the cover
by roughly three door pitches. That is the same class as the servo-runaway
incident.

**Wire the limit switches before flipping SIM_MODE to 0.** Neither this document
nor `SIM_MODE_VALIDATION_PLAN.md` listed that as a prerequisite for servo
bring-up. It is one. (The plan's step 7a did predict the 1740ms figure, so the
behaviour was expected \u2014 but its consequence for real servos was never drawn
out.)

---

## PHASE 4 RESEARCH (2026-08-25/26): HOW DO WE KNOW A WAYPOINT IS REACHED?

**Status: RESEARCH ONLY, no code written. The conclusion is that the current
design is right and should NOT be replaced — but that the real accuracy
problem is somewhere else entirely.**

The question was whether the launcher should stop deriving "waypoint reached"
from position, and instead read it from the host aircraft.

### DJI M600 / A3 — not available, and not fixable

DJI's own `DataBroadcast` reference lists every broadcast field: timestamp,
quaternion, accel, angular rate, velocity, global/relative position, GPS info,
RTK, magnetometer, RC, gimbal, flight status, battery, SDK control mode.
**There is no waypoint mission progress, no mission state and no "waypoint
reached" in the broadcast.** That list maps exactly onto the passFlag bits
`osdk_sniff.py` already decodes, so we are already reading everything it
carries.

Waypoint progress lives in OSDK's `WaypointMission` push-data callbacks, which
require activation and **control authority** — which is precisely what this
project refuses (TX physically disconnected, never request authority, keep
DJI's Return-Home failsafe out of reach). So on the M600 there is no
read-only path to it, and the launcher deriving it itself is not merely the
current design, it is the only one.

### Cube Orange / ArduPilot — available read-only, but not trustworthy

- `MISSION_CURRENT` (#42) is in the `SRx_EXT_STAT` stream group. Set
  `SR2_EXT_STAT` and reboot and the autopilot **pro-actively** streams it,
  no request needed — so the read-only posture survives, same as the OSDK link.
- `MISSION_ITEM_REACHED` (#46) is an event, not a stream, and **it does not
  mean "I am at the waypoint."** ArduPilot chases a virtual lookahead point
  and marks the waypoint complete when THAT point arrives: *"this can be 10m
  (or more) ahead"*, scaling with speed. Also reported unreliable across
  command types, and ArduCopter 4.1+ S-curves changed the behaviour.

**Conclusion: asking the aircraft is LESS accurate than deciding locally.**
The launcher's 50Hz fused position gives ~10cm of travel between samples at
5 m/s; `MISSION_ITEM_REACHED` could be 10m+ early. Do not switch. Phase 4
should use MAVLink mission data as a **cross-check and mode/abort detector**,
never as the drop trigger.

Worth having anyway from MAVLink, when Phase 4 happens:
- log `MISSION_CURRENT` next to the launcher's own decision, so post-flight
  you can see the two disagree;
- **detect the aircraft leaving AUTO or going RTL.** Nothing currently tells
  the launcher a mission was aborted; it would keep dropping. Real safety gain
  from the same read-only stream.

### THE ACTUAL ACCURACY PROBLEM: nobody is modelling the fall

A propagule keeps the aircraft's horizontal velocity while it falls. The
launcher currently releases *at* the waypoint, so every seed lands downrange:

| Height | Fall time | @2 m/s | @5 m/s | @8 m/s |
|---|---|---|---|---|
| 5 m  | 1.01 s | 2.0 m | **5.0 m** | 8.1 m |
| 8 m  | 1.28 s | 2.6 m | **6.4 m** | 10.2 m |
| 15 m | 1.75 s | 3.5 m | **8.7 m** | 14.0 m |

At 5 m/s from 8m that is **6.4m past target — twice the 3m radius.** Chasing
10cm of sampling accuracy while ignoring this is straining a gnat.

### The design: predict the IMPACT point, not the release point

Do not compute "where do I release". Compute where the propagule would land if
released right now, every cycle, and fire when that enters the target radius:

    impact = position + v_horizontal x (t_exit + t_fall)
    fire when haversine(impact, target) <= radius

    t_fall = (-vz + sqrt(vz^2 + 2*g*h)) / g        (vz positive DOWN)

This handles turns, speed changes and wind-on-the-aircraft for free, it
degenerates to exactly today's behaviour at zero velocity, and it is a
substitution of `cur` in `DropLogicEngine`'s distance check rather than a
rewrite. It also makes `radius` mean an accuracy tolerance instead of a fudge
factor for sampling.

### Everything the physics needs is ALREADY on the wire

| Needed | Status | Where |
|---|---|---|
| lat / lon | in use | `POSITION` +0, +8 |
| **height above takeoff** | **parsed then DISCARDED** | `POSITION` +20 |
| barometric altitude | in use as `alt_m` | `POSITION` +16 |
| horizontal velocity vx, vy | on the wire, not decoded | `VELOCITY` +0, +4 |
| vertical velocity vz | on the wire, not decoded | `VELOCITY` +8 |
| velocity health | on the wire, not decoded | `VELOCITY` +12 |
| turn rate (gate the straight-line assumption) | enabled, not decoded | `ANG_RATE` |
| airborne | enabled, not decoded | `STATUS` |

The live passFlag **0x0A79** decodes to ON: TIME, VELOCITY, ANG_RATE,
POSITION, GPS_DETAIL, RC, STATUS — OFF: QUATERNION, ACCEL, RTK, MAG, GIMBAL,
BATTERY, DEVICE. **No new hardware and no DJI Assistant 2 change is needed.**
It is decode-and-store work only.

**The one concrete bug found by this research:** `OsdkPositionSource.cpp:144`
reads `float alt, height;`, memcpy's BOTH, then assigns only `_last.alt_m =
alt`. DJI's `height` is height above the takeoff point — the exact number
ballistics needs — and it is being parsed off the wire and thrown away because
`Position` has no field for it. One struct member and one assignment.

### What is genuinely NOT available

1. **`t_exit`** — when the propagule actually leaves during the 580ms travel.
   Up to **2.9m of along-track spread at 5 m/s** until measured. Bench job,
   240fps phone, same technique as the drive-window check. **No drone needed.**
2. **Terrain / tide offset.** `height` is above the TAKEOFF point, not the drop
   surface. On a tidal mangrove flat that matters. Suggest a per-waypoint
   `ground_offset_m` in `mission.json` — the operator knows it, the aircraft
   does not.
3. **Wind on the falling propagule.** The aircraft's velocity already includes
   wind drift on itself, but not what wind does to the seed over 1.3s of fall.
4. **Propagule drag.** Rhizophora propagules are dense darts, so near-ballistic
   is a fair first model, but it needs a calibration factor from real drops.

### Error budget at 5 m/s, 8m

| Term | Magnitude | Status |
|---|---|---|
| Fall carry | **6.4 m** | computable today from data in hand |
| Exit-time uncertainty | up to **2.9 m** | bench measurement, no drone |
| Wind + drag | ~1-2 m? | empirical, needs real drops |
| Tide / terrain | site-dependent | operator config |
| GPS position | 0.5-1 m | accepted |
| Sampling @50Hz | 0.1 m | negligible |

### Bonus finding: the RC trigger could avoid GPIO 35 — but only partly

`Telemetry::RC` in the broadcast is exactly six `int16_t`: **roll, pitch, yaw,
throttle, mode, gear. No auxiliary channels.** So the A3's physical F-channel
output pin — what Phase 3c plans to wire to GPIO 35 — is NOT in the broadcast.

What IS available is the **mode** switch (P/A/F) and the **gear** switch. If
the M600's gear switch is otherwise unused, a pilot trigger could come over
the existing serial link with **no new wire, no 10k pull-down, and none of the
unverified A3 output-voltage risk**, on a link already proven at 46.8 fps.
Do NOT use `mode` — F mode is needed for other things. `RcPwmInput`'s
band/debounce/edge logic ports nearly verbatim onto an int16 stream, and
signal-loss fail-safe becomes broadcast staleness, which is already detected.

`osdk_sniff.py --diff` exists precisely to settle which switch moves which
bytes. Capture with the switch up, capture with it down.

### Sources

- DJI `DataBroadcast` reference and `Telemetry::RC` struct reference
  (developer.dji.com/onboard-api-reference)
- ArduPilot "Requesting Data From The Autopilot"
  (ardupilot.org/dev/docs/mavlink-requesting-data.html)
- ArduPilot discourse: "When is MISSION_ITEM_REACHED sent" (t/21107) and
  "Problem monitoring Mission using MISSION_CURRENT and MISSION_ITEM_REACHED"
  (t/98988)
- Mission Planner built-in SITL (ardupilot.org/planner/docs/mission-planner-simulation.html)

### Testable with NO aircraft

- Mission Planner has **built-in SITL** (Simulation tab, one click, no build).
  Fly a mission, log `MISSION_ITEM_REACHED` and `MISSION_CURRENT` against
  `GLOBAL_POSITION_INT`, and measure how many metres early the event really
  fires. Turns the "10m or more" folklore into a number.
- The impact predictor is pure math — build and test it offline, the same way
  the CRC and passFlag work was validated before touching hardware.
- **The 602-frame OSDK capture is GONE** (`*.bin` is gitignored, nothing on
  disk). A fresh capture is needed for offline replay — grab one, plus a
  `--diff` pair for the gear switch, in a single session rather than two trips.

---

## PHASE 4a (2026-09-01): THE IMPACT PREDICTOR \u2014 BUILT AND SELF-TESTED

**Status: IMPLEMENTED and verified by two independent implementations. NEVER
RUN ON REAL AIRCRAFT DATA \u2014 SIM injects position with no velocity, so the
predictor has correctly never fired in anger. Read the gaps below before
trusting a drop.**

### What was built

- **`Position` gains ballistics fields** (`PositionSource.h`): `height_m`,
  `height_valid`, `vn_mps` / `ve_mps` / `vd_mps` (NED, **vd positive DOWN**),
  `vel_valid`, and a `horizontalSpeed()` helper. Only `OsdkPositionSource`
  fills them; GNSS and SIM leave them invalid.
- **`OsdkPositionSource` now keeps `height`** \u2014 the field that had been
  memcpy'd and discarded since Phase 3d, gated on position health and
  plausibility. **And decodes VELOCITY (bit 3)**, which cost nothing: bit 3
  sits below POSITION so its size was already in `FIELD_SZ` and the same
  offset-from-flag walk reaches it. No new capture, no DJI Assistant 2 change.
- **`Ballistics.h`** \u2014 new, pure arithmetic, no state:
  `fallTime()`, `leadTime()`, `carryDistance()`, `predictImpact()`.
- **`DropLogicEngine`** now tests the PREDICTED IMPACT POINT against the
  waypoint instead of the aircraft position. It is a substitution at the
  haversine call, not a rewrite \u2014 everything downstream is untouched.

### It fails closed, and that is the design

`predictImpact()` returns false and leaves its outputs alone whenever position,
velocity or height is missing or implausible (height >120m, speed >30 m/s,
degenerate latitude). The caller then uses the aircraft position, which is
**exactly** the pre-Phase-4 behaviour. So the GNSS fallback, every SIM build,
and any frame with bad velocity health all behave as they did yesterday. At zero
velocity the prediction degenerates to the aircraft position anyway \u2014 verified
to be *exactly* 0.000000000 degrees of offset, so hover drops are unchanged.

### Verified by two implementations, on the real target

`balltest` on the serial console runs the vectors on the actual ESP32 \u2014 real
float behaviour, not a host approximation \u2014 and `tools/test_ballistics.py` is an
independent Python reimplementation. They agree to five decimals:

    fallTime  5m/8m/15m     1.00981 / 1.27732 / 1.74904 s
    fallTime  8m, vd=+1/-1  1.17941 / 1.38336 s
    carry @8m, 2/5/8 m/s    2.555 / 6.387 / 10.219 m
    dLat 5 m/s north @8m    0.000057372 deg  (6.39 m)
    v=0 degenerate          dLat=0.000000000 dLon=0.000000000
    all five gates          ok=0

The Python file also re-derives the fall-carry table below rather than quoting
it \u2014 **and caught an error in it.** The h=5m @5 m/s cell read 5.1m; it is
5.049m, which rounds to 5.0. The 2 and 8 m/s cells were right, so it was a
single rounding slip. Corrected here and in `Ballistics.h`.

### WHAT IS STILL MISSING \u2014 read before trusting a real drop

1. **The predictor has never seen real aircraft data.** SIM has no velocity, so
   it has correctly refused to predict every time it has ever run. The
   `[DLE] WP%d impact=... carry=... lead=...` log line **has never been printed
   on hardware.** First real test needs the M600 powered with the OSDK link
   live; a walk test with the SIM build would do it, since GPIO 14 shows the
   trigger without anything moving.
2. **`T_EXIT_S` is 0.0 and UNMEASURED** \u2014 worth up to **2.9m** of along-track
   spread at 5 m/s. It is deliberately 0 rather than a guess: a guessed value
   biases every drop in one direction while looking authoritative in the log.
   Bench job, 240fps phone, no drone needed \u2014 the same technique that measured
   the drive window.
3. **The DJI vertical-velocity sign is ASSUMED, not verified.**
   `OSDK_VZ_IS_UP = true` in `OsdkPositionSource.cpp`. Getting it wrong is
   second-order \u2014 about 1m at 8m for 1 m/s of vertical motion, against the 6.4m
   the horizontal terms fix \u2014 but it is a real error and it is free to check:
   climb steadily and confirm `vd` goes NEGATIVE while barometric `alt_m` rises.
4. **`height` is above TAKEOFF, not above the drop surface.** On a tidal
   mangrove flat that matters. A per-waypoint `ground_offset_m` in
   `mission.json` is the fix \u2014 the operator knows the tide, the aircraft does not.
5. **No drag model.** Rhizophora propagules are dense darts so near-ballistic is
   a fair first approximation, but it needs a calibration factor from real drops.

### The radius means something different now

With the predictor active, `radius_m` stops being a fudge factor absorbing GPS
noise and becomes an accuracy tolerance: "drop when I predict I will land within
X metres". The 5.0m currently saved was chosen under the OLD meaning (smallest
value that still survives the 1Hz GNSS fallback). Once the predictor is running
on real data, revisit it \u2014 and consider the further change of firing at CLOSEST
PREDICTED APPROACH rather than on circle ENTRY, which would decouple "when to
fire" from "how big is the circle" entirely and make radius purely a reject gate.

---

## FLIGHT CONTROLLER (Cube Orange \u2014 separate from launcher)

- User has Cube Orange Standard Set. Successfully flashed ArduCopter V4.6.3, CubeOrangePlus.
- Frame: needs Hexa X (FRAME_CLASS=2, FRAME_TYPE=1) — user's final drone is a 6-rotor.
  (Currently user only has a DIY quadcopter with insufficient lift; drone not final yet.)
- Driver issue was resolved (CubePilot driver + reboot).
- For later MAVLink position source (Phase 4): TELEM2 = SERIAL2.
  Set SERIAL2_PROTOCOL=2, SERIAL2_BAUD=57, SR2_POSITION=4.
  TELEM2 JST-GH 6pin: pin2 TX, pin3 RX, pin6 GND. 3.3V, no shifter.
- **PIN CORRECTION (2026-08-12):** this section previously specified ESP32
  **GPIO 16/17** for that UART. That is WRONG on this hardware and would have
  failed exactly like the GPIO 9/10 bug: **GPIO 16 and 17 are wired to the
  PSRAM chip on WROVER modules** and are not available as general I/O.
  Use **GPIO 25 (RX) / GPIO 26 (TX)** instead — both free, neither a
  strapping pin, no boot-time glitch. UART1 is already taken by the GNSS on
  GPIO 4/5, so this is UART2. Same correction applies to the DJI OSDK API
  port if that path is pursued.
- The launcher is INDEPENDENT of the Cube. They only share nothing but the user may later
  feed MAVLink position. Default is onboard GNSS.
- This independence is a deliberate architectural decision, not an accident of how the project
  grew — full reasoning in `DESIGN_RATIONALE.md`. Key point for anyone extending this: the
  launcher may CONSUME data from the Cube (e.g. the Phase 4 MAVLink position source above), but
  it must never COMMAND through it and must never run code on it. Drop logic stays on the
  launcher's own MCU.

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

Added since, in the shipped `Servo_Web_Calibrator_WROVER` firmware (Phases 3b-3d):
- OsdkPositionSource - DJI A3 broadcast decoder, read-only, offsets from passFlag
- FallbackPositionSource - prefers OSDK, degrades to the M8N after 500ms stale
- SimPositionSource - `/simpos` injection for indoor tests; falls through to the
  real source until a position is injected (compiled out of flight builds)
- RcPwmInput       - IRAM_ATTR edge ISR pilot trigger on GPIO 35, never `pulseIn()`
- Logger           - now JSONL to `/logs/flight.jsonl`, 1.1MB cap, events + 1Hz tlm

`mission.json` also now holds the `rc_trigger` block (band thresholds) and the
waypoint list is editable at runtime through the dashboard's Waypoints tab
rather than being seeded once from `MissionDefault.h`.

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

9. NEVER hang an LED (or any load) on the EN / CHIP_PU pin. EN is the chip's reset
   INPUT, not a GPIO — firmware cannot drive it, so it can only ever indicate
   "not currently in reset", which is also true during a boot loop. Worse, an LED
   there clamps EN to ~2.05V against its 10k pull-up, below the ~2.475V V_IH
   threshold, so the chip may never leave reset — and it loads the EN reset RC cap,
   which breaks the CH340 auto-reset during flashing (uploads start failing
   intermittently, which is easy to misdiagnose as a bad cable). For a
   hardware-only "alive" light, tap the 3V3 rail instead.

10. GPIO 34-39 are INPUT-ONLY on the classic ESP32 and have NO internal pull-up or
    pull-down. That input-only property is a genuine safety feature for signals
    coming FROM the flight controller (no firmware bug can back-drive it), which is
    why the RC trigger uses GPIO 35 — but it means an external pull resistor is
    mandatory, or a disconnected wire floats and reads as noise instead of silence.

11. Arduino IDE's default "Compiler warnings: None" setting SUPPRESSES `#warning`
    directives. The flight-build warning about open defects (a)-(c) only shows if
    Preferences -> Compiler warnings is set to Default or higher. Worth setting
    permanently on this project.

12. `git commit` in this repo needed an identity — it is now set repo-locally to
    Carlos Odreman <codreman2@gmail.com>, matching the existing history.

13. **A FRAME IS NOT A FIX.** The A3 broadcasts happily indoors with lat/lon at
    exactly 0.0 and health 0. Accepting a well-formed frame as a valid position
    hands drop logic a coordinate off the coast of Africa. Gate on health >= 3
    AND non-zero coordinates. Same shape of error as trusting an M8N sentence
    that parsed cleanly but carries no lock.

14. **Do not let an unverified CRC gate anything.** The first live capture scored
    0/601 on header CRC because the constants were guessed wrong, not because
    the link was bad. Had CRC gated frame acceptance rather than merely
    reporting, a perfectly healthy link would have been declared dead. Report
    first, gate only once the constants are proven against real frames.

15. **Compute protocol field offsets from the flag word; never hardcode them.**
    The M600's broadcast was trimmed between sessions (passFlag 0x067F ->
    0x0A79) and POSITION moved from offset 72 to 44. Computed offsets decoded it
    first try; a hardcoded 72 would have produced plausible garbage.

16. **A delegating wrapper must delegate `begin()` too.** `SimPositionSource`
    forwarded `update()`, `read()` and `hasFix()` to its fallback but not
    `begin()`, so UART2 was never initialised and BOTH position sources were
    dead in every SIM build for as long as the fallback existed. The boot log
    looked healthy — the only symptom was an init line that never printed.
    **Absence of expected output is a symptom; scan for what is missing, not
    only for what looks wrong.**

18. **Two implementations of a safety-critical action is one too many.** The
    e-stop existed twice — `handleStop()` for the dashboard route and a
    separate copy inside the serial command handler. Fixing confirmed defects
    (a) and (b) in `handleStop()` left the serial path still firing module B
    on an emergency stop, and only the regression test caught it. Worse, the
    test could easily have been run through the browser, passed, and left the
    bench path — the one actually used, tethered to a laptop with no dashboard
    — broken. If a third entry point ever appears, it calls the same function.

19. **Fix interacting defects together, not one at a time.** Fixing (a) alone
    would have stranded `DropLogicEngine::_phase` in `WAITING_A` with nothing
    left to advance it: the state machine would have sat in DROPPING forever,
    a worse failure than the one being repaired. The two were only visible as
    a pair because the run showed (a) *completing the mission*, which is what
    masked (b)'s real symptom.

17. **Serial and filesystem work in `loop()` IS drop latency.** Three separate
    instances on this project: a `printf()` of the waypoint distance every
    iteration (~2.6ms each at 115200, and only while ARMED — i.e. exactly when
    it hurts); an HTTP handler reading a 752KB log byte-by-byte every 5s; and
    open/append/close per telemetry row, where LittleFS walks metadata on each
    open for 80-200ms once the file is large. Hold the handle and `flush()`
    instead — `lfs_file_sync()` gives the same crash safety that closing was
    supposed to buy. And **instrument the worst loop iteration** so this is
    measured rather than argued about.

---

## CURRENT EXACT STATE / WHERE WE ARE

*(Last reviewed 2026-09-01, after the PHASE 3g bench run.)*

- Board settings correct: ESP32 Wrover Module, 8MB flash, QIO, 80MHz,
  Default 4MB with spiffs, upload 921600. **The COM port moves between
  sessions** — COM10 early on, COM6 for the Phase 3d flashes, and back to
  COM10 on 2026-08-25. Check `Tools -> Port` every time rather than trusting
  a number written down.
- servo_minimal.ino boot instability is resolved — base is solid.
- Module A: **open=1470 stop=1670 travel=580ms**, confirmed from the board's
  own boot banner on 2026-08-25. The **1450** this document used to carry was
  stale history — settled. Module B reads open=1300 stop=1500 travel=350ms,
  which are still defaults and still unmeasured. The stop values are worth
  re-verifying against real servos — see the plan's step 1.
- Module B has NEVER been calibrated and its **hardware is not assembled**.
  `Servo_Calibration_Tool_B.ino` is written and ready for whenever it exists.
- **Door-to-door travel deviation on Module A is still un-characterised** (35mm
  then 41mm at the same 580ms). Still the main open MECHANICAL question, and it
  compounds across doors — see KNOWN ISSUE above.
- **SIM mode has RUN on hardware; the RC pilot trigger has not.** The GPS
  auto-fire path was observed end to end (Phase 3d): waypoint detected at 0.0m,
  A fired 580ms, B 350ms, both logged, auto-disarm. No real RC signal has ever
  been read. `SIM_MODE` is still **1** in `Pins.h`, so the sketch as committed
  is the dry-run build: it will not drive the servos. Flip to 0 for a flight
  build (and read the `#warning` it emits).
- **The servos have still never been driven by this firmware. Not once.**
  Everything proven so far was proven on LEDs with the servo UBEC unplugged.
- **The plan's results table is now over half filled.** Steps 1, 3, 4, 6, 7a,
  9, 10, 11 and 12 all have observed results. Still open: 2, 5, 7b, 8 (browser
  and limit-switch), and the entire RC block (B1-B4, 13-20).
- **The saved mission is now 10 waypoints at r=5.0m**, built around the real
  coordinate 9.0222514, -79.5322405 with ~22m spacing so the circles stay
  disjoint. It is the defect (c) fixture; it is NOT a real plot.
- **The flight log is NOT full** — ~474KB of a 1.1MB cap. Earlier notes saying
  otherwise were stale.
- Git: work is on branch **`sim-mode-and-rc-trigger`**. Merge to main when the
  bench run has validated the rest — the RC path in particular has still never
  seen a real signal.

### Compile status (arduino-cli, core 3.3.8 — NOT PlatformIO)

Measured 2026-08-25, after the Phase 3e latency work:

| Build | Flash | RAM |
|---|---|---|
| Dashboard, SIM_MODE 1 | 1,102,428 (84%) | 50,064 (15%) |
| Dashboard, SIM_MODE 0 | 1,097,427 (83%) | 50,084 (15%) |

SIM_MODE 1 measured 2026-09-01 after PHASE 4a. The ballistics work added
+8,556 bytes, a good part of it the SIM-only `balltest` vectors, taking flash
from 83% to 84%. ~208KB headroom left. Zero warnings from project code.

**Zero warnings from project code, both builds** (compiled with
`--warnings default`). The only diagnostic emitted is the intentional
`#warning` on flight builds listing defects (a), (b) and (c) — which is the
guard working, not a problem. The two `%d`-vs-`uint32_t` warnings were fixed
back in `68cbe30`.

The older `RC_Input_Calibration_Tool` figure was 304,640 (23%) flash /
22,756 (6%) RAM.

**arduino-cli lives inside the Arduino IDE install** — it is not on PATH:

    "C:/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"

with `ARDUINO_DIRECTORIES_DATA=C:/Users/carli/AppData/Local/Arduino15` so it
picks up the same esp32 core 3.3.8 the IDE uses. Build:

    arduino-cli compile --fqbn esp32:esp32:esp32wrover --warnings default \
        Servo_Web_Calibrator_WROVER

---

## IMMEDIATE NEXT GOAL

*(Rewritten 2026-09-01. All four planted defects are now closed: (a), (b) and
(e) confirmed-and-fixed on hardware, (c)'s symptoms proven unreachable in
PHASE 3g.)*

**Three things are now in front, in this order:**

1. **Finish the ballistics work** — the predictor is BUILT and self-tested
   (PHASE 4a) but **has never seen real aircraft data**, because SIM has no
   velocity and it correctly refuses to predict without it. Three things close
   it, in increasing cost:
   a. **Measure `T_EXIT_S`** — bench, 240fps phone, no drone. Currently 0.0 and
      worth up to 2.9m of spread.
   b. **Verify the DJI vertical-velocity sign** — `OSDK_VZ_IS_UP` is assumed.
      Climb steadily; `vd` must go NEGATIVE while barometric `alt_m` rises.
   c. **Power the M600 and watch a real `[DLE] ... impact=` line appear.** That
      log line has never been printed. A walk test with the SIM build proves the
      whole path with nothing moving — GPIO 14 shows the trigger.
2. **Wire the limit switches** — promoted to a blocker by PHASE 3g. Until they
   are wired, the last door of each module overdrives 3x. This gates ALL servo
   work, and it also unlocks plan step 7b.
3. **The rest of the plan needs a human at the bench** — every LED criterion,
   steps 2 and 8 (browser), and the whole RC path, which still needs the A3
   F-channel voltage measured first. Note PHASE 4 RESEARCH's finding that the
   pilot trigger could come over the existing OSDK link via the `gear` switch,
   which needs no wiring and none of the A3 output-voltage risk — worth
   settling with `osdk_sniff.py --diff` before soldering anything to GPIO 35.

Everything below still applies. Follow `SIM_MODE_VALIDATION_PLAN.md` — it has
the pre-run checklist, per-step pass criteria and a partly-filled results
table.

Work splits cleanly into a software half (needs only two LEDs and a
breadboard) and a mechanical half (needs servos, and Module B hardware
that doesn't exist yet). Do the software half first — it is cheap, safe,
and gates everything else.

### Software half — no servos, no aircraft
0. Housekeeping: **DONE 2026-09-01.** The log was never actually full
   (~474KB of 1.1MB) and was left intact. Radius is now 5.0m on all 10
   waypoints — see PHASE 3g for why 5 and not 10.
1. Wire the two indicator LEDs (GPIO 14 announce, GPIO 27 yellow wait,
   330R each to GND). **Leave the servo UBEC unplugged.**
2. Flash the dashboard with `SIM_MODE 1` and work the plan's results table.
   The auto-fire A→B sequence (step 11) is already demonstrated; what is
   NOT are the multi-door steps (6, 7a, 7b), the guard toast (8), and above
   all **steps 9, 10 and 12** — the three with inverted pass criteria, which
   exist to confirm defects (a), (b) and (c) rather than to pass.
3. Fix whatever the run confirms. Those are flight-build bugs in shared
   code, not sim-mode bugs. Re-run the affected steps.
4. Chase the remaining **60-200ms worst-loop spikes around the drop** (see
   Phase 3d open items). The instrumentation to measure them is already on
   the dashboard, so this is now a measurement job, not a guessing one.
5. RC trigger bring-up: **measure the A3 F-channel voltage first**, fit
   the 10k pull-down on GPIO 35, then `RC_Input_Calibration_Tool.ino`
   (plan steps B1–B4), then the dashboard RC card (steps 13–20). Steps
   15, 19 and 20 are the "must NOT fire" cases — those matter most.
6. Verify the browser UI from a device that can join the `SmartLauncher` AP.
   It has never been exercised from this environment.

### Mechanical half — needs real servos
5. Re-measure Module A's true stop fresh on the Calibration tab. Do not
   trust 1670 carried forward, even though it came from this toolchain.
6. **Characterize the door-to-door travel deviation** (KNOWN ISSUE above):
   run 580ms across all 5 door positions in one sequence, marking and
   measuring each, to see whether door 1 is uniquely short or the error
   accumulates. Determines whether one global `travel_ms` is usable, needs
   a first-door exception, or points at a mechanical fault worth fixing
   before this module is trusted with real seeds.
7. Bench Test 4 from `SMART_LAUNCHER_COMPLETE_GUIDE.md` for Module A alone
   — 5 consecutive door drops ending in a confirmed limit-switch stop.
8. When Module B hardware exists: `Servo_Calibration_Tool_B.ino` for its
   true stop, then the dashboard's Calibration tab for open/close/
   travel_ms, then repeat the 5-door consistency check.
9. Both modules calibrated -> full two-module bench test (sequential
   A-then-B confirm) -> Phase 4 (MAVLink position from the Cube's TELEM2).

### Standing rule that keeps paying off
Build with the **Arduino IDE / arduino-cli toolchain (core 3.3.8)**, not
PlatformIO. Every calibration number measured through it has held up on
re-verification; the PlatformIO path's first live drop test caused a real
servo-runaway safety incident.

---

## DEVELOPMENT ROADMAP

- Phase 1  Electronics/schematic/pins/power - DONE (spec'd)
- Phase 2  Firmware skeleton                - DONE (ported to WROVER-E, flashed, 2 bugs fixed)
- Phase 3  Mission Dashboard (Wi-Fi config + drop logic + calibration tab) - BUILT,
           partially validated (browser UI still never exercised)
- Phase 3b SIM mode (LED dry-run + fake position injection) - RUN ON HARDWARE;
           GPS auto-fire path proven, e-stop and multi-door steps outstanding
- Phase 3c Pilot RC trigger (M600 A3 F-channel) + calibrator - WRITTEN, NOT RUN.
           No real A3 signal has ever been read.
- Phase 3d OSDK position + Waypoints tab + JSONL flight log - DONE, verified
           outdoors at 46.8 frames/sec with a real fix
- Phase 3e Drop-latency fix (TX buffering, drop-window logging, direct
           overshoot measurement) - DONE, verified on hardware: +0ms overshoot.
           Platform migration to a Pi Zero 2 W evaluated and REJECTED.
- Phase 3f E-stop defects (a), (b) + newly found (e) - CONFIRMED on hardware,
           FIXED, and re-verified. Both e-stop entry points unified.
- Phase 3g Defect (c) CLOSED - symptoms proven unreachable on hardware, though
           the inner guard is unreachable by construction and never ran.
           SIM-gated waypoint serial commands added. Housekeeping done.
           Found a LATENT SERVO HAZARD: unwired limit switches make the last
           door of each module overdrive 3x. Blocks servo bring-up.
- Phase 4a Impact predictor (fall carry) - BUILT and self-tested on target,
           agreeing with an independent Python implementation. NEVER run on
           real aircraft data; t_exit unmeasured; DJI vz sign unverified.
- Phase 4  MAVLink position (Cube Orange TELEM2) - pending
- Phase 5  Python config tool (import QGC/UgCS missions -> mission.json) - pending
- Phase 6  Testing (bench -> static drops -> low altitude -> full mission) - pending

### Document map
- `START_HERE.md` — beginner's onboarding guide, assumes no prior context.
  Point a newcomer here, not at this document. `docs/GUIA_INICIO.pdf` is the
  Spanish edition of the same material.
- `SMART_LAUNCHER_COMPLETE_GUIDE.md` — wiring, BOM, calibration, bench checklists.
- `DESIGN_RATIONALE.md` — why the launcher runs on its own MCU, not the Cube.
  Reference answer for that question; keep in sync if new incidents change it.
- `SIM_MODE_VALIDATION_PLAN.md` — the bench script for Phases 3b/3c, with
  per-step pass criteria and a results table. **This is the next thing to do.**
- `Servo_Web_Calibrator_WROVER/Pins.h` — the authoritative pin table, with the
  reasoning for every unsafe pin recorded inline.
- `tools/osdk_sniff.py` — PC-side DJI OSDK broadcast decoder and `--diff` field
  locator. Use it when the A3's broadcast config changes.
- `Servo_Web_Calibrator_WROVER/Ballistics.h` — the impact predictor. Pure
  arithmetic, no state, no I/O, so it is readable and testable on its own.
  Run `balltest` on the serial console to execute its vectors on the target.
- `tools/test_ballistics.py` — independent Python reimplementation of the same
  math, and the expected values `balltest` is checked against. Run it after any
  change to `Ballistics.h`; it also re-derives the fall-carry table.
- `tools/bench_defect_c.py` — plan step 12. Resets the board, arms, walks a
  10-waypoint / 5-door mission through `simpos`, and shows what happens when
  the doors run out. It is what closed defect (c), and it surfaced the
  limit-switch overdrive hazard as a side effect. Re-run it after any change to
  `DropLogicEngine`'s completion logic.
- `tools/bench_serial.py` — PC-side serial bench driver. `boot`, `listen`,
  `cmd`, `drop` and `estop` modes. The `estop` mode schedules the emergency
  stop INSIDE module A's drive window, which a human cannot reliably do by
  hand; it is what confirmed defects (a), (b) and (e) and then caught the
  first fix landing on only one of the two e-stop paths. Prints the live port
  list if the CH340 has moved.
