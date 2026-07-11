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

Phase 2 firmware has now been ported to the WROVER-E pinout/API and
build-verified (see above) — it is NOT yet flashed, waiting on the
Module A OPEN pwm value and the travel deviation question below.

Current sequence:
1. Get the exact "OPEN=" value the web calibrator currently shows for
   Module A and write it into `mission.json`'s
   `modules.A.servo_open_pwm_us` (currently a stale placeholder, 1700).
2. **Characterize the door-to-door travel deviation on Module A** (see
   KNOWN ISSUE above) — run the current 580ms setting across all 5 door
   positions in one sequence, marking/measuring each, to see whether
   door 1 is uniquely short or whether the deviation grows/accumulates.
   This determines whether a single global travel_ms is usable, needs a
   first-door exception, or points to a mechanical problem worth
   inspecting before trusting this module with real seeds.
3. Flash the ported Phase 2 firmware (+ `pio run --target uploadfs` for
   mission.json) and re-verify via the serial bench commands (`arm`,
   `dropa`, `status`, `log`) that it matches what the calibrator showed —
   same live-hardware-verification discipline as the rest of this session.
4. Once travel is acceptably consistent (or compensated for), run bench
   Test 4 from `SMART_LAUNCHER_COMPLETE_GUIDE.md` for Module A alone —
   5 consecutive door drops ending in a confirmed limit-switch stop.
5. When Module B hardware exists: `Servo_Calibration_Tool_B.ino` for its
   true stop, then the same web calibrator workflow (already supports it)
   for open/close/travel_ms, then repeat the 5-door consistency check.
6. Once both modules are calibrated: full two-module bench Test
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
