# SIM Mode Validation Plan — LED launch announcer (GPIO 14)

Status: **PLAN ONLY — nothing implemented yet.** Written 2026-08-11.

Goal: validate the Phase 3 Mission Dashboard's timing, state and HTTP behaviour
end-to-end with **no servo motion at all**, by replacing the actuator's output
stage with a single LED on GPIO 14. Unblocks the software half of the handoff's
verification checklist while Module B hardware doesn't exist and the mechanical
travel-deviation question is still open.

---

## 1. Rationale — what is actually unverified

`PROJECT_HANDOFF.md` lists checklist items 2–7 as "needs the user physically
present, real hardware motion." But every one of them is timing, state or HTTP
behaviour. The servo is only the output stage:

| Open checklist item | Needs physical servo motion? |
|---|---|
| Mission Control tab renders (state, GPS, door counters) | No |
| ARM → dashboard reflects ARMED | No |
| Non-blocking actuator stops on `travel_ms` | No — a `millis()` comparison |
| Non-blocking actuator stops on limit switch (last door) | No — switch triggered by hand |
| `/status` polling keeps responding *during* a drop | No |
| EMERGENCY STOP halts a running drop | No |
| GPS-radius auto-fire, sequential A→B | No |
| Round-2 toast / HTTP-status fix (never reproduced in a browser) | No |

Swapping the output stage for an LED validates all of it with **zero risk of a
one-way rack retraction, zero risk of a repeat of the servo-runaway incident,
and no dependency on Module B hardware.**

### Explicitly NOT validated by this test

Sim mode says nothing about: `travel_ms` accuracy in millimetres, the
door-to-door travel deviation, backlash, servo direction / `reverse` flags, or
the LEDC-core runaway (a signal-generation defect — invisible when no signal is
generated). Handoff checklist item 1 (re-measure Module A's true stop) and item
8 (Module B calibration) still require real servos. This plan does not replace
them.

---

## 2. Candidate defects this run is designed to confirm

Found by **reading the code, not by reproduction.** The sim run is the safe way
to confirm or refute each. Do not treat them as established until observed.

### (a) Emergency stop mid-auto-sequence probably fires Module B

`handleStop()` (`Servo_Web_Calibrator_WROVER.ino:547`) calls `modA.stop()`,
which sets `_active = false`. It does not touch `DropLogicEngine`'s `_phase`.
With the engine in `WAITING_A`, its next `update()` tick sees
`!_modA.isDropping()`, treats that as "A completed successfully", records the
drop, and calls `_modB.startDrop()` (`DropLogicEngine.cpp:30`).

**The stop button would start the other module.** On the bench with an LED
that's a blink; with servos and seeds loaded it is not.

### (b) Emergency stop does not actually disarm

`handleStop()` calls `sm.requestDisarm()`, but that is guarded to the `ARMED`
state only (`StateMachine.cpp:41`), and `setDropping(true)` has already moved
the state to `DROPPING`. The call silently no-ops. Same class of bug as the
`completeBoot()` one already fixed on this project.

**Revised expectation after implementation:** the manual-drop cleanup added in
`loop()` (see defect (d), now fixed) calls `setDropping(false)` on the next
tick, which transitions DROPPING → **ARMED**. So the observable symptom is not
"stuck in DROPPING" — it is **"emergency stop leaves the system ARMED instead
of disarmed."** Still wrong, still worth fixing, but that is what test step 9
should actually look for.

### (c) Waypoints outstrip doors — drops 6–10 log false successes

`MissionDefault.h` ships **10 drop waypoints**, but each module has **5 doors**.
Once doors are spent, `startDrop()` returns `false` — and that return value is
ignored at `DropLogicEngine.cpp:76`. The engine advances to `WAITING_A`, sees
`isDropping() == false` immediately, and records the drop using a stale
`lastResult()` (defaults to `OK`). Drops 6–10 would complete instantly, log as
successful, and increment the drop index with no motion whatsoever.

The LED sim shows this unmistakably: 5 real flash windows, then 5 instant no-op
"drops" in the log.

### (d) Manual drops never enter the DROPPING state — **FIXED during implementation**

`handleDropA/B()` called `modX.startDrop()` but never `sm.setDropping(true)`, so
the state LED stayed at the 2Hz ARMED cadence during a manual drop and the
dashboard reported ARMED for the whole drop window.

Fixed rather than left for the test run, because the yellow WAIT LED cannot be
truthful without it — yellow must go dark exactly when a drop is in progress.
`handleDropA/B()` now set DROPPING on a successful start, and `loop()` clears it
once neither module is dropping and the engine is idle.

**Deliberate side effect:** because `setDropping(true)` makes `sm.isArmed()`
false, a second manual drop cannot be started while one is running — the other
module's button now returns `409 not armed`. This matches the project's stated
sequential-confirm principle ("Module A fires, its limit switch confirms, then
Module B fires. This preserves balance") and is treated as correct, not as a
regression.

---

## 3. Design decisions (locked)

| Decision | Choice | Reason |
|---|---|---|
| Announce LED | **GPIO 14, shared by both modules** | As requested. Modules are told apart by confirm-blip length (A short, B long); the critical test (defect a) reads fine on one LED — see §6 step 10. |
| Yellow WAIT LED | **GPIO 27** — *not* EN | EN (CHIP_PU) is the chip's reset **input**, not a GPIO: firmware cannot drive it. An LED there also clamps EN to ≈2.05V against its 10k pull-up, below the ≈2.475V V_IH, so the chip may never leave reset — and it loads the EN reset-RC cap, breaking auto-reset during flashing. GPIO 27 is free, not a strapping pin, no boot glitch, and sits adjacent to GPIO 14 on the Freenove WROVER header. Compiled into flight builds too, not sim-only. |
| Mode switch | **Compile-time `#define SIM_MODE` only** | Hard guarantee, no runtime ambiguity about which mode the board is in. |
| Fake position injection | **In scope** | Only way to test auto-fire indoors with no GPS fix; covers roadmap's "NMEA bench replay" item and is required for test steps 9–10. |

### Injection point — one line, one branch

The sim branch goes in **`ActuatorModule::_writePWM()` only**
(`ActuatorModule.cpp:90`) — the single point where a pulse reaches hardware.

Everything above it — `startDrop()`, `poll()`, `_timeout_ms`, `_is_last_door`,
the limit-switch read, door counting, `DropLogicEngine`, `StateMachine`, every
HTTP handler — runs **byte-identically to the flight build**. A separate
`SimActuator` class is explicitly rejected: it would mean validating code that
never flies.

### LED semantics — 1:1 truth, no distortion of timing

- `pulse == stop_pwm_us` → LED **OFF**
- `pulse != stop_pwm_us` → LED **ON**

The LED is lit for exactly the window the servo would be driven. A 580ms drop
is a 580ms flash — measurable to ±4ms on a 240fps phone slo-mo, enough to
confirm `travel_ms` is honoured. No speed multiplier, no stretching: the test
must not lie about the timing it exists to verify.

After the drive window closes, a **post-drop confirmation pattern** makes the
event human-readable without touching the drive window:

- Module A: `door_num` × short blips (80ms on / 120ms off)
- Module B: `door_num` × long blips (400ms on / 200ms off)

Distinct enough to tell the modules apart on one LED. Must be non-blocking
(millis-driven, advanced from `loop()`) — a blocking flash routine would
reintroduce exactly the watchdog/dashboard-freeze defect class this firmware was
rewritten to avoid.

### Hard safety guarantee

With `SIM_MODE 1`, `ActuatorModule::begin()` **skips `ledcAttach()` on GPIO
12/13 entirely.** The servo pins stay high-impedance — no PWM is generated at
all, rather than "PWM we believe equals the stop value." Compile-time, so it
cannot be left in the wrong position.

**In addition: physically unplug the servo UBEC for every sim run.** Costs
nothing and removes the entire runaway risk class instead of reasoning about it.

### GPIO 14 — pin safety

- **Not** a strapping pin on classic ESP32 (unlike GPIO 2), and not on the
  WROVER flash/PSRAM bus (unlike GPIO 9/10). Safe as `OUTPUT`.
- It **does** emit a brief PWM burst from the boot ROM at power-on. A short
  flicker before checkpoint `[1]` prints is **expected, not a fault.**
- Wiring: `GPIO14 → 330Ω → LED anode`, `LED cathode → GND`. ≈4mA at 3.3V with a
  red LED — well inside the ~12mA recommended per-pin limit.
- Shares the WROVER's common ground with everything else, as always.

### Fake position injection

Sim-only `SimPositionSource` implementing `IPositionSource`
(`PositionSource.h:15`), selected in place of `gnssSource` when `SIM_MODE 1`.
Holds one `Position`, returns `valid == true` only after a position has been
injected. New sim-only route:

- `GET /simpos?lat=<deg>&lon=<deg>` — set the fake position
- `GET /simpos?clear=1` — return to invalid/no-fix

Both compiled out entirely when `SIM_MODE 0`, so no injection surface exists in
a flight build. Waypoint 1 of the default mission is `9.345678, -79.876543`
with a 3.0m radius — inject that to trigger drop 1.

---

## 4. Files touched

| File | Change | Approx |
|---|---|---|
| `Pins.h` | `SIM_MODE`, `PIN_SIM_LED_A 14` + safety comment | ~10 lines |
| `ActuatorModule.h/.cpp` | sim output branch in `_writePWM()`, skip `ledcAttach`, non-blocking confirm-flash state | ~45 lines |
| `SimPositionSource.h` | new, sim-only fake position source | ~35 lines |
| `Servo_Web_Calibrator_WROVER.ino` | `/simpos` route, SIM banner in HTML, `"sim"` field in `/status`, checkpoint text | ~30 lines |

**Servo code path stays identical when `SIM_MODE 0`.** Verify this by diffing
the `SIM_MODE 0` preprocessor output against the current build if in doubt.

Compile and flash via the **Arduino IDE / arduino-cli toolchain (core 3.3.8)** —
not PlatformIO. Per the runaway incident, every number measured through the
Arduino IDE toolchain has held up; the PlatformIO path's first live drop caused
a safety incident.

---

## 5. Pre-run checklist

- [ ] Servo power (UBEC) **physically unplugged**
- [ ] Announce LED + 330Ω wired GPIO 14 → GND
- [ ] Yellow WAIT LED + 330Ω wired GPIO 27 → GND (adjacent pin, same rail)
- [ ] Limit switch A on **GPIO 32** (not 9 — see `Pins.h`)
- [ ] Board set to "ESP32 Wrover Module" in Arduino IDE
- [ ] `SIM_MODE` is **1** in `Pins.h` (banner on the dashboard confirms it)
- [ ] Arduino IDE → Preferences → *Compiler warnings* set to **Default** or
      higher, otherwise the flight-build `#warning` about open defects (a)–(c)
      is silently suppressed
- [ ] Serial monitor open at 115200 before power-on
- [ ] Browser ready to join the `SmartLauncher` AP → `http://192.168.4.1`

---

## 6. Test protocol

Each step has an explicit pass criterion. Record the result in §7.

1. **Boot.** All 12 checkpoints print; `[5]` announces SIM mode; GPIO 14 dark
   after its boot flicker.
   *Pass: no `rst:0x8 (TG1WDT_SYS_RESET)`, full checkpoint sequence.*

2. **Dashboard renders.** Banner reads `SIM MODE — NO SERVO OUTPUT`. Mission
   Control tab shows state, GPS status (no fix), door counters 0/5.
   *Pass: both tabs render, no missing fields.*

3. **ARM.** Press ARM (confirm).
   *Pass: dashboard shows ARMED, GPIO 22 state LED at 2Hz.*

4. **Manual DROP A, door 1.** Confirm-tap DROP A.
   *Pass: GPIO 14 lit for one continuous window, then 1 short confirm blip;
   `doorsA` 0→1; measured on-time within ±10% of `travel_ms` (580ms).*
   Measure with a 240fps slo-mo video.

5. **`/status` responsiveness during the drop.** Poll `/status` continuously
   while GPIO 14 is lit (browser auto-poll is enough; watch for a stall).
   *Pass: no gap in responses during the lit window. This is the exact defect
   class the non-blocking rewrite exists to prevent.*

6. **Doors 2–4.** Repeat manual DROP A.
   *Pass: `doorsA` reaches 4, confirm blips count up 2, 3, 4.*

7. **Door 5 — TIMEOUT branch.** Drop door 5 **without** touching the limit
   switch.
   *Pass: LED stays lit ~3× travel (1740ms), then serial logs
   `Door 5 — limit switch: TIMEOUT`.*
   Then `reset` door counters and repeat door 5 **with** GPIO 32 shorted to GND
   during the window.
   *Pass: LED extinguishes early, logs `limit switch: OK`.*

8. **Guard / toast check.** With the system ARMED, click a Calibration-tab
   OPEN button.
   *Pass: toast displays `409 blocked: armed/dropping` — not a silent failure.
   This is the first real reproduction of the Round-2 fix.*

9. **E-STOP mid-drop.** Start a manual DROP A, press EMERGENCY STOP while
   GPIO 14 is lit.
   *Pass: GPIO 14 extinguishes immediately and no confirm blips follow.*
   Then check **defect (b)** on the state badge: expected-correct is **IDLE**;
   if it reads **ARMED**, defect (b) is confirmed (yellow LED steady rather
   than slow-blinking is the same signal). Record which.

10. **E-STOP mid-auto-sequence — defect (a).** `GET /simpos?lat=9.345678&
    lon=-79.876543` while ARMED to trigger waypoint 1. Press EMERGENCY STOP
    while module A's window is lit.
    *Pass criterion is deliberately inverted: if GPIO 14 lights again for a
    fresh ~580ms window (Module B, long confirm blips) after the stop, **defect
    (a) is confirmed** — fix it before any further servo work.*

11. **Auto-fire happy path.** Clear position, re-arm, reset doors, then inject
    waypoint 1 and let the sequence run untouched.
    *Pass: A window → A confirm blips → B window → B confirm blips → drop
    logged, `dropIdx` advances to 2.*

12. **Doors vs waypoints — defect (c).** Continue injecting waypoints 2…6.
    *Pass criterion inverted again: if drop 6 completes instantly with no LED
    window and logs `OK`, **defect (c) is confirmed.***

---

## 6b. Pilot RC trigger (M600 A3 F-channel) — test protocol

The RC trigger is validated on the same LED rig, which is the whole payoff:
a pilot-commanded drop can be proven end-to-end with **no servos and no
aircraft**.

### Bring-up first — standalone tool

Do **not** start with the dashboard. Flash `RC_Input_Calibration_Tool.ino`
(repo root) and prove the signal in isolation, the same discipline that caught
the GPIO 2 and GPIO 9/10 bugs.

- [ ] **Measure the A3 F-channel output voltage before connecting.** Should be
      3.3V. If it reads 5V, fit a 10k/20k divider or a level shifter — the
      ESP32 is not 5V tolerant.
- [ ] External **10k pull-down** GPIO 35 → GND (input-only pins have no
      internal pull; without it a disconnected wire floats and reads as noise)
- [ ] Common ground between the M600 and the launcher

| # | Test | Pass criterion |
|---|---|---|
| B1 | Frame rate | `~50 Hz`, glitches not climbing. Anything else = wiring/ground/voltage — stop and fix |
| B2 | Full sweep | Move the switch through every position; `min`/`max` span ≥200µs |
| B3 | `g` suggest | Thresholds returned; each switch position lands in the band you expect |
| B4 | Disconnect signal | Reads `NO SIGNAL` within ~500ms — **not** a stuck last value |

### Then, on the dashboard (SIM build, servos still unplugged)

13. **Live readout.** Calibration tab → RC PILOT TRIGGER card shows live µs,
    band, and ~50Hz. *Pass: matches what the standalone tool reported.*
14. **Apply + Save.** Set thresholds, Apply + Save, reboot.
    *Pass: values survive; `[CFG]` summary line prints them at boot.*
15. **Fire while DISARMED.** Flick the switch to FIRE with the system IDLE.
    *Pass: GPIO 14 stays dark; serial logs `FIRE ignored — not ARMED`. This is
    the two-authority rule; if it drops here, stop everything.*
16. **Fire while ARMED.** ARM from the dashboard, then flick to FIRE.
    *Pass: A window → A short blips → B window → B long blips; yellow LED dark
    throughout, steady again after. `dropIdx` must **not** advance — a pilot
    drop must not consume a planned GPS waypoint.*
17. **Hold at FIRE.** Leave the switch at FIRE.
    *Pass: exactly one pair fires. Holding must not dump every door.*
18. **Re-fire.** Return to SAFE, then FIRE again.
    *Pass: a second pair fires — confirms the edge re-arms.*
19. **Cold start at FIRE.** Power-cycle with the switch already at FIRE, then
    ARM. *Pass: nothing fires until the switch is cycled back below `fire_min`
    and returned. This is the boot-with-switch-on guard.*
20. **Signal loss mid-flight.** Unplug the signal wire while ARMED.
    *Pass: serial logs `SIGNAL LOST`; nothing fires. Loss must never trigger.*

> **Since 2026-10-15 the RC trigger ships disabled** (the F-channel is not in
> the wiring guides). For steps 13–20, tick *RC trigger enabled* on the
> Calibration tab and Apply + Save first; untick and save again afterwards.

---

## 6c. Release gate settings (added 2026-10-15) — SIM build, serial only

Max speed, dwell and default radius are now `mission.json` settings edited
from the Waypoints tab. The logic was host-tested (real `DropLogicEngine.cpp`
against a fake position source, 17/17) but has **not run on the board**.
Drive it with the serial commands — timing-critical, so script it rather than
hand-type it. Each firing step consumes a waypoint (and a door per module):
`<lat> <lon>` below is always the waypoint `status` reports as `wp=`, taken
from `wp`. Four drops are needed, so start with all doors at 0 (reboot).

21. **Defaults survive an old mission.json.** Boot the new build on a board whose
    mission.json predates it. *Pass: `[CFG]` summary prints
    `Release : speed<=2.0 m/s dwell=0.0 s default r=5.0 m`; `rel` prints the same.*
22. **Dwell 0 = old behaviour.** `arm`, `simpos <lat> <lon>`, `simvel 1 0`.
    *Pass: fires at once, as step 11 did.* Then `stop`.
23. **Dwell counts.** `rel 2.0 2.0`, re-arm, `simpos`, `simvel 0.5 0`.
    *Pass: `status` shows `approach=SETTLING ... dwell=x/2.0`; `>>> ... after
    2.0s — executing drop` about 2 s after `settling 2.0s before release`.*
24. **Exit restarts it.** Same, but `simvel 3 0` for one second mid-count, then
    `simvel 0.5 0`. *Pass: SLOW DOWN, then a fresh 2.0 s before firing.*
25. **Speed limit is live.** `rel 0.5 0`, `simvel 1 0` → SLOW DOWN, holds;
    `rel 1.5 0` is refused while armed (`blocked`) — disarm, set, re-arm →
    fires. *Pass: both behaviours.*
26. **Log row.** Events-only flight log after any drop above. *Pass: a
    `release` row with dist, r, spd, max and dwell precedes the two `drop` rows.*
    (Needs room in the log — it was FULL on 2026-09-30.)

---

## 7. Results table (fill in at the bench)

| # | Test | Expected | Observed | Pass |
|---|---|---|---|---|
| 1 | Boot | 12 checkpoints, no reset | **PASS** 2026-08-25 | all 12, SIM confirms GPIO12/13 unattached |
| 2 | Dashboard renders | SIM banner, both tabs | | |
| 3 | ARM | ARMED, 2Hz | **PASS** 2026-08-25 | via `arm` serial cmd; LED rate NOT observed |
| 4 | DROP A door 1 | 580ms ±10%, 1 blip — read it straight off the `[ACT]` line, which now prints `target / actual / +/-drift` and flags `*** OVER 10% ***` itself (Phase 3e). No slo-mo needed for this one. | **PASS** 2026-08-25 | +0ms x3 runs; blip count NOT observed |
| 5 | `/status` during drop | no stall | | |
| 6 | Doors 2–4 | counters 2,3,4 | **PASS** 2026-09-01 | doorsA/B → 2,3,4; every window +0ms (A 580, B 350). Via `tools/bench_defect_c.py` |
| 7a | Door 5 no switch | TIMEOUT @1740ms | **PASS** 2026-09-01 | exactly as predicted: A `TIMEOUT (drove 1740 ms)`, B `TIMEOUT (drove 1050 ms)` — both 3× travel_ms. **See the hazard note below** |
| 7b | Door 5 switch hit | OK, early stop | | |
| 8 | Guard toast | `409 blocked` | | |
| 9 | E-STOP mid-drop | LED off; state? | **(b) CONFIRMED then FIXED** 2026-08-25 | was `state=3 dropping=1` 68ms after the stop; now `state=1 armed=0 dropping=0`, transition 3->1 direct. LED NOT observed |
| 10 | E-STOP mid-sequence | **B must NOT fire** | **(a) CONFIRMED then FIXED** 2026-08-25 | B fired a full 350ms window; after the fix B never starts, `doorsB=0`, waypoint stays pending |
| 11 | Auto-fire A→B | full sequence | **PASS** 2026-08-25 | A 580ms, B 350ms, logged, auto-disarm |
| 12 | Drop 6 of 10 | **must NOT log OK** | **PASS (with a caveat)** 2026-09-01 | run ends at WP5 with `MISSION COMPLETE`; WP6 produces **no `[DLE]` output at all**. No false OK, no B start, 5 waypoints left pending. But no `EMPTY` row either — `missionComplete()` short-circuits at line 73 before the guard at line 123, so the inner guard was never executed and is unreachable by construction. See PROJECT_HANDOFF.md PHASE 3g |
| B1 | RC frame rate | ~50 Hz, no glitch climb | | |
| B2 | RC full sweep | span ≥200µs | | |
| B3 | RC suggest | bands match switch positions | | |
| B4 | RC disconnect | `NO SIGNAL` ≤500ms | | |
| 13 | RC live readout | matches standalone tool | | |
| 14 | RC save + reboot | values persist | | |
| 15 | RC fire DISARMED | **must NOT fire** | | |
| 16 | RC fire ARMED | A→B pair, `dropIdx` unchanged | | |
| 17 | RC held at FIRE | exactly one pair | | |
| 18 | RC re-fire | second pair fires | | |
| 19 | RC cold start at FIRE | **must NOT fire** | | |
| 20 | RC signal loss | **must NOT fire** | | |
| 21 | Release defaults on old mission.json | 2.0 / 0.0 / 5.0 | | |
| 22 | Dwell 0 | fires at once | | |
| 23 | Dwell 2.0 s | SETTLING, fires at ~2.0 s | | |
| 24 | Speed spike mid-dwell | count restarts | | |
| 25 | Speed limit 0.5 → 1.5 | holds, then fires; blocked while armed | | |
| 26 | `release` log row | precedes the `drop` rows | | |

---

## 7b. HAZARD surfaced by the 2026-09-01 run — read before SIM_MODE goes to 0

Step 7a's `TIMEOUT @1740ms` was predicted here and observed exactly. What was
not drawn out is what it means once servos are attached.

The last door of each module waits for limit-switch confirmation. With the
switches on GPIO 32/33 unwired — which is how the bench has always been — it
falls through to a **3× travel_ms** timeout: module A drove **1740ms against a
580ms target**, module B **1050ms against 350ms**.

In SIM that is a log line. With servos connected it is the final door of every
module driving three times its intended travel, overshooting the cover by
roughly three door pitches. Same class as the servo-runaway incident.

**Wire the limit switches before flipping SIM_MODE to 0.** This is a
prerequisite for servo bring-up and was not previously listed as one.

---

## 8. After the run

1. Fix whichever of defects (a)–(d) the run confirms — these are flight-build
   bugs, not sim-mode bugs, and the fixes belong in the shared code path.
2. Re-run the affected steps to confirm the fixes.
3. Fold the results into `PROJECT_HANDOFF.md` (checklist items 2–7).
4. Only then return to the servo-side work: re-measure Module A's true stop,
   then characterise the door-to-door travel deviation across all 5 doors.
