# Measuring `T_EXIT_S` — bench procedure

**Status: WRITTEN, NOT RUN.** No trial has been taken. `T_EXIT_S` is still
`0.0f`.

Phase 4a, item (a) of the IMMEDIATE NEXT GOAL in `PROJECT_HANDOFF.md`.
Bench job. **No drone, no flight, no sky view.** One person, one phone, one
afternoon.

---

## 1. What is being measured, and what it is worth

`t_exit` is the time between the drop command and the propagule actually
leaving the module. `Ballistics.h::leadTime()` adds it to the fall time:

```cpp
inline float leadTime(float height_m, float vd_mps) {
    return T_EXIT_S + fallTime(height_m, vd_mps);
}
```

The propagule keeps the aircraft's horizontal velocity for the whole of
`leadTime`, not just for the fall. So `t_exit` is carry the aircraft applies to
the seed **before it is even in the air**, and it is currently modelled as zero.

The drive window is 580 ms on module A. If the seed leaves at the very end of
it, at 5 m/s that is **2.9 m** of unmodelled along-track error — comparable to
the 6.4 m the whole ballistics rewrite exists to remove, and nearly the entire
error budget in PHASE 4 RESEARCH. If it leaves at the very start, it is zero
and the constant should stay 0. The answer is somewhere in between and nobody
knows where.

`T_EXIT_S` is deliberately `0.0f` and not a guess, for the reason written into
`Ballistics.h`: a guessed value biases every drop in one direction while
looking authoritative in the log. **This procedure replaces it with a measured
number or leaves it alone.**

---

## 2. Preconditions — read all of this before plugging in the UBEC

This is the first procedure in the project that drives a **real servo**.
Everything to date ran with `SIM_MODE 1` and the UBEC unplugged.

### 2.1 `SIM_MODE` must go to 0

`Pins.h:100`. In SIM the servo pin is never `ledcAttach`ed, so GPIO 12/13 stay
high-impedance and nothing moves — which is exactly what makes SIM useless
here. `t_exit` is a mechanical quantity; it cannot be simulated.

### 2.2 The limit-switch hazard, and why doors 1–4 are safe anyway

`PROJECT_HANDOFF.md` PHASE 3g and the project memory both say: **wire the limit
switches before `SIM_MODE` goes to 0.** That instruction stands, and wiring
them (GPIO 32 / 33, `INPUT_PULLUP`, active LOW) is blocker item 2 in the
handoff regardless of this measurement. **Doing that first is the preferred
route** — it removes the hazard instead of steering around it, and it unlocks
plan step 7b at the same time.

If the switches are not wired yet, this measurement is still safe **provided no
trial ever fires the last door**, and that is true by construction rather than
by care. From `ActuatorModule::startDrop()`:

```cpp
_is_last_door = (door_num == _door_count);
_timeout_ms   = _is_last_door ? (_travel_ms * TIMEOUT_MULTIPLIER) : _travel_ms;
```

Doors 1–4 are purely timed: the window closes at `travel_ms` and the limit
switch is never read. Only door 5 gets `travel_ms * 3` and depends on the
switch to stop it early — which is why door 5 drove 1740 ms against a 580 ms
target in the `bench_defect_c.py` run.

**The rule, then: fire doors 1–4 only. Never door 5.**

- `status` on serial prints `doorsA=n doorsB=n`. **Read it before every trial.**
- Door counters are RAM only. A reboot (`bench_serial.py boot`) resets them to
  0.
- `tools/t_exit_reduce.py` rejects any trial row with `door >= 5` rather than
  averaging it in.

### 2.3 Do not let the autonomous path fire

`/dropa` requires the board armed, and an armed flight build with a valid GPS
fix will evaluate the drop radius every cycle. The shipped default mission's
waypoint 1 is `9.022251, -79.532240` with `radius_m = 5.0`. If the bench
happens to sit inside that circle, arming fires the **full A→B sequence**
unattended and will walk straight into door 5.

**Before arming, confirm on serial that `status` reports `pos valid=0`.**
Indoors with no sky view it will. If there is a fix, confirm you are not inside
any waypoint radius (`wp` prints the list) before going further.

### 2.4 Physical

- Servo UBEC (5 V / 5 A) plugged in and powering **only** the servo rail.
- **Keep the UBEC's plug in reach.** There has been a servo-runaway incident on
  this project before (see `git log`, `main`).
- `stop` on the serial console calls `emergencyStopAll()` and is available in a
  flight build — not SIM-gated. Have `bench_serial.py listen` open on the
  laptop so it can be typed instantly.
- The rack retracts one way. Re-cocking is a reverse jog from the Calibration
  tab, which is blocked while armed — so `disarm` first.

### 2.5 Payload

`t_exit` depends on the mass, shape and friction of what is sitting on the
cover. **Use real Rhizophora propagules.** If none are to hand, use a stand-in
of matched mass and length, and record the result as **provisional** in the
results table — do not set `T_EXIT_S` from a stand-in and then forget that is
what it was.

---

## 3. The rig

One camera, one shot, both events in frame:

- The **announce LED (GPIO 14, 330R to GND)** — it is lit for exactly the drive
  window and nothing else. `ActuatorModule::_writePWM()` sets it from
  `pulse_us != stop_pwm`, with no stretching and no speed multiplier, which is
  the property SIM mode was built around and the reason it is trustworthy here.
- The **drop aperture**, where the propagule leaves.

Notes that actually matter:

- **Put the LED physically next to the aperture, in the same horizontal band of
  the frame.** Phone sensors are rolling-shutter: the top of the frame is
  exposed up to a frame before the bottom. Two events at the same height cancel
  that skew; two events at opposite ends of the frame do not.
- **Do not let the LED bloom.** An over-exposed LED smears across neighbouring
  frames and stretches its apparent on-time by a frame or two. Mask it down,
  point it away from the lens, or drop the exposure. A dim, sharp LED is worth
  more than a bright one.
- Light the aperture hard. 240fps means a very short exposure; an underlit seed
  is a blur that cannot be read to a frame.
- Plain contrasting backdrop behind the aperture, so first movement is
  unmistakable.
- Camera on something rigid. The drop is 4 ms per frame; hand shake is not.

---

## 4. Why the frame rate never enters the answer

This is the part that makes the measurement solid, so it is worth stating
plainly.

The board prints the true width of each drive window on the completion line,
measured immediately after the stop write:

```
[ACT] Door 1 — timed drop OK (580 ms target, 580 ms actual, +0 ms)
```

That number has been verified exact — +0 ms across three consecutive runs on
both modules, after the TX-buffering fix. So the video does not need to supply
time at all. It only needs to supply a **ratio**:

```
t_exit = drive_ms × (release − led_on) / (led_off − led_on)
```

The frame rate cancels. That is not a nicety — phone "240fps" is routinely
239.76fps, and slo-mo clips are routinely exported into a 30fps container at 8×
slow. Anything that trusted the metadata could be wrong by a factor of eight
and still look plausible. Here the clip contributes counted frames and the
already-verified drive window contributes the seconds.

Practical consequence: **it does not matter whether you shoot 240fps or 120fps,
or whether the export is re-timed.** Shoot the highest rate the phone offers
and count frames.

---

## 5. Run procedure

Per rack load, four trials (doors 1–4).

1. **Flash** `SIM_MODE 0`.
   ```
   set ARDUINO_DIRECTORIES_DATA=C:/Users/carli/AppData/Local/Arduino15
   "C:/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe" \
       compile --fqbn esp32:esp32:esp32wrover --warnings default Servo_Web_Calibrator_WROVER
   ```
   The launcher is the **CH340**, and its port moves (COM6, COM10 seen). It is
   not the Cube Orange+.

2. **Boot and capture.** Leave this running for the whole session; it is the
   record of every `[ACT]` line.
   ```
   python tools/bench_serial.py boot 8
   python tools/bench_serial.py listen 600
   ```
   (Second window, or re-run `listen` between trials. Only one process may hold
   the port.)

3. **Load** the rack with propagules and cock the cover closed.

4. **Check the preconditions on serial**: `status` → `pos valid=0`,
   `doorsA=0 doorsB=0`.

5. **Join the laptop to the `SmartLauncher` AP** (no password) →
   `http://192.168.4.1`. This costs the laptop its internet for the session;
   that is expected. Serial and HTTP work simultaneously.

6. **Arm.** Serial `arm` is simplest and works in a flight build.

7. **Start recording** at the phone's highest frame rate. Let it run — the clip
   is cheap, a missed trigger is not.

8. **Fire one door.**
   ```
   curl "http://192.168.4.1/dropa?confirm=1"
   ```
   or the dashboard's **DROP A** button (two-tap confirm). Expect the occasional
   dropped connection: `server.handleClient()` runs inside a `loop()` that
   stalls 100–182 ms on `lfs_file_sync()`, so handshakes get refused. Retry.
   **This is not timing-critical** — unlike the e-stop test, nothing here
   depends on when the PC sends the command, because the timing reference lives
   entirely inside the video frame. A hand-tapped browser button is a perfectly
   good trigger.

9. **Stop recording.** Note which `[ACT]` line belongs to this clip.

10. **Repeat** for doors 2, 3, 4 — reload the aperture between each. **Stop at
    door 4.**

11. **After door 4:** `disarm`, reverse-jog the cover closed from the
    Calibration tab, reload, then `bench_serial.py boot` to reset the door
    counters, and go again from step 4.

12. **Repeat all of it for module B** with `/dropb?confirm=1`. B's window is
    350 ms, not 580 — the reduction reads the real value off the `[ACT]` line,
    so nothing needs changing, but expect fewer frames per trial and be
    correspondingly more careful reading them.

**Target: at least 8 trials per module** (two rack loads). Five is the minimum
the reduction script will accept without complaint.

---

## 6. Reading the frames

Extract frames rather than scrubbing a phone timeline — every frame gets an
index and nothing is judged by a progress bar:

```
ffmpeg -i trial_A1.mov -vsync 0 frames/A1_%05d.png
```

Then, per trial, record three indices:

| Field     | The frame where…                                                   |
|-----------|--------------------------------------------------------------------|
| `led_on`  | the announce LED **first** shows lit                               |
| `led_off` | it **first** shows dark again                                      |
| `release` | the propagule **first moves** — first frame of motion, not first frame of clear air |

`release` is the judgement call. Take first movement consistently; if you take
first-clear-air on some trials and first-movement on others, the spread you
measure is your own inconsistency, not the mechanism's.

Indices may start anywhere and need not be consecutive between rows — only
differences within a row are used.

Optional, if the LED edge is at all ambiguous: dump per-frame brightness of a
crop around the LED and take the edge off the numbers instead of the eye.

```
ffmpeg -i trial_A1.mov -vf "crop=40:40:X:Y,signalstats,metadata=print:file=-" -f null -
```

---

## 7. Reduction

Fill in `tools/t_exit_trials.csv`, one row per trial:

```
module,door,drive_ms,led_on,led_off,release
A,1,580,1412,1551,1496
```

`drive_ms` is the number **the board printed for that drop**, not the nominal
580.

```
python tools/t_exit_reduce.py
```

It reports per-trial `t_exit`, per-module mean/sd/min/max, where in the drive
window the seed leaves as a fraction, and what the correction is worth in
metres at 2 / 5 / 8 m/s. It rejects rows that cannot mean anything — door 5, a
release before the LED lit, a release after the window closed — rather than
averaging them in.

Check its own arithmetic first, against synthetic frames with a known answer:

```
python tools/t_exit_reduce.py --selftest
```

The implied-fps column is a sanity check only, never an input. If it reads
~240 the counting is consistent; if it reads ~30 the export was re-timed and
frames were skipped in extraction.

---

## 8. Acceptance

- **n ≥ 5 per module**, preferably 8.
- **sd ≤ 0.05 s.** At 5 m/s that is ±0.25 m of residual spread, inside the
  0.5–1 m of GPS position error already accepted. Above it, understand the
  spread before trusting a single constant — it usually means the seed is
  hanging up inconsistently, which is a mechanical finding worth more than the
  number.
- Every trial's `t_exit` inside its own drive window. A trial outside it is not
  measuring `t_exit`; discard it and say so.

---

## 9. What changes afterwards, and the trap waiting there

1. Set the constant in **both** places, or the two implementations silently
   diverge and the cross-check that has been catching errors stops working:
   - `Servo_Web_Calibrator_WROVER/Ballistics.h` — `static const float T_EXIT_S`
   - `tools/test_ballistics.py` — `T_EXIT_S`

2. **`tools/test_ballistics.py` will fail, and that is correct.** It checks the
   PHASE 4 RESEARCH fall-carry table through `carry()`, which now includes
   `t_exit`, against a table that is pure fall. Split it: keep the handoff table
   checked against fall time alone, and add separate cases for the total.
   **Do not widen the tolerance until it passes** — that deletes the only
   independent check this constant has.

3. Re-run `balltest` on the target. `fallTime` is unchanged; every `carry` and
   `dLat` expectation moves. Update the verification table in
   `PROJECT_HANDOFF.md`.

4. Note in the handoff that the `radius_m` conversation shifts again: with
   `t_exit` in `leadTime`, the predictor fires that much earlier along track.

---

## 10. What this measurement will surface, whatever the number

Two things are already visible from the code and the run will make them
concrete:

**Modules A and B have different drive windows** — 580 ms and 350 ms
(`MissionDefault.h`). If their `t_exit` values differ by more than the trial
noise, one global `T_EXIT_S` cannot describe both, and `Ballistics.h` has no
per-module concept today. The reduction script tests exactly this and says so.

**Module B fires only after module A's window has closed.**
`DropLogicEngine::update()` holds in `WAITING_A` until `_modA.isDropping()`
goes false, then calls `_modB.startDrop()`. So B's seed leaves roughly
`580 ms + t_exit_B` after the trigger while A's leaves at `t_exit_A` — both
from a **single** predicted impact point. At 5 m/s that is metres of separation
between two seeds meant for one waypoint, and **no value of `T_EXIT_S` fixes
it**, because it is a sequencing question, not a constant question. Worth
raising deliberately once the numbers are in; out of scope for this procedure.

---

## 11. Results

Fill in as trials are taken. Raw rows live in `tools/t_exit_trials.csv`.

| Date | Module | n | mean t_exit | sd | payload | notes |
|------|--------|---|-------------|----|---------|-------|
|      | A      |   |             |    |         |       |
|      | B      |   |             |    |         |       |

**Constant set to:** _(not yet — still `0.0f`)_
