# Design Rationale — Why the Launcher Runs on a Dedicated MCU, Not on the Flight Controller

**Question addressed:** the drone already carries a Cube Orange+ running ArduCopter, with spare
PWM outputs, spare GPIO, Lua scripting, and a GPS. Why was the Smart Launcher built as an
independent ESP32 module instead of implementing the drop logic directly on the flight
controller?

**Answer in one sentence:** the launcher is a *payload*, not an avionics function. Implementing it
on the Cube+ would have placed non-flight-critical, actively-debugged code inside the
flight-critical fault domain, and would have bound the launcher permanently to ArduPilot —
violating the project's core requirement that the module be host-platform agnostic.

Everything below elaborates that, and each argument is backed by an event that actually occurred
during development (see `PROJECT_HANDOFF.md` for the full engineering log).

---

## 0. The requirement this decision serves

From the project definition:

> A self-contained, platform-agnostic drone payload module that autonomously releases mangrove
> propagules at predefined GPS coordinates. **The launcher does ALL drop logic onboard. The host
> drone only provides power and flies a waypoint mission.** No DJI OSDK/PSDK. Works on a DJI M600
> or a DIY ArduPilot drone by changing config only.

Host-independence is not a nice-to-have that fell out of the implementation; it is the stated
requirement. Every argument below follows from taking it seriously.

---

## 1. Fault isolation — with evidence, not speculation

The strongest argument is not theoretical. Development produced five real defects, every one of
which would have been a flight-critical failure had the code been running on the Cube+:

| Defect (all real, all hit during development) | Symptom on the ESP32 | Same defect on the Cube+ |
|---|---|---|
| GPIO 2 (strapping pin) driven as an LED output | `rst:0x8` boot loop | Flight controller boot loop |
| Limit switches on GPIO 9/10 (tied to the WROVER flash/PSRAM SPI bus — unfixable in firmware) | Hard crash at `pinMode()` | Flight controller crash |
| Web handler blocked for several seconds | Watchdog reset (`TG1WDT_SYS_RESET`) | Scheduler starvation on the flight loop |
| Blocking servo drop loop | Froze the system for the whole drop | Froze the system for the whole drop |
| **`dropa` servo runaway** — servo kept rotating after the drop completed *and* after a full reboot | **Power to the servo had to be physically cut** | A runaway actuator on the computer holding the aircraft in the air |

Each was found by reflashing a spare development board on a bench with no aircraft present. Each was harmless
*specifically because* the launcher owns its own MCU.

The last row is the decisive one. The remedy for that fault was "unplug it." That is an acceptable
remedy for a payload sitting on a table. It is not an acceptable remedy for a flight controller.

This is not an unusual conclusion — it is standard aerospace practice to separate the payload
computer from the flight control system, and it is why DJI's own Payload SDK executes user code on
a *separate* onboard computer rather than allowing injection into the flight stack. The Smart
Launcher follows the same principle.

---

## 2. Host-platform independence

Placing the drop logic in the Cube+ binds it to ArduPilot. ArduPilot Lua does not run on a DJI
M600; that path requires DJI OSDK/PSDK — a different language, different hardware, a different
approval process, and a second complete implementation of the same drop logic.

The result would be two codebases, two calibration workflows, and two independent sets of bugs, for
one mechanism. The module as built is a single artefact that is indifferent to what is carrying it.

---

## 3. ArduPilot's built-in payload facilities are not sufficient — and the Lua path buys nothing

This must be argued honestly: implementing the launcher on the Cube+ is **technically feasible**.
ArduPilot offers `DO_SET_SERVO`, `DO_REPEAT_SERVO`, gripper / payload-place, and Lua scripting with
`SRV_Channels:set_output_pwm()` and GPIO access. The claim is not that it is impossible. The claim
is that the built-in mission items are far short of what this mechanism needs, and that the Lua
route incurs every cost of the dedicated MCU while surrendering its central benefit.

What this mechanism actually requires:

- A **continuous-rotation** servo whose true neutral is **1670 µs, not the textbook 1500 µs** —
  measured empirically, and *different for each of the two modules*.
- **Time-based** travel (`travel_ms = 580` for Module A), because a CR servo provides no positional
  feedback.
- **Cumulative state across five doors.** The cover retracts one way only; the firmware must always
  know which door it is on and must never lose that state.
- **Limit-switch-confirmed sequencing:** Module A fires, its limit switch confirms, *then* Module B
  fires — required to preserve airframe balance.
- **Non-blocking execution**, so an in-progress drop does not stall telemetry, the dashboard, or the
  state machine.

`DO_SET_SERVO` is a fire-and-forget mission item. It does not read a limit switch, does not carry
door state, does not sequence A-then-B on confirmation, and has no defined behaviour when a
continuous-rotation servo fails to stop.

To obtain any of that, one must write a Lua state machine — which is the same firmware that exists
today, only now running in a memory-limited sandbox, sharing a scheduler with the flight loop, on
the computer keeping the aircraft airborne. Same software complexity; strictly worse fault domain.

---

## 4. Bench testing without arming an aircraft

On the Cube+, servo-rail outputs are gated behind the safety switch and the arming state by
default. This is loosenable (`BRD_SAFETY_MASK`), but the default workflow for actuating a seed door
becomes: **arm a hexacopter.**

Calibration on this project took dozens of iterations (servo stop identification, open/close pulse
selection, the mark-run-measure-adjust `travel_ms` loop, five-door consistency checks). Under the
flight-controller architecture, every one of those iterations happens beside a live, armed airframe.

With a dedicated MCU, a servo and a rack are calibrated on a table with no aircraft in the room.
Given the `dropa` runaway (§1), this is not a matter of convenience.

---

## 5. The Phase 3 mission dashboard exists *because* of this decision

The Cube Orange+ has no Wi-Fi. The current field workflow — join the `SmartLauncher` access point
from a phone, open `192.168.4.1`, read live state / GPS status / door counters, execute a timed
servo run, use the *Apply Correction* helper to converge `travel_ms`, press the always-visible
**EMERGENCY STOP**, and persist tuned values directly to `mission.json` via `/savecal` — is possible
only because the payload carries its own radio and its own filesystem.

The flight-controller equivalent is a laptop, a USB cable or telemetry link, Mission Planner
parameter tables, and a script reload per iteration.

Worth stating plainly: to obtain a wireless configuration link on the Cube+ at all, one would bolt
on an ESP-based telemetry bridge. The "avoid an external microcontroller" position therefore ends up
adding an external microcontroller regardless — merely a dumb one.

---

## 6. Calibration data belongs with the mechanism, not the airframe

Development established that each servo has its own true stop value (Module A = 1670 µs; Module B
deliberately *not* assumed to mirror it), and that `travel_ms` depends on the backlash of one
specific rack-and-pinion assembly.

That is per-unit data about **this launcher**, not about **this drone**.

Holding it in the launcher's own `mission.json` means the calibration travels with the hardware it
describes: move the module to another aircraft and it remains correct. Held as flight-controller
parameters, the calibration would live with the airframe — to be re-entered on every drone swap or
FC reflash, with no mechanism to keep two launchers' calibration sets distinct on one controller.

---

## 7. Iteration cost and defensibility

- **Iteration cost.** Reflashing the ESP32 takes seconds over USB and risks nothing. Reflashing a
  flight controller is a heavier operation carrying a genuine bricking risk on a unit whose failure
  grounds the aircraft. Given how many flash cycles this project consumed, the cost of iteration is
  a real engineering variable, not a footnote.
- **Defensibility.** The aircraft runs **stock, unmodified ArduCopter 4.6.3**. Its flight behaviour
  is therefore a known, community-tested, reproducible quantity. "The flight controller runs stock
  firmware; the payload is an independent module" is a materially easier statement to defend — to a
  thesis committee, to a regulator, or in an incident review — than "the flight stack was modified."

---

## 8. Costs accepted (stated explicitly)

The decision is not free, and the debit column should be stated before anyone else states it:

| Cost | Assessment |
|---|---|
| One additional MCU, a UBEC, wiring, ~30–50 g | Negligible against a hexarotor's payload budget. |
| A second GNSS receiver (u-blox M8N) duplicating the Cube's GPS | Bounded, and already addressed by design — see below. |
| Two subsystems to power and maintain | Real, but the fault-isolation benefit is precisely what this buys. |

On the duplicated GPS: **Phase 4** specifies taking position from the Cube+ over MAVLink on TELEM2
(`SERIAL2_PROTOCOL=2`, `SERIAL2_BAUD=57`, `SR2_POSITION=4`), and the `IPositionSource` abstraction
exists so that this is a configuration change (`position_source: "GNSS" | "MAVLINK"`), not a rewrite.

Note the direction of that coupling, because it is the crux of the whole architecture: **the
launcher *consumes* data from the flight controller. It never *commands* through it, and it never
runs code on it.** That is the correct dependency direction for a payload, and it is available
without surrendering any of the properties argued above.

---

## Summary

| | Launcher on the Cube+ | Launcher on a dedicated MCU (chosen) |
|---|---|---|
| Fault domain | Payload bugs can take down the flight controller | Payload can crash, boot-loop, or run away; the aircraft still flies and lands |
| Host platforms | ArduPilot only (DJI needs a second implementation) | Any host that supplies power |
| Bench testing | Requires arming the aircraft (by default) | No aircraft required |
| Field configuration | Laptop + Mission Planner + script reload | Phone → Wi-Fi AP → web dashboard |
| Calibration data | Lives with the airframe | Lives with the mechanism |
| Flight firmware | Modified / heavily scripted | Stock ArduCopter 4.6.3 |
| Marginal cost | — | One MCU, a UBEC, ~30–50 g, one redundant GNSS |

The costs incurred are small, bounded, and fully characterised. The costs avoided — a flight-critical
fault domain that includes payload defects, and lock-in to a single flight stack — are neither.

**In short:** a drone does not put sprayer logic, winch logic, or gimbal logic in the flight stack;
payloads carry their own computer, and this is a payload. This particular payload also had a servo
run away on the bench and required its power to be physically cut. That is an event best confined to
a development board on a table, not to the computer holding a hexarotor in the air.
