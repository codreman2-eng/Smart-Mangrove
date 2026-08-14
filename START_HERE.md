# START HERE — Smart Launcher, for someone new

Written 2026-08-13 for a student taking the project over for a week.

This assumes **no prior knowledge of this project**. It does assume you can
use a computer and are willing to be careful. Read section 1 before you plug
anything in.

If you want the deep history and the reasoning behind every decision, that
lives in `PROJECT_HANDOFF.md`. **Read this file first** — the handoff is
dense and written for someone already up to speed.

---

# 1. Safety — read before touching anything

These are not formalities. Every one of these has already bitten this
project or is a known hazard.

### The big one: the launcher is a machine that moves

Right now the firmware is in **SIM mode**, which means the servos are
electrically disconnected in software and a **LED lights instead of the
motors turning**. That is the safe state. **Leave it that way** unless
someone experienced is with you.

- **Keep the servo power supply (the UBEC) UNPLUGGED.** Even in SIM mode.
  Belt and braces.
- The seed cover only moves **one way**. If it runs, it does not go back
  without taking the mechanism apart. A mistake here costs real work.
- There has already been one incident where a servo ran continuously and
  would not stop. The power had to be physically pulled. That is why we
  test with LEDs.

### Electrical rules

- **The ESP32 is a 3.3 V device and is NOT 5 V tolerant.** Connecting a 5 V
  signal directly to any of its pins can destroy it.
- The Arduino Mega is a **5 V** device. Anything going from the Mega to the
  ESP32 needs a voltage divider (see §4).
- **Common ground is mandatory.** Any two boards that exchange a signal must
  have their GND pins connected. Signals without a shared ground do not
  work, and the failure looks like a software bug.
- Never connect two outputs (two TX pins) together.

### Aircraft rules

- **Propellers OFF** whenever the M600 is powered on a bench. Always. No
  exceptions, not even "just for a second".
- Do not fly anything. Nothing in this project is flight-validated yet.

---

# 2. What this project actually is

A drone drops mangrove seeds to replant mangrove forests.

The **Smart Launcher** is a box that hangs under the drone. It holds seeds
behind 5 sliding doors per module, and there are 2 modules (A and B) mounted
symmetrically so the drone stays balanced.

**The key idea:** the launcher does all its own thinking. It has its own
computer (an ESP32), its own GPS, its own battery feed. The drone just flies
and provides power. This means the same launcher works on a DJI M600 or on a
home-built drone, by changing a config file rather than rewriting software.

When the launcher decides it has arrived at a drop point, it runs a servo
motor which slides the cover across by one door width (35 mm), and a seed
falls out by gravity. Module A fires first, waits for confirmation, then
Module B fires. That order keeps the aircraft balanced.

### How it knows where it is

Three possible sources, in order of preference:

1. **OSDK** — reads position directly from the DJI M600's flight controller
   over a serial cable, 50 times per second. This is the best one and it
   works today.
2. **GNSS** — its own small GPS module (a u-blox M8N). Backup.
3. **Simulated** — a fake position you type in, for testing indoors.

### How it decides to drop

Two ways:

1. **Automatically** — you save a list of waypoints (GPS coordinates). When
   the launcher gets within a set radius of one, it drops.
2. **Pilot trigger** — the pilot flips a switch on the remote control and
   the launcher drops. This is built but has never been tested end to end.

---

# 3. The hardware — what each thing is

| Board | What it does | Where |
|---|---|---|
| **ESP32 WROVER-E** | The launcher's brain. Runs everything. | The main board |
| **Arduino Mega 2560** | A test tool. Generates a fake RC signal. | Bench only, never flies |
| **FTDI FT232 adapter** | A USB-to-serial cable, for listening to the DJI. | Bench only |
| **DJI M600 / A3 Pro** | The drone and its flight controller. | — |
| **FS5106R servos** | The motors that slide the seed covers. | Currently disconnected |

### ESP32 pin assignments — memorise the "never use" list

| Pin | Used for |
|---|---|
| GPIO 12 | Servo A signal |
| GPIO 13 | Servo B signal |
| GPIO 32 | Limit switch A |
| GPIO 33 | Limit switch B |
| GPIO 22 | Status LED (blinks a pattern per state) |
| GPIO 27 | Yellow "waiting" LED |
| GPIO 14 | Drop announce LED |
| GPIO 35 | RC pilot trigger input (input-only pin) |
| GPIO 25 | DJI OSDK serial in |
| GPIO 26 | DJI OSDK serial out (wired, never used) |
| GPIO 4 / 5 | GPS module |

### ⛔ NEVER USE THESE PINS

- **GPIO 2** — the board refuses to boot if you drive it. Cost a debugging session.
- **GPIO 9, 10** — physically wired to the memory chip inside the module. Cost another.
- **GPIO 16, 17** — same problem, also wired to memory. Found before it cost anything.

If you need a spare pin, use **18, 19, 21, 23** — those are free and safe.

The authoritative list, with reasons, is in
`Servo_Web_Calibrator_WROVER/Pins.h`. Read it before wiring anything.

---

# 4. Wiring that already exists

### The three LEDs

Each LED goes: **ESP32 pin → 330 Ω resistor → LED long leg (+) → LED short
leg (−) → GND.**

| LED | Pin | What it tells you |
|---|---|---|
| Status | 22 | Blink pattern = what state the system is in |
| Yellow "wait" | 27 | **Solid on** = armed and waiting for a waypoint |
| Drop announce | 14 | Lights when a drop fires |

### DJI OSDK cable (position data)

```
A3 "API" port TX  →  [330 Ω]  →  ESP32 GPIO 25
A3 "API" port GND →              ESP32 GND       ← mandatory
A3 API RX / VCC   →  NOT CONNECTED
```

Both sides are 3.3 V so no converter is needed. **This link is
receive-only** — the launcher listens, it never sends commands to the
aircraft. That is a deliberate design rule, not a limitation.

### Arduino Mega test signal (only when testing the RC input)

The Mega is 5 V, so this **needs a divider**:

```
Mega pin 11 ──[ 1 kΩ ]──┬────── ESP32 GPIO 35
                        │
                     [ 2 kΩ ]
                        │
Mega GND ───────────────┴────── ESP32 GND
```

**Use 1 kΩ / 2 kΩ. Do NOT use 10 kΩ / 20 kΩ** — there's already a 10 kΩ
resistor on GPIO 35, and the combination drops the voltage too low to
register. It looks exactly like a broken wire.

---

# 5. Setting up your computer

### Install Arduino IDE

Version 2.x. It is already installed on the project machine at
`C:\Program Files\Arduino IDE\`.

### Install ESP32 board support

`File → Preferences → Additional board manager URLs`, add:

```
https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

Then `Tools → Board → Boards Manager`, search **esp32**, install it.
The project machine has **version 3.3.8** — use that if you can.

### Board settings — these matter

`Tools →` set:

| Setting | Value |
|---|---|
| Board | **ESP32 Wrover Module** |
| Upload Speed | 921600 |
| Flash Frequency | 80 MHz |
| Partition Scheme | Default 4MB with spiffs |

**"ESP32 Wrover Module" is not optional.** Choosing "ESP32 Dev Module"
misconfigures the memory chip and the board crashes on boot. This already
happened once.

### Turn compiler warnings on

`File → Preferences → Compiler warnings → Default` (or higher).

The code contains a deliberate warning that fires when you build a
**flight** version, reminding you that three known bugs are still unfixed.
At the IDE's default setting of "None" you would never see it.

---

# 6. Finding your COM port

**The port number changes.** Unplugging and replugging can move the ESP32
from COM6 to COM11 to anything else. Never assume.

1. Plug in the board.
2. Arduino IDE → `Tools → Port`. Take note of what appears.
3. Unplug the board. Look again. The one that disappeared is yours.

Recently observed on the project machine (do not trust these):

| Device | Was on |
|---|---|
| ESP32 WROVER | COM6 |
| Arduino Mega | COM8 |
| FTDI adapter | COM11 |

**Only one program can use a port at a time.** If uploading fails with
"Access denied" or "port busy", the Serial Monitor is almost certainly still
open. Close it and try again. This has caught us three times.

---

# 7. Flashing (uploading code to a board)

### The main firmware — the dashboard

1. Open `Servo_Web_Calibrator_WROVER/Servo_Web_Calibrator_WROVER.ino`
2. Check `Pins.h` line with `#define SIM_MODE` — **it must say `1`**
3. Set the board and port (§5, §6)
4. Click the **→** arrow (Upload)
5. Wait for `Hash of data verified` and `Hard resetting`

### Confirm it worked

Open the Serial Monitor (magnifier icon, top right), set to **115200 baud**.
Press the board's **EN/RST** button. You should see twelve numbered steps:

```
=== Smart Launcher Mission Dashboard ===
[1] Config (LittleFS + mission.json, self-seed if missing)
[2] Logger
[3] State machine (LED GPIO22, ARM button GPIO0 best-effort)
[4] Position source (SIM — real OSDK/GNSS until /simpos injects)
[5] Actuators *** SIM MODE — announce LED GPIO14, servo GPIO12/13 NOT attached ***
[6] Drop logic engine
[6b] RC pilot trigger (GPIO35 ...)
[7] WiFi AP
[8] IP: 192.168.4.1
[9] HTTP server
[10] BOOT -> IDLE
[11] Ready — connect to SmartLauncher -> http://192.168.4.1
[12] setup() complete — loop starting
```

**If you see `rst:0x8 (TG1WDT_SYS_RESET)` repeating, the board is
crash-looping.** Stop and read §10.

Line `[5]` saying **"servo GPIO12/13 NOT attached"** is your proof that SIM
mode is active and nothing can physically move.

### The other programs (test tools, not the main firmware)

- `RC_Input_Calibration_Tool.ino` — ESP32. Measures an RC signal.
- `RC_Pulse_Generator_Mega2560.ino` — **Arduino Mega**, board setting
  `Arduino Mega or Mega 2560`. Generates a test RC signal.
- `tools/osdk_sniff.py` — runs on the **PC**, not a board. Needs Python and
  `pip install pyserial`.

---

# 8. Using the dashboard

1. Power the ESP32.
2. On your phone or laptop, join the Wi-Fi network **`SmartLauncher`**
   (no password).
3. Browse to **`http://192.168.4.1`**

You will lose internet while connected to it. That is normal — it is a
standalone network from the board itself.

You should see an orange banner reading **SIM MODE — NO SERVO OUTPUT**. If
that banner is missing, the firmware can drive real motors. Stop and check
`SIM_MODE`.

### The three tabs

**Control** — arm/disarm, manual drops, the emergency stop, the flight log.

- **ARM** requires two taps (it asks you to confirm).
- **EMERGENCY STOP** is the red bar fixed at the bottom. One tap, always
  available, never asks for confirmation.
- **worst loop** shows how long the software's slowest cycle took. Under a
  few ms is healthy. Hundreds of ms means something is stalling.

**Waypoints** — where you plan the mission.

- **ADD WAYPOINT HERE** captures wherever the launcher currently is. Stand
  at the spot, tap once. It refuses if there is no GPS fix.
- Radius is how close you must get before it drops. **Use 10 m for
  testing**, not 3 — you are testing logic, not fighting GPS accuracy.
- Edits are in memory only until you press **Save**.

**Calibration** — servo tuning and RC trigger setup. **Do not use the servo
controls** unless someone has told you to; they drive real motors in a
flight build.

---

# 9. Where the project actually stands

### What works and has been proven on real hardware

- The dashboard runs, all three tabs render.
- Reading position from the DJI M600 at ~47 updates/second. Verified
  outdoors with a real GPS fix.
- The drop logic fires correctly when a waypoint is reached — tested by
  walking to a waypoint and watching the LED.
- The flight log records events to a file on the board.
- The RC-signal reading tool works against a test signal generator.

### What is built but NOT proven

- The pilot RC trigger has never been tested end to end with a real remote.
- The servos have never been driven by this firmware. Not once.
- Module B's hardware does not exist yet.
- Nothing has ever flown.

### ⚠ Three known bugs are still in the code ON PURPOSE

They were found by reading the code but never confirmed by running it. They
were deliberately left in so the next test run can prove or disprove them
rather than someone "fixing" a problem that may not exist.

1. **Emergency stop may fire Module B.** If you press stop during an
   automatic sequence, module B might start anyway.
2. **Emergency stop may not disarm.** It might leave the system ARMED.
3. **10 waypoints vs 5 doors.** If a mission has more waypoints than doors,
   the extra ones report success without anything happening.

**Do not connect servos until these are resolved.** The Waypoints tab warns
you about #3 automatically.

### Immediate housekeeping

**The flight log is FULL** (it hit its 1.1 MB limit) and has stopped
recording. Go to the Control tab and press **Clear** twice. Do this first —
nothing is being logged until you do.

---

# 10. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| Upload fails, "Access denied" / "port busy" | Serial Monitor is open. Close it. |
| Upload fails, "port not found" | Port number changed. Recheck `Tools → Port`. |
| `rst:0x8` looping at boot | Wrong board selected, or something wired to GPIO 2 / 9 / 10 / 16 / 17. |
| Serial prints garbage | Baud rate wrong. Set 115200. |
| Dashboard won't load | Are you on the `SmartLauncher` Wi-Fi? Address is `192.168.4.1`, no https. |
| "no fix" outdoors | Check `[OSDK]` lines on serial. `frames=0` means the cable is wrong or the A3 is off. Frames but `health` under 3 means the drone hasn't locked onto satellites yet — wait a minute. |
| Drop doesn't fire | Is it ARMED? Is a waypoint saved? Is the radius big enough? Is the waypoint actually near you? |
| No drops in the log | Tick "Events only". Telemetry writes one row per second and buries them. |
| Everything feels laggy | Check "worst loop". If it's high, the log is probably full — clear it. |

### Reading the LEDs

| Yellow (27) | Announce (14) | Meaning |
|---|---|---|
| slow blink | off | Idle, not armed |
| **solid** | off | Armed, waiting |
| off | **on ~0.6 s** | Module A firing |
| off | 1 short blip | Module A door confirmed |
| off | **on ~0.35 s** | Module B firing |
| off | 1 long blip | Module B door confirmed |
| rapid blink | off | Fault |

Short blips = Module A, long blips = Module B. That's how you tell them
apart on one LED.

---

# 11. What to do next

In this order. Do not skip ahead.

1. **Clear the flight log.** It's full.
2. **Re-run the walk test** with the log cleared, and check whether the
   lag we were chasing is gone. Watch "worst loop".
3. **Run the emergency-stop tests.** Arm, trigger a drop, and press
   EMERGENCY STOP while the announce LED is lit.
   - If the LED lights again afterwards → bug #1 is real. Write it down.
   - If the state stays ARMED → bug #2 is real. Write it down.
4. **Record the results in `SIM_MODE_VALIDATION_PLAN.md`** — it has a table
   with blank cells waiting for exactly this.

`SIM_MODE_VALIDATION_PLAN.md` is the test script. It has 20 numbered steps
with a pass criterion for each. **Some steps are written so that "it fired"
is a FAILURE** — those are the ones testing that something must *not*
happen. Read the criterion, don't assume.

---

# 12. Rules

**Do:**
- Keep `SIM_MODE 1` unless supervised
- Keep the servo UBEC unplugged
- Write down what you observe, even when it works
- Ask before changing pin assignments

**Don't:**
- Don't connect servos
- Don't fly anything
- Don't "fix" the three known bugs before testing them — the test is the point
- Don't use GPIO 2, 9, 10, 16, 17
- Don't merge the git branch into `main`

### Git

Work is on branch **`sim-mode-and-rc-trigger`**, pushed to GitHub. It is
deliberately **not merged** into `main` because of the three known bugs.

```bash
git status                  # what have I changed?
git add -A                  # stage everything
git commit -m "message"     # save a checkpoint
git push                    # send to GitHub
```

Commit whenever something works. It is free and it means you can always get
back.

---

# 13. Glossary

| Term | Meaning |
|---|---|
| **ESP32** | The small computer running the launcher |
| **Flash / upload** | Copying your program onto a board |
| **Serial Monitor** | A window showing text the board prints. Debugging lifeline. |
| **Baud** | Speed of the serial connection. Ours is 115200. |
| **COM port** | Windows' name for a connected board (COM6, COM8...) |
| **GPIO** | A numbered pin on the ESP32 |
| **PWM** | A pulse whose *width* carries a value. Used for servos and RC. |
| **UART / serial** | Two wires sending data one bit at a time |
| **OSDK** | DJI's Onboard SDK — how we read position from the M600 |
| **LittleFS** | A tiny filesystem on the ESP32 that stores `mission.json` and the log |
| **mission.json** | Config file on the board: waypoints, servo values, RC settings |
| **JSONL** | The log format — one JSON record per line |
| **Waypoint** | A GPS coordinate where a seed should drop |
| **Arm** | Switch the system to a state where it is allowed to drop |
| **UBEC** | A small power supply that feeds the servos 5 V |
| **Haversine** | The formula for distance between two GPS coordinates |
| **SIM mode** | Safe test mode — LED instead of servos |

---

# 14. Document map

| File | What it's for |
|---|---|
| **`START_HERE.md`** | This file. Read first. |
| `SIM_MODE_VALIDATION_PLAN.md` | The test script. **This is the next work.** |
| `PROJECT_HANDOFF.md` | Full history and reasoning. Dense but authoritative. |
| `DESIGN_RATIONALE.md` | Why the launcher runs its own computer |
| `SMART_LAUNCHER_COMPLETE_GUIDE.md` | Wiring, parts list, mechanical build |
| `Servo_Web_Calibrator_WROVER/` | **The main firmware** |
| `Servo_Web_Calibrator_WROVER/Pins.h` | Pin assignments — the authority |
| `RC_Input_Calibration_Tool.ino` | Test tool: measures an RC signal |
| `RC_Pulse_Generator_Mega2560.ino` | Test tool: makes a fake RC signal |
| `tools/osdk_sniff.py` | PC tool: decodes DJI telemetry |

---

## If you get stuck

Write down: what you did, what you expected, what happened, and anything the
Serial Monitor printed. That single habit has solved most of the hard
problems on this project — several bugs were found purely because someone
noticed a line that *should* have been there and wasn't.

Nothing here is so fragile that careful work will break it. The one thing
that genuinely cannot be undone is running the servos, and that is why they
are unplugged.
