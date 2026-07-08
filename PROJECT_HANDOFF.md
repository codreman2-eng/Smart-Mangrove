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

- Board settings NOW correct: ESP32 Wrover Module, COM10, 8MB flash, QIO, 80MHz,
  Default 4MB with spiffs, upload 921600.
- We stripped everything back to a MINIMAL serial-only servo sketch (servo_minimal.ino):
  no WiFi, no LED, no GPIO 2, no limit switches, no PROGMEM. Just ledcAttach on GPIO 12/13
  and serial commands (a+ a- a0 b+ b- b0, and "t NNN" for timed run).
- WAITING ON: user to flash servo_minimal.ino on the corrected board profile and report
  whether it (a) boots clean and controls servos, or (b) still boot-loops.
  - If it boots clean: base is solid -> add back limit switches, then web UI calibrator
    on a SAFE led pin (GPIO 22, NOT GPIO 2).
  - If it still loops: not a code issue -> do full flash erase
    (Tools -> Erase All Flash Before Sketch Upload -> Enabled, flash once, set back).

---

## IMMEDIATE NEXT GOAL

Get a stable servo calibrator running so the user can find:
  (a) true servo STOP/neutral PWM (may not be exactly 1500us), and
  (b) travel_ms = time to move exactly 35mm (one door), via mark-run-measure-adjust.
User wanted a WEB UI for calibration, but boot instability forced a fall back to serial.
Once the minimal serial sketch is confirmed stable, rebuild the web UI calibrator carefully
(chunked HTML, non-blocking timed runs, safe LED pin, correct WROVER board profile).

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
