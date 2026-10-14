#pragma once

// Smart Launcher — Mission Dashboard + Calibrator (Phase 3)
// Board: Freenove ESP32-WROVER
//
// Arduino IDE has no equivalent of PlatformIO's `build_flags -D...`, so
// pins live here as plain #defines instead. Values are the ones validated
// on real hardware this session — do not change without re-running the
// checkpoint-print bisection that found the two unsafe pins below.
//
// NEVER USE:
//   GPIO 2      — strapping pin, boot-loops (rst:0x8 TG1WDT_SYS_RESET) if
//                 driven as OUTPUT during/around boot.
//   GPIO 9/10   — permanently wired to the shared flash+PSRAM SPI bus on
//                 WROVER modules (unconditionally, regardless of flash
//                 mode). Confirmed by live serial bisection: touching
//                 GPIO 9 as INPUT_PULLUP crashes the board instantly.

#define PIN_SERVO_A   12
#define PIN_SERVO_B   13
#define PIN_LIMIT_A   32   // NOT GPIO 9
#define PIN_LIMIT_B   33   // NOT GPIO 10

// Limit switches on GPIO 32/33 physically connected? While 0, the last door
// of each module is driven for travel_ms exactly like doors 1..N-1 and the
// switch is never read. An unwired pin reads HIGH (INPUT_PULLUP) forever, so
// with 1 and no switch the last door ran to its 3x timeout — measured at
// 1740 ms against a 580 ms target on 2026-09-01. Set 1 only once both
// switches are wired and seen to go LOW when pressed.
#define LIMIT_SWITCHES_WIRED 0
#define PIN_LED       22   // NOT GPIO 2 — plain 2-pin LED, not WS2812
#define PIN_GNSS_RX    4
#define PIN_GNSS_TX    5

// ─── RC PWM input — pilot trigger from the M600's A3 Pro F-channel ────
//
// GPIO 35 is INPUT-ONLY (34-39 are): it physically cannot be configured as
// an output, so no firmware bug can ever back-drive the flight controller.
// That is the reason for the choice, not convenience.
//
// CATCH: input-only pins have NO internal pull-up/pull-down. An external
// 10k pull-down from GPIO35 to GND is REQUIRED, otherwise a disconnected
// signal wire floats and reads as noise instead of "no signal".
//
// VOLTAGE: the A3's F-channel should be 3.3V logic — MEASURE IT before
// connecting. If it reads 5V, fit a 10k/20k divider or a level shifter;
// the ESP32 is not 5V tolerant. Common ground with the M600 is mandatory.
//
// Bring-up/calibration: use RC_Input_Calibration_Tool.ino (repo root)
// before trusting this path in the dashboard firmware.
#define PIN_RC_IN     35

// ─── DJI OSDK API port (M600 A3 Pro) — UART2, read-only ───────────────
//
// NOT GPIO 16/17. Serial2's DEFAULT pins are 16/17, and on WROVER modules
// those are the PSRAM bus — the same trap as GPIO 9/10. The pins must be
// passed explicitly to Serial2.begin(). UART0 is the console and UART1 is
// the GNSS (GPIO 4/5), so OSDK gets UART2.
//
//   A3 API TX -> GPIO 25   (330R series optional)
//   A3 API GND-> ESP32 GND (MANDATORY)
//   A3 API RX / VCC: NOT CONNECTED — this link is receive-only by design
//
// Both sides are 3.3V; no level shifter. 921600 baud as set in DJI
// Assistant 2. Trimming the broadcast there to POSITION+GPS+RC+STATUS cuts
// the frame from 231 to ~60 bytes; the decoder computes field offsets from
// the passFlag, so that change is safe.
#define PIN_OSDK_RX   25
#define PIN_OSDK_TX   26   // wired for future use; never driven today
#define OSDK_BAUD     921600

// ─── Indicator LEDs ───────────────────────────────────────────────────
//
// Yellow "WAIT" LED — system-state semaphore, present in BOTH sim and
// flight builds. GPIO 27: free, not a strapping pin, no boot-time glitch,
// and physically adjacent to GPIO 14 on the Freenove WROVER header so the
// two indicator LEDs share one ground rail.
//
//   IDLE      slow blink   — powered, not armed
//   ARMED     STEADY ON    — armed, waiting for a trigger  <- the "wait" state
//   DROPPING  off          — the announce LED has the floor
//   FAULT     rapid blink
//
// NOT wired to EN/CHIP_PU: EN is the chip's reset *input*, not a GPIO —
// firmware cannot drive it. An LED there would also clamp EN to ~2.05V
// against its 10k pull-up, below the ~2.475V V_IH threshold, so the chip
// may never leave reset; it would additionally load the EN reset-RC cap
// and break auto-reset during flashing.
#define PIN_WAIT_LED  27   // yellow, 330R to GND
// Wire: GPIO -> 330R -> LED anode, cathode -> GND (~4mA at 3V3).

// ─── SIM MODE ─────────────────────────────────────────────────────────
//
// 1 = dry run. The actuator's output stage is swapped for a single LED on
// GPIO 14; ledcAttach() is NEVER called on the servo pins, so no PWM is
// generated at all — the servo signal lines stay high-impedance. This is a
// compile-time guarantee on purpose: a runtime toggle could be left in the
// wrong position, and this project has already had one servo-runaway
// incident. ALSO UNPLUG THE SERVO UBEC for sim runs.
//
// Everything upstream of ActuatorModule::_writePWM() — drop timing, state
// machine, door counting, limit switches, DropLogicEngine, every HTTP
// handler — runs byte-identically to the flight build. That is the point:
// a separate "SimActuator" class would mean validating code that never
// flies. See SIM_MODE_VALIDATION_PLAN.md.
//
// 0 = flight build. Real servo PWM on GPIO 12/13.
//
// The committed default is 1 so that anyone flashing a fresh clone gets the
// safe build. The 2026-09-30 flight flash was built from 22d753a with 0 here;
// set 0 only for a supervised flight build, and set it back before committing.
#define SIM_MODE 1

// Drop announce LED. Lit for exactly the window the servo is (or would be)
// driven, then blips out which door just dropped.
//
// ACTIVE IN BOTH SIM AND FLIGHT BUILDS. In SIM it is the only output; in a
// flight build it runs alongside the real servo PWM. That makes the LED
// behaviour identical in both, so what you validate on the bench is what
// you see on the aircraft — and it gives a visible drop confirmation during
// ground checks without needing the dashboard.
//
// GPIO 14: not a strapping pin (0/2/5/12/15 are), not on the WROVER
// flash/PSRAM bus (9-11). Caveat: the boot ROM emits a brief PWM burst on
// GPIO 14, so a flicker at power-on before checkpoint [1] prints is
// EXPECTED, not a fault. Also JTAG MTMS — only matters if a hardware
// debugger is ever attached to this board.
#define PIN_DROP_LED  14   // 330R to GND

// Confirm-flash patterns after a drop completes — how the two modules are
// told apart on a single shared LED. Non-blocking (millis-driven).
#define DROP_FLASH_LEAD_MS   200   // dark gap between drive window and confirm
#define DROP_FLASH_A_ON_MS    80   // Module A: short blips
#define DROP_FLASH_A_OFF_MS  120
#define DROP_FLASH_B_ON_MS   400   // Module B: long blips
#define DROP_FLASH_B_OFF_MS  200

// No external ARM button wired — only the board's onboard EN/RST and
// BOOT/IO0 buttons exist. GPIO0 is the BOOT button; holding it 2s after
// boot (not during) is a standard safe ESP32 pattern, kept here as a
// secondary/best-effort ARM path. The dashboard's ARM button is primary.
// If this ever causes a conflict during hands-on testing, remove it —
// don't fight it to keep it working.
#define PIN_ARM_BUTTON 0

#define SERVO_FREQ_HZ   50
#define SERVO_RES_BITS  16
