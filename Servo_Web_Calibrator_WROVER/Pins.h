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
#define PIN_LED       22   // NOT GPIO 2 — plain 2-pin LED, not WS2812
#define PIN_GNSS_RX    4
#define PIN_GNSS_TX    5

// No external ARM button wired — only the board's onboard EN/RST and
// BOOT/IO0 buttons exist. GPIO0 is the BOOT button; holding it 2s after
// boot (not during) is a standard safe ESP32 pattern, kept here as a
// secondary/best-effort ARM path. The dashboard's ARM button is primary.
// If this ever causes a conflict during hands-on testing, remove it —
// don't fight it to keep it working.
#define PIN_ARM_BUTTON 0

#define SERVO_FREQ_HZ   50
#define SERVO_RES_BITS  16
