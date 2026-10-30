#pragma once
#include <Arduino.h>

// Self-seeded default mission.json — written to LittleFS on first boot only
// if /mission.json doesn't already exist. Means a fresh flash needs no
// separate LittleFS-upload step (Arduino IDE has no bundled equivalent of
// PlatformIO's `pio run --target uploadfs`).
//
// IMPORTANT: Module A's stop_pwm_us here (1670) is the most recent value
// measured via this same calibrator toolchain — but per PROJECT_HANDOFF.md,
// do NOT trust it for a real drop without re-verifying fresh first. This is
// a reasonable starting point for the dashboard's calibration tab, not a
// promise it's still exactly correct. Module B has never been calibrated —
// its values are rough theoretical placeholders (see SMART_LAUNCHER_COMPLETE_GUIDE.md
// gear math) and MUST be measured with Servo_Calibration_Tool_B.ino /
// the dashboard's Calibration tab before use.
//
// The rc_trigger band values (1300/1700) are likewise nominal RC-PWM
// thresholds, NOT measured against the real A3 F-channel. Run
// RC_Input_Calibration_Tool.ino and write in what that reports before the
// pilot trigger is trusted to fire anything. It ships DISABLED: the
// F-channel -> GPIO 35 wire is not in the wiring guides (see RcTriggerConfig).
//
// The release block is the 2026-09-30 behaviour (2.0 m/s gate, no dwell);
// it is edited from the dashboard's RELEASE SETTINGS card.
const char MISSION_DEFAULT_JSON[] PROGMEM = R"json({
  "mission_name": "mangrove_plot_A",
  "min_gps_fix_duration_s": 10,
  "position_source": "GNSS",
  "modules": {
    "A": {
      "servo_open_pwm_us": 1470,
      "servo_stop_pwm_us": 1670,
      "travel_ms": 580,
      "door_count": 5,
      "reverse": false,
      "offset_us": 200
    },
    "B": {
      "servo_open_pwm_us": 1300,
      "servo_stop_pwm_us": 1500,
      "travel_ms": 350,
      "door_count": 5,
      "reverse": false,
      "offset_us": 200
    }
  },
  "rc_trigger": {
    "enabled": false,
    "safe_max_us": 1300,
    "fire_min_us": 1700,
    "frames_to_confirm": 5,
    "signal_timeout_ms": 500
  },
  "release": {
    "max_speed_mps": 2.0,
    "dwell_s": 0.0,
    "default_radius_m": 5.0
  },
  "hover": {
    "enabled": false,
    "min_flight_min": 0.25,
    "min_height_m": 5.0,
    "max_height_m": 15.0,
    "min_home_dist_m": 30.0,
    "max_speed_mps": 1.0,
    "max_vspeed_mps": 1.0,
    "hold_s": 5.0,
    "rearm_dist_m": 10.0,
    "takeoff_height_m": 3.0,
    "use_b": false
  },
  "drops": [
    { "id": 1, "lat_deg": 9.345678, "lon_deg": -79.876543, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 2, "lat_deg": 9.345700, "lon_deg": -79.876520, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 3, "lat_deg": 9.345722, "lon_deg": -79.876497, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 4, "lat_deg": 9.345744, "lon_deg": -79.876474, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 5, "lat_deg": 9.345766, "lon_deg": -79.876451, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 6, "lat_deg": 9.345788, "lon_deg": -79.876428, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 7, "lat_deg": 9.345810, "lon_deg": -79.876405, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 8, "lat_deg": 9.345832, "lon_deg": -79.876382, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 9, "lat_deg": 9.345854, "lon_deg": -79.876359, "alt_m": 8.0, "radius_m": 3.0 },
    { "id": 10,"lat_deg": 9.345876, "lon_deg": -79.876336, "alt_m": 8.0, "radius_m": 3.0 }
  ]
})json";
