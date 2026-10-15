#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// Per-module actuator settings loaded from mission.json
struct ModuleConfig {
    uint16_t servo_open_pwm_us  = 1700;   // PWM to run servo (open direction)
    uint16_t servo_stop_pwm_us  = 1500;   // PWM to stop servo (neutral)
    uint32_t travel_ms          = 350;    // ms to travel exactly one door (35mm)
    uint8_t  door_count         = 5;      // doors per module
    bool     reverse            = false;  // flip open/close direction (per-unit quirk)
    uint16_t offset_us          = 200;    // open/close = stop +/- offset
};

// Mirrors the calibrator's proven client-side JS formula exactly
// (opensHigh(mod) = (mod==='A') === isRev(mod)) so server-side automatic
// drops (triggered by DropLogicEngine on GPS radius, no browser involved)
// compute the identical OPEN/CLOSE pulse the dashboard's UI would show.
inline bool opensHigh(bool isModuleA, bool reverse) {
    return isModuleA == reverse;
}
inline uint16_t computeOpenPwm(bool isModuleA, uint16_t stop, uint16_t offset, bool reverse) {
    int v = opensHigh(isModuleA, reverse) ? (int)stop + (int)offset : (int)stop - (int)offset;
    return (uint16_t)constrain(v, 1000, 2000);
}
inline uint16_t computeClosePwm(bool isModuleA, uint16_t stop, uint16_t offset, bool reverse) {
    int v = opensHigh(isModuleA, reverse) ? (int)stop - (int)offset : (int)stop + (int)offset;
    return (uint16_t)constrain(v, 1000, 2000);
}

// One drop waypoint
struct DropPoint {
    uint8_t  id;
    double   lat_deg;
    double   lon_deg;
    float    alt_m;
    float    radius_m;
    bool     completed = false;
};

// Position source selection
enum class PositionSource { GNSS, MAVLINK };

// Pilot RC trigger (M600 A3 Pro F-channel) — SUPPLEMENTS GPS waypoint
// auto-fire, does not replace it. Calibrate with RC_Input_Calibration_Tool.ino.
//
// DISABLED by default since 2026-10-15: the F-channel -> GPIO 35 wire was
// taken out of the wiring guides, and an unwired input-only pin has no pull
// resistor — it floats, in a box carrying 50 Hz servo PWM. Enable it from the
// Calibration tab only once the wire and its 10k pull-down are fitted.
struct RcTriggerConfig {
    bool     enabled           = false;
    uint16_t safe_max_us       = 1300;   // <= this is SAFE
    uint16_t fire_min_us       = 1700;   // >= this is FIRE
    uint8_t  frames_to_confirm = 5;      // consecutive frames to accept a band
    uint16_t signal_timeout_ms = 500;    // no valid frame for this long = no signal
};

// When an ARMED engine actually fires at a waypoint. Edited from the
// dashboard (Waypoints tab -> RELEASE SETTINGS), persisted in mission.json
// under "release". An absent block gives these defaults, which reproduce the
// behaviour flashed on 2026-09-30 (22d753a): 2.0 m/s gate, no dwell.
//
// The release fires once the PREDICTED IMPACT point (aircraft position when
// there is no velocity) has been inside the waypoint's radius_m, at or below
// max_speed_mps, continuously for dwell_s.
struct ReleaseConfig {
    // Horizontal speed at or below which a release is allowed. Needs OSDK
    // velocity; on the GNSS fallback there is no speed and the gate is open.
    float max_speed_mps    = 2.0f;
    // Seconds the in-radius-and-slow condition must hold, ARMED, before the
    // drop starts. 0 = fire on the first qualifying sample. A hover drop wants
    // 1-2 s so the aircraft has settled; any exit from the radius or speed
    // window restarts the count.
    float dwell_s          = 0.0f;
    // Radius given to waypoints added from the dashboard or `wpadd`. 5 m is
    // the smallest that still survives the 1 Hz GNSS fallback at 5 m/s.
    float default_radius_m = 5.0f;

    static constexpr float SPEED_MIN  = 0.1f,  SPEED_MAX  = 15.0f;
    static constexpr float DWELL_MIN  = 0.0f,  DWELL_MAX  = 30.0f;
    static constexpr float RADIUS_MIN = 0.5f,  RADIUS_MAX = 100.0f;
};

// Full mission config
struct MissionConfig {
    String          mission_name;
    uint8_t         min_gps_fix_duration_s = 10;
    PositionSource  position_source        = PositionSource::GNSS;
    ModuleConfig    module_a;
    ModuleConfig    module_b;
    RcTriggerConfig rc_trigger;
    ReleaseConfig   release;
    DropPoint       drops[10];
    uint8_t         drop_count             = 0;
    bool            loaded                 = false;
};

class ConfigManager {
public:
    // Mount LittleFS and load /mission.json.
    // If /mission.json does not exist and defaultJson is provided (a PROGMEM
    // JSON string), seeds the file from it first — lets the dashboard work
    // from a fresh flash with no separate LittleFS-upload step.
    // Returns true on success.
    bool begin(const char* defaultJson = nullptr);

    // Reload config without rebooting (useful after Wi-Fi upload)
    bool reload();

    // Serialize current config back to /mission.json (used by /savecal).
    bool save();

    // Access loaded config (read-only)
    const MissionConfig& config() const { return _cfg; }

    // Mutators for the dashboard's "Save Calibration" flow — update in RAM,
    // caller must still call save() to persist.
    void setModuleA(uint16_t open_pwm_us, uint16_t stop_pwm_us,
                    uint32_t travel_ms, bool reverse, uint16_t offset_us);
    void setModuleB(uint16_t open_pwm_us, uint16_t stop_pwm_us,
                    uint32_t travel_ms, bool reverse, uint16_t offset_us);
    void setRcTrigger(bool enabled, uint16_t safe_max_us, uint16_t fire_min_us,
                      uint8_t frames_to_confirm, uint16_t signal_timeout_ms);
    // Clamps each value into ReleaseConfig's limits; read config().release
    // back for what was actually stored.
    void setRelease(float max_speed_mps, float dwell_s, float default_radius_m);

    // Waypoint editing (dashboard Waypoints tab). All operate in RAM; the
    // caller must still call save() to persist. Drop ids are renumbered
    // 1..n on every change so they always match list position — the flight
    // log records the id, and an id that no longer matches what is on the
    // board makes a post-flight log impossible to interpret.
    bool    addDrop(double lat_deg, double lon_deg, float alt_m, float radius_m);
    bool    removeDrop(uint8_t idx);
    void    clearDrops();
    bool    setDropRadius(uint8_t idx, float radius_m);
    void    setAllDropRadius(float radius_m);
    uint8_t maxDrops() const { return MAX_DROPS; }

    static const uint8_t MAX_DROPS = 10;

    // Print config summary to Serial for verification
    void printSummary() const;

private:
    MissionConfig _cfg;
    bool _parse(const char* path);
};
