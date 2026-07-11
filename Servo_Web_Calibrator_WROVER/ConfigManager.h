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

// Full mission config
struct MissionConfig {
    String          mission_name;
    uint8_t         min_gps_fix_duration_s = 10;
    PositionSource  position_source        = PositionSource::GNSS;
    ModuleConfig    module_a;
    ModuleConfig    module_b;
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

    // Print config summary to Serial for verification
    void printSummary() const;

private:
    MissionConfig _cfg;
    bool _parse(const char* path);
};
