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
};

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
    // Mount LittleFS and load /mission.json
    // Returns true on success
    bool begin();

    // Reload config without rebooting (useful after Wi-Fi upload)
    bool reload();

    // Access loaded config (read-only)
    const MissionConfig& config() const { return _cfg; }

    // Print config summary to Serial for verification
    void printSummary() const;

private:
    MissionConfig _cfg;
    bool _parse(const char* path);
};
