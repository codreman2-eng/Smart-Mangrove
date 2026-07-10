#include "ConfigManager.h"

bool ConfigManager::begin() {
    if (!LittleFS.begin(true)) {
        Serial.println("[CFG] LittleFS mount FAILED");
        return false;
    }
    Serial.println("[CFG] LittleFS mounted OK");
    return _parse("/mission.json");
}

bool ConfigManager::reload() {
    return _parse("/mission.json");
}

bool ConfigManager::_parse(const char* path) {
    _cfg = MissionConfig{};   // reset

    File f = LittleFS.open(path, "r");
    if (!f) {
        Serial.printf("[CFG] Cannot open %s\n", path);
        return false;
    }

    // ArduinoJson 7 — use JsonDocument directly
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();

    if (err) {
        Serial.printf("[CFG] JSON parse error: %s\n", err.c_str());
        return false;
    }

    // --- Top-level fields ---
    _cfg.mission_name             = doc["mission_name"] | "unnamed";
    _cfg.min_gps_fix_duration_s   = doc["min_gps_fix_duration_s"] | 10;

    String src = doc["position_source"] | "GNSS";
    _cfg.position_source = (src == "MAVLINK") ? PositionSource::MAVLINK
                                               : PositionSource::GNSS;

    // --- Module A config ---
    JsonObject ma = doc["modules"]["A"];
    if (!ma.isNull()) {
        _cfg.module_a.servo_open_pwm_us = ma["servo_open_pwm_us"] | 1700;
        _cfg.module_a.servo_stop_pwm_us = ma["servo_stop_pwm_us"] | 1500;
        _cfg.module_a.travel_ms         = ma["travel_ms"]         | 350;
        _cfg.module_a.door_count        = ma["door_count"]        | 5;
    }

    // --- Module B config ---
    JsonObject mb = doc["modules"]["B"];
    if (!mb.isNull()) {
        _cfg.module_b.servo_open_pwm_us = mb["servo_open_pwm_us"] | 1300;
        _cfg.module_b.servo_stop_pwm_us = mb["servo_stop_pwm_us"] | 1500;
        _cfg.module_b.travel_ms         = mb["travel_ms"]         | 350;
        _cfg.module_b.door_count        = mb["door_count"]        | 5;
    }

    // --- Drop points ---
    JsonArray drops = doc["drops"];
    _cfg.drop_count = 0;
    for (JsonObject d : drops) {
        if (_cfg.drop_count >= 10) break;
        DropPoint& dp      = _cfg.drops[_cfg.drop_count++];
        dp.id              = d["id"]      | 0;
        dp.lat_deg         = d["lat_deg"] | 0.0;
        dp.lon_deg         = d["lon_deg"] | 0.0;
        dp.alt_m           = d["alt_m"]   | 0.0f;
        dp.radius_m        = d["radius_m"]| 3.0f;
        dp.completed       = false;
    }

    _cfg.loaded = (_cfg.drop_count > 0);
    Serial.printf("[CFG] Loaded '%s' — %d drop points\n",
                  _cfg.mission_name.c_str(), _cfg.drop_count);
    return _cfg.loaded;
}

void ConfigManager::printSummary() const {
    if (!_cfg.loaded) { Serial.println("[CFG] No config loaded"); return; }
    Serial.println("=== MISSION CONFIG ===");
    Serial.printf("  Name        : %s\n", _cfg.mission_name.c_str());
    Serial.printf("  GPS fix min : %d s\n", _cfg.min_gps_fix_duration_s);
    Serial.printf("  Pos source  : %s\n",
                  _cfg.position_source == PositionSource::MAVLINK ? "MAVLINK" : "GNSS");
    Serial.printf("  Module A    : open=%d stop=%d travel=%dms doors=%d\n",
                  _cfg.module_a.servo_open_pwm_us, _cfg.module_a.servo_stop_pwm_us,
                  _cfg.module_a.travel_ms, _cfg.module_a.door_count);
    Serial.printf("  Module B    : open=%d stop=%d travel=%dms doors=%d\n",
                  _cfg.module_b.servo_open_pwm_us, _cfg.module_b.servo_stop_pwm_us,
                  _cfg.module_b.travel_ms, _cfg.module_b.door_count);
    Serial.printf("  Drop points : %d\n", _cfg.drop_count);
    for (uint8_t i = 0; i < _cfg.drop_count; i++) {
        const DropPoint& dp = _cfg.drops[i];
        Serial.printf("    [%2d] %.6f, %.6f  r=%.1fm\n",
                      dp.id, dp.lat_deg, dp.lon_deg, dp.radius_m);
    }
    Serial.println("======================");
}
