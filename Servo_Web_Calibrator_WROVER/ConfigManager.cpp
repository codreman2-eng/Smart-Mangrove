#include "ConfigManager.h"

bool ConfigManager::begin(const char* defaultJson) {
    if (!LittleFS.begin(true)) {
        Serial.println("[CFG] LittleFS mount FAILED");
        return false;
    }
    Serial.println("[CFG] LittleFS mounted OK");

    if (defaultJson && !LittleFS.exists("/mission.json")) {
        Serial.println("[CFG] /mission.json not found — seeding default");
        File f = LittleFS.open("/mission.json", "w");
        if (f) {
            f.print(defaultJson);
            f.close();
            Serial.println("[CFG] Default mission.json written");
        } else {
            Serial.println("[CFG] Failed to write default mission.json");
        }
    }

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
        _cfg.module_a.reverse           = ma["reverse"]           | false;
        _cfg.module_a.offset_us         = ma["offset_us"]         | 200;
    }

    // --- Module B config ---
    JsonObject mb = doc["modules"]["B"];
    if (!mb.isNull()) {
        _cfg.module_b.servo_open_pwm_us = mb["servo_open_pwm_us"] | 1300;
        _cfg.module_b.servo_stop_pwm_us = mb["servo_stop_pwm_us"] | 1500;
        _cfg.module_b.travel_ms         = mb["travel_ms"]         | 350;
        _cfg.module_b.door_count        = mb["door_count"]        | 5;
        _cfg.module_b.reverse           = mb["reverse"]           | false;
        _cfg.module_b.offset_us         = mb["offset_us"]         | 200;
    }

    // --- RC pilot trigger ---
    // Absent block = defaults (enabled, 1300/1700). Note the defaults are
    // only a starting point: they MUST be checked against the real A3
    // channel with RC_Input_Calibration_Tool.ino before being trusted.
    JsonObject rc = doc["rc_trigger"];
    if (!rc.isNull()) {
        _cfg.rc_trigger.enabled           = rc["enabled"]           | true;
        _cfg.rc_trigger.safe_max_us       = rc["safe_max_us"]       | 1300;
        _cfg.rc_trigger.fire_min_us       = rc["fire_min_us"]       | 1700;
        _cfg.rc_trigger.frames_to_confirm = rc["frames_to_confirm"] | 5;
        _cfg.rc_trigger.signal_timeout_ms = rc["signal_timeout_ms"] | 500;
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

bool ConfigManager::save() {
    JsonDocument doc;
    doc["mission_name"]           = _cfg.mission_name;
    doc["min_gps_fix_duration_s"] = _cfg.min_gps_fix_duration_s;
    doc["position_source"]        = (_cfg.position_source == PositionSource::MAVLINK)
                                     ? "MAVLINK" : "GNSS";

    JsonObject ma = doc["modules"]["A"].to<JsonObject>();
    ma["servo_open_pwm_us"] = _cfg.module_a.servo_open_pwm_us;
    ma["servo_stop_pwm_us"] = _cfg.module_a.servo_stop_pwm_us;
    ma["travel_ms"]         = _cfg.module_a.travel_ms;
    ma["door_count"]        = _cfg.module_a.door_count;
    ma["reverse"]           = _cfg.module_a.reverse;
    ma["offset_us"]         = _cfg.module_a.offset_us;

    JsonObject mb = doc["modules"]["B"].to<JsonObject>();
    mb["servo_open_pwm_us"] = _cfg.module_b.servo_open_pwm_us;
    mb["servo_stop_pwm_us"] = _cfg.module_b.servo_stop_pwm_us;
    mb["travel_ms"]         = _cfg.module_b.travel_ms;
    mb["door_count"]        = _cfg.module_b.door_count;
    mb["reverse"]           = _cfg.module_b.reverse;
    mb["offset_us"]         = _cfg.module_b.offset_us;

    JsonObject rc = doc["rc_trigger"].to<JsonObject>();
    rc["enabled"]           = _cfg.rc_trigger.enabled;
    rc["safe_max_us"]       = _cfg.rc_trigger.safe_max_us;
    rc["fire_min_us"]       = _cfg.rc_trigger.fire_min_us;
    rc["frames_to_confirm"] = _cfg.rc_trigger.frames_to_confirm;
    rc["signal_timeout_ms"] = _cfg.rc_trigger.signal_timeout_ms;

    JsonArray drops = doc["drops"].to<JsonArray>();
    for (uint8_t i = 0; i < _cfg.drop_count; i++) {
        JsonObject d = drops.add<JsonObject>();
        d["id"]       = _cfg.drops[i].id;
        d["lat_deg"]  = _cfg.drops[i].lat_deg;
        d["lon_deg"]  = _cfg.drops[i].lon_deg;
        d["alt_m"]    = _cfg.drops[i].alt_m;
        d["radius_m"] = _cfg.drops[i].radius_m;
    }

    File f = LittleFS.open("/mission.json", "w");
    if (!f) {
        Serial.println("[CFG] save(): cannot open /mission.json for write");
        return false;
    }
    bool ok = serializeJson(doc, f) > 0;
    f.close();
    Serial.printf("[CFG] save(): %s\n", ok ? "OK" : "FAILED");
    return ok;
}

void ConfigManager::setModuleA(uint16_t open_pwm_us, uint16_t stop_pwm_us,
                               uint32_t travel_ms, bool reverse, uint16_t offset_us) {
    _cfg.module_a.servo_open_pwm_us = open_pwm_us;
    _cfg.module_a.servo_stop_pwm_us = stop_pwm_us;
    _cfg.module_a.travel_ms         = travel_ms;
    _cfg.module_a.reverse           = reverse;
    _cfg.module_a.offset_us         = offset_us;
}

void ConfigManager::setModuleB(uint16_t open_pwm_us, uint16_t stop_pwm_us,
                               uint32_t travel_ms, bool reverse, uint16_t offset_us) {
    _cfg.module_b.servo_open_pwm_us = open_pwm_us;
    _cfg.module_b.servo_stop_pwm_us = stop_pwm_us;
    _cfg.module_b.travel_ms         = travel_ms;
    _cfg.module_b.reverse           = reverse;
    _cfg.module_b.offset_us         = offset_us;
}

void ConfigManager::setRcTrigger(bool enabled, uint16_t safe_max_us, uint16_t fire_min_us,
                                 uint8_t frames_to_confirm, uint16_t signal_timeout_ms) {
    _cfg.rc_trigger.enabled           = enabled;
    _cfg.rc_trigger.safe_max_us       = safe_max_us;
    _cfg.rc_trigger.fire_min_us       = fire_min_us;
    _cfg.rc_trigger.frames_to_confirm = frames_to_confirm;
    _cfg.rc_trigger.signal_timeout_ms = signal_timeout_ms;
}

bool ConfigManager::addDrop(double lat_deg, double lon_deg, float alt_m, float radius_m) {
    if (_cfg.drop_count >= MAX_DROPS) return false;
    DropPoint& dp = _cfg.drops[_cfg.drop_count];
    dp.lat_deg   = lat_deg;
    dp.lon_deg   = lon_deg;
    dp.alt_m     = alt_m;
    dp.radius_m  = radius_m;
    dp.completed = false;
    dp.id        = ++_cfg.drop_count;      // ids stay 1..n, matching position
    _cfg.loaded  = true;
    Serial.printf("[CFG] +WP%d %.7f, %.7f r=%.1fm\n",
                  dp.id, lat_deg, lon_deg, radius_m);
    return true;
}

bool ConfigManager::removeDrop(uint8_t idx) {
    if (idx >= _cfg.drop_count) return false;
    for (uint8_t i = idx; i + 1 < _cfg.drop_count; i++) _cfg.drops[i] = _cfg.drops[i + 1];
    _cfg.drop_count--;
    for (uint8_t i = 0; i < _cfg.drop_count; i++) _cfg.drops[i].id = i + 1;
    Serial.printf("[CFG] -WP index %d, %d remain\n", idx, _cfg.drop_count);
    return true;
}

void ConfigManager::clearDrops() {
    _cfg.drop_count = 0;
    Serial.println("[CFG] all waypoints cleared");
}

bool ConfigManager::setDropRadius(uint8_t idx, float radius_m) {
    if (idx >= _cfg.drop_count) return false;
    _cfg.drops[idx].radius_m = radius_m;
    return true;
}

void ConfigManager::printSummary() const {
    if (!_cfg.loaded) { Serial.println("[CFG] No config loaded"); return; }
    Serial.println("=== MISSION CONFIG ===");
    Serial.printf("  Name        : %s\n", _cfg.mission_name.c_str());
    Serial.printf("  GPS fix min : %d s\n", _cfg.min_gps_fix_duration_s);
    Serial.printf("  Pos source  : %s\n",
                  _cfg.position_source == PositionSource::MAVLINK ? "MAVLINK" : "GNSS");
    Serial.printf("  Module A    : open=%d stop=%d travel=%lums doors=%d rev=%d off=%d\n",
                  _cfg.module_a.servo_open_pwm_us, _cfg.module_a.servo_stop_pwm_us,
                  (unsigned long)_cfg.module_a.travel_ms, _cfg.module_a.door_count,
                  _cfg.module_a.reverse, _cfg.module_a.offset_us);
    Serial.printf("  Module B    : open=%d stop=%d travel=%lums doors=%d rev=%d off=%d\n",
                  _cfg.module_b.servo_open_pwm_us, _cfg.module_b.servo_stop_pwm_us,
                  (unsigned long)_cfg.module_b.travel_ms, _cfg.module_b.door_count,
                  _cfg.module_b.reverse, _cfg.module_b.offset_us);
    Serial.printf("  RC trigger  : %s  SAFE<=%d  FIRE>=%d  confirm=%d  timeout=%dms\n",
                  _cfg.rc_trigger.enabled ? "enabled" : "DISABLED",
                  _cfg.rc_trigger.safe_max_us, _cfg.rc_trigger.fire_min_us,
                  _cfg.rc_trigger.frames_to_confirm, _cfg.rc_trigger.signal_timeout_ms);
    Serial.printf("  Drop points : %d\n", _cfg.drop_count);
    for (uint8_t i = 0; i < _cfg.drop_count; i++) {
        const DropPoint& dp = _cfg.drops[i];
        Serial.printf("    [%2d] %.6f, %.6f  r=%.1fm\n",
                      dp.id, dp.lat_deg, dp.lon_deg, dp.radius_m);
    }
    Serial.println("======================");
}
