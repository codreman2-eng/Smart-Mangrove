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
    // Absent block = defaults (DISABLED, 1300/1700 — see RcTriggerConfig).
    // The thresholds are only a starting point: they MUST be checked against
    // the real A3 channel with RC_Input_Calibration_Tool.ino before being
    // trusted. An existing mission.json keeps whatever "enabled" it saved.
    JsonObject rc = doc["rc_trigger"];
    if (!rc.isNull()) {
        _cfg.rc_trigger.enabled           = rc["enabled"]           | false;
        _cfg.rc_trigger.safe_max_us       = rc["safe_max_us"]       | 1300;
        _cfg.rc_trigger.fire_min_us       = rc["fire_min_us"]       | 1700;
        _cfg.rc_trigger.frames_to_confirm = rc["frames_to_confirm"] | 5;
        _cfg.rc_trigger.signal_timeout_ms = rc["signal_timeout_ms"] | 500;
    }

    // --- Release gate ---
    // Absent block = ReleaseConfig defaults, which are the 2026-09-30 flight
    // build's behaviour (2.0 m/s, no dwell), so a board carrying an older
    // mission.json keeps firing exactly as before. Routed through
    // setRelease() so a hand-edited file cannot store an out-of-range value.
    JsonObject rel = doc["release"];
    if (!rel.isNull()) {
        const ReleaseConfig def;
        setRelease(rel["max_speed_mps"]    | def.max_speed_mps,
                   rel["dwell_s"]          | def.dwell_s,
                   rel["default_radius_m"] | def.default_radius_m);
    }

    // --- Hover release mode ---
    // Absent block = HoverConfig defaults (disabled). Routed through
    // setHover() for the same reason as the release block.
    JsonObject hv = doc["hover"];
    if (!hv.isNull()) {
        const HoverConfig def;
        HoverConfig h;
        h.enabled          = hv["enabled"]          | def.enabled;
        h.min_flight_min   = hv["min_flight_min"]   | def.min_flight_min;
        h.min_height_m     = hv["min_height_m"]     | def.min_height_m;
        h.max_height_m     = hv["max_height_m"]     | def.max_height_m;
        h.min_home_dist_m  = hv["min_home_dist_m"]  | def.min_home_dist_m;
        h.max_speed_mps    = hv["max_speed_mps"]    | def.max_speed_mps;
        h.max_vspeed_mps   = hv["max_vspeed_mps"]   | def.max_vspeed_mps;
        h.hold_s           = hv["hold_s"]           | def.hold_s;
        h.rearm_dist_m     = hv["rearm_dist_m"]     | def.rearm_dist_m;
        h.takeoff_height_m = hv["takeoff_height_m"] | def.takeoff_height_m;
        h.use_b            = hv["use_b"]            | def.use_b;
        setHover(h);
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

    // Hover mode needs no waypoints, so an empty list is a valid mission there
    _cfg.loaded = (_cfg.drop_count > 0) || _cfg.hover.enabled;
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

    JsonObject rel = doc["release"].to<JsonObject>();
    rel["max_speed_mps"]    = _cfg.release.max_speed_mps;
    rel["dwell_s"]          = _cfg.release.dwell_s;
    rel["default_radius_m"] = _cfg.release.default_radius_m;

    JsonObject hv = doc["hover"].to<JsonObject>();
    hv["enabled"]          = _cfg.hover.enabled;
    hv["min_flight_min"]   = _cfg.hover.min_flight_min;
    hv["min_height_m"]     = _cfg.hover.min_height_m;
    hv["max_height_m"]     = _cfg.hover.max_height_m;
    hv["min_home_dist_m"]  = _cfg.hover.min_home_dist_m;
    hv["max_speed_mps"]    = _cfg.hover.max_speed_mps;
    hv["max_vspeed_mps"]   = _cfg.hover.max_vspeed_mps;
    hv["hold_s"]           = _cfg.hover.hold_s;
    hv["rearm_dist_m"]     = _cfg.hover.rearm_dist_m;
    hv["takeoff_height_m"] = _cfg.hover.takeoff_height_m;
    hv["use_b"]            = _cfg.hover.use_b;

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

void ConfigManager::setRelease(float max_speed_mps, float dwell_s, float default_radius_m) {
    // !(x >= lo) also catches NaN, which constrain() would pass straight through
    auto clampf = [](float v, float lo, float hi) {
        return !(v >= lo) ? lo : (v > hi ? hi : v);
    };
    _cfg.release.max_speed_mps    = clampf(max_speed_mps,
                                           ReleaseConfig::SPEED_MIN,  ReleaseConfig::SPEED_MAX);
    _cfg.release.dwell_s          = clampf(dwell_s,
                                           ReleaseConfig::DWELL_MIN,  ReleaseConfig::DWELL_MAX);
    _cfg.release.default_radius_m = clampf(default_radius_m,
                                           ReleaseConfig::RADIUS_MIN, ReleaseConfig::RADIUS_MAX);
}

void ConfigManager::setHover(const HoverConfig& h) {
    auto clampf = [](float v, float lo, float hi) {
        return !(v >= lo) ? lo : (v > hi ? hi : v);
    };
    HoverConfig& c = _cfg.hover;
    c.enabled          = h.enabled;
    c.min_flight_min   = clampf(h.min_flight_min,   0.0f, HoverConfig::FLIGHT_MAX);
    c.min_height_m     = clampf(h.min_height_m,     0.0f, HoverConfig::HEIGHT_MAX);
    // Never below the minimum: an empty height window could never fire, and
    // would look like an OSDK fault in the field rather than a typo.
    c.max_height_m     = clampf(h.max_height_m,     c.min_height_m, HoverConfig::HEIGHT_MAX);
    c.min_home_dist_m  = clampf(h.min_home_dist_m,  0.0f, HoverConfig::HOME_MAX);
    c.max_speed_mps    = clampf(h.max_speed_mps,    HoverConfig::SPEED_MIN, HoverConfig::SPEED_MAX);
    c.max_vspeed_mps   = clampf(h.max_vspeed_mps,   HoverConfig::VSPEED_MIN, HoverConfig::VSPEED_MAX);
    c.hold_s           = clampf(h.hold_s,           0.0f, HoverConfig::HOLD_MAX);
    c.rearm_dist_m     = clampf(h.rearm_dist_m,     0.0f, HoverConfig::REARM_MAX);
    c.takeoff_height_m = clampf(h.takeoff_height_m, HoverConfig::TAKEOFF_MIN, HoverConfig::TAKEOFF_MAX);
    c.use_b            = h.use_b;
    if (c.enabled) _cfg.loaded = true;
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

void ConfigManager::setAllDropRadius(float radius_m) {
    for (uint8_t i = 0; i < _cfg.drop_count; i++) _cfg.drops[i].radius_m = radius_m;
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
    Serial.printf("  Release     : speed<=%.1f m/s  dwell=%.1f s  default r=%.1f m\n",
                  _cfg.release.max_speed_mps, _cfg.release.dwell_s,
                  _cfg.release.default_radius_m);
    const HoverConfig& h = _cfg.hover;
    Serial.printf("  Hover mode  : %s  after %.2f min  h %.1f-%.1f m  home>=%.0f m  "
                  "speed<=%.1f vert<=%.1f m/s for %.1f s  rearm %.0f m  takeoff %.1f m  %s\n",
                  h.enabled ? "ENABLED (waypoints ignored)" : "off",
                  h.min_flight_min, h.min_height_m, h.max_height_m, h.min_home_dist_m,
                  h.max_speed_mps, h.max_vspeed_mps, h.hold_s, h.rearm_dist_m,
                  h.takeoff_height_m, h.use_b ? "A+B" : "A only");
    Serial.printf("  Drop points : %d\n", _cfg.drop_count);
    for (uint8_t i = 0; i < _cfg.drop_count; i++) {
        const DropPoint& dp = _cfg.drops[i];
        Serial.printf("    [%2d] %.6f, %.6f  r=%.1fm\n",
                      dp.id, dp.lat_deg, dp.lon_deg, dp.radius_m);
    }
    Serial.println("======================");
}
