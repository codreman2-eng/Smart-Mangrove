#include "Logger.h"

// .jsonl, not .log — the extension states the format, and it keeps the old
// CSV file (if one is still on the device) from being appended to with
// incompatible rows.
const char* Logger::LOG_PATH = "/logs/flight.jsonl";

bool Logger::begin() {
    if (!LittleFS.exists("/logs")) {
        LittleFS.mkdir("/logs");
    }
    _ready = true;

    // Pick up where a previous session left off so the size guard survives a
    // reboot mid-mission (a brown-out and restart must not reset the budget).
    File f = LittleFS.open(LOG_PATH, "r");
    if (f) { _bytes = f.size(); f.close(); }
    _full = (_bytes >= MAX_LOG_BYTES);

    Serial.printf("[LOG] JSONL logger ready — %s, %lu bytes used%s\n",
                  LOG_PATH, (unsigned long)_bytes, _full ? " (FULL)" : "");

    logEvent("boot");
    return true;
}

void Logger::_writeLine(const char* line) {
    if (!_ready) return;
    if (_full) return;

    if (_bytes >= MAX_LOG_BYTES) {
        _full = true;
        Serial.println("[LOG] *** LOG FULL — no further rows will be written ***");
        return;
    }

    File f = LittleFS.open(LOG_PATH, "a");
    if (!f) { Serial.println("[LOG] Cannot open log file"); return; }
    size_t n = f.println(line);
    f.close();          // commit now — see the crash-safety note in Logger.h
    _bytes += n;
}

void Logger::logTelemetry(const TelemetrySample& s) {
    char line[320];
    snprintf(line, sizeof(line),
        "{\"t\":%lu,\"ev\":\"tlm\",\"st\":%u,\"arm\":%s,"
        "\"gps\":%s,\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"sats\":%u,\"hdop\":%.1f,"
        "\"rc\":%s,\"rcus\":%u,\"band\":\"%s\",\"rchz\":%u,\"rcgl\":%lu,"
        "\"dA\":%u,\"dB\":%u,\"idx\":%u,\"lA\":%s,\"lB\":%s}",
        (unsigned long)millis(), s.state, s.armed ? "true" : "false",
        s.gps_valid ? "true" : "false", s.lat_deg, s.lon_deg, s.alt_m, s.sats, s.hdop,
        s.rc_signal ? "true" : "false", s.rc_us, s.rc_band, s.rc_hz,
        (unsigned long)s.rc_glitches,
        s.doors_a, s.doors_b, s.drop_idx,
        s.limit_a ? "true" : "false", s.limit_b ? "true" : "false");
    _writeLine(line);
}

void Logger::logDrop(const DropEvent& evt) {
    // Always print to serial, regardless of flash state
    Serial.printf("[DROP] id=%d mod=%c door=%d lat=%.6f lon=%.6f alt=%.1f %s t=%lu\n",
                  evt.drop_id,
                  evt.module == 0 ? 'A' : 'B',
                  evt.door_num,
                  evt.lat_deg, evt.lon_deg, evt.alt_m,
                  evt.result_str,
                  evt.timestamp_ms);

    char line[256];
    snprintf(line, sizeof(line),
        "{\"t\":%lu,\"ev\":\"drop\",\"id\":%u,\"mod\":\"%c\",\"door\":%u,"
        "\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"ok\":%s,\"res\":\"%s\"}",
        (unsigned long)evt.timestamp_ms, evt.drop_id,
        evt.module == 0 ? 'A' : 'B', evt.door_num,
        evt.lat_deg, evt.lon_deg, evt.alt_m,
        evt.success ? "true" : "false", evt.result_str);
    _writeLine(line);
}

void Logger::logEvent(const char* type, const char* extra_json) {
    char line[256];
    if (extra_json && extra_json[0]) {
        snprintf(line, sizeof(line), "{\"t\":%lu,\"ev\":\"%s\",%s}",
                 (unsigned long)millis(), type, extra_json);
    } else {
        snprintf(line, sizeof(line), "{\"t\":%lu,\"ev\":\"%s\"}",
                 (unsigned long)millis(), type);
    }
    Serial.printf("[EVT] %s\n", line);
    _writeLine(line);
}

void Logger::log(const char* tag, const char* msg) {
    Serial.printf("[%s] %s\n", tag, msg);
}

void Logger::dumpLog() {
    File f = LittleFS.open(LOG_PATH, "r");
    if (!f) { Serial.println("[LOG] No log file found"); return; }
    Serial.println("=== FLIGHT LOG (JSONL) ===");
    while (f.available()) Serial.write(f.read());
    f.close();
    Serial.println("==========================");
}

void Logger::streamToHttp(WebServer& server, bool events_only) {
    File f = LittleFS.open(LOG_PATH, "r");
    if (!f) {
        server.send(200, "text/plain", "(no log file yet)");
        return;
    }
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/plain", "");

    if (!events_only) {
        char buf[256];
        size_t n;
        while ((n = f.readBytes(buf, sizeof(buf))) > 0) server.sendContent(buf, n);
        f.close();
        return;
    }

    // Line-at-a-time so telemetry can be dropped without loading the file.
    // Over-long lines are flushed rather than truncated silently — a
    // half-line of JSON in a flight log is worse than an obviously long one.
    char line[288];
    uint16_t len = 0;
    uint16_t kept = 0;
    while (f.available()) {
        char c = (char)f.read();
        if (c == '\n' || len >= sizeof(line) - 1) {
            line[len] = '\0';
            if (len && !strstr(line, "\"ev\":\"tlm\"")) {
                server.sendContent(line);
                server.sendContent("\n");
                kept++;
            }
            len = 0;
        } else if (c != '\r') {
            line[len++] = c;
        }
    }
    if (len) {                       // trailing line with no newline
        line[len] = '\0';
        if (!strstr(line, "\"ev\":\"tlm\"")) { server.sendContent(line); kept++; }
    }
    f.close();
    if (kept == 0) server.sendContent("(no events logged yet)\n");
}

void Logger::clearLog() {
    LittleFS.remove(LOG_PATH);
    _bytes = 0;
    _full  = false;
    Serial.println("[LOG] Log cleared");
    logEvent("log_cleared");
}
