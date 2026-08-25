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

    // Handle held open, committed with flush(), rather than open/append/close
    // per row. The close-every-time version cost 80-200ms once the file grew
    // past a few hundred KB — LittleFS has to walk metadata on every open —
    // and that stall lands inside loop(), so it became drop latency.
    // flush() still calls lfs_file_sync(), so each row is committed to flash
    // before the call returns; the crash-safety argument for closing was
    // satisfied by flush() all along.
    if (!_fh) {
        _fh = LittleFS.open(LOG_PATH, "a");
        if (!_fh) { Serial.println("[LOG] Cannot open log file"); return; }
    }
    size_t n = _fh.println(line);
    // Inside a drive window the sync is deferred to setDropWindow(false).
    // lfs_file_sync() is not free, and this call site sits in loop().
    if (!_drop_window) _fh.flush();
    _bytes += n;
}

void Logger::setDropWindow(bool active) {
    if (active == _drop_window) return;
    _drop_window = active;
    // Falling edge: commit everything the window accumulated. The drive
    // window is over, so a stall here costs nothing that matters.
    if (!active && _fh) _fh.flush();
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
    // Echo suppressed inside a drive window — see setDropWindow(). The row
    // still reaches flash on the next line.
    if (!_drop_window) Serial.printf("[EVT] %s\n", line);
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

void Logger::streamToHttp(WebServer& server, bool events_only, uint32_t tail_bytes) {
    if (_fh) _fh.flush();          // reader must see everything written so far
    File f = LittleFS.open(LOG_PATH, "r");
    if (!f) {
        server.send(200, "text/plain", "(no log file yet)");
        return;
    }

    // Bound the read from the END. This handler runs inside loop(), so its
    // cost is stolen directly from position updates and drop detection — and
    // the previous version read the WHOLE file one byte at a time, which at
    // 80KB was hundreds of milliseconds to over a second of stall, growing
    // with every flight. Seeking to the tail makes the cost constant.
    size_t sz = f.size();
    size_t start = 0;
    bool truncated = false;
    if (tail_bytes && sz > tail_bytes) { start = sz - tail_bytes; truncated = true; }

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/plain", "");

    f.seek(start);
    if (truncated) {
        while (f.available() && (char)f.read() != '\n') {}   // drop partial line
        server.sendContent("... older entries omitted (fetch /log?tail=0 for all)\n");
    }

    // Block reads, and lines accumulated into one output buffer before being
    // sent. Both matter: byte-at-a-time reads and per-line sendContent() are
    // each an order of magnitude slower than doing it in chunks.
    char   in[256];
    char   line[288];
    String out;
    out.reserve(1200);
    uint16_t len = 0;
    uint16_t kept = 0;
    size_t n;

    while ((n = f.readBytes(in, sizeof(in))) > 0) {
        for (size_t i = 0; i < n; i++) {
            char c = in[i];
            if (c == '\n' || len >= sizeof(line) - 1) {
                line[len] = '\0';
                // Over-long lines are flushed, not silently truncated — half
                // a line of JSON in a flight log is worse than a long one.
                if (len && (!events_only || !strstr(line, "\"ev\":\"tlm\""))) {
                    out += line; out += '\n'; kept++;
                    if (out.length() >= 1024) { server.sendContent(out); out = ""; }
                }
                len = 0;
            } else if (c != '\r') {
                line[len++] = c;
            }
        }
    }
    if (len) {                                   // trailing line, no newline
        line[len] = '\0';
        if (!events_only || !strstr(line, "\"ev\":\"tlm\"")) { out += line; out += '\n'; kept++; }
    }
    if (out.length()) server.sendContent(out);
    f.close();
    if (kept == 0) server.sendContent("(no events logged yet)\n");
}

void Logger::clearLog() {
    if (_fh) _fh.close();          // must release the append handle first
    LittleFS.remove(LOG_PATH);
    _bytes = 0;
    _full  = false;
    Serial.println("[LOG] Log cleared");
    logEvent("log_cleared");       // reopens the handle
}
