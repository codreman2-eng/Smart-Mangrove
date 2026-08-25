#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include <WebServer.h>

// Drop event record written to flash log
struct DropEvent {
    uint32_t timestamp_ms;
    uint8_t  drop_id;         // 0 = pilot/RC commanded, else the waypoint id
    uint8_t  module;          // 0 = A, 1 = B
    uint8_t  door_num;
    double   lat_deg;
    double   lon_deg;
    float    alt_m;
    bool     success;
    char     result_str[12];  // "OK" or "TIMEOUT"
};

// One periodic telemetry row. Grouped into a struct rather than passed as a
// dozen arguments so adding a field later doesn't touch every call site.
struct TelemetrySample {
    uint8_t     state       = 0;
    bool        armed       = false;
    bool        gps_valid   = false;
    double      lat_deg     = 0.0;
    double      lon_deg     = 0.0;
    float       alt_m       = 0.0f;
    float       hdop        = 99.0f;
    uint8_t     sats        = 0;
    bool        rc_signal   = false;
    uint16_t    rc_us       = 0;
    const char* rc_band     = "NONE";
    uint32_t    rc_glitches = 0;
    uint16_t    rc_hz       = 0;
    uint8_t     doors_a     = 0;
    uint8_t     doors_b     = 0;
    uint8_t     drop_idx    = 0;
    bool        limit_a     = false;
    bool        limit_b     = false;
};

// Flight logger — JSON Lines (one JSON object per line).
//
// JSONL rather than a single JSON array on purpose:
//   * Appendable. No closing bracket to write, so the file is valid at every
//     instant, not only after a clean shutdown.
//   * Crash-safe. If the battery is pulled or the aircraft comes down hard
//     mid-write you lose ONE line, not the whole file. That is precisely the
//     flight whose log you most need.
//   * Streams over the existing /log route unchanged, and drops straight into
//     Python/pandas (`pd.read_json(path, lines=True)`).
//
// The handle is held open and each row committed with flush(), which calls
// lfs_file_sync() — so a record still reaches flash before the write returns.
// (It formerly did open/append/close per row, which cost 80-200ms once the
// file grew past a few hundred KB, and that stall landed inside loop().)
// The one exception is a drive window — see setDropWindow().
class Logger {
public:
    bool begin();

    // Periodic telemetry row — call at TELEMETRY_INTERVAL_MS from loop()
    void logTelemetry(const TelemetrySample& s);

    // Drop event (also printed to serial)
    void logDrop(const DropEvent& evt);

    // Generic event row. `extra_json` is an optional pre-formatted fragment
    // of JSON fields, e.g. "\"from\":2,\"to\":3" — no leading/trailing comma.
    void logEvent(const char* type, const char* extra_json = nullptr);

    // Serial-only message (unchanged — does not touch flash)
    void log(const char* tag, const char* msg);

    // ── Drop window ───────────────────────────────────────────────────
    // Set true for exactly the span a servo is being driven. Inside it, two
    // things are suppressed, both of which are pure loop() stall:
    //
    //   * the [EVT] serial echo — the row is already going to flash, so the
    //     echo is a duplicate that costs ~87us/byte at 115200, and the drive
    //     window is the one time that latency is safety-critical;
    //   * flush() on each row — the write still happens, it just is not
    //     sync'd to flash until the window closes.
    //
    // Nothing is dropped or deferred: full fidelity is preserved in the file.
    // The only thing relaxed is the crash-safety guarantee, and only for the
    // sub-second span of a drop. That is the right trade: a stall inside the
    // drive window directly overshoots the servo, which is a mechanical
    // failure, whereas losing the last row of a log to a brown-out that
    // happens to land in that same sub-second is a cosmetic one.
    void setDropWindow(bool active);

    void dumpLog();

    // events_only skips the 1 Hz "tlm" rows. Filtering SERVER-side matters:
    // at 1 row/second the two lines from a drop are buried under hundreds of
    // telemetry rows, and shipping the whole file to the browser just to
    // throw most of it away is wasteful over a WiFi AP.
    //
    // tail_bytes bounds the work: the file only grows, and this runs inside
    // an HTTP handler, so an unbounded read means loop() stalls for longer
    // every time the log gets bigger. 0 = whole file.
    void streamToHttp(WebServer& server, bool events_only = false,
                      uint32_t tail_bytes = TAIL_DEFAULT);

    static const uint32_t TAIL_DEFAULT = 24576;   // ~25KB, bounded and quick
    void clearLog();

    uint32_t bytesUsed() const { return _bytes; }
    bool     isFull()    const { return _full; }

    // Partition is ~1.5MB ("Default 4MB with spiffs"). Cap well below that so
    // a runaway log cannot wedge mission.json's filesystem. At ~150 bytes per
    // row and 1 Hz this is roughly 2 hours of flight.
    static const uint32_t MAX_LOG_BYTES = 1100000UL;

private:
    bool     _ready       = false;
    bool     _full        = false;
    bool     _drop_window = false;
    uint32_t _bytes       = 0;
    File     _fh;                 // held open; flush() commits each row
    static const char* LOG_PATH;

    // Common open/append/close with the size guard applied
    void _writeLine(const char* line);
};
