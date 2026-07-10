#pragma once
#include <Arduino.h>
#include <LittleFS.h>

// Drop event record written to flash log
struct DropEvent {
    uint32_t timestamp_ms;
    uint8_t  drop_id;
    uint8_t  module;          // 0 = A, 1 = B
    uint8_t  door_num;
    double   lat_deg;
    double   lon_deg;
    float    alt_m;
    bool     success;
    char     result_str[12];  // "OK" or "TIMEOUT"
};

class Logger {
public:
    // Opens (or creates) /logs/flight.log on LittleFS
    bool begin();

    // Log a drop event to serial and flash
    void logDrop(const DropEvent& evt);

    // Log a general message to serial only
    void log(const char* tag, const char* msg);

    // Print entire log file to Serial (for post-flight review)
    void dumpLog();

    // Clear log file (call before each new mission)
    void clearLog();

private:
    bool _ready = false;
    static const char* LOG_PATH;
};
