#include "Logger.h"

const char* Logger::LOG_PATH = "/logs/flight.log";

bool Logger::begin() {
    // Ensure /logs directory exists
    if (!LittleFS.exists("/logs")) {
        LittleFS.mkdir("/logs");
    }
    _ready = true;
    Serial.println("[LOG] Logger ready");
    return true;
}

void Logger::logDrop(const DropEvent& evt) {
    // Always print to serial
    Serial.printf("[DROP] id=%d mod=%c door=%d lat=%.6f lon=%.6f alt=%.1f %s t=%lu\n",
                  evt.drop_id,
                  evt.module == 0 ? 'A' : 'B',
                  evt.door_num,
                  evt.lat_deg, evt.lon_deg, evt.alt_m,
                  evt.result_str,
                  evt.timestamp_ms);

    // Write to flash log
    if (!_ready) return;
    File f = LittleFS.open(LOG_PATH, "a");
    if (!f) { Serial.println("[LOG] Cannot open log file"); return; }
    f.printf("%lu,%d,%c,%d,%.6f,%.6f,%.1f,%s\n",
             evt.timestamp_ms,
             evt.drop_id,
             evt.module == 0 ? 'A' : 'B',
             evt.door_num,
             evt.lat_deg, evt.lon_deg, evt.alt_m,
             evt.result_str);
    f.close();
}

void Logger::log(const char* tag, const char* msg) {
    Serial.printf("[%s] %s\n", tag, msg);
}

void Logger::dumpLog() {
    File f = LittleFS.open(LOG_PATH, "r");
    if (!f) { Serial.println("[LOG] No log file found"); return; }
    Serial.println("=== FLIGHT LOG ===");
    Serial.println("ms,id,mod,door,lat,lon,alt,result");
    while (f.available()) Serial.write(f.read());
    f.close();
    Serial.println("==================");
}

void Logger::clearLog() {
    LittleFS.remove(LOG_PATH);
    Serial.println("[LOG] Log cleared");
}
